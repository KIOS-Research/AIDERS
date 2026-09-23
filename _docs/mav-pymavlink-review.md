# MAV container (pymavlink): review and rework plan

## Context
The `mav` container connects MAVLink UAVs to the platform: Django (`web/django_api/aiders/views.py` → `httpRequests.postRequestForMavlink`) POSTs to the aiohttp API in `mav/app/api.py` → `requestHandler.py` → a per-drone `Uav2` object (`mav/app/uav2.py`, pymavlink). `Uav2` streams telemetry into MySQL and Kafka and runs commands. You asked whether the structure and approach are right, and how to support as many MAVLink vehicles as possible: ArduPilot Copter/Plane/QuadPlane, PX4 MC/FW/VTOL, and their different modes. The MAVSDK path (`uav.py`) is out of scope and stays as it is.

**Verdict:** The layering (web → HTTP API → requestHandler → one object per UAV → DB/Kafka) is fine. The inside of `Uav2` is not. Two threads read from the same pymavlink connection, blocking calls run on the event loop, commands are fire-and-forget, and the mission upload doesn't follow the protocol. Most of the per-autopilot bugs come from those four problems, so fix the core first and then add ArduPilot and PX4 adapters.

Checked against: pymavlink 2.4.49 (the version in the `aiders-platform-mav` image); ArduPilot master (`QuadPlane::handle_do_vtol_transition`, `MissionItemProtocol.cpp`, `GCS_MAVLink_Copter.cpp`); PX4 main (`mavlink_mission.{h,cpp}`, `navigator_main.cpp`, `rcAndDataLinkCheck.cpp`).

## Status and decisions (2026-09-14)
- **Scope: review only for now.** No code changes until you decide. The structure below is a proposal for discussion.
- **GCS heartbeat: per-drone opt-in, off by default.** This needs a boolean on the Drone model (Django model and migration, a toggle on `mavlink-manage.html`, and the flag sent in the `connectToUav` payload). With the flag off, today's behaviour stays: the vehicle's own link-loss failsafes won't react to the platform, and a PX4 vehicle with `NAV_DLL_ACT>0` only arms when QGC is also connected. The platform-side LINK_LOST detection works either way.
- **Telemetry schema: Kafka and DB together.** New fields: `flightMode`, `armed`, `landedState`, `gpsFixType`, `altitudeAmsl`. This needs Django model and migration changes for `aiders_telemetry` and `aiders_telemetrylatest`, updated INSERT/UPDATE statements in `mav/app/database/queries.py`, and the new keys in the `kafka_broadcaster.py` payload. Every Kafka topic consumer must tolerate the extra keys. Open question: should `gpsSignal` (a 0–5 level, DJI-style) also be derived from fix_type so the UI stays consistent?
- **HTTP replies: on ACK.** Flight-phase events go to the mavlink log.

## Findings

### Critical: broken today
1. **Two readers on one connection.** The telemetry thread calls `recv_match()` (uav2.py:171). At the same time, `arm`/`takeoff`/`land`/`sendMission`/`wait_until_landed` call `recv_match(type=…)` from the asyncio thread (uav2.py:292, 298, 368, 380, 425, 469, 501, 544, 556). Each message goes to whichever reader gets it first. ACKs, MISSION_REQUESTs and HEARTBEATs get stolen, which causes random timeouts and hangs. pymavlink is not thread-safe.
2. **Blocked event loop.** `Uav2.connect()` is synchronous and can take about 20 s (uav2.py:57). `arm()` calls `recv_match(blocking=True)` with no timeout, then loops forever waiting for the armed bit (uav2.py:292-302; disarm has the same loop at 317-322). A single denied arm, such as a pre-arm failure, freezes the whole API, including land and kill for other drones.
3. **`await` on sync methods.** `requestHandler.py:37,43` await `Uav2.connect()`/`disconnect()`, which return None. After they finish, Python raises `TypeError: object NoneType can't be used in 'await' expression`. So every pymavlink connect and disconnect returns `status: error`, even though the DB is already updated. The UI hides this: `mavlink-manage.html` only logs the connect response and polls the DB flag every 2 s for 20 s. The other side of that is that real failures (heartbeat timeout, drone missing from the DB) only show up as "Timed out. Please try again." with no reason. And `connect()` can wait up to about 20 s on its own timeouts, which is as long as the polling window.
4. **Missing actions.** `Uav2` has no `kill`, `reboot`, `shutdown`, `pauseMission`, `resumeMission` or `cancelMission`, so the existing routes fail with AttributeError. Django also calls `/mav/arm` and `/mav/disarm`, which have no route (404). The broken kill is the biggest safety gap.
5. **The mission upload doesn't follow the protocol** (uav2.py:446-579):
   - MISSION_COUNT says N+2 items, but the items sent are seq 0, 1..N, then seq 0 again: "re-add last waypoint" reuses seq 0. Seq N+1 is never sent.
   - The code waits for one `MISSION_REQUEST`, then sends all items at once without answering each request. PX4 defaults to `MISSION_REQUEST_INT` (`_int_mode{true}`) and silently drops items that arrive out of sequence. It also replies `MAV_MISSION_ERROR` to a new MISSION_COUNT while a transfer is still unfinished. ArduPilot rejects a wrong seq with `MAV_MISSION_INVALID_SEQUENCE`, but it has already stored the earlier items, so uploads happen to work on ArduPilot. This likely explains the DeltaQuad TODO at uav2.py:491.
   - The final MISSION_ACK is read once, without blocking, and ignored. MISSION_START is sent anyway, and "START UNCERTAIN" still sets `inMission=True`.
   - Seq 0 is the home position on ArduPilot but a real waypoint on PX4. The duplicated-first-waypoint trick only makes sense on ArduPilot.
   - `_speed` is ignored: no DO_CHANGE_SPEED item is added.
6. **VTOL transitions fail on ArduPilot.** The code sends 1 and 2 (uav2.py:623, 662). ArduPilot only accepts `MAV_VTOL_STATE_MC` (3) and `MAV_VTOL_STATE_FW` (4), and only in AUTO mode (it reports "VTOL transition only in AUTO"). Outside AUTO, the transition has to be done with a mode change.
7. **No GCS heartbeat is sent.** PX4 with `NAV_DLL_ACT>0` refuses to arm with "No connection to the ground control station", and the vehicle's data-link-loss failsafes can't work. The problem is hidden today when QGC is connected through the same MAVProxy.

### High: compatibility and reliability
8. **No source filtering or vehicle identity.** HEARTBEATs from GCSs, cameras and companion computers overwrite `base_mode`/`custom_mode` (uav2.py:188-190). Any message refreshes the liveness timestamp (uav2.py:224). `autopilot_type == 0` is used to mean "unset", but 0 is `MAV_AUTOPILOT_GENERIC` (uav2.py:72). The fix is to lock onto the autopilot's (sysid, compid) and filter everything else.
9. **No stream requests.** ArduPilot only streams what its SRx params or another GCS asked for; MAVProxy is doing that for you today. Request every message you use with `MAV_CMD_SET_MESSAGE_INTERVAL` (supported by both stacks), falling back to `REQUEST_DATA_STREAM` for old ArduPilot.
10. **Fire-and-forget commands.** land, RTL, setSpeed, transition and disarm never check `COMMAND_ACK`. arm and takeoff take the next ACK for *any* command and ignore its `result`. Nothing is retransmitted; the spec says to resend with an incremented `confirmation`. Mode changes use the deprecated `SET_MODE` message (uav2.py:288, 343), which gets no ACK.
11. **Telemetry loop throughput.** `time.sleep(0.01)` after every message caps processing at about 100 msg/s (uav2.py:236). On PX4 or other fast links, messages back up and latency grows. DB and Kafka writes happen every 6 messages of any type (uav2.py:225), so the write rate depends on the link instead of time.
12. **Link loss tears the connection down.** After 15 s of silence, `disconnect()` closes the socket (uav2.py:230-233), so a short radio dropout needs a manual reconnect. A LINK_LOST state with automatic recovery would be better.
13. **Landing and state detection.** `droneState` is derived from `relative_alt > 0.5` (uav2.py:181), and ArduPilot's `wait_until_landed` checks for alt < 0.5 m. Both break with barometer drift or a take-off from high ground. Use `EXTENDED_SYS_STATE.landed_state` (implemented by ArduCopter and PX4) together with the armed bit.
14. **Long-running HTTP calls.** takeoff waits until it reaches altitude, and land/RTL wait until landed. That never happens for a fixed-wing RTL, which loiters instead. Django's `requests.post` has no timeout.
15. **Pause/resume.** ArduCopter supports `MAV_CMD_DO_PAUSE_CONTINUE`, but the PX4 navigator doesn't. On PX4, pause by switching to Hold mode (`LOITER` in pymavlink's px4_map) and resume by switching to `MISSION`.

### Medium: data quality
16. `vtolState` is hard-coded to "MC". `EXTENDED_SYS_STATE.vtol_state` maps one-to-one to MC/FW/TRANSITION_TO_FW/TRANSITION_TO_MC.
17. "Unknown" sentinel values aren't handled: `GPS_RAW_INT.vel` and `GLOBAL_POSITION_INT.hdg` of 65535, `satellites_visible` of 255, `battery_remaining` of -1.
18. `BATTERY_STATUS` isn't filtered by `id`, so values alternate on vehicles with several batteries. Use `SYS_STATUS.battery_remaining`, plus `BATTERY_STATUS` id 0.
19. These fields are never filled in: `gpsSignal` (fix_type), `homeLatitude`/`homeLongitude` (request HOME_POSITION with `MAV_CMD_REQUEST_MESSAGE`), flight mode and armed state. pymavlink's `mode_string_v10()` already decodes both ArduPilot and PX4 modes.
20. Mission completion is detected as "seq ≥ N and within 101 m" (uav2.py:194-210). Use `MISSION_ITEM_REACHED` for the last seq, plus `MISSION_CURRENT.mission_state` when the vehicle sends it. PX4 staying in AUTO.MISSION after the mission ends (noted in mav-test.py) is normal PX4 behaviour.
21. Protocol strings: the UI offers `tcpout`, which pymavlink doesn't recognise, so it tries to open it as a serial device. In pymavlink, `udp:` means listen. TCP connections should use `autoreconnect=True`. The `timeout=10` passed to `mavlink_connection` (uav2.py:61) lands in `**opts`, which the UDP/TCP branches ignore, so it does nothing.
22. The MAVLink version switches process-wide when the first v2 byte arrives. Set `MAVLINK20=1` and the dialect explicitly before importing mavutil. Pin pymavlink too: the image has 2.4.49, while web pins 2.4.29.
23. Takeoff: `int(self.altitude_amsl)` truncates (uav2.py:335), and PX4 receives lat/lon as float32 params, which loses precision. Send NaN instead; PX4 then uses the current position.

### Housekeeping
24. `mav/notes.txt` puts vehicle console and WiFi passwords in git. Rotate them and purge them from history.
25. `entrypoint.mav.sh` runs Python in the background and then `tail -f /dev/null`, so if Python crashes the container still shows as up. Use `exec` instead. `python:3.9` is end-of-life. Dead code: `_ref/`, `check_vehicle_type`, the commented-out handlers.

## Recommended structure
Keep `Uav2` as a thin facade with the same method names, so `requestHandler.py` and `api.py` barely change. Move the MAVLink logic into a package:

```
mav/app/mavlink/
  link.py        MavLink: owns the mavutil connection (explicit source sysid/compid, MAVLINK20, dialect)
                 - ONE reader thread: recv_match(blocking=True, timeout=0.5), no sleep; filter by the locked
                   autopilot (sysid, compid); dispatch to VehicleState and to pending waiters
                 - waiters = (predicate, asyncio.Future), resolved with loop.call_soon_threadsafe
                 - 1 Hz GCS HEARTBEAT only when the drone's opt-in flag is set; all sends go through one lock
                 - link state CONNECTING / CONNECTED / LINK_LOST, based on the age of the autopilot HEARTBEAT; auto-recovers
  state.py       VehicleState: position, rel/AMSL alt, heading, groundspeed, GPS fix/sats, battery, armed,
                 mode (mode_string_v10), landed_state, vtol_state, home, mission seq/state, STATUSTEXT ring
                 buffer -> builds today's telemetryObj; emits transition events (TOOK OFF, LANDED, MODE ...)
  commands.py    send_command(cmd, *p, timeout, retries): COMMAND_LONG with an incremented `confirmation` on each
                 retry, matched on ack.command (+ target fields); handles IN_PROGRESS/DENIED/UNSUPPORTED; returns the
                 result plus the last STATUSTEXT. set_mode(name): MAV_CMD_DO_SET_MODE from mode_mapping(), confirmed
                 by HEARTBEAT. set_message_interval(), request_message().
  mission.py     Upload state machine: MISSION_COUNT -> answer each MISSION_REQUEST_INT|MISSION_REQUEST
                 with the requested seq (MISSION_ITEM_INT) -> MISSION_ACK ACCEPTED; timeout and resend per step;
                 clear_all; start; progress from MISSION_CURRENT / MISSION_ITEM_REACHED
  autopilots/    base.py (generic MAVLink: standard MAV_CMDs only), ardupilot.py, px4.py
```

**On connect:** wait for the autopilot HEARTBEAT (skip GCS, gimbal, camera and onboard types, and `MAV_AUTOPILOT_INVALID`). Lock the target (sysid, compid) and pick the adapter from `autopilot`. Get the vehicle class from `type` (MC / FW / VTOL = `MAV_TYPE_VTOL_*`, or `vtol_state != UNDEFINED`). Request `AUTOPILOT_VERSION` and log the firmware version, then set message intervals and request HOME_POSITION. All of this is async with timeouts; nothing blocks the loop.

**GCS identity:** make it configurable with `MAV_GCS_SYSID` (default 255) and `MAV_GCS_COMPID`. 255 is ArduPilot's `SYSID_MYGCS` default, which its GCS failsafe and `SYSID_ENFORCE` need. The compid should differ from QGC's 190, for example a `MAV_COMP_ID_USER*` value, so ACKs and mission replies don't get mixed up when both are connected.

**Persistence:** a fixed 2 Hz loop takes a snapshot of VehicleState and reuses `kafka_broadcaster.broadcastTelemetry`, `database.queries.saveDroneTelemetry` / `updateDroneTelemetryLatest` and `CameraFootprintCalculator`. That makes the write rate independent of the link.

**Adapter behaviour** (pymavlink 2.4.49 mode names; rows marked ✱ need confirming on SITL):

| Intent | ArduCopter | ArduPlane / QuadPlane | PX4 (MC/FW/VTOL) |
|---|---|---|---|
| Takeoff | GUIDED → arm → NAV_TAKEOFF (param7 = rel alt) | QuadPlane GUIDED+NAV_TAKEOFF or TAKEOFF mode ✱; FW TAKEOFF mode ✱ | arm → NAV_TAKEOFF (param7 AMSL, lat/lon NaN) |
| Land | NAV_LAND (→ LAND) | QuadPlane QLAND; FW mission landing ✱ | NAV_LAND |
| RTL | NAV_RETURN_TO_LAUNCH | RTL (QuadPlane: Q_RTL_MODE) | NAV_RETURN_TO_LAUNCH |
| Pause / Resume | DO_PAUSE_CONTINUE 0/1, fallback BRAKE/LOITER ↔ AUTO | LOITER or QLOITER ↔ AUTO | mode LOITER (Hold) ↔ MISSION |
| Cancel mission | pause, then MISSION_CLEAR_ALL | same | same |
| VTOL transition | n/a | AUTO: DO_VTOL_TRANSITION 3/4; otherwise mode change QLOITER/QHOVER ↔ FBWA/CRUISE ✱ | DO_VTOL_TRANSITION 3/4 |
| Mission seq 0 | home placeholder | home placeholder | first real item |
| Kill / Reboot / Shutdown | ARM_DISARM(0, 21196) / PREFLIGHT_REBOOT_SHUTDOWN 1 / 2 | same | same |

**HTTP semantics (decided: reply on ACK):** each action returns as soon as the ACK arrives, with `{"status","message": MAV_RESULT + last STATUSTEXT}`. Flight-phase events (TOOK OFF, TARGET ALT REACHED, LANDED, MISSION COMPLETED, MODE CHANGED, LINK LOST/RESTORED) are written to `aiders_mavlinklog` from state transitions using `database.queries.saveMavlinkLog`. Add the `/mav/arm` and `/mav/disarm` routes. An unknown drone name returns a clear error instead of a KeyError.

## Phases
1. **Core:** `link.py`, `state.py`, `commands.py`; fix the async/await bug; GCS heartbeat; source filtering; message intervals; fixed-rate persistence; LINK_LOST with auto-recovery.
2. **Actions and adapters:** ArduPilot and PX4 adapters; every action including arm, disarm, kill, reboot, shutdown, pause, resume and cancel; the VTOL fix; ACK-checked mode changes.
3. **Mission:** protocol-compliant upload, speed item, completion tracking.
4. **Telemetry correctness:** landed/vtol state, sentinel values, battery, GPS fix, home; the new Kafka and DB fields (web model and migration, deployed together with the mav change).
5. **Housekeeping:** entrypoint `exec`, pinned deps, base image, dead code, notes.txt credentials.

## Critical files
- `mav/app/uav2.py` (becomes the facade), new `mav/app/mavlink/*`
- `mav/app/requestHandler.py` (await fix, name validation), `mav/app/api.py` (arm/disarm routes, error handling)
- `mav/requirements.txt`, `mav/entrypoint.mav.sh`, `mav/Dockerfile.mav`
- Web container (heartbeat flag and schema): `web/django_api/aiders/models/drone.py` (`Drone` :16, `Telemetry` :86, `TelemetryLatest` :155) plus migrations, the toggle in `web/django_api/aiders/templates/aiders/mavlink-manage.html`, the connect payload in `views.mavlinkConnect` (views.py:3157)
- Mav side of the schema change: `mav/app/database/queries.py`, `mav/app/kafka_broadcaster.py`
- Untouched: `mav/app/uav.py` (MAVSDK)

## Verification
- **SITL matrix (docker):** ArduCopter, ArduPlane `-f quadplane`, PX4 `gz_x500`, PX4 `gz_standard_vtol`. Connect each one directly, with no MAVProxy or QGC, to test the heartbeat and stream requests. Then connect through `start_mavproxy.sh` with QGC attached, to test coexistence.
- **Per vehicle:**
  - Connect; check identity, mode and armed state in the mavlink log.
  - Arm with a forced pre-arm failure; the error and STATUSTEXT come back and the API stays responsive.
  - Takeoff, then setSpeed.
  - Upload a mission 3 times in a row (the DeltaQuad regression).
  - Pause, resume and cancel.
  - Transition FW↔MC on the VTOLs.
  - RTL, then land; kill on the ground; reboot.
  - Cut the link for 20 s; the state goes to LINK_LOST and recovers on its own.
- **Concurrency:** run two SITL vehicles on different ports. While drone A's arm is pending, drone B's land responds immediately.
- **Unit tests (pytest, no SITL):**
  - A fake vehicle over pymavlink UDP loopback drives the mission uploader: MISSION_REQUEST vs _INT, re-requests, dropped requests, NACK.
  - The same fake vehicle drives `send_command`: retry, IN_PROGRESS, unrelated ACKs.
- **Rates:** log received vs processed msg/s; DB rows stay at 2 Hz per drone whatever the stream rate.
