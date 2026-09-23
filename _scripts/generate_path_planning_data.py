import time
import json
import random
from datetime import datetime, timezone
from kafka import KafkaProducer

# — Configuration —
BOOTSTRAP_SERVER = "apps.edutel.uniwa.gr:9092"
# BOOTSTRAP_SERVER = "192.168.0.10:9092"

TOPIC_NAME      = "PathPlanning_Output"
INTERVAL_S      = 5  # seconds between sends

# — Base mission definitions —
BASE_MISSIONS = [
    {
        "name": "SIM_1",
        "missionSpeed": 22,
        "action": "START_MISSION",
        "missionPath": [[23.774278969589794, 37.973917861638704, 100], [23.788578358664267, 37.97306065833104, 100], [23.788655784520774, 37.97386917456843, 100], [23.767206343022938, 37.97515434156251, 100], [23.767129150110218, 37.97434581142547, 100], [23.77325757189701, 37.97397902393023, 100], [23.773103058016538, 37.97236197086799, 100], [23.778209918566315, 37.9720560773338, 100], [23.778287230123954, 37.972864600676026, 100], [23.774201700712993, 37.97310933588633, 100], [23.774278969589794, 37.973917861638704, 100]]
    },
    # {
    #     "name": "SIM_Bravo",
    #     "missionSpeed": 20,
    #     "action": "START_MISSION",
    #     "missionPath": [
    #         [23.77416387297398, 37.96730188204659, 120],
    #         [23.77426387397398, 37.96730188204659, 120],
    #         [23.774366387397398, 37.96730188204659, 120],
    #         [23.774466387497398, 37.96730188204659, 120],
    #         [23.774566387597398, 37.96730188204659, 120],
    #         [23.774666387697398, 37.96730288204659, 120],
    #         [23.774766387797389, 37.96730188204677, 139]
    #     ]
    # }
]

def perturb_path(path, max_offset=0.0001, max_alt_offset=2):
    """
    Randomly jitter each [lon, lat, alt]:
      - lon/lat by up to ±max_offset degrees
      - alt by up to ±max_alt_offset meters
    """
    new_path = []
    for lon, lat, alt in path:
        nl = lon + random.uniform(-max_offset, max_offset)
        nt = lat + random.uniform(-max_offset, max_offset)
        na = alt + random.randint(-max_alt_offset, max_alt_offset)
        new_path.append([round(nl, 8), round(nt, 8), na])
    return new_path

counter = 0

def make_missions_payload(missions):
    """
    Returns the raw {"missions": [...]} structure, with each missionPath
    slightly perturbed.
    """
    out = []
    for m in missions:
        m_copy = m.copy()
        # m_copy["missionPath"] = perturb_path(m["missionPath"])
        m_copy["missionPath"] = m["missionPath"]
        out.append(m_copy)
    global counter
    counter += 1
    # return {"msg_id": counter, "missions": out} // TESTING
    return out


def main():
    global counter
    producer = KafkaProducer(
        bootstrap_servers=BOOTSTRAP_SERVER,
        value_serializer=lambda v: json.dumps(v).encode("utf-8")
    )
    print(f"Streaming raw missions to '{TOPIC_NAME}' every {INTERVAL_S}s. Ctrl-C to stop.")
    try:
        while True:
            payload = make_missions_payload(BASE_MISSIONS)
            producer.send(TOPIC_NAME, payload)
            producer.flush()
            
            # simple log with timestamp
            current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            print(f"[{current_timestamp}] Sent payload #{counter}")            
            # ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
            # print(f"[{ts}] Sent payload:", json.dumps(payload))
            time.sleep(INTERVAL_S)
    except KeyboardInterrupt:
        print("\nStopped by user.")

if __name__ == "__main__":
    main()
