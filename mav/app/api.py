import asyncio
from aiohttp import web


# custom libs
import requestHandler


app = web.Application()

# start the http server, called on launch
def start(_port):
    web.run_app(app, port=int(_port))


async def handleConnectToUav(request):
    data = await request.json()
    name = data.get('name')
    ip = data.get('ip')
    port = data.get('port')
    operationId = data.get('operationId')
    # type = data.get('type')
    model = data.get('model')
    try:
        # await requestHandler.connectToUav(name, ip, port, type, model, operationId)
        await requestHandler.connectToUav(name, ip, port, model, operationId)
        response_data = {"status": "success", "message": "Connected to the drone"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleDisconnectFromUav(request):
    data = await request.json()
    name = data.get('name')
    try:
        await requestHandler.disconnectFromUav(name)
        response_data = {"status": "success", "message": "Disconnected from the drone"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleTakeoff(request):
    data = await request.json()
    name = data.get('name')
    alt = data.get('alt')
    try:
        await requestHandler.publishTakeoffCommand(name, alt)
        response_data = {"status": "success", "message": "ok"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleLand(request):
    data = await request.json()
    name = data.get('name')
    try:
        await requestHandler.publishLandCommand(name)
        response_data = {"status": "success", "message": "ok"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleMission(request):
    data = await request.json()
    name = data.get('name')
    missionPoints = data.get('missionPath')
    speed = data.get('missionSpeed')
    action = data.get('action')

    try:
        if action == "PAUSE_MISSION":
            await requestHandler.publishPauseMissionCommand(name)
        elif action == "RESUME_MISSION":
            await requestHandler.publishResumeMissionCommand(name)
        elif action == "CANCEL_MISSION":
            await requestHandler.publishCancelMissionCommand(name)
        else:
            await requestHandler.publishMissionCommand(name, missionPoints, speed)
        response_data = {"status": "success", "message": "ok"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleSetSpeed(request):
    data = await request.json()
    name = data.get('name')
    speed = data.get('speed')
    try:
        await requestHandler.publishSetSpeedCommand(name, speed)
        response_data = {"status": "success", "message": "ok"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleTransition(request):
    data = await request.json()
    name = data.get('name')
    mode = data.get('mode')
    try:
        await requestHandler.publishTransitionCommand(name, mode)
        response_data = {"status": "success", "message": "ok"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleReturnHome(request):
    data = await request.json()
    name = data.get('name')
    try:
        await requestHandler.publishReturnCommand(name)
        response_data = {"status": "success", "message": "ok"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleKill(request):
    data = await request.json()
    name = data.get('name')
    try:
        await requestHandler.publishKillCommand(name)
        response_data = {"status": "success", "message": "ok"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleReboot(request):
    data = await request.json()
    name = data.get('name')
    try:
        await requestHandler.publishRebootCommand(name)
        response_data = {"status": "success", "message": "ok"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


async def handleShutdown(request):
    data = await request.json()
    name = data.get('name')
    try:
        await requestHandler.publishShutdownCommand(name)
        response_data = {"status": "success", "message": "ok"}
    except Exception as e:
        response_data = {"status": "error", "message": str(e)}
    return web.json_response(response_data)


# async def handleGoTo(request):
#     data = await request.json()
#     name = data.get('name')
#     lat = data.get('lat')
#     lon = data.get('lon')
#     alt = data.get('alt')
#     try:
#         await requestHandler.publishGoToCommand(name, lat, lon, alt)
#         response_data = {"status": "success", "message": "ok"}
#     except Exception as e:
#         response_data = {"status": "error", "message": str(e)}
#     return web.json_response(response_data)


# async def handleTelemetryStart(request):
#     data = await request.json()
#     name = data.get('name')
#     asyncio.create_task(requestHandler.startTelemetry(name))
#     return web.Response(text='Telemetry task triggered!')


# async def handlePauseMission(request):
#     data = await request.json()
#     name = data.get('name')
#     try:
#         await requestHandler.publishPauseMissionCommand(name)
#         response_data = {"status": "success", "message": "ok"}
#     except Exception as e:
#         response_data = {"status": "error", "message": str(e)}
#     return web.json_response(response_data)


# async def handleResumeMission(request):
#     data = await request.json()
#     name = data.get('name')
#     try:
#         await requestHandler.publishResumeMissionCommand(name)
#         response_data = {"status": "success", "message": "ok"}
#     except Exception as e:
#         response_data = {"status": "error", "message": str(e)}
#     return web.json_response(response_data)


# async def handleCancelMission(request):
#     data = await request.json()
#     name = data.get('name')
#     try:
#         await requestHandler.publishCancelMissionCommand(name)
#         response_data = {"status": "success", "message": "ok"}
#     except Exception as e:
#         response_data = {"status": "error", "message": str(e)}
#     return web.json_response(response_data)


##########
# ROUTES #
##########

app.router.add_post('/connectToUav', handleConnectToUav)
app.router.add_post('/disconnectFromUav', handleDisconnectFromUav)
app.router.add_post('/takeoff', handleTakeoff)
app.router.add_post('/land', handleLand)
app.router.add_post('/transition', handleTransition)
app.router.add_post('/setSpeed', handleSetSpeed)
app.router.add_post('/returnHome', handleReturnHome)
app.router.add_post('/kill', handleKill)
app.router.add_post('/mission', handleMission)

app.router.add_post('/reboot', handleReboot)
app.router.add_post('/shutdown', handleShutdown)

# app.router.add_post('/pauseMission', handlePauseMission)
# app.router.add_post('/resumeMission', handleResumeMission)
# app.router.add_post('/cancelMission', handleCancelMission)

# app.router.add_post('/startTelemetry', handleTelemetryStart)
# app.router.add_post('/goTo', handleGoTo)