import asyncio

import requestHandler
from aiohttp import web

app = web.Application()


# start the http server, called on launch
def start(_port):
    web.run_app(app, port=int(_port))


async def processLidarPointCloudToMeshBySessionId(request):
    data = await request.json()
    sessionId = data.get("sessionId")
    response_data = requestHandler.processPointCloud(sessionId)
    return web.json_response(response_data)


app.router.add_post("/processPointCloud", processLidarPointCloudToMeshBySessionId)
