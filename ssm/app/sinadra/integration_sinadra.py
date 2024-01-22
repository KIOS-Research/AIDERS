import math
import time
import database.queries
import utils
import threading
from . import publishers
from . import subscribers

runningSinadraIntegration = {}

def startSinadraIntegrationForOperation(_operationId):
    if len(runningSinadraIntegration) != 0:
        return False
    threadName = f"sinadraIntegrationThread_{_operationId}"
    sinadraActivated = database.queries.getSinadraStatusFormOperationId(
        _operationId)
    if sinadraActivated[0] is not None or sinadraActivated[0] is True:
        if utils.threadStarted(threadName):
            return False
    if not utils.threadStarted(threadName):
        database.queries.updateSinadraStatusFormOperationId(_operationId, 1)
        thread = threading.Thread(target=sinadraIntegrationThread, args=(_operationId,))
        thread.name = threadName
        thread.start()
        subscribers.startHumanInjuryCriticalityInArea(_operationId)


def stopSinadraIntegrationForOperation(_operationId):
    if len(runningSinadraIntegration) == 0:
        return False 
    runningSinadraIntegration[_operationId].stop()
    subscribers.stopHumanInjuryCriticalityInArea()
    del runningSinadraIntegration[_operationId]
    database.queries.updateSinadraStatusFormOperationId(_operationId, 0)


def sinadraIntegrationThread(_operationId):
    sinadraIntegration = SinadraIntegration(_operationId)
    runningSinadraIntegration[_operationId] = sinadraIntegration
    sinadraIntegration.run()

class SinadraIntegration:
    def __init__(self, _operationId):
        self.operationId = _operationId
        self.running = True
    def run(self):
        while self.running:
            results=database.queries.getOperationDisasterDataFromOperationId(self.operationId)
            disasterEpicenterLatitude, disasterEpicenterLongtitude, denseAreaOfBuildings, extremeTemperature, riskOfFireAndExplosion = results[:5]
            if disasterEpicenterLatitude is not None and disasterEpicenterLongtitude is not None:
                allDronesGPS=database.queries.getActiveDronesLatitudeAndLongitudeForOperation(self.operationId)
                distance = None
                for droneGPS in allDronesGPS:
                    tempDistance = haversine(disasterEpicenterLatitude, disasterEpicenterLongtitude, droneGPS[1], droneGPS[2])
                    if distance == None or distance > tempDistance:
                        distance=tempDistance
                publishers.areaDistanceToCatastropyEpicenter(distance)
            if denseAreaOfBuildings is not None:
                publishers.denseBuildingArea(denseAreaOfBuildings)
            if extremeTemperature is not None:
                publishers.highestTemperatureInArea(extremeTemperature)
            if riskOfFireAndExplosion is not None:
                publishers.riskOfFireAndExplosionsInArea(riskOfFireAndExplosion)
            time.sleep(5)
    def stop(self):
        self.running = False


def haversine(lat1, lon1, lat2, lon2):
    # Radius of the Earth in meters
    R = 6371000  # approximately 6,371 km

    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    return distance
