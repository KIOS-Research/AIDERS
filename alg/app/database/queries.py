import os
from datetime import datetime

import pytz

# custom libs
from database.connection import MySQLConnector

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))


def getAllPointsByLidarSessionId(_sessionId):
    query = f"SELECT x, y, z, red, green, blue FROM aiders_lidarpoint WHERE lidar_point_session_id = %s"
    params = (_sessionId, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, False)
    connector.close()
    return result

def updateProcessedSessionBySessionIdAndPath(_sessionId, _fullPath):
    query = f"UPDATE aiders_lidarpointsession SET path = %s, is_process = 1 WHERE id = %s"
    params = (_fullPath, _sessionId)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()