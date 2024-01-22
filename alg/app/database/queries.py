import os
from datetime import datetime
import pytz
# custom libs
from database.connection import MySQLConnector

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))

# def getActiveDroneLiveSession(_droneId):
#     query = f"SELECT id FROM aiders_livestreamsession WHERE drone_id = %s AND is_active = 1 LIMIT 1"
#     params = (_droneId, )
#     connector = MySQLConnector()
#     result = connector.executeQuery(query, params, True)
#     connector.close()
#     return result


# def deactivateSessionsAndCreateNew(_droneId):
#     updateQuery = f"UPDATE aiders_livestreamsession SET is_active = 0 WHERE drone_id = %s"
#     updateParams = (_droneId, )

#     insertQuery = (
#         "INSERT INTO aiders_livestreamsession "
#         "(drone_id, start_time, is_active, latest_frame_url) "
#         "VALUES (%s, %s, %s, %s)"
#     )
#     insertParams = (
#         _droneId, datetime.now(timezone), 1, "/static/aiders/imgs/drone_img.jpg"
#     )

#     connector = MySQLConnector()
#     connector.executeQuery(updateQuery, updateParams, False)
#     sessionId = connector.executeQuery(insertQuery, insertParams, False)
#     connector.close()
#     return sessionId


# def saveFrame(_droneId, _sessionId, _framePath):
#     frameQuery = (
#         "INSERT INTO aiders_rawframe "
#         "(drone_id, live_stream_session_id, frame, time) "
#         "VALUES (%s, %s, %s, %s)"
#     )
#     frameParams = (_droneId, _sessionId, _framePath, datetime.now(timezone))

#     sessionQuery = "UPDATE aiders_livestreamsession SET latest_frame_url = %s WHERE id = %s"
#     sessionParams = (_framePath, _sessionId)

#     connector = MySQLConnector()
#     connector.executeQuery(frameQuery, frameParams, False)
#     connector.executeQuery(sessionQuery, sessionParams, False)
#     connector.close()


# def updateSessionEnd(_sessionId):
#     query = "UPDATE aiders_livestreamsession SET is_active = 0, end_time = %s WHERE id = %s"
#     params = (datetime.now(timezone), _sessionId)
#     connector = MySQLConnector()
#     connector.executeQuery(query, params, False)
#     connector.close()
