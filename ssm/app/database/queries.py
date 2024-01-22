
from datetime import datetime
from database.connection import MySQLConnector
import pytz
timezone = pytz.utc

#####################################
############## SINADRA ##############
#####################################

def getSinadraStatusFormOperationId(_operationId):
    query = f"SELECT sinadra_active FROM aiders_operation WHERE id = %s"
    params = (_operationId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def updateSinadraStatusFormOperationId(_operationId, newStatus):
    query = "UPDATE aiders_operation SET sinadra_active = %s WHERE id = %s"
    params = (newStatus, _operationId)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def getOperationDisasterDataFromOperationId(_operationId):
    query = f"SELECT disaster_epicenter_latitude, disaster_epicenter_longtitude, dense_area_of_buildings, max_extreme_temperature, risk_of_explosion_and_fire FROM aiders_operation WHERE id = %s"
    params = (_operationId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def getActiveDronesLatitudeAndLongitudeForOperation(_operationId):
    query = f"""SELECT aiders_drone.id, aiders_telemetry.lat, aiders_telemetry.lon 
                FROM aiders_drone 
                INNER JOIN ( 
                    SELECT drone_id, MAX(time) AS last_time FROM aiders_telemetry GROUP BY drone_id ) last_telemetry ON aiders_drone.id = last_telemetry.drone_id 
                INNER JOIN aiders_telemetry ON last_telemetry.drone_id = aiders_telemetry.drone_id AND last_telemetry.last_time = aiders_telemetry.time 
                INNER JOIN aiders_operation ON aiders_drone.operation_id = aiders_operation.id 
                WHERE aiders_operation.id = %s 
                AND aiders_drone.is_connected_with_platform = 1;"""
    params = (_operationId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, False)
    connector.close()

    return result

def saveHumanInjuryCriticalityInAreaData(_data, _operationId):
    query = f"""INSERT INTO aiders_sinadradata 
        (operation_id, human_injury_risk_prediction, time)
        VALUES (%s, %s, %s)"""
    
    params = ( _operationId, _data, datetime.now(timezone))
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()