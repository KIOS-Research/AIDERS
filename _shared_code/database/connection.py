import os
import sys
import mysql.connector
from mysql.connector import pooling
from mysql.connector.errors import PoolError


dbconfig = {
    "host": "localhost",
    "user": os.environ['SQL_USER'],
    "password": os.environ['SQL_PASSWORD'],
    "database": os.environ['SQL_DATABASE'],
    "connect_timeout": 30
}

numberOfPools = 0
currentPoolIndex = 0
connectionPools = []

# create the connection pools
def init(_noOfPools, _poolNamePefix):
    global numberOfPools
    numberOfPools = _noOfPools
    try:
        for i in range(1, numberOfPools + 1):
            poolName = f"{_poolNamePefix}_{i}"
            connectionPool = pooling.MySQLConnectionPool(pool_name=poolName, pool_size=32, **dbconfig)
            connectionPools.append(connectionPool)
        print(f"\n\U0001F525 MySQL connection pools: {numberOfPools}")
        with open("/db_states.txt", "a") as file:
            file.write(f"{_poolNamePefix}\n")
    except Exception as e:
        print(f"Error connecting to DB: {e}")
        with open("/db_states.txt", "a") as file:
            file.write(f"ERROR: {_poolNamePefix}\n")

class MySQLConnector:
    def __init__(self):
        global currentPoolIndex
        try:
            self.connection = connectionPools[currentPoolIndex].get_connection()   # get a connection from a pool
            currentPoolIndex += 1
            if currentPoolIndex == numberOfPools:
                currentPoolIndex = 0
            # print(f"{currentPoolIndex}." , end='', flush=True)
            # sys.stdout.flush()
        except PoolError as e:
            print("\n\U0001F6A9 MYSQL Connection pool error!")
            sys.stdout.flush()
        
    def close(self):
        if self.connection:
            self.connection.close()

    def executeQuery(self, _query, _params, _fetchOne=False):
        result = ""
        try:
            cursor = self.connection.cursor()
            if _params is not None:
                cursor.execute(_query, _params)
            else:
                cursor.execute(_query)
            if _query.lower().startswith("select"):
                if _fetchOne:
                    result = cursor.fetchone()
                else:
                    result = cursor.fetchall()
            else:
                self.connection.commit()
                if _query.lower().startswith("insert"):
                    result = cursor.lastrowid
                else:
                    result = None
        except mysql.connector.Error as e:
            print(f"MySQL Query Error: {e}")
        cursor.close()
        return result
    
    def executeBatchQuery(self, _query, _params):
        result = ""
        try:
            cursor = self.connection.cursor()
            if _params is not None:
                cursor.executemany(_query, _params)
            else:
                cursor.execute(_query)
            if _query.lower().startswith("select"):
                result = cursor.fetchall()
            else:
                self.connection.commit()
                if _query.lower().startswith("insert"):
                    result = cursor.lastrowid
                else:
                    result = None
        except mysql.connector.Error as e:
            print(f"MySQL Batch Query Error: {e}")
            cursor.close()
        return result
