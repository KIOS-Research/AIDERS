import os
from threading import Thread

# custom libs
import database.connection
# from database.connection import MySQLConnector
import ros.subscribers as rosSubscribers
import api

def main():
    database.connection.init(int(os.environ['ROS_MYSQL_CONNECTION_POOLS']), "rosPool")
    apiThread = Thread(target=api.start, args=(os.environ['ROS_API_PORT'],), daemon=True)
    apiThread.name = "ApiThread"
    apiThread.start()    
    rosSubscribers.createCoreSubscribers()  # initialize the core ROS subscribers

if __name__ == '__main__':
    main()


