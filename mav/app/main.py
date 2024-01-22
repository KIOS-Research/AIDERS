import os

# custom libs
import database.connection
import api


def main():
    database.connection.init(int(os.environ['MAV_MYSQL_CONNECTION_POOLS']), "mavPool")
    api.start(os.environ['MAV_API_PORT'])   # start the http server

if __name__ == '__main__':
    main()


