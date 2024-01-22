import os

# custom libs
import database.connection
import api


def main():
    database.connection.init(int(os.environ['CV_MYSQL_CONNECTION_POOLS']), "cvPool")
    api.start(os.environ['CV_API_PORT'])   # start the http server

if __name__ == '__main__':
    main()
