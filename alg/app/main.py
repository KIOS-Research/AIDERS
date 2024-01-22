import os

# custom libs
import database.connection
import api


def main():
    database.connection.init(int(os.environ['ALG_MYSQL_CONNECTION_POOLS']), "algPool")
    api.start(os.environ['ALG_API_PORT'])   # start the http server

if __name__ == '__main__':
    main()


