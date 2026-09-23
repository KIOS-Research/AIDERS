import os

# custom libs
import database.connection
import api


def main():
    database.connection.init(int(os.environ['LSCR_DB_CONNECTION_POOLS']), "lscrPool")
    api.start(os.environ['LSCR_API_PORT'])   # start the http server

if __name__ == '__main__':
    main()


