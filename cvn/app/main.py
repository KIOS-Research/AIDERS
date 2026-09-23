import os

# custom libs
import database.connection
import api

def main():
    database.connection.init(int(os.environ['CVN_DB_CONNECTION_POOLS']), "cvnPool")
    api.start(os.environ['CVN_API_PORT'])   # start the http server

if __name__ == '__main__':
    main()
