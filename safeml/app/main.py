import os

import database.connection
import api

def main():
    database.connection.init(int(os.environ['SAFEML_MYSQL_CONNECTION_POOLS']), "safemlPool")
    api.start(os.environ['SAFEML_API_PORT'])   # start the http server

if __name__ == '__main__':
    main()
