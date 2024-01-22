import os
from threading import Thread
from flask import Flask, request, jsonify
from sinadra import integration_sinadra, publishers
import database.connection
app = Flask(__name__)

def start(_port):
    app.run(port=_port)   # start the http server

# Start the Sinadra
@app.route('/sinadraStartOrStop', methods=['POST'])
def handleSinadraIntegration():
    if request.method == 'POST':
        data = request.get_json()
        if data["command"] == 'start':
            starts = integration_sinadra.startSinadraIntegrationForOperation(data["operationId"])
            if starts != False:
                return "200"
        elif data["command"] == 'stop':
            stops=integration_sinadra.stopSinadraIntegrationForOperation(data["operationId"])
            if stops != False:
                return "200"
    return "400"

def main():
    database.connection.init(int(os.environ['SSM_MYSQL_CONNECTION_POOLS']), "rosPool")
    publishers.rosInit()
    start(os.environ['SSM_API_PORT'])

if __name__ == '__main__':
    main()