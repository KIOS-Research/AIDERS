import threading

from flask import Flask, jsonify, request

app = Flask(__name__)

# # start the http server, called on launch
def start(_port):
    app.run(host='0.0.0.0', port=_port)   
    

# # returns the active threads in this app
@app.route('/kc/threads', methods=['GET'])
def getThreads():
    threadList = [{'name': t.name} for t in threading.enumerate() if not t.name.startswith(('Thread-', 'Dummy-'))]
    return jsonify(threadList)



@app.route('/kc/healthCheck', methods=['GET'])
def handleHealthCheckRequest():
    response_data = {
        "status": "OK",
        "message": "Health check successful"
    }
    return jsonify(response_data), 200
