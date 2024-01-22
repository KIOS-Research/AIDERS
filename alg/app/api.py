import threading
from flask import Flask, request, jsonify

# custom libs


app = Flask(__name__)

# start the http server, called on launch
def start(_port):
    app.run(port=_port)   
    

# returns the active threads in this app
@app.route('/threads', methods=['GET'])
def getThreads():
    threadList = [{'name': t.name} for t in threading.enumerate() if not t.name.startswith(('Thread-', 'Dummy-'))]
    return jsonify(threadList)

