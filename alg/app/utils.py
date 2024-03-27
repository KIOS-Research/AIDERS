import threading


# returns true if a thread with the same name is already running
def threadStarted(_threadName):
    return any(thread.name == _threadName for thread in threading.enumerate())
