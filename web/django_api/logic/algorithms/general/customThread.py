import threading


class MyThread(threading.Thread):
    def __init__(self, thread_id, target, args=(), kwargs=None):
        super().__init__()
        self._stop_event = threading.Event()
        self.thread_id = thread_id
        self.target = target
        self.args = args
        self.kwargs = kwargs or {}

    def run(self):
        while not self._stop_event.is_set():
            self.target(*self.args, **self.kwargs)

    def stop(self):
        self._stop_event.set()


def thread_exists(name):
    return any(thread.name == name for thread in threading.enumerate())


def get_thread_by_name(name):
    return next(
        (thread for thread in threading.enumerate() if thread.name == name),
        None,
    )
