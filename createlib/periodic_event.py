import threading

class PeriodicEvent(threading.Thread):
    def __init__(self, period,  delegate_fn):
        self.delegate = delegate_fn
        self.is_active = False
        self.wait_period = period
        self.active_evt = threading.Event()
        self.pause_evt = threading.Event()
        self.stop_condition = threading.Event()
        super().__init__(daemon=True)

    def run(self):
        while True:
            self.active_evt.wait()
            pause = self.pause_evt.wait(self.wait_period)
            if pause:
                if self.stop_condition.is_set():
                    break
                self.pause_evt.clear()
                self.active_evt.clear()
                continue
            else:
                self.delegate()

    def go(self):
        self.is_active = True
        self.active_evt.set()

    def pause(self):
        self.is_active = False
        self.pause_evt.set()

    def stop(self):
        self.is_active = False
        self.stop_condition.set()
        self.active_evt.set()
        self.pause_evt.set()

    def toggle(self):
        print("Toggle called")
        if self.is_active:
            self.pause()
        else:
            self.go()