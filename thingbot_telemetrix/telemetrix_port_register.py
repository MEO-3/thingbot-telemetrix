class TelemetrixPortRegister:
    """
    This is a Singleton Class to share active ports
    used by Telemetrix instances.

    """
    _instance = None

    def __new__(self):
        if self._instance is None:
            self._instance = super(TelemetrixPortRegister, self).__new__(self)
            self.active = []
        return self._instance

    def add(self, port):
        self.active.append(port)

    def remove(self, port):
        self.active.remove(port)