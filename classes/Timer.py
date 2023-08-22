from datetime import datetime


class Timer:
    def __init__(self) -> None:
        self.start_time = None

    def start(self):
        self.start_time = datetime.now()

    def get_delta_sec(self):
        return (datetime.now().timestamp() - self.start_time.timestamp())
