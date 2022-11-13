from contextlib import contextmanager
import threading
import _thread

class TimeoutException(Exception):
    pass

@contextmanager
def time_limit(seconds):
    timer = threading.Timer(seconds, lambda: _thread.interrupt_main())
    timer.start()
    try:
        yield
    except KeyboardInterrupt:
        raise TimeoutException("Timed out for operation")
    finally:
        # if the action ends in specified time, timer is canceled
        timer.cancel()