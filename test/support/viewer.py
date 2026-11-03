"""Shared lifetime management for the two independent model viewers."""

from contextlib import contextmanager
import signal
import threading

import mujoco.viewer


@contextmanager
def open_viewer(model, data, key_callback=None):
    """Close and join viewer threads before Python tears down GLFW."""
    interrupted = threading.Event()
    previous = signal.signal(signal.SIGINT, lambda *_: interrupted.set())
    existing_threads = set(threading.enumerate())
    viewer_threads = []
    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            # This launch is synchronous; no other Python workers are started here.
            # Capture its threads through threading's public API, without MuJoCo internals.
            viewer_threads = [thread for thread in threading.enumerate()
                              if thread not in existing_threads]
            yield viewer, interrupted
    finally:
        # Handle.close() requests exit but does not wait for native rendering to finish.
        # Keep Python and GLFW alive until the viewer has released its resources.
        for thread in viewer_threads:
            thread.join()
        signal.signal(signal.SIGINT, previous)
