"""Bounded asynchronous JSONL recording; disk failures never reach the detector."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import tempfile
import threading
import time


def default_log_dir():
    """Follow ROS log directory overrides without importing ROS."""
    root = os.environ.get('ROS_LOG_DIR')
    if not root:
        root = Path(os.environ.get('ROS_HOME') or '~/.ros').expanduser() / 'log'
    return Path(root).expanduser() / 'yolo_diagnostics'


class DetectorJsonl:
    def __init__(self, directory='', max_bytes=10*1024*1024, backup_count=4, capacity=256):
        if max_bytes < 1 or backup_count < 0 or capacity < 1:
            raise ValueError('JSONL size/capacity must be positive; backup count must be nonnegative')
        self.max_bytes, self.backup_count = max_bytes, backup_count
        self.path = None
        self._file, self._thread = None, None
        self._queue = queue.Queue(maxsize=capacity)
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._closed, self._error = False, ''
        self._sequence = self._written = self._dropped = self._size = 0
        self._serialization_errors = 0
        try:
            base = Path(directory).expanduser() if directory else default_log_dir()
            base.mkdir(parents=True, exist_ok=True)
            prefix = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + f'-pid{os.getpid()}-'
            run = Path(tempfile.mkdtemp(prefix=prefix, dir=base))
            self.path = (run / 'events.jsonl').resolve()
            self._file = self.path.open('xb')
            self._thread = threading.Thread(target=self._run, name='yolo-jsonl', daemon=True)
            self._thread.start()
        except (OSError, RuntimeError) as error:
            self._error = str(error)
            self._thread = None
            if self._file:
                try:
                    self._file.close()
                except OSError:
                    pass

    def record(self, event, **fields):
        """Enqueue owned JSON-compatible fields. No serialization or disk I/O here."""
        with self._lock:
            if self._closed or self._error:
                return False
            self._sequence += 1
            row = dict(schema_version=1, session_id=self.path.parent.name,
                       sequence=self._sequence, wall_time_ns=time.time_ns(),
                       monotonic_ns=time.monotonic_ns(), event=event, data=fields,
                       dropped_total=self._dropped)
            try:
                self._queue.put_nowait(row)
            except queue.Full:
                self._dropped += 1
                return False
            return True

    def status(self):
        with self._lock:
            return dict(path=str(self.path) if self.path else '', active=not self._closed and not self._error,
                        written=self._written, dropped=self._dropped, queued=self._queue.qsize(),
                        serialization_errors=self._serialization_errors, error=self._error)

    def _write(self, row):
        try:
            line = (json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8')
        except (TypeError, ValueError):
            with self._lock:
                self._dropped += 1
                self._serialization_errors += 1
            return
        # A single oversized event must not defeat the configured disk limit.
        if len(line) > self.max_bytes:
            with self._lock:
                self._dropped += 1
            return
        if self._size + len(line) > self.max_bytes:
            self._file.close()
            for index in range(self.backup_count, 1, -1):
                old = Path(f'{self.path}.{index-1}')
                if old.exists():
                    old.replace(f'{self.path}.{index}')
            if self.backup_count:
                self.path.replace(f'{self.path}.1')
            self._file = self.path.open('wb')
            self._size = 0
        self._file.write(line)
        self._file.flush()  # readable during a trial; no fsync in the camera thread
        self._size += len(line)
        with self._lock:
            self._written += 1

    def _run(self):
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    row = self._queue.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    self._write(row)
                except OSError as error:
                    with self._lock:
                        self._error = str(error)
                        self._dropped += 1
                    break
                finally:
                    self._queue.task_done()
        finally:
            # On disk failure, release queued payloads and expose lost event count.
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
                with self._lock:
                    self._dropped += 1
                self._queue.task_done()
            try:
                self._file.close()
            except OSError as error:
                with self._lock:
                    self._error = str(error)

    def close(self, timeout=2.0):
        with self._lock:
            self._closed = True
            self._stop.set()
        if self._thread:
            self._thread.join(timeout=timeout)
            return not self._thread.is_alive()
        return True
