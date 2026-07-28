"""Thread-safe latest-frame stores used by ROS and FastAPI threads."""

from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class FrameSnapshot:
    """A source image and its monotonically increasing sequence number."""

    message: object
    sequence: int
    received_at: float


@dataclass(frozen=True)
class JpegSnapshot:
    """The latest encoded JPEG and associated stream metadata."""

    data: bytes
    sequence: int
    encoded_at: float
    source_sequence: int
    encode_duration_ms: float


class LatestFrameStore:
    """Keep only the newest ROS image message without building a queue."""

    def __init__(self):
        self._condition = threading.Condition()
        self._message = None
        self._sequence = 0
        self._received_at = 0.0
        self._input_count = 0

    def put(self, message):
        """Replace the pending image and wake the encoder worker."""
        with self._condition:
            self._message = message
            self._sequence += 1
            self._input_count += 1
            self._received_at = time.monotonic()
            self._condition.notify_all()

    def latest(self):
        """Return the newest image, or None before the first message."""
        with self._condition:
            if self._message is None:
                return None
            return FrameSnapshot(
                self._message,
                self._sequence,
                self._received_at,
            )

    def wait_for_new(self, previous_sequence, timeout):
        """Wait until a newer image is available, returning only the latest."""
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence > previous_sequence,
                timeout=timeout,
            )
            if self._message is None or self._sequence <= previous_sequence:
                return None
            return FrameSnapshot(
                self._message,
                self._sequence,
                self._received_at,
            )

    @property
    def input_count(self):
        with self._condition:
            return self._input_count


class LatestJpegStore:
    """Share one encoded JPEG among every connected web client."""

    def __init__(self):
        self._lock = threading.Lock()
        self._snapshot = None
        self._sequence = 0
        self._encoded_count = 0
        self._client_count = 0

    def put(self, data, source_sequence, encode_duration_ms):
        """Replace the web frame without retaining older JPEG buffers."""
        with self._lock:
            self._sequence += 1
            self._encoded_count += 1
            self._snapshot = JpegSnapshot(
                data=data,
                sequence=self._sequence,
                encoded_at=time.monotonic(),
                source_sequence=source_sequence,
                encode_duration_ms=encode_duration_ms,
            )

    def latest(self):
        with self._lock:
            return self._snapshot

    def add_client(self):
        with self._lock:
            self._client_count += 1

    def remove_client(self):
        with self._lock:
            self._client_count = max(0, self._client_count - 1)

    def metrics(self):
        with self._lock:
            return {
                'encoded_count': self._encoded_count,
                'client_count': self._client_count,
            }
