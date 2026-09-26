"""Optional private factory; the existing/default BootIdentityGuard is untouched.

No generic path argument: real use owns only the original procfs boot-id fd.
The guard retains the original Python lock, initializer, close and call cadence.
This helper has a finite byte/retry bound, not a hard syscall-time bound.
"""
import sys

from singularitydog_hw.sensor_pipeline_benchmark import BootIdentityGuard
from singularitydog_hw.policy_observer_live import require
from _native_boot_guard import fresh_boot_matches


class NativeBootIdentityGuard(BootIdentityGuard):
    def __init__(self):
        if sys.platform != "linux":
            raise RuntimeError("Runtime native boot guard requires genuine Linux procfs")
        super().__init__()  # Exact existing /proc/sys/kernel/random/boot_id path.

    def check(self):
        with self._lock:
            require(self._fd is not None, "Boot monitor already closed")
            require(fresh_boot_matches(self._fd, self._expected),
                    "Boot changed or invalid boot read")
