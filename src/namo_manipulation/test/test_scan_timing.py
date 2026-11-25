"""A phase offset between 10 Hz scans and TF must not starve valid scans."""
from types import SimpleNamespace
from namo_manipulation.mission import latest_ready_scan


def scan(nanoseconds):
    return SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=10,nanosec=nanoseconds)))


def test_new_scan_without_tf_does_not_replace_previous_transformable_sample():
    previous,newest=scan(100_000_000),scan(200_000_000)
    assert latest_ready_scan([previous,newest],10.,10.2,lambda m:m is previous) is previous


def test_stale_or_already_processed_scans_never_refresh_watchdog():
    old=scan(0)
    assert latest_ready_scan([old],9.,11.,lambda m:True) is None
    assert latest_ready_scan([old],10.,10.,lambda m:True) is None
