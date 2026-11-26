from types import SimpleNamespace

import pytest

from franka_capture.episode import Sample, select_frames


def frames(fps, length=1200, gap=0):
    camera = {"configuration": {"fps": 30}}
    manifest = {"max_skew_ns": 35_000_000}
    stamps = [10**12 + round(i * 10**9 / fps) for i in range(length)]
    if gap:
        stamps[length // 2 :] = [x + gap for x in stamps[length // 2 :]]
    matched = tuple(Sample(SimpleNamespace(timestamp_ns=t), None, t) for t in stamps)
    return SimpleNamespace(camera=camera, manifest=manifest, matched=matched)


def test_nominal_capture_rate_drift_does_not_accumulate_a_false_gap():
    episode = frames(29.97)
    selected = select_frames(episode, 30)
    assert [x.frame for x in selected] == [x.frame for x in episode.matched]
    assert max(abs(x.target_ns - x.frame.timestamp_ns) for x in selected) < 10


def test_real_gap_is_still_rejected():
    with pytest.raises(ValueError, match="gap|repeat"):
        select_frames(frames(29.97, gap=80_000_000), 30)
