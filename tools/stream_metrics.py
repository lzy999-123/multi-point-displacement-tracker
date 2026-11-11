"""Acceptance metrics for source frames, independent of JPEG pixel changes."""
from bisect import bisect_right


def summarize_frames(frames, duration, warmup, width=320, height=240):
    valid = [f for f in frames if 0 <= f["seconds"] < duration]
    bins = [0] * int(duration)
    new_bins = bins.copy()
    hash_bins = [set() for _ in bins]
    new_times = []
    previous_seq = None
    duplicate_sequences = out_of_order = missing_sequences = 0
    wrong_resolution = 0
    source_ages = []
    for frame in valid:
        t = frame["seconds"]
        index = int(t)
        if index < len(bins):
            bins[index] += 1
            hash_bins[index].add(frame["sha256"])
        wrong_resolution += (frame["width"], frame["height"]) != (width, height)
        seq = frame.get("seq")
        if seq is None:
            missing_sequences += 1
            continue  # Unknown provenance cannot satisfy the source-frame target.
        if previous_seq is not None:
            delta = (seq - previous_seq) & 0xFFFFFFFF
            if delta == 0:
                duplicate_sequences += 1
                continue
            if delta >= 0x80000000:
                out_of_order += 1
                continue
        previous_seq = seq
        new_times.append(t)
        if index < len(new_bins):
            new_bins[index] += 1
        if t >= warmup and frame.get("source_age_ms") is not None:
            source_ages.append(frame["source_age_ms"])

    steady = [t for t in new_times if t >= warmup]
    boundaries = [warmup] + steady + [duration]
    max_gap = max(b - a for a, b in zip(boundaries, boundaries[1:]))
    # Counts can fall only when a frame leaves a moving one-second window.
    starts = [float(warmup), duration - 1]
    starts.extend(t for t in steady if t <= duration - 1)
    rolling_min = min(bisect_right(steady, t + 1) - bisect_right(steady, t) for t in starts)
    return {
        "frames_within_requested_window": len(valid),
        "source_frames": len(new_times),
        "frames_per_second": bins,
        "source_frames_per_second": new_bins,
        "unique_jpegs_per_second": [len(x) for x in hash_bins],
        "steady_min_fps": min(bins[warmup:], default=0),
        "steady_min_source_fps": min(new_bins[warmup:], default=0),
        "steady_min_rolling_source_fps": rolling_min,
        "steady_max_no_frame_ms": round(max_gap * 1000, 2),
        "duplicate_sequences": duplicate_sequences,
        "out_of_order_sequences": out_of_order,
        "missing_sequences": missing_sequences,
        "wrong_resolution_frames": wrong_resolution,
        "source_age_at_send_max_ms": max(source_ages, default=None),
    }
