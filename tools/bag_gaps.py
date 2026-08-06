#!/usr/bin/env python3
"""
Find dropouts in a recorded topic by looking at message ARRIVAL times.

    python3 bag_gaps.py conn_base_robot_2_*  conn_r2_robot_2_*

Pass the base-station bag and the robot's bag for the SAME topic and the same
run. The base bag is what was sent; the robot bag is what arrived. If the robot
bag has fewer messages, or has gaps the base bag does not, the link dropped.

Reports per bag: message count, duration, mean rate, and every gap larger than
--threshold (default 0.25 s, i.e. 5 missed messages at 20 Hz).
"""

import os
import sys
import glob
import argparse

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
except ImportError:
    sys.exit("rosbag2_py not found. Source your ROS 2 setup first:\n"
             "    source /opt/ros/$ROS_DISTRO/setup.bash")


def storage_id_of(path):
    """Read the storage plugin from metadata.yaml; fall back to sqlite3."""
    meta = os.path.join(path, 'metadata.yaml')
    if os.path.exists(meta):
        with open(meta) as f:
            for line in f:
                if 'storage_identifier' in line:
                    return line.split(':', 1)[1].strip().strip('"\'')
    return 'sqlite3'


def arrival_times(path):
    """Returns {topic: [arrival_time_seconds, ...]} for one bag."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id=storage_id_of(path)),
                ConverterOptions('', ''))
    out = {}
    while reader.has_next():
        topic, _data, t_ns = reader.read_next()
        out.setdefault(topic, []).append(t_ns * 1e-9)
    return out


def report(path, threshold):
    print(f"\n=== {os.path.basename(path)} ===")
    try:
        per_topic = arrival_times(path)
    except Exception as exc:
        print(f"   could not read: {exc}")
        return {}

    summary = {}
    for topic, times in per_topic.items():
        times.sort()
        n = len(times)
        if n < 2:
            print(f"   {topic}: only {n} message(s)")
            continue

        dur = times[-1] - times[0]
        gaps = [(times[i + 1] - times[i], times[i] - times[0])
                for i in range(n - 1)]
        big = [g for g in gaps if g[0] > threshold]
        worst = max(gaps)[0]

        print(f"   {topic}")
        print(f"      {n} messages over {dur:.1f} s   mean rate {n / dur:.1f} Hz")
        print(f"      largest gap {worst * 1000:.0f} ms")
        if big:
            print(f"      {len(big)} gap(s) over {threshold * 1000:.0f} ms:")
            for g, t in big[:20]:
                print(f"         t = {t:7.2f} s   gap = {g * 1000:7.0f} ms")
            if len(big) > 20:
                print(f"         ... and {len(big) - 20} more")
            lost = sum(g for g, _ in big)
            print(f"      total time with no messages: {lost:.1f} s "
                  f"({100 * lost / dur:.1f}% of the run)")
        else:
            print(f"      no gaps over {threshold * 1000:.0f} ms -- link looks clean")

        summary[topic] = (n, dur)
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', help='bag directories (globs are expanded)')
    ap.add_argument('--threshold', type=float, default=0.25,
                    help='gap size in seconds to report (default 0.25)')
    args = ap.parse_args()

    paths = []
    for b in args.bags:
        paths.extend(sorted(glob.glob(b)) or [b])

    all_summaries = {}
    for p in paths:
        if not os.path.isdir(p):
            print(f"\n=== {p} ===\n   not a bag directory, skipping")
            continue
        all_summaries[os.path.basename(p)] = report(p, args.threshold)

    # Cross-bag comparison: same topic recorded in more than one place.
    by_topic = {}
    for bag, topics in all_summaries.items():
        for topic, (n, dur) in topics.items():
            by_topic.setdefault(topic, []).append((bag, n, dur))

    shared = {t: v for t, v in by_topic.items() if len(v) > 1}
    if shared:
        print("\n=== SENT vs ARRIVED ===")
        for topic, entries in shared.items():
            print(f"   {topic}")
            most = max(n for _, n, _ in entries)
            for bag, n, dur in entries:
                miss = most - n
                pct = 100.0 * miss / most if most else 0.0
                tag = "  <-- reference (most messages)" if n == most else \
                      f"  MISSING {miss} messages ({pct:.1f}%)"
                print(f"      {bag:<45} {n:6d} msgs{tag}")


if __name__ == '__main__':
    main()
