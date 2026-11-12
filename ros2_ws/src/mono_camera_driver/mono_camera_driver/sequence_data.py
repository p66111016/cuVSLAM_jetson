"""Strict timestamp and IMU readers; no ROS or cuVSLAM dependency."""

from bisect import bisect_right
import csv
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import math


def microseconds_to_ns(value):
    try:
        ns = Decimal(value) * 1000
        if not ns.is_finite() or ns <= 0 or ns != ns.to_integral_value():
            raise ValueError()
        return int(ns)
    except (InvalidOperation, ValueError):
        raise ValueError(f'invalid microsecond timestamp: {value!r}') from None


def load_image_timestamps(path, files):
    """Match by filename, never by CSV row index; reject missing/duplicate times."""
    with open(path, newline='') as stream:
        reader = csv.DictReader(stream)
        columns = reader.fieldnames or []
        column = next((c for c in ('unix_us', 'unix_utc+8') if c in columns), None)
        if 'Label' not in columns or column is None:
            raise ValueError('image CSV requires Label and unix_us (or unix_utc+8)')
        rows = {}
        for row in reader:
            label = row['Label']
            if label in rows:
                raise ValueError(f'duplicate image label: {label}')
            rows[label] = row[column]
    stamps = []
    for file in files:
        if not rows.get(file.name):
            raise ValueError(f'missing image timestamp: {file.name}')
        stamps.append(microseconds_to_ns(rows[file.name]))
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('image timestamps must strictly increase; rounded scientific '
                         'notation cannot synchronize IMU. Use the full-precision '
                         'IMU/EOP_firefly_unix_aligned_imu.csv mapping.')
    return stamps


@dataclass(frozen=True)
class ImuSample:
    stamp_ns: int
    gyro: tuple
    accel: tuple


def load_imu(path):
    samples = []
    with open(path, newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'unix_us', 'GyrX', 'GyrY', 'GyrZ', 'AccX', 'AccY', 'AccZ'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f'IMU CSV requires {sorted(required)}')
        for row in reader:
            stamp = microseconds_to_ns(row['unix_us'])
            gyro = tuple(float(row[c]) for c in ('GyrX', 'GyrY', 'GyrZ'))
            accel = tuple(float(row[c]) for c in ('AccX', 'AccY', 'AccZ'))
            if not all(math.isfinite(v) for v in gyro + accel):
                raise ValueError(f'non-finite IMU at {stamp}')
            if samples and stamp <= samples[-1].stamp_ns:
                raise ValueError('IMU timestamps must strictly increase; select one sensor stream')
            samples.append(ImuSample(stamp, gyro, accel))
    if len(samples) < 2:
        raise ValueError('IMU CSV must contain at least two samples')
    return samples


class ImuReplay:
    """Partition raw IMU samples into (previous image, current image] batches."""

    def __init__(self, samples, image_stamps, warmup_s=1.0, max_gap_s=0.05):
        if warmup_s < 0 or max_gap_s <= 0:
            raise ValueError('warmup must be nonnegative; max IMU gap must be positive')
        self.samples = samples
        self.stamps = [s.stamp_ns for s in samples]
        begin = image_stamps[0] - round(warmup_s * 1e9)
        end = image_stamps[-1]
        if self.stamps[0] > begin or self.stamps[-1] < end:
            raise ValueError('IMU does not cover selected images and requested warmup')
        self.index = max(0, bisect_right(self.stamps, begin) - 1)
        stop = min(len(samples), bisect_right(self.stamps, end) + 1)
        gaps = [b - a for a, b in zip(self.stamps[self.index:stop],
                                    self.stamps[self.index + 1:stop])]
        self.max_gap_ns = max(gaps, default=0)
        if self.max_gap_ns > max_gap_s * 1e9:
            raise ValueError(f'IMU gap {self.max_gap_ns / 1e9:.6f}s exceeds {max_gap_s}s')
        self.last_image_ns = -1

    def through(self, stamp_ns):
        if stamp_ns <= self.last_image_ns:
            raise ValueError('image timestamps must strictly increase')
        stop = bisect_right(self.stamps, stamp_ns)
        batch = self.samples[self.index:stop]
        self.index = stop
        self.last_image_ns = stamp_ns
        return batch
