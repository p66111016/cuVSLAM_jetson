"""Input validation and timestamp partitioning, independent of ROS/CUDA."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/mono_camera_driver'))
from mono_camera_driver.sequence_data import (ImuReplay, ImuSample, load_image_timestamps,
                                              load_imu, microseconds_to_ns)


class SequenceDataTests(unittest.TestCase):
    def write_csv(self, rows):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = Path(tmp.name) / 'data.csv'
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerows(rows)
        return path

    def test_microsecond_precision(self):
        self.assertEqual(microseconds_to_ns('1782089908520829'), 1782089908520829000)
        for value in ('', 'nan', 'inf', '-1', '0.0001'):
            with self.assertRaises(ValueError):
                microseconds_to_ns(value)

    def test_filename_mapping_and_invalid_timestamps(self):
        files = [Path('a.png'), Path('b.png')]
        path = self.write_csv([['Label', 'unix_us'], ['b.png', '101'], ['a.png', '100']])
        self.assertEqual(load_image_timestamps(path, files), [100000, 101000])
        for rows in ([['a.png', '1.78209E+15'], ['b.png', '1.78209E+15']],
                     [['a.png', '100'], ['b.png', '']],
                     [['a.png', '100'], ['a.png', '101']],
                     [['a.png', '101'], ['b.png', '100']]):
            path = self.write_csv([['Label', 'unix_us']] + rows)
            with self.assertRaises(ValueError):
                load_image_timestamps(path, files)

    def test_batches_do_not_repeat_or_include_future_samples(self):
        samples = [ImuSample(i * 10000000, (1, 2, 3), (4, 5, 6)) for i in range(1, 11)]
        replay = ImuReplay(samples, [30000000, 55000000, 90000000], warmup_s=0.02)
        batches = [replay.through(t) for t in (30000000, 55000000, 90000000)]
        self.assertEqual([[s.stamp_ns for s in b] for b in batches],
                         [[10000000, 20000000, 30000000], [40000000, 50000000],
                          [60000000, 70000000, 80000000, 90000000]])
        with self.assertRaises(ValueError):
            replay.through(90000000)
        with self.assertRaises(ValueError):
            ImuReplay(samples, [30000000, 110000000], warmup_s=0)
        with self.assertRaises(ValueError):
            ImuReplay([samples[0], samples[-1]], [30000000, 90000000], warmup_s=0)

    def test_raw_imu_validation(self):
        header = ['unix_us', 'GyrX', 'GyrY', 'GyrZ', 'AccX', 'AccY', 'AccZ']
        path = self.write_csv([header, ['100', 1, 2, 3, 4, 5, -9.8],
                               ['105', 1, 2, 3, 4, 5, -9.8]])
        samples = load_imu(path)
        self.assertEqual(samples[0].accel, (4., 5., -9.8))
        for last in (['100', 1, 2, 3, 4, 5, 6], ['105', 'nan', 2, 3, 4, 5, 6]):
            path = self.write_csv([header, ['100', 1, 2, 3, 4, 5, 6], last])
            with self.assertRaises(ValueError):
                load_imu(path)


if __name__ == '__main__':
    unittest.main()
