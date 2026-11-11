"""Regression cases that could falsely certify a wireless 20fps stream."""
import unittest
from stream_metrics import summarize_frames


def frame(t, seq, width=320, height=240, sha="static-scene"):
    return dict(seconds=t, seq=seq, width=width, height=height, sha256=sha)


class StreamMetricsTest(unittest.TestCase):
    def test_new_captures_of_static_scene_are_real_frames(self):
        frames = [frame(i / 25 + .001, i) for i in range(250)]
        report = summarize_frames(frames, 10, 2)
        self.assertEqual(report["steady_min_source_fps"], 25)
        self.assertGreaterEqual(report["steady_min_rolling_source_fps"], 24)
        self.assertEqual(report["unique_jpegs_per_second"][2], 1)
        self.assertLess(report["steady_max_no_frame_ms"], 41)

    def test_repeated_sequence_cannot_inflate_rate(self):
        frames = [frame(i / 25, 7, sha=str(i)) for i in range(250)]
        report = summarize_frames(frames, 10, 2)
        self.assertEqual(report["source_frames"], 1)
        self.assertEqual(report["steady_min_source_fps"], 0)
        self.assertEqual(report["duplicate_sequences"], 249)
        self.assertEqual(report["steady_max_no_frame_ms"], 8000)

    def test_final_freeze_includes_trailing_silence(self):
        report = summarize_frames([frame(i / 25, i) for i in range(100)], 10, 2)
        self.assertGreater(report["steady_max_no_frame_ms"], 6000)
        self.assertEqual(report["steady_min_rolling_source_fps"], 0)

    def test_wrong_size_midstream_is_not_hidden_by_last_frame(self):
        frames = [frame(i / 25, i) for i in range(100)]
        frames[40]["width"] = 160
        self.assertEqual(summarize_frames(frames, 4, 2)["wrong_resolution_frames"], 1)

    def test_bursts_can_pass_fixed_bins_but_fail_rolling_window(self):
        frames = [frame(s + (.05 if s % 2 == 0 else .9) + i * .001, s * 25 + i)
                  for s in range(10) for i in range(25)]
        report = summarize_frames(frames, 10, 2)
        self.assertEqual(report["steady_min_source_fps"], 25)
        self.assertEqual(report["steady_min_rolling_source_fps"], 0)

    def test_missing_sequence_and_post_deadline_frames(self):
        report = summarize_frames([frame(2.1, None), frame(5, 1)], 5, 2)
        self.assertEqual(report["missing_sequences"], 1)
        self.assertEqual(report["frames_within_requested_window"], 1)
        self.assertEqual(report["steady_min_source_fps"], 0)

    def test_uint32_wrap_is_valid_but_reversal_is_not(self):
        frames = [frame(2.1, 0xFFFFFFFF), frame(2.2, 0), frame(2.3, 1), frame(2.4, 0)]
        report = summarize_frames(frames, 4, 2)
        self.assertEqual(report["source_frames"], 3)
        self.assertEqual(report["out_of_order_sequences"], 1)


if __name__ == "__main__":
    unittest.main()
