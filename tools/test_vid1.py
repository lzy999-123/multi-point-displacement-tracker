import unittest

from check_esp32_udp_stream import summarize
from vid1 import Assembler, HEADER, newer


def packet(seq, index=0, data=b'jpeg', total=4, count=1):
    return HEADER.pack(0x31444956, 1, 0, seq, index, count, len(data), total) + data


class Vid1Test(unittest.TestCase):
    def test_out_of_order_chunks_and_duplicate_packets(self):
        assembler = Assembler()
        first = packet(7, 0, b'a' * 1400, 1500, 2)
        last = packet(7, 1, b'b' * 100, 1500, 2)
        self.assertIsNone(assembler.feed(last, 0))
        self.assertIsNone(assembler.feed(last, .001))
        device, seq, jpeg = assembler.feed(first, .002)
        self.assertEqual(jpeg, b'a' * 1400 + b'b' * 100)
        assembler.delivered(device, seq)
        self.assertIsNone(assembler.feed(first, .003))
        self.assertIsNone(assembler.feed(packet(6), .004))

    def test_inconsistent_metadata_and_truncation_are_rejected(self):
        assembler = Assembler()
        self.assertIsNone(assembler.feed(packet(1, 0, b'a' * 1400, 1500, 2), 0))
        self.assertIsNone(assembler.feed(packet(1, 4, b'b' * 1400, 7000, 5), .001))
        self.assertIsNone(assembler.feed(packet(2)[:-1], .002))
        self.assertEqual(assembler.invalid_packets, 2)

    def test_sequence_wrap_and_restart_reset(self):
        assembler = Assembler()
        assembler.delivered(1, 65535)
        self.assertIsNotNone(assembler.feed(packet(0), 0))
        assembler.delivered(1, 10000)
        self.assertIsNone(assembler.feed(packet(0), .1))
        assembler.reset()
        self.assertIsNotNone(assembler.feed(packet(0), .2))
        self.assertFalse(newer(7, 7))

    def test_only_successful_decode_advances_sequence(self):
        assembler = Assembler()
        assembler.feed(packet(1000), 0)  # caller failed to decode
        self.assertIsNotNone(assembler.feed(packet(1), .01))

    def test_trailing_silence_and_wrong_resolution_fail(self):
        frames = [dict(time_s=i / 25 + .001, width=320, height=240) for i in range(75)]
        result = summarize(frames, 4, 2, 20, 150, 0)
        self.assertFalse(result['passed'])
        self.assertGreater(result['max_gap_ms'], 1000)
        frames = [dict(time_s=i / 25 + .001, width=160, height=120) for i in range(100)]
        result = summarize(frames, 4, 2, 20, 150, 0)
        self.assertFalse(result['passed'])
        self.assertEqual(result['wrong_resolution_frames'], 100)

    def test_good_stream_passes_but_decode_errors_do_not(self):
        frames = [dict(time_s=i / 25 + .001, width=320, height=240) for i in range(100)]
        self.assertTrue(summarize(frames, 4, 2, 20, 150, 0)['passed'])
        self.assertFalse(summarize(frames, 4, 2, 20, 150, 1)['passed'])


if __name__ == '__main__':
    unittest.main()
