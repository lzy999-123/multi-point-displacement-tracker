"""Bounded VID1 reassembly shared by UDP validation and loopback tests."""
import struct

HEADER = struct.Struct("<IBBHBBHI")
PAYLOAD_SIZE = 1400
MAX_FRAME_SIZE = 65536


def newer(seq, previous):
    return 0 < ((seq - previous) & 65535) < 32768


class Assembler:
    def __init__(self):
        self.slots = {}
        self.last_seq = None
        self.device_id = None
        self.invalid_packets = 0
        self.duplicate_or_old_packets = 0

    def reset(self):
        self.slots.clear()
        self.last_seq = None

    def feed(self, data, now):
        if len(data) < HEADER.size:
            self.invalid_packets += 1
            return None
        magic, device, _, seq, index, count, payload_size, size = HEADER.unpack_from(data)
        expected_count = (size + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
        expected_size = min(PAYLOAD_SIZE, size - index * PAYLOAD_SIZE)
        if (magic != 0x31444956 or device not in (1, 2, 3)
                or (self.device_id is not None and device != self.device_id)
                or not 0 < size <= MAX_FRAME_SIZE or count != expected_count
                or index >= count or payload_size != expected_size
                or len(data) != HEADER.size + payload_size):
            self.invalid_packets += 1
            return None
        if self.last_seq is not None and not newer(seq, self.last_seq):
            self.duplicate_or_old_packets += 1
            return None
        key = seq % 8
        slot = self.slots.get(key)
        if slot and now - slot["started"] >= .25:
            slot = None
        if slot and slot["seq"] != seq and not newer(seq, slot["seq"]):
            return None
        if not slot or slot["seq"] != seq:
            slot = dict(seq=seq, size=size, count=count, started=now, chunks={})
            self.slots[key] = slot
        elif slot["size"] != size or slot["count"] != count:
            self.invalid_packets += 1
            return None
        slot["chunks"][index] = data[HEADER.size:]
        if len(slot["chunks"]) != count:
            return None
        del self.slots[key]
        return device, seq, b"".join(slot["chunks"][i] for i in range(count))

    def delivered(self, device, seq):
        # Only successful decodes advance the chronology.
        self.device_id = device
        self.last_seq = seq
