import unittest
from pathlib import Path

from orthostream_injector import link, link_vectors

VECTORS = Path(__file__).resolve().parents[3] / "interfaces" / "link" / "test_vectors.txt"


class LinkTest(unittest.TestCase):
    def test_vectors_file_is_current(self):
        self.assertEqual(VECTORS.read_text(), link_vectors.render(),
                         "run: python3 -m orthostream_injector.link_vectors ../../interfaces/link/test_vectors.txt")

    def test_sizes(self):
        self.assertEqual(link.SIZES, {"HEADER": 16, "TELEMETRY": 104, "HEARTBEAT": 24, "SESSION_START": 200,
                                      "IMAGE": 32, "SESSION_END": 16})

    def test_header_fields(self):
        data = link.session_end(1, 2, 3, sequence=0xFFFFFFFF + 6)
        magic, version, msg_type, length, seq = link.HEADER.unpack_from(data)
        self.assertEqual((magic, version, msg_type, length, seq), (b"OSTL", 1, 18, 16, 5))

    def test_image_length_counts_data(self):
        img = link.Image(1, 2, 3, 4, b"x" * 1000)
        head = link.image_head(img, 0)
        self.assertEqual(len(head), 16 + 32)
        self.assertEqual(link.HEADER.unpack_from(head)[3], 32 + 1000)


if __name__ == "__main__":
    unittest.main()
