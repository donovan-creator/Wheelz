"""Bounded JPEG extraction from an ESP MJPEG stream."""


class JpegBuffer:
    def __init__(self, max_bytes=2 * 1024 * 1024):
        self.buffer = bytearray()
        self.max_bytes = max_bytes

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while True:
            start = self.buffer.find(b'\xff\xd8')
            if start < 0:
                self.buffer[:] = self.buffer[-1:]
                break
            if start:
                del self.buffer[:start]
            end = self.buffer.find(b'\xff\xd9', 2)
            if end < 0:
                if len(self.buffer) > self.max_bytes:
                    self.buffer.clear()
                    raise ValueError('MJPEG frame exceeded buffer limit')
                break
            if end + 2 > self.max_bytes:
                self.buffer.clear()
                raise ValueError('MJPEG frame exceeded buffer limit')
            frames.append(bytes(self.buffer[:end + 2]))
            del self.buffer[:end + 2]
        return frames
