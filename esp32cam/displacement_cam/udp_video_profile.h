#pragma once

#include <stddef.h>
#include <stdint.h>

namespace VideoProfile {
// Existing IDs/URL keys remain compatible. fps=0 explicitly means uncapped.
enum Id : uint8_t { Vga12 = 0, Qvga25 = 1, Baseline10 = 2, Svga = 3 };

struct Preset {
  uint16_t width, height;
  uint8_t fps, minimumQuality, maximumQuality;
  uint16_t targetBytes;
};

inline bool valid(unsigned id) { return id <= Svga; }
inline Preset preset(unsigned id) {
  if (id == Baseline10) return Preset{320, 240, 10, 32, 32, 6500};
  if (id == Svga) return Preset{800, 600, 0, 26, 46, 10000};
  return id == Qvga25 ? Preset{320, 240, 0, 24, 42, 2600}
                      : Preset{640, 480, 0, 34, 62, 5600};
}

// Production modes keep protocol fps=0 (no rigid legacy cap), while using a
// smooth per-resolution pacing floor paired with AIMD congestion backoff so
// three cameras sharing one 2.4GHz hotspot do not saturate the AP queue.
inline unsigned sendIntervalMs(unsigned id) {
  return preset(id).fps ? 1000 / preset(id).fps : 0;
}

inline unsigned adaptiveFloorMs(unsigned id) {
  if (id == Baseline10) return 100;
  if (id == Svga) return 105;
  return id == Qvga25 ? 40 : 60;
}

inline bool validTransport(unsigned fps, unsigned minimum, unsigned maximum, unsigned targetBytes) {
  return (fps == 0 || (fps >= 5 && fps <= 40)) && minimum >= 10 && minimum <= 40
      && maximum >= minimum && maximum <= 62 && targetBytes >= 1000 && targetBytes <= 16000;
}

// Parse JPEG SOF dimensions, so a frame queued before a sensor change cannot be
// labelled with the new resolution merely because sensor metadata has changed.
inline bool jpegDimensions(const uint8_t *data, size_t size, unsigned &width, unsigned &height) {
  if (size < 4 || data[0] != 0xff || data[1] != 0xd8) return false;
  size_t offset = 2;
  while (offset + 4 <= size) {
    if (data[offset++] != 0xff) return false;
    while (offset < size && data[offset] == 0xff) ++offset;
    if (offset >= size) return false;
    const unsigned marker = data[offset++];
    if (marker == 0xda || marker == 0xd9) return false;
    if (marker == 1 || (marker >= 0xd0 && marker <= 0xd7)) continue;
    if (offset + 2 > size) return false;
    const unsigned length = unsigned(data[offset]) * 256 + data[offset + 1];
    if (length < 2 || length > size - offset) return false;
    if ((marker >= 0xc0 && marker <= 0xc3) || (marker >= 0xc5 && marker <= 0xc7)
        || (marker >= 0xc9 && marker <= 0xcb) || (marker >= 0xcd && marker <= 0xcf)) {
      if (length < 8) return false;
      height = unsigned(data[offset + 3]) * 256 + data[offset + 4];
      width = unsigned(data[offset + 5]) * 256 + data[offset + 6];
      return width != 0 && height != 0;
    }
    offset += length;
  }
  return false;
}
} // namespace VideoProfile
