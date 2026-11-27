#pragma once

#include <stdint.h>

// All deadlines use unsigned elapsed time, including across millis() rollover.
// One frame owns the send budget; a temporary full driver queue does not consume
// a fixed retry count and discard an otherwise recoverable JPEG immediately.
class UdpSendPolicy {
public:
  UdpSendPolicy(uint32_t intervalMs, uint32_t budgetMs)
      : m_intervalMs(intervalMs), m_budgetMs(budgetMs) {}

  static bool due(uint32_t now, uint32_t deadline) {
    return int32_t(now - deadline) >= 0;
  }

  bool expired(uint32_t now, uint32_t started) const {
    return uint32_t(now - started) >= m_budgetMs;
  }

  // If chunk 0 cannot even enter the driver queue within 35ms, abort before
  // putting a single partial fragment on the air and fetch a fresher image.
  bool firstChunkExpired(uint32_t now, uint32_t started) const {
    const uint32_t limit = m_budgetMs < 35 ? m_budgetMs : 35;
    return uint32_t(now - started) >= limit;
  }

  uint32_t retryDelay(uint32_t attempt, uint32_t now, uint32_t started) const {
    const uint32_t spent = uint32_t(now - started);
    const uint32_t delay = attempt >= 3 ? 8 : 2 + attempt * 2;
    return spent < m_budgetMs && delay < m_budgetMs - spent ? delay : 0;
  }

  uint32_t chunkDelay(uint32_t chunkCount) const {
    if (chunkCount < 2) return 0;
    // Leave 20ms for send calls/retries even for a maximum-size VID1 frame.
    const uint32_t allowance = m_budgetMs > 20 ? (m_budgetMs - 20) / (chunkCount - 1) : 0;
    return m_gapMs < allowance ? m_gapMs : allowance;
  }

  void pressure() {
    if (m_gapMs < 4) ++m_gapMs;
    m_cleanFrames = 0;
  }

  uint32_t finish(uint32_t started, uint32_t now, bool success, bool retried) {
    uint32_t recovery = 0;
    if (success) {
      m_failures = 0;
      if (retried) {
        // Additive backoff even when retries succeed: keep the Wi-Fi/hotspot
        // queue shallow instead of immediately re-triggering ENOMEM storms.
        m_cleanFrames = 0;
        m_adaptiveMs = m_adaptiveMs + 8 > kMaxAdaptiveMs ? kMaxAdaptiveMs : m_adaptiveMs + 8;
      } else {
        ++m_cleanFrames;
        if (m_adaptiveMs > 0 && (m_cleanFrames % 5 == 0)) {
          const uint32_t step = m_adaptiveMs / 4 > 4 ? m_adaptiveMs / 4 : 4;
          m_adaptiveMs = m_adaptiveMs > step ? m_adaptiveMs - step : 0;
        }
        if (m_cleanFrames >= 30) {
          if (m_gapMs > 1) --m_gapMs;
          m_cleanFrames = 0;
        }
      }
    } else {
      pressure();
      if (m_failures < 3) ++m_failures;
      m_adaptiveMs = m_adaptiveMs + 24 > kMaxAdaptiveMs ? kMaxAdaptiveMs : m_adaptiveMs + 24;
      // A partial JPEG also occupies the radio queue. Give it time to drain,
      // even though it cannot produce an application ACK.
      recovery = m_failures * 50; // 50/100/150ms, bounded and reset on success.
    }
    const uint32_t baseInterval = m_intervalMs > m_floorMs ? m_intervalMs : m_floorMs;
    const uint32_t effectiveInterval = baseInterval + m_adaptiveMs;
    const uint32_t spent = uint32_t(now - started);
    const uint32_t cadenceWait = spent < effectiveInterval ? effectiveInterval - spent : 0;
    // A late frame never creates a catch-up burst; always yield to the driver.
    const uint32_t wait = cadenceWait > recovery ? cadenceWait : (recovery ? recovery : 1);
    return now + wait;
  }

  void setInterval(uint32_t intervalMs) {
    if (intervalMs != m_intervalMs) {
      m_intervalMs = intervalMs;
      m_adaptiveMs = 0;
    }
  }
  void setFloorInterval(uint32_t floorMs) {
    if (floorMs != m_floorMs) {
      m_floorMs = floorMs;
      m_adaptiveMs = 0;
      m_cleanFrames = 0;
    }
  }
  uint32_t interval() const { return m_intervalMs; }
  uint32_t effectiveInterval() const {
    return (m_intervalMs > m_floorMs ? m_intervalMs : m_floorMs) + m_adaptiveMs;
  }
  uint32_t adaptiveDelay() const { return m_adaptiveMs; }
  uint32_t gap() const { return m_gapMs; }

private:
  static constexpr uint32_t kMaxAdaptiveMs = 180;
  uint32_t m_intervalMs;
  uint32_t m_floorMs = 0;
  uint32_t m_adaptiveMs = 0;
  uint32_t m_budgetMs;
  uint32_t m_gapMs = 1;
  uint32_t m_cleanFrames = 0;
  uint32_t m_failures = 0;
};

// Negotiate this window with VREQA; legacy VREQ clients need no ACK support.
class UdpFrameFeedback {
public:
  explicit UdpFrameFeedback(uint32_t timeoutMs) : m_timeoutMs(timeoutMs) {}
  void enable(bool enabled) {
    if (enabled != m_enabled) { m_limit = m_minimum; m_cleanAcks = 0; }
    m_enabled = enabled;
    if (!enabled) for (Slot &slot : m_slots) slot.active = false;
  }
  void submitted(uint16_t sequence, uint32_t started) {
    if (!m_enabled) return;
    for (Slot &slot : m_slots) {
      if (!slot.active) {
        slot = Slot{started, sequence, true};
        return;
      }
    }
  }
  bool acknowledge(uint16_t sequence) {
    for (Slot &slot : m_slots) {
      if (slot.active && sequence == slot.sequence) {
        slot.active = false;
        if (++m_cleanAcks >= 30) { m_limit = m_capacity; m_cleanAcks = 0; }
        return true;
      }
    }
    return false;
  }
  bool waiting(uint32_t now) const {
    unsigned pending = 0;
    for (const Slot &slot : m_slots)
      if (slot.active && uint32_t(now - slot.started) < m_timeoutMs) ++pending;
    return pending >= m_limit;
  }
  bool expire(uint32_t now) {
    for (Slot &slot : m_slots) {
      if (slot.active && uint32_t(now - slot.started) >= m_timeoutMs) {
        slot.active = false;
        return true;
      }
    }
    return false;
  }
  bool enabled() const { return m_enabled; }
  bool acknowledgeLatest(uint16_t sequence) {
    // A completed newer image makes older pending images obsolete. Release
    // their send credits; never use an unmatched/stale ACK to release credits.
    if (!acknowledge(sequence)) return false;
    for (Slot &slot : m_slots) {
      const uint16_t delta = uint16_t(sequence - slot.sequence);
      if (slot.active && delta > 0 && delta < 32768) slot.active = false;
    }
    return true;
  }
  void pressure() { m_limit = m_minimum; m_cleanAcks = 0; }
  void setMinimumWindow(unsigned minimum) {
    m_minimum = minimum >= 2 ? 2 : 1;
    if (m_limit < m_minimum) m_limit = m_minimum;
  }
  void setWindow(unsigned minimum, unsigned maximum) {
    if (minimum < 1 || minimum > maximum || maximum > kCapacity) return;
    if (m_minimum == minimum && m_capacity == maximum) return;
    m_minimum = minimum;
    m_capacity = maximum;
    m_limit = minimum;
    m_cleanAcks = 0;
  }
  unsigned limit() const { return m_limit; }

private:
  uint32_t m_timeoutMs;
  struct Slot { uint32_t started = 0; uint16_t sequence = 0; bool active = false; };
  static constexpr unsigned kCapacity = 8;
  Slot m_slots[kCapacity];
  unsigned m_limit = 1;
  unsigned m_minimum = 1;
  unsigned m_capacity = 2;
  unsigned m_cleanAcks = 0;
  bool m_enabled = false;
};

// Raise the hardware JPEG quantizer (smaller images) under sustained byte/driver
// pressure. Recover detail slowly. Resolution and sensor cadence never change.
class UdpJpegQuality {
public:
  UdpJpegQuality(unsigned quality, unsigned target, unsigned minimum, unsigned maximum)
      : m_quality(quality), m_target(target), m_minimum(minimum), m_maximum(maximum) {}
  void configure(unsigned target, unsigned minimum, unsigned maximum) {
    m_target = target;
    m_minimum = minimum;
    m_maximum = maximum < minimum ? minimum : maximum;
    if (m_quality < m_minimum) m_quality = m_minimum;
    if (m_quality > m_maximum) m_quality = m_maximum;
  }
  unsigned observe(unsigned bytes, bool pressure) {
    m_average = m_average ? (m_average * 7 + bytes) / 8 : bytes;
    if (pressure || m_average > m_target) {
      m_clean = 0;
      const bool heavy = m_average > m_target * 3 / 2;
      if (++m_pressure >= ((pressure || heavy) ? 2U : 4U)) {
        const unsigned step = heavy ? 2U : 1U;
        m_quality = m_quality + step > m_maximum ? m_maximum : m_quality + step;
        m_pressure = 0;
      }
    } else {
      m_pressure = 0;
      if (m_average < m_target * 3 / 4 && ++m_clean >= 90) {
        if (m_quality > m_minimum) --m_quality;
        m_clean = 0;
      }
    }
    return m_quality;
  }
  unsigned quality() const { return m_quality; }
  unsigned average() const { return m_average; }
private:
  unsigned m_quality, m_target, m_minimum, m_maximum;
  unsigned m_average = 0, m_pressure = 0, m_clean = 0;
};
