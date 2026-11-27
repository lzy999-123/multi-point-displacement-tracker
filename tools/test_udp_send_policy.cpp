#include "../esp32cam/displacement_cam/udp_send_policy.h"
#include <cassert>
#include <cstdio>
#include "../esp32cam/displacement_cam/udp_video_profile.h"

int main() {
  UdpSendPolicy policy(45, 80);
  assert(!policy.expired(79, 0));
  assert(policy.expired(80, 0));
  assert(policy.retryDelay(0, 0, 0) == 2);
  assert(policy.retryDelay(100, 71, 0) == 8);
  assert(policy.retryDelay(100, 72, 0) == 0);
  // JPEGs larger than 21KB must still fit the deliberate pacing budget.
  for (unsigned chunks = 2; chunks <= 47; ++chunks)
    assert(policy.chunkDelay(chunks) * (chunks - 1) <= 60);

  assert(policy.finish(0, 30, true, false) == 45);
  assert(policy.finish(100, 180, true, false) == 181);
  for (unsigned i = 0; i < 100; ++i) {
    const unsigned next = policy.finish(200, 280, false, true);
    assert(next >= 330 && next <= 430);
  }
  assert(policy.gap() == 4);
  for (unsigned i = 0; i < 90; ++i) policy.finish(0, 20, true, false);
  assert(policy.gap() == 1);

  policy.setInterval(40);
  assert(policy.finish(10, 30, true, false) == 50);
  policy.setInterval(0);
  assert(policy.finish(10, 30, true, false) == 31);

  const uint32_t start = 0xfffffff0U;
  assert(!policy.expired(0x3fU, start));
  assert(policy.expired(0x40U, start));
  assert(!UdpSendPolicy::due(0xfffffff0U, 10));
  assert(UdpSendPolicy::due(10, 10));
  assert(UdpSendPolicy::due(11, 10));
  UdpFrameFeedback feedback(150);
  feedback.submitted(1, 0);
  assert(!feedback.waiting(1)); // Legacy clients never wait for an ACK.
  feedback.enable(true);
  assert(feedback.limit() == 1);
  for (unsigned i = 0; i < 30; ++i) {
    feedback.submitted(uint16_t(i), 0);
    assert(feedback.waiting(1) == (feedback.limit() == 1));
    assert(feedback.acknowledge(uint16_t(i)));
  }
  assert(feedback.limit() == 2);
  feedback.submitted(65534, start);
  feedback.submitted(65535, start);
  assert(feedback.waiting(0x85));
  assert(!feedback.acknowledge(0)); // Unrelated/stale acknowledgements do not release the window.
  assert(feedback.acknowledge(65535));
  assert(!feedback.waiting(0x86));
  assert(feedback.expire(0x86));
  assert(feedback.limit() == 2); // Lost ACKs alone do not prove driver pressure.
  assert(!feedback.expire(0x86));
  for (unsigned i = 0; i < 30; ++i) {
    feedback.submitted(uint16_t(i), 90);
    assert(feedback.acknowledge(uint16_t(i)));
  }
  feedback.submitted(0, 100);
  feedback.submitted(1, 100);
  assert(feedback.waiting(200));
  assert(feedback.acknowledge(0));
  assert(!feedback.waiting(200));
  feedback.submitted(2, 200);
  assert(feedback.waiting(201));
  feedback.pressure();
  assert(feedback.expire(250));
  assert(feedback.waiting(250)); // One pending frame fills the reduced window.
  assert(!feedback.acknowledge(1));
  assert(feedback.acknowledge(2));
  assert(!feedback.waiting(250));
  feedback.pressure();
  assert(feedback.limit() == 1);
  feedback.submitted(1, 0);
  feedback.enable(false);
  assert(!feedback.waiting(1));
  feedback.setMinimumWindow(2);
  feedback.enable(true);
  feedback.pressure();
  feedback.submitted(1, 100);
  assert(!feedback.waiting(101));
  feedback.submitted(2, 100);
  assert(feedback.waiting(101));
  feedback.enable(false);
  feedback.setWindow(6, 8);
  feedback.enable(true);
  for (unsigned i=0;i<6;++i) feedback.submitted(uint16_t(65532+i), 100);
  assert(feedback.waiting(101));
  assert(!feedback.acknowledgeLatest(999));
  assert(feedback.waiting(101));
  assert(feedback.acknowledgeLatest(1)); // Releases 65532..65535 and 0 across wrap.
  assert(!feedback.waiting(101));
  feedback.enable(false);
  const auto vga = VideoProfile::preset(VideoProfile::Vga12);
  const auto qvga = VideoProfile::preset(VideoProfile::Qvga25);
  assert(vga.width == 640 && vga.height == 480 && vga.fps == 0);
  assert(qvga.width == 320 && qvga.height == 240 && qvga.fps == 0);
  assert(VideoProfile::sendIntervalMs(VideoProfile::Vga12) == 0);
  assert(VideoProfile::sendIntervalMs(VideoProfile::Qvga25) == 0);
  assert(VideoProfile::sendIntervalMs(VideoProfile::Baseline10) == 100);
  // Both shipped presets must pass the same gate that applies cadence/quality
  // on the camera. Rejecting the 2600-byte QVGA target leaves the VGA cadence.
  const auto baseline = VideoProfile::preset(VideoProfile::Baseline10);
  assert(baseline.width == 320 && baseline.height == 240 && baseline.fps == 10);
  assert(baseline.minimumQuality == 32 && baseline.maximumQuality == 32);
  const auto svga = VideoProfile::preset(VideoProfile::Svga);
  assert(svga.width == 800 && svga.height == 600 && svga.fps == 0);
  assert(VideoProfile::sendIntervalMs(VideoProfile::Svga) == 0);
  const VideoProfile::Preset presets[] = {vga, qvga, baseline, svga};
  for (const auto preset : presets)
    assert(VideoProfile::validTransport(preset.fps, preset.minimumQuality,
                                       preset.maximumQuality, preset.targetBytes));
  assert(VideoProfile::validTransport(0, 24, 42, 2600));
  assert(!VideoProfile::validTransport(1, 24, 42, 2600));
  assert(!VideoProfile::validTransport(25, 42, 24, 2600));
  assert(!VideoProfile::validTransport(25, 24, 42, 0));
  const uint8_t jpeg[] = {0xff,0xd8,0xff,0xc0,0,8,8,1,0xe0,2,0x80,1};
  unsigned width=0,height=0;
  assert(VideoProfile::jpegDimensions(jpeg,sizeof(jpeg),width,height));
  assert(width == 640 && height == 480);
  assert(!VideoProfile::jpegDimensions(jpeg,sizeof(jpeg)-1,width,height));
  UdpJpegQuality quality(26, 6000, 22, 38);
  for (unsigned i=0;i<100;++i) quality.observe(12000, true);
  assert(quality.quality() == 38);
  for (unsigned i=0;i<2500;++i) quality.observe(3000, false);
  assert(quality.quality() == 22);
  quality.configure(4500, 28, 32);
  assert(quality.quality() == 28);
  for (unsigned i=0;i<200;++i) quality.observe(9000, false);
  assert(quality.quality() == 32);
  // Shipped 350ms budget: a slow link may finish a whole VGA frame instead of
  // abandoning it after a few chunks; retries keep going well past 80ms.
  UdpSendPolicy slow(83, 350);
  assert(!slow.expired(349, 0));
  assert(slow.expired(350, 0));
  assert(slow.retryDelay(10, 200, 0) == 8);
  assert(slow.retryDelay(10, 342, 0) == 0);
  for (unsigned chunks = 2; chunks <= 47; ++chunks)
    assert(slow.chunkDelay(chunks) * (chunks - 1) <= 330);
  // A frame that took longer than the cadence is followed immediately (no catch-up burst).
  assert(slow.finish(0, 300, true, false) == 301);
  // AIMD adaptive pacing backs off even when retries succeed, and recovers on clean frames.
  UdpSendPolicy adaptive(0, 350);
  adaptive.setFloorInterval(VideoProfile::adaptiveFloorMs(VideoProfile::Vga12));
  assert(adaptive.effectiveInterval() == 60);
  assert(VideoProfile::adaptiveFloorMs(VideoProfile::Qvga25) == 40);
  assert(VideoProfile::adaptiveFloorMs(VideoProfile::Svga) == 105);
  assert(!adaptive.firstChunkExpired(34, 0));
  assert(adaptive.firstChunkExpired(35, 0));
  adaptive.finish(0, 20, true, true);
  assert(adaptive.adaptiveDelay() == 8);
  assert(adaptive.effectiveInterval() == 68);
  for (unsigned i = 0; i < 20; ++i) adaptive.finish(0, 10, true, false);
  assert(adaptive.adaptiveDelay() == 0);
  std::puts("UDP pacing deadline, large-frame, AIMD recovery and rollover checks passed.");
}
