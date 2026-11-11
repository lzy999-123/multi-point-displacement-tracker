#include "tracker.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b)) < 0.001f)
#define RUN(x) do { x(); puts("PASS " #x); ++passed; } while (0)

typedef struct { uint8_t data[36], size; unsigned count; int busy; } Sink;

static int capture(void *user, const uint8_t *frame, uint8_t size)
{
    Sink *s = (Sink *)user;
    if (s->busy) return 0;
    memcpy(s->data, frame, size);
    s->size = size;
    ++s->count;
    return 1;
}

static Tracker make_tracker(uint32_t now)
{
    Tracker t;
    TrackerConfig config;
    Tracker_DefaultConfig(&config);
    CHECK(Tracker_Init(&t, &config, now));
    return t;
}

static void feed(Tracker *t, uint8_t type, const uint8_t *payload, uint8_t size, uint32_t now)
{
    uint8_t data[36], n, i;
    n = Tracker_Frame(type, payload, size, data);
    CHECK(n != 0);
    for (i = 0; i < n; ++i) Tracker_Receive(t, data[i], now);
}

static void command(Tracker *t, uint8_t cmd, uint32_t now) { feed(t, 2, &cmd, 1, now); }

static void track(Tracker *t, int16_t x, int16_t y, uint32_t now)
{
    uint8_t p[4] = {(uint8_t)((uint16_t)x >> 8), (uint8_t)x,
                    (uint8_t)((uint16_t)y >> 8), (uint8_t)y};
    feed(t, 1, p, 4, now);
}

static void startup_and_fresh_data(void)
{
    Tracker t = make_tracker(0);
    CHECK(!t.output_enabled && Tracker_Flags(&t, 0) == 1);
    track(&t, 320, 320, 1); /* Manual input must not be cached for a later start. */
    command(&t, 0x11, 2);
    Tracker_Tick(&t, 20);
    CHECK(t.mode == 1 && !t.output_enabled);
    track(&t, 320, -320, 21);
    Tracker_Tick(&t, 40);
    CHECK(t.output_enabled);
    NEAR(t.angle[0], 88.0f);
    NEAR(t.angle[1], 92.0f);
    Tracker_Tick(&t, 60);
    NEAR(t.angle[0], 88.0f); /* Each sample is consumed only once. */
}

static void crc_and_wire_format(void)
{
    static const uint8_t heartbeat[] = {0xA5, 0x5A, 1, 3, 0x1C};
    Tracker t = make_tracker(0);
    uint8_t frame[36];
    unsigned i;
    CHECK(Tracker_Crc8((const uint8_t *)"123456789", 9) == 0xF4);
    CHECK(Tracker_Frame(3, NULL, 0, frame) == sizeof(heartbeat));
    CHECK(memcmp(heartbeat, frame, sizeof(heartbeat)) == 0);
    for (i = 0; i < sizeof(heartbeat); ++i) Tracker_Receive(&t, heartbeat[i], 10 + i);
    CHECK(t.link_seen && !t.output_enabled);
}

static void resynchronization(void)
{
    Tracker t = make_tracker(0);
    uint8_t cmd = 0x02, good[36], bad[36];
    uint8_t n = Tracker_Frame(2, &cmd, 1, good);
    unsigned i;
    /* Repeated sync byte and an out-of-range length followed by a good frame. */
    const uint8_t noise[] = {0x77, 0xA5, 0xA5, 0x5A, 0xFF};
    for (i = 0; i < sizeof(noise); ++i) Tracker_Receive(&t, noise[i], 1);
    memcpy(bad, good, n);
    bad[n-1] ^= 1;
    for (i = 0; i < n; ++i) Tracker_Receive(&t, bad[i], 2);
    CHECK(!t.output_enabled);
    for (i = 0; i < n; ++i) Tracker_Receive(&t, good[i], 3);
    CHECK(t.output_enabled && t.ack_count == 1 && t.invalid_frames >= 2);
}

static void embedded_frame_recovery(void)
{
    Tracker t = make_tracker(0);
    uint8_t command_payload = 2, frame[36];
    uint8_t n = Tracker_Frame(2, &command_payload, 1, frame);
    uint8_t corrupt[] = {0xA5, 0x5A, 8, 0xEE};
    unsigned i;
    for (i = 0; i < sizeof(corrupt); ++i) Tracker_Receive(&t, corrupt[i], 1);
    for (i = 0; i < n; ++i) Tracker_Receive(&t, frame[i], 1);
    Tracker_Receive(&t, 0, 1);
    Tracker_Receive(&t, 0, 1);
    CHECK(t.output_enabled && t.ack_count == 1);
}

static void malformed_does_not_refresh_link(void)
{
    Tracker t = make_tracker(0);
    uint8_t old_step[] = {0x22, 0, 5};
    feed(&t, 3, NULL, 0, 10);
    feed(&t, 4, old_step, 3, 900); /* Old FOC TYPE 04 is incompatible. */
    feed(&t, 0xEE, NULL, 0, 950);
    CHECK(t.last_link_ms == 10 && !t.output_enabled && t.ack_count == 0);
    CHECK(Tracker_Flags(&t, 1010) & 1U);
}

static void incomplete_frame_gap_recovery(void)
{
    Tracker t = make_tracker(0);
    uint8_t prefix[] = {0xA5, 0x5A, 32, 1, 0};
    unsigned i;
    for (i = 0; i < sizeof(prefix); ++i) Tracker_Receive(&t, prefix[i], 1);
    CHECK(!t.link_seen);
    command(&t, 2, 201);
    CHECK(t.output_enabled && t.ack_count == 1);
}

static void limits_and_pulse_mapping(void)
{
    Tracker t = make_tracker(0);
    const uint8_t p[] = {0x7F, 0xFF, 0x80, 0};
    feed(&t, 4, p, 4, 1);
    NEAR(t.angle[0], 165.0f);
    NEAR(t.angle[1], 15.0f);
    CHECK(Tracker_Flags(&t, 1) == 2);
    CHECK(Tracker_Pulse(&t, 0) == 2333 && Tracker_Pulse(&t, 1) == 667);
    command(&t, 2, 2);
    CHECK(Tracker_Pulse(&t, 0) == 1500 && Tracker_Flags(&t, 2) == 0);
}

static void manual_semantics_and_signs(void)
{
    Tracker t = make_tracker(0);
    command(&t, 0x20, 1); NEAR(t.angle[1], 91.0f);
    command(&t, 0x21, 2); NEAR(t.angle[1], 90.0f);
    command(&t, 0x22, 3); NEAR(t.angle[0], 91.0f);
    command(&t, 0x23, 4); NEAR(t.angle[0], 90.0f);
    command(&t, 0x11, 5);
    command(&t, 0x23, 6); NEAR(t.angle[0], 90.0f);
    command(&t, 1, 7);
    command(&t, 1, 8); CHECK(!t.mode); /* Repeating manual is idempotent. */
    command(&t, 0xEE, 9); NEAR(t.angle[0], 90.0f);
}

static void timeout_holds_and_does_not_resume(void)
{
    Tracker t = make_tracker(0);
    command(&t, 0x11, 1);
    track(&t, 160, 0, 2);
    Tracker_Tick(&t, 20);
    NEAR(t.angle[0], 89.0f);
    track(&t, 320, 0, 21);
    feed(&t, 3, NULL, 0, 200);
    Tracker_Tick(&t, 221); /* Track timed out, heartbeat still valid. */
    NEAR(t.angle[0], 89.0f); CHECK(t.mode == 1);
    Tracker_Tick(&t, 1200);
    CHECK(!t.mode && t.output_enabled && (Tracker_Flags(&t, 1200) & 1U));
    feed(&t, 3, NULL, 0, 1201);
    track(&t, 320, 0, 1202);
    Tracker_Tick(&t, 1220);
    CHECK(!t.mode); NEAR(t.angle[0], 89.0f);
}

static void late_packet_cannot_hide_expiry(void)
{
    Tracker t = make_tracker(0);
    command(&t, 0x11, 1);
    track(&t, 320, 0, 1002); /* No Tick in between: expiry must still be enforced. */
    Tracker_Tick(&t, 1002);
    CHECK(!t.mode && !t.output_enabled);
    command(&t, 0x11, 1003);
    track(&t, 320, 0, 1004);
    command(&t, 0x11, 1005);
    Tracker_Tick(&t, 1022);
    CHECK(!t.output_enabled); /* Repeated start clears stale data. */
}

static void control_deadzone_and_step_limit(void)
{
    Tracker t = make_tracker(0);
    command(&t, 0x11, 1);
    track(&t, 8, -8, 2);
    Tracker_Tick(&t, 20);
    CHECK(!t.output_enabled);
    track(&t, 32767, -32768, 21);
    Tracker_Tick(&t, 39); CHECK(!t.output_enabled);
    Tracker_Tick(&t, 40);
    NEAR(t.angle[0], 88.0f); NEAR(t.angle[1], 92.0f);
    track(&t, 320, 0, 41);
    Tracker_Tick(&t, 100); /* No catch-up loop. */
    NEAR(t.angle[0], 86.0f);
}

static void ack_retention_and_telemetry(void)
{
    Tracker t = make_tracker(0);
    Sink s = {{0}, 0, 0, 1};
    command(&t, 2, 1);
    Tracker_ServiceTx(&t, 100, capture, &s);
    CHECK(t.ack_count == 1 && !s.count);
    s.busy = 0;
    Tracker_ServiceTx(&t, 101, capture, &s);
    CHECK(!t.ack_count && s.count == 1 && s.size == 6 && s.data[3] == 0x82 && s.data[4] == 2);
    Tracker_ServiceTx(&t, 102, capture, &s);
    CHECK(s.size == 11 && s.data[3] == 0x81 && s.data[4] == 0);
    CHECK(s.data[5] == 3 && s.data[6] == 0x84 && s.data[9] == 0);
    CHECK(Tracker_Crc8(s.data + 2, s.size - 3U) == s.data[s.size - 1U]);
    Tracker_ServiceTx(&t, 150, capture, &s); CHECK(s.count == 2);
    Tracker_ServiceTx(&t, 202, capture, &s); CHECK(s.count == 3);
}

static void full_ack_queue_does_not_execute(void)
{
    Tracker t = make_tracker(0);
    unsigned i;
    for (i = 0; i < TRACKER_ACK_CAPACITY; ++i) command(&t, 0xEE, i + 1);
    command(&t, 2, 30);
    CHECK(t.ack_count == TRACKER_ACK_CAPACITY && !t.output_enabled);
}

static void three_channel_isolation(void)
{
    Tracker t[3];
    unsigned i;
    for (i = 0; i < 3; ++i) t[i] = make_tracker(0);
    command(&t[1], 0x11, 1);
    track(&t[1], -160, 160, 2);
    for (i = 0; i < 3; ++i) Tracker_Tick(&t[i], 20);
    CHECK(!t[0].output_enabled && t[1].output_enabled && !t[2].output_enabled);
    NEAR(t[1].angle[0], 91.0f); NEAR(t[1].angle[1], 89.0f);
    Tracker_ResetRx(&t[1]);
    CHECK(!t[1].mode && t[1].output_enabled && Tracker_Flags(&t[1], 21) == 1);
}

static void config_270_and_rejection(void)
{
    Tracker t;
    TrackerConfig c;
    Tracker_DefaultConfig(&c);
    c.axis[0].span_deg = 270;
    c.axis[0].center_deg = 135;
    CHECK(Tracker_Init(&t, &c, 0));
    CHECK(Tracker_Pulse(&t, 0) == 1500);
    c.axis[0].center_deg = 999;
    CHECK(!Tracker_Init(&t, &c, 0));
    Tracker_DefaultConfig(&c); c.axis[0].direction = 0;
    CHECK(!Tracker_Init(&t, &c, 0));
    Tracker_DefaultConfig(&c); c.max_step_deg = NAN;
    CHECK(!Tracker_Init(&t, &c, 0));
    Tracker_DefaultConfig(&c); c.axis[0].span_deg = 3600;
    CHECK(!Tracker_Init(&t, &c, 0));
}

static void tick_wraparound(void)
{
    uint32_t start = UINT32_MAX - 30U;
    Tracker t = make_tracker(start);
    command(&t, 0x11, start);
    track(&t, 160, 0, start + 1U);
    Tracker_Tick(&t, start + 20U);
    NEAR(t.angle[0], 89.0f);
    Tracker_Tick(&t, 100U); CHECK(t.mode == 1);
    Tracker_Tick(&t, 1000U); CHECK(!t.mode);
}

static void corrupted_stream_fuzz(void)
{
    Tracker t = make_tracker(0);
    uint32_t state = 1;
    unsigned i;
    for (i = 0; i < 100000U; ++i) {
        state = state * 1664525U + 1013904223U;
        Tracker_Receive(&t, (uint8_t)(state >> 24), i);
        Tracker_Tick(&t, i);
        CHECK(t.parser_size < sizeof(t.parser));
        CHECK(t.angle[0] >= 15 && t.angle[0] <= 165);
    }
    Tracker_ResetRx(&t);
    command(&t, 2, 100001U);
    CHECK(t.output_enabled);
}

int main(void)
{
    unsigned passed = 0;
    RUN(startup_and_fresh_data);
    RUN(crc_and_wire_format);
    RUN(resynchronization);
    RUN(embedded_frame_recovery);
    RUN(malformed_does_not_refresh_link);
    RUN(incomplete_frame_gap_recovery);
    RUN(limits_and_pulse_mapping);
    RUN(manual_semantics_and_signs);
    RUN(timeout_holds_and_does_not_resume);
    RUN(late_packet_cannot_hide_expiry);
    RUN(control_deadzone_and_step_limit);
    RUN(ack_retention_and_telemetry);
    RUN(full_ack_queue_does_not_execute);
    RUN(three_channel_isolation);
    RUN(config_270_and_rejection);
    RUN(tick_wraparound);
    RUN(corrupted_stream_fuzz);
    printf("%u scenarios passed (real tracker.c; no hardware attached).\n", passed);
    return 0;
}
