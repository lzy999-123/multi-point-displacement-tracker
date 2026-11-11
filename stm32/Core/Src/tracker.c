#include "tracker.h"

#include <math.h>
#include <string.h>

static float clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

void Tracker_DefaultConfig(TrackerConfig *config)
{
    unsigned i;
    memset(config, 0, sizeof(*config));
    for (i = 0; i < 2U; ++i) {
        config->axis[i].min_deg = 15.0f;
        config->axis[i].max_deg = 165.0f;
        config->axis[i].center_deg = 90.0f;
        config->axis[i].span_deg = 180.0f;
        config->axis[i].pulse_min_us = 500U;
        config->axis[i].pulse_max_us = 2500U;
        config->axis[i].kp = 0.00625f;
        config->axis[i].direction = -1;
    }
    config->deadzone_px = 8U;
    config->max_step_deg = 2.0f;
}

int Tracker_Init(Tracker *tracker, const TrackerConfig *config, uint32_t now)
{
    unsigned i;
    /* Reject invalid calibration before any PWM can be enabled. */
    if (!isfinite(config->max_step_deg) || config->max_step_deg <= 0.0f) return 0;
    for (i = 0; i < 2U; ++i) {
        const TrackerAxisConfig *a = &config->axis[i];
        if (!isfinite(a->min_deg) || !isfinite(a->max_deg) ||
            !isfinite(a->center_deg) || !isfinite(a->span_deg) ||
            !isfinite(a->kp) || a->kp < 0.0f || a->span_deg <= 0.0f || a->span_deg > 270.0f ||
            a->min_deg < 0.0f || a->max_deg > a->span_deg ||
            a->min_deg >= a->max_deg || a->center_deg < a->min_deg ||
            a->center_deg > a->max_deg || a->pulse_min_us < 500U ||
            a->pulse_max_us > 2500U || a->pulse_min_us >= a->pulse_max_us ||
            (a->direction != -1 && a->direction != 1)) return 0;
    }
    memset(tracker, 0, sizeof(*tracker));
    tracker->config = *config;
    tracker->angle[0] = config->axis[0].center_deg;
    tracker->angle[1] = config->axis[1].center_deg;
    tracker->last_control_ms = now;
    tracker->last_telemetry_ms = now;
    return 1;
}

uint8_t Tracker_Crc8(const uint8_t *data, unsigned size)
{
    uint8_t crc = 0;
    unsigned i, bit;
    for (i = 0; i < size; ++i) {
        crc ^= data[i];
        for (bit = 0; bit < 8U; ++bit)
            crc = (uint8_t)((crc & 0x80U) ? (crc << 1U) ^ 0x07U : crc << 1U);
    }
    return crc;
}

uint8_t Tracker_Frame(uint8_t type, const uint8_t *payload, uint8_t size, uint8_t *out)
{
    if (size >= TRACKER_MAX_LEN) return 0;
    out[0] = 0xA5U;
    out[1] = 0x5AU;
    out[2] = (uint8_t)(size + 1U);
    out[3] = type;
    if (size) memcpy(out + 4, payload, size);
    out[4U + size] = Tracker_Crc8(out + 2, size + 2U);
    return (uint8_t)(size + 5U);
}

static int16_t read_i16(const uint8_t *p)
{
    int32_t value = (int32_t)p[0] * 256 + p[1];
    return (int16_t)(value >= 32768 ? value - 65536 : value);
}

static void write_i16(uint8_t *p, int16_t value)
{
    uint16_t bits = (uint16_t)value;
    p[0] = (uint8_t)(bits >> 8U);
    p[1] = (uint8_t)bits;
}

static void manual(Tracker *t)
{
    t->mode = 0;
    t->track_pending = 0;
}

static void set_angle(Tracker *t, unsigned axis, float angle)
{
    const TrackerAxisConfig *a = &t->config.axis[axis];
    t->angle[axis] = clamp(angle, a->min_deg, a->max_deg);
    t->output_enabled = 1;
}

static int dispatch(Tracker *t, uint8_t type, const uint8_t *p, uint8_t size, uint32_t now)
{
    uint8_t cmd = 0;
    if (!((type == 1U && size == 4U) || (type == 2U && size == 1U) ||
          (type == 3U && size == 0U) || (type == 4U && size == 4U))) return 0;
    /* Do not execute a command for which an ACK cannot be retained. */
    if ((type == 2U || type == 4U) && t->ack_count == TRACKER_ACK_CAPACITY) return 0;
    /* An expired link must be stopped BEFORE processing a newly arrived frame. */
    if (t->link_seen && (uint32_t)(now - t->last_link_ms) >= TRACKER_LINK_TIMEOUT_MS)
        manual(t);
    t->link_seen = 1;
    t->last_link_ms = now;
    if (type == 1U) {
        if (t->mode) {
            t->offset[0] = read_i16(p);
            t->offset[1] = read_i16(p + 2);
            t->last_track_ms = now;
            t->track_pending = 1;
        }
    } else if (type == 2U) {
        cmd = p[0];
        switch (cmd) {
        case 0x01: /* Explicit manual mode, not the old mode toggle. */
        case 0x12:
            manual(t);
            break;
        case 0x02:
            manual(t);
            set_angle(t, 0, t->config.axis[0].center_deg);
            set_angle(t, 1, t->config.axis[1].center_deg);
            break;
        case 0x11:
            t->mode = 1;
            t->track_pending = 0; /* Require fresh data after every start. */
            break;
        case 0x20:
        case 0x21:
        case 0x22:
        case 0x23:
            if (!t->mode) {
                unsigned axis = cmd < 0x22U ? 1U : 0U;
                float sign = (cmd == 0x20U || cmd == 0x22U) ? -1.0f : 1.0f;
                /* Manual direction and tracking share the same sign mapping. */
                set_angle(t, axis, t->angle[axis] + sign * t->config.axis[axis].direction);
            }
            break;
        default: /* Unknown command: acknowledge receipt, do not move. */
            break;
        }
    } else if (type == 4U) {
        cmd = 4;
        if (!t->mode) {
            set_angle(t, 0, read_i16(p) / 10.0f);
            set_angle(t, 1, read_i16(p + 2) / 10.0f);
        }
    }
    if (type == 2U || type == 4U) {
        t->acks[(t->ack_head + t->ack_count) % TRACKER_ACK_CAPACITY] = cmd;
        ++t->ack_count;
    }
    return 1;
}

static void drop_byte(Tracker *t)
{
    --t->parser_size;
    memmove(t->parser, t->parser + 1, t->parser_size);
}

void Tracker_Receive(Tracker *t, uint8_t byte, uint32_t now)
{
    unsigned total;
    if ((uint32_t)(now - t->last_byte_ms) >= TRACKER_TRACK_TIMEOUT_MS) t->parser_size = 0;
    t->last_byte_ms = now;
    t->parser[t->parser_size++] = byte;
    while (t->parser_size >= 2U) {
        if (t->parser[0] != 0xA5U || t->parser[1] != 0x5AU) { drop_byte(t); continue; }
        if (t->parser_size < 3U) return;
        if (t->parser[2] < 1U || t->parser[2] > TRACKER_MAX_LEN) {
            ++t->invalid_frames;
            drop_byte(t);
            continue;
        }
        total = t->parser[2] + 4U;
        if (t->parser_size < total) return;
        if (Tracker_Crc8(t->parser + 2, total - 3U) != t->parser[total - 1U]) {
            ++t->invalid_frames;
            drop_byte(t); /* Recover even when a valid frame starts inside a bad one. */
            continue;
        }
        if (!dispatch(t, t->parser[3], t->parser + 4, (uint8_t)(t->parser[2] - 1U), now))
            ++t->invalid_frames;
        t->parser_size = (uint8_t)(t->parser_size - total);
        memmove(t->parser, t->parser + total, t->parser_size);
    }
}

void Tracker_ResetRx(Tracker *t)
{
    t->parser_size = 0;
    t->link_seen = 0;
    manual(t);
}

void Tracker_Tick(Tracker *t, uint32_t now)
{
    unsigned i;
    if (!t->link_seen || (uint32_t)(now - t->last_link_ms) >= TRACKER_LINK_TIMEOUT_MS)
        manual(t); /* Hold current PWM; never return to center on link loss. */
    if ((uint32_t)(now - t->last_track_ms) >= TRACKER_TRACK_TIMEOUT_MS) t->track_pending = 0;
    if ((uint32_t)(now - t->last_control_ms) < TRACKER_CONTROL_MS) return;
    t->last_control_ms = now; /* No catch-up burst following a delayed loop. */
    if (!t->mode || !t->track_pending) return;
    t->track_pending = 0;
    for (i = 0; i < 2U; ++i) {
        float offset = (float)t->offset[i];
        if (fabsf(offset) > t->config.deadzone_px) {
            float delta = offset * t->config.axis[i].kp * t->config.axis[i].direction;
            delta = clamp(delta, -t->config.max_step_deg, t->config.max_step_deg);
            set_angle(t, i, t->angle[i] + delta);
        }
    }
}

uint8_t Tracker_Flags(const Tracker *t, uint32_t now)
{
    uint8_t flags = 0;
    unsigned i;
    if (!t->link_seen || (uint32_t)(now - t->last_link_ms) >= TRACKER_LINK_TIMEOUT_MS) flags |= 1U;
    for (i = 0; i < 2U; ++i)
        if (t->angle[i] <= t->config.axis[i].min_deg || t->angle[i] >= t->config.axis[i].max_deg)
            flags |= 2U;
    return flags;
}

uint16_t Tracker_Pulse(const Tracker *t, unsigned axis)
{
    const TrackerAxisConfig *a = &t->config.axis[axis];
    float pulse = a->pulse_min_us + t->angle[axis] / a->span_deg * (a->pulse_max_us - a->pulse_min_us);
    return (uint16_t)(pulse + 0.5f);
}

void Tracker_ServiceTx(Tracker *t, uint32_t now, TrackerSend send, void *user)
{
    uint8_t frame[TRACKER_MAX_LEN + 4U], payload[6], size;
    if (t->ack_count) {
        size = Tracker_Frame(0x82U, &t->acks[t->ack_head], 1, frame);
        if (send(user, frame, size)) {
            t->ack_head = (uint8_t)((t->ack_head + 1U) % TRACKER_ACK_CAPACITY);
            --t->ack_count;
        }
    } else if ((uint32_t)(now - t->last_telemetry_ms) >= TRACKER_TELEMETRY_MS) {
        payload[0] = t->mode;
        /* These are commanded angles. Ordinary PWM servos have no feedback. */
        write_i16(payload + 1, (int16_t)(t->angle[0] * 10.0f + 0.5f));
        write_i16(payload + 3, (int16_t)(t->angle[1] * 10.0f + 0.5f));
        payload[5] = Tracker_Flags(t, now);
        size = Tracker_Frame(0x81U, payload, sizeof(payload), frame);
        if (send(user, frame, size)) t->last_telemetry_ms = now;
    }
}
