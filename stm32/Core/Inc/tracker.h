#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>

#define TRACKER_CHANNELS 3U
#define TRACKER_MAX_LEN 32U
#define TRACKER_ACK_CAPACITY 16U
#define TRACKER_CONTROL_MS 20U
#define TRACKER_TRACK_TIMEOUT_MS 200U
#define TRACKER_LINK_TIMEOUT_MS 1000U
#define TRACKER_TELEMETRY_MS 100U

/* Units: degrees, microseconds, degrees per pixel per control update.
 * Each physical axis has its own calibration and installation direction. */
typedef struct {
    float min_deg, max_deg, center_deg;
    float span_deg;
    uint16_t pulse_min_us, pulse_max_us;
    float kp;
    int8_t direction;
} TrackerAxisConfig;

typedef struct {
    TrackerAxisConfig axis[2];
    uint16_t deadzone_px;
    float max_step_deg;
} TrackerConfig;

typedef struct {
    TrackerConfig config;
    float angle[2];
    int16_t offset[2];
    uint8_t mode, output_enabled, link_seen, track_pending;
    uint32_t last_link_ms, last_track_ms, last_control_ms, last_telemetry_ms;
    uint8_t parser[TRACKER_MAX_LEN + 4U];
    uint8_t parser_size;
    uint32_t last_byte_ms;
    uint8_t acks[TRACKER_ACK_CAPACITY];
    uint8_t ack_head, ack_count;
    uint32_t invalid_frames;
} Tracker;

/* Return 1 only after copying/accepting the complete frame. Busy returns 0;
 * the pending ACK or telemetry will be retried in the next main-loop pass. */
typedef int (*TrackerSend)(void *user, const uint8_t *frame, uint8_t size);

void Tracker_DefaultConfig(TrackerConfig *config);
int Tracker_Init(Tracker *tracker, const TrackerConfig *config, uint32_t now);
void Tracker_Receive(Tracker *tracker, uint8_t byte, uint32_t now);
void Tracker_Tick(Tracker *tracker, uint32_t now);
void Tracker_ServiceTx(Tracker *tracker, uint32_t now, TrackerSend send, void *user);
void Tracker_ResetRx(Tracker *tracker);
uint8_t Tracker_Flags(const Tracker *tracker, uint32_t now);
uint16_t Tracker_Pulse(const Tracker *tracker, unsigned axis);
uint8_t Tracker_Crc8(const uint8_t *data, unsigned size);
uint8_t Tracker_Frame(uint8_t type, const uint8_t *payload, uint8_t size, uint8_t *out);

#endif
