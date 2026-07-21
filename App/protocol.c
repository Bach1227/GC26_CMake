#include "protocol.h"

#include "ChassisControl.h"
#include "config.h"
#include "statemachine.h"

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    PARSER_WAIT_HEADER_0 = 0,
    PARSER_WAIT_HEADER_1,
    PARSER_READ_COMMAND,
    PARSER_READ_LENGTH,
    PARSER_READ_PAYLOAD,
    PARSER_READ_CHECKSUM,
} ParserState_t;

typedef struct {
    ParserState_t state;
    uint8_t command;
    uint8_t length;
    uint8_t payload_index;
    uint8_t checksum;
    uint8_t payload[PROTOCOL_MAX_PAYLOAD_LEN];
} ProtocolParser_t;

static ProtocolParser_t parser;
static ProtocolTransmitCallback_t transmit_callback;

volatile ProtocolVisionFeedback_t g_vision_feedback = {
    .color = 0u,
    .offset_x = 0,
    .offset_y = 0,
    .is_static = 0u,
};

#if CONFIG_VISION_ADJUST_ONLY && CONFIG_USE_CHASSIS
/*
 * Pure vision mode is a single adjustment session. Once a session has
 * started, later vision motion (for example, moving the material after
 * alignment) must not start the chassis again.
 */
static bool pure_vision_adjust_started = false;
#endif

#if !CONFIG_VISION_ADJUST_ONLY
static bool is_adjust_state(State_t state)
{
    return state == STATE_ADJUST_RAW_1 ||
           state == STATE_ADJUST_ROUGH_1 ||
           state == STATE_ADJUST_TEMP_1 ||
           state == STATE_ADJUST_RAW_2 ||
           state == STATE_ADJUST_ROUGH_2 ||
           state == STATE_ADJUST_TEMP_2;
}

static bool use_feedback_for_adjust(State_t state, uint8_t color)
{
    bool rough_or_temp =
        state == STATE_ADJUST_ROUGH_1 ||
        state == STATE_ADJUST_TEMP_1 ||
        state == STATE_ADJUST_ROUGH_2 ||
        state == STATE_ADJUST_TEMP_2;

    if (!rough_or_temp) {
        return true;
    }

    switch ((ProtocolColor_t)color) {
    case PROTOCOL_COLOR_GREEN:
        return true;
    case PROTOCOL_COLOR_RED:
        /* Reserved: add red-target adjustment behavior here later. */
        return false;
    case PROTOCOL_COLOR_BLUE:
        /* Reserved: add blue-target adjustment behavior here later. */
        return false;
    case PROTOCOL_COLOR_NONE:
    default:
        /* Reserved: handle target loss or unknown colors here later. */
        return false;
    }
}
#endif

static bool valid_material_sequence(const uint8_t order[3])
{
    uint8_t seen = 0u;

    for (uint8_t i = 0u; i < 3u; ++i) {
        if (order[i] < 1u || order[i] > 3u) {
            return false;
        }
        seen |= (uint8_t)(1u << order[i]);
    }

    return seen == 0x0Eu;
}

static void echo_frame(uint8_t command, const uint8_t *payload, uint8_t length)
{
    if (transmit_callback == NULL) {
        return;
    }

    uint8_t frame[PROTOCOL_MAX_FRAME_LEN];
    uint8_t checksum = (uint8_t)(command + length);
    frame[0] = PROTOCOL_HEADER_0;
    frame[1] = PROTOCOL_HEADER_1;
    frame[2] = command;
    frame[3] = length;
    for (uint8_t i = 0u; i < length; ++i) {
        frame[4u + i] = payload[i];
        checksum = (uint8_t)(checksum + payload[i]);
    }
    frame[4u + length] = checksum;
    transmit_callback(frame, (uint16_t)(5u + length));
}

static void dispatch_frame(uint8_t command, const uint8_t *payload,
                           uint8_t length)
{
    switch (command) {
    case PROTOCOL_CMD_MATERIAL_SEQUENCE:
        if (length == sizeof(ProtocolMaterialSequences_t) &&
            valid_material_sequence(payload) &&
            valid_material_sequence(payload + 3u)) {
            /*
             * Firmware integration contract: statemachine.h must expose
             * SM_SetMaterialSequences(first, second) and save both rounds.
             */
            (void)SM_SetMaterialSequences(payload, payload + 3u);
            /*
             * Always echo repeated valid frames. The host retries until one
             * echo reaches it, even if the first echo was lost.
             */
            echo_frame(command, payload, length);
        }
        break;

    case PROTOCOL_CMD_VISION_FEEDBACK:
        if (length == sizeof(ProtocolVisionFeedback_t) &&
            payload[0] <= 3u &&
            payload[3] <= 1u) {
            ProtocolVisionFeedback_t feedback = {
                .color = payload[0],
                .offset_x = (int8_t)payload[1],
                .offset_y = (int8_t)payload[2],
                .is_static = payload[3],
            };

            g_vision_feedback.color = feedback.color;
            g_vision_feedback.offset_x = feedback.offset_x;
            g_vision_feedback.offset_y = feedback.offset_y;
            g_vision_feedback.is_static = feedback.is_static;
            SM_SetCurrentColor(feedback.color);

#if !CONFIG_VISION_ADJUST_ONLY
            State_t adjust_state = SM_GetState();
            if (!is_adjust_state(adjust_state) ||
                !use_feedback_for_adjust(adjust_state, feedback.color)) {
                break;
            }
#endif

            /*
             * is_static only gates whether this sample participates in the
             * control loop. Completion is determined exclusively by the
             * X/Y error thresholds in ChassisControl.
             */
            if (feedback.is_static == 0u) {
                break;
            }

#if CONFIG_USE_CHASSIS
#if CONFIG_VISION_ADJUST_ONLY
            if (!g_chassis_adjust_heading_active) {
                if (pure_vision_adjust_started ||
                    !Chassis_BeginVisionAdjust()) {
                    break;
                }
                pure_vision_adjust_started = true;
            }
#endif
            if (g_chassis_adjust_heading_active) {
                Chassis_UpdateVisionAdjust(feedback.offset_x,
                                           feedback.offset_y);
            }
#else
            (void)feedback.offset_x;
            (void)feedback.offset_y;
#endif
        }
        break;

    default:
        break;
    }
}

void Protocol_Reset(void)
{
    parser.state = PARSER_WAIT_HEADER_0;
    parser.command = 0u;
    parser.length = 0u;
    parser.payload_index = 0u;
    parser.checksum = 0u;
}

void Protocol_SetTransmitCallback(ProtocolTransmitCallback_t callback)
{
    transmit_callback = callback;
}

static void process_byte(uint8_t byte)
{
    switch (parser.state) {
    case PARSER_WAIT_HEADER_0:
        if (byte == PROTOCOL_HEADER_0) {
            parser.state = PARSER_WAIT_HEADER_1;
        }
        break;

    case PARSER_WAIT_HEADER_1:
        if (byte == PROTOCOL_HEADER_1) {
            parser.state = PARSER_READ_COMMAND;
        } else if (byte != PROTOCOL_HEADER_0) {
            parser.state = PARSER_WAIT_HEADER_0;
        }
        break;

    case PARSER_READ_COMMAND:
        parser.command = byte;
        parser.checksum = byte;
        parser.state = PARSER_READ_LENGTH;
        break;

    case PARSER_READ_LENGTH:
        parser.length = byte;
        parser.checksum = (uint8_t)(parser.checksum + byte);
        parser.payload_index = 0u;

        if (parser.length > PROTOCOL_MAX_PAYLOAD_LEN) {
            Protocol_Reset();
        } else if (parser.length == 0u) {
            parser.state = PARSER_READ_CHECKSUM;
        } else {
            parser.state = PARSER_READ_PAYLOAD;
        }
        break;

    case PARSER_READ_PAYLOAD:
        parser.payload[parser.payload_index++] = byte;
        parser.checksum = (uint8_t)(parser.checksum + byte);
        if (parser.payload_index >= parser.length) {
            parser.state = PARSER_READ_CHECKSUM;
        }
        break;

    case PARSER_READ_CHECKSUM:
        if (byte == parser.checksum) {
            dispatch_frame(parser.command, parser.payload, parser.length);
            Protocol_Reset();
        } else {
            Protocol_Reset();
            if (byte == PROTOCOL_HEADER_0) {
                parser.state = PARSER_WAIT_HEADER_1;
            }
        }
        break;

    default:
        Protocol_Reset();
        break;
    }
}

void Protocol_ProcessBytes(const uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return;
    }

    for (uint16_t i = 0u; i < length; ++i) {
        process_byte(data[i]);
    }
}
