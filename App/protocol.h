#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_HEADER_0          0xAAu
#define PROTOCOL_HEADER_1          0x55u
#define PROTOCOL_MAX_PAYLOAD_LEN   6u
#define PROTOCOL_MAX_FRAME_LEN     (5u + PROTOCOL_MAX_PAYLOAD_LEN)

typedef enum {
    PROTOCOL_CMD_MATERIAL_SEQUENCE = 0x01u,
    PROTOCOL_CMD_VISION_FEEDBACK   = 0x02u,
} ProtocolCommand_t;

typedef struct {
    uint8_t first[3];    /* 第一轮物料顺序 */
    uint8_t second[3];   /* 第二轮物料顺序 */
} ProtocolMaterialSequences_t;

_Static_assert(sizeof(ProtocolMaterialSequences_t) == 6u,
               "Material sequence payload must be 6 bytes");

typedef struct {
    uint8_t color;       /* 0=未识别，1=红，2=绿，3=蓝 */
    int8_t  offset_x;    /* X 归一化偏差，右正，范围 -127~127 */
    int8_t  offset_y;    /* Y 归一化偏差，下正，范围 -127~127 */
    uint8_t is_static;   /* 0=运动中，1=静止 */
} ProtocolVisionFeedback_t;

_Static_assert(sizeof(ProtocolVisionFeedback_t) == 4u,
               "Vision feedback payload must be 4 bytes");

/** Latest valid vision feedback received from the upper computer. */
extern volatile ProtocolVisionFeedback_t g_vision_feedback;

/**
 * UART integration hook supplied by the lower-computer project.
 * The parser uses it to echo a valid material-sequence frame.
 */
typedef void (*ProtocolTransmitCallback_t)(
    const uint8_t *data, uint16_t length);

void Protocol_SetTransmitCallback(ProtocolTransmitCallback_t callback);

/** Reset the byte-stream parser to the header-search state. */
void Protocol_Reset(void);

/**
 * Feed arbitrary UART byte chunks to the parser.
 * Chunks may contain noise, partial frames, or multiple frames.
 */
void Protocol_ProcessBytes(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H__ */
