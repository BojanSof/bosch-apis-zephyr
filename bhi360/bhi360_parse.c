/**
* Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bhi360_parse.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360.h"
#include "bhi360_parse.h"

/*SCALE FACTOR from BHI3 data sheet, IAQ data format*/
#define SCALE_IAQ_VOC                100.0
#define SCALE_IAQ_TEMP               256.0
#define SCALE_IAQ_HUMI               500.0

#define MAXIMUM_VIRTUAL_SENSOR_LIST  UINT16_C(256)

/**
* @brief Function to convert time in tick to seconds and nanoseconds
* @param[in] time_ticks : Time in ticks
* @param[out] s         : Second part of time
* @param[out] ns        : Nanosecond part of time
* @param[out] tns       : Total time in nanoseconds
*/
static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

/**
* @brief Function to parse sensor status meta event
* @param[in] event_text    : Event text
* @param[in] s             : Second part of time
* @param[in] ns            : Nanosecond part of time
* @param[in] byte1         : Byte 1 in meta event
* @param[in] byte2         : Byte 2 in meta event
* @param[in] parse_table   : Pointer to parse table
*/
static void parse_meta_event_sensor_status(char *event_text,
                                           uint32_t s,
                                           uint32_t ns,
                                           uint8_t byte1,
                                           uint8_t byte2,
                                           struct bhi360_parse_ref *parse_table)
{
    struct bhi360_parse_sensor_details *sensor_details;

    sensor_details = bhi360_parse_get_sensor_details(byte1, parse_table);

    /*lint -e774 */
    if (parse_table && sensor_details)
    {
        sensor_details->accuracy = byte2;
    }
}

/**
* @brief Function to parse meta event
* @param[in] callback_info : Pointer to callback information
* @param[in] event_text    : Event text
* @param[in] s             : Second part of time
* @param[in] ns            : Nanosecond part of time
* @param[in] parse_table   : Pointer to parse table
*/
static void parse_meta_event_type(const struct bhi360_fifo_parse_data_info *callback_info,
                                  char *event_text,
                                  uint32_t s,
                                  uint32_t ns,
                                  struct bhi360_parse_ref *parse_table)
{
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];

    // TODO: maybe add logs
    switch (meta_event_type)
    {
        case BHI360_META_EVENT_FLUSH_COMPLETE:
            break;
        case BHI360_META_EVENT_SAMPLE_RATE_CHANGED:
            break;
        case BHI360_META_EVENT_POWER_MODE_CHANGED:
            break;
        case BHI360_META_EVENT_ALGORITHM_EVENTS:
            break;
        case BHI360_META_EVENT_SENSOR_STATUS:
            parse_meta_event_sensor_status(event_text, s, ns, byte1, byte2, parse_table);
            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_MAIN:
            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_CALIB:
            break;
        case BHI360_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            break;
        case BHI360_META_EVENT_SENSOR_ERROR:
            break;
        case BHI360_META_EVENT_FIFO_OVERFLOW:
            break;
        case BHI360_META_EVENT_DYNAMIC_RANGE_CHANGED:
            break;
        case BHI360_META_EVENT_FIFO_WATERMARK:
            break;
        case BHI360_META_EVENT_INITIALIZED:
            break;
        case BHI360_META_TRANSFER_CAUSE:
            break;
        case BHI360_META_EVENT_SENSOR_FRAMEWORK:
            break;
        case BHI360_META_EVENT_RESET:
            break;
        case BHI360_META_EVENT_SPACER:
            break;
        default:
            break;
    }
}

/**
* @brief Function to get sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi360_parse_sensor_details *bhi360_parse_get_sensor_details(uint8_t id, struct bhi360_parse_ref *ref)
{
    uint8_t i;

    for (i = 0; i < BHI360_MAX_SIMUL_SENSORS; i++)
    {
        if (ref->sensor[i].id == id)
        {
            return &ref->sensor[i];
        }
    }

    return NULL;
}

/**
* @brief Function to add sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi360_parse_sensor_details *bhi360_parse_add_sensor_details(uint8_t id, struct bhi360_parse_ref *ref)
{
    uint8_t i = 0;

    struct bhi360_parse_sensor_details *sensor_details;

    sensor_details = bhi360_parse_get_sensor_details(id, ref);
    if (sensor_details)
    {
        /* Slot for the sensor ID is already used */
        return sensor_details;
    }
    else
    {
        /* Find a new slot */
        for (i = 0; i < BHI360_MAX_SIMUL_SENSORS; i++)
        {
            if (ref->sensor[i].id == 0)
            {
                ref->sensor[i].id = id;

                return &ref->sensor[i];
            }
        }
    }

    return NULL;
}

/**
* @brief Function to parse meta event (wake-up and non-wake-up)
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint32_t s, ns;
    uint64_t tns;
    char *event_text;

    if (!callback_info)
    {
        return;
    }

    if (callback_info->sensor_id == BHI360_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHI360_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    parse_meta_event_type(callback_info, event_text, s, ns, parse_table);
}

/**
* @brief Function to parse 3-axis format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_3axis_s16(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_xyz data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    scaling_factor = sensor_details->scaling_factor;

    bhi360_event_data_parse_xyz(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse euler format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_euler(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_orientation data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    scaling_factor = sensor_details->scaling_factor;

    bhi360_event_data_parse_orientation(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse quaternion format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_quaternion(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_quaternion data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    bhi360_event_data_parse_quaternion(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse 16-bit signed format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_s16_as_float(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    int16_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    scaling_factor = sensor_details->scaling_factor;

    data = BHI360_LE2S16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse 32-bit scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_u32(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    data = BHI360_LE2U32(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }
}

/**
* @brief Function to parse scalar event format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse activity format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_activity(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t activity;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    activity = BHI360_LE2U16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse 24-bit unsigned format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_u24_as_float(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    scaling_factor = sensor_details->scaling_factor;

    data = BHI360_LE2U24(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse 8-bit unsigned scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_u8(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    data = callback_info->data_ptr[0];

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse generic format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_generic(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse device orientation format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_device_ori(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    char *ori;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    switch (callback_info->data_ptr[0])
    {
        case 0:
            ori = "Portrait upright";
            break;
        case 1:
            ori = "Landscape left";
            break;
        case 2:
            ori = "Portrait upside down";
            break;
        case 3:
            ori = "Landscape right";
            break;
        default:
            ori = "Unknown orientation";
            break;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse debug message
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_debug_message(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;

    uint32_t s, ns;
    uint64_t tns;
    uint8_t msg_length;
    uint8_t debug_msg[17] = { 0 }; /* Max payload size is 16 bytes, adds a trailing zero if the payload is full */

    if (!callback_info)
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    msg_length = callback_info->data_ptr[0];

    memcpy(debug_msg, &callback_info->data_ptr[1], msg_length);
    debug_msg[msg_length] = '\0'; /* Terminate the string */
}

/**
* @brief Function to parse for Air quality
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_air_quality(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t parse_flag;
    uint32_t s = 0, ns = 0;
    uint64_t tns = 0;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;
    bhi360_event_data_iaq_output_t air_quality = { 0 };

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    bhi360_event_data_parse_air_quality(callback_info->data_ptr, &air_quality);
}

/**
* @brief Function to parse for Multi-tap
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_multitap(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    bhi360_event_data_multi_tap multitap_data = BHI360_NO_TAP;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    (void)bhi360_event_data_multi_tap_parsing(callback_info->data_ptr, (uint8_t *)&multitap_data);
}

/**
* @brief Function to parse for Wrist Gesture Detector
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_wrist_gesture_detect(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    bhi360_event_data_wrist_gesture_detect_t wrist_gesture_detect_data;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    (void)bhi360_event_data_wrist_gesture_detect_parsing(callback_info->data_ptr, &wrist_gesture_detect_data);
}

/**
* @brief Function to parse for Head Misalignment Calibration
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_hmc(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_head_orientation_quat data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    bhi360_event_data_head_orientation_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse for Head Orientation Quaternion
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_oc(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_head_orientation_quat data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    bhi360_event_data_head_orientation_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
* @brief Function to parse for Head Orientation Euler
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_ec(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_head_orientation_eul data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        return;
    }

    bhi360_event_data_head_orientation_eul_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
}

/**
 * @brief Function to parse FIFO frame data into temperature
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] temperature  : Reference to the data buffer to store temperature in degree C
 */
void bhi360_parse_temperature_celsius(const uint8_t *data, bhi360_float *temperature)
{
    /* 1 LSB = 1/100 degC */
    float scale_factor = (float)1 / 100;

    *temperature = BHI360_LE2S16(data) * scale_factor;
}

/**
 * @brief Function to parse FIFO frame data into humidity
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] humidity : Reference to the data buffer to store humidity in %
 */
void bhi360_parse_humidity(const uint8_t *data, bhi360_float *humidity)
{
    float scale_factor = (float)1;

    *humidity = data[0] * scale_factor;
}

/**
 * @brief Function to parse FIFO frame data into barometric pressure
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] pressure : Reference to the data buffer to store pressure in Pascals
 */
void bhi360_parse_pressure(const uint8_t *data, bhi360_float *pressure)
{
    /* 1 LSB = 1/128 Pa */
    float scale_factor = (float)1 / 128;

    *pressure = (float)BHI360_LE2U24(data) * scale_factor;
}

/**
 * @brief Function to parse FIFO frame data into altitude
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] altitude : Reference to the data buffer to store altitude
 */
void bhi360_parse_altitude(const uint8_t *data, bhi360_float *altitude)
{
    *altitude = (float)(data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}
