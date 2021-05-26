#pragma once
// MESSAGE GPS_METADATA PACKING

#define MAVLINK_MSG_ID_GPS_METADATA 95


typedef struct __mavlink_gps_metadata_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
 uint64_t epoch_time_usec; /*< [us] GPS Epoch time*/
 int16_t avg_cno; /*< [dB] Avg CN0 of overhead only satellites.*/
 uint8_t use_feature; /*< [bitmask] User feature override*/
} mavlink_gps_metadata_t;

#define MAVLINK_MSG_ID_GPS_METADATA_LEN 19
#define MAVLINK_MSG_ID_GPS_METADATA_MIN_LEN 19
#define MAVLINK_MSG_ID_95_LEN 19
#define MAVLINK_MSG_ID_95_MIN_LEN 19

#define MAVLINK_MSG_ID_GPS_METADATA_CRC 64
#define MAVLINK_MSG_ID_95_CRC 64



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_METADATA { \
    95, \
    "GPS_METADATA", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_metadata_t, time_usec) }, \
         { "avg_cno", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_gps_metadata_t, avg_cno) }, \
         { "epoch_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_gps_metadata_t, epoch_time_usec) }, \
         { "use_feature", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_gps_metadata_t, use_feature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_METADATA { \
    "GPS_METADATA", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_metadata_t, time_usec) }, \
         { "avg_cno", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_gps_metadata_t, avg_cno) }, \
         { "epoch_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_gps_metadata_t, epoch_time_usec) }, \
         { "use_feature", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_gps_metadata_t, use_feature) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_metadata message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param avg_cno [dB] Avg CN0 of overhead only satellites.
 * @param epoch_time_usec [us] GPS Epoch time
 * @param use_feature [bitmask] User feature override
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_metadata_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, int16_t avg_cno, uint64_t epoch_time_usec, uint8_t use_feature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_METADATA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, epoch_time_usec);
    _mav_put_int16_t(buf, 16, avg_cno);
    _mav_put_uint8_t(buf, 18, use_feature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_METADATA_LEN);
#else
    mavlink_gps_metadata_t packet;
    packet.time_usec = time_usec;
    packet.epoch_time_usec = epoch_time_usec;
    packet.avg_cno = avg_cno;
    packet.use_feature = use_feature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_METADATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_METADATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_METADATA_MIN_LEN, MAVLINK_MSG_ID_GPS_METADATA_LEN, MAVLINK_MSG_ID_GPS_METADATA_CRC);
}

/**
 * @brief Pack a gps_metadata message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param avg_cno [dB] Avg CN0 of overhead only satellites.
 * @param epoch_time_usec [us] GPS Epoch time
 * @param use_feature [bitmask] User feature override
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_metadata_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,int16_t avg_cno,uint64_t epoch_time_usec,uint8_t use_feature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_METADATA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, epoch_time_usec);
    _mav_put_int16_t(buf, 16, avg_cno);
    _mav_put_uint8_t(buf, 18, use_feature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_METADATA_LEN);
#else
    mavlink_gps_metadata_t packet;
    packet.time_usec = time_usec;
    packet.epoch_time_usec = epoch_time_usec;
    packet.avg_cno = avg_cno;
    packet.use_feature = use_feature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_METADATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_METADATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_METADATA_MIN_LEN, MAVLINK_MSG_ID_GPS_METADATA_LEN, MAVLINK_MSG_ID_GPS_METADATA_CRC);
}

/**
 * @brief Encode a gps_metadata struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_metadata C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_metadata_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_metadata_t* gps_metadata)
{
    return mavlink_msg_gps_metadata_pack(system_id, component_id, msg, gps_metadata->time_usec, gps_metadata->avg_cno, gps_metadata->epoch_time_usec, gps_metadata->use_feature);
}

/**
 * @brief Encode a gps_metadata struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_metadata C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_metadata_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_metadata_t* gps_metadata)
{
    return mavlink_msg_gps_metadata_pack_chan(system_id, component_id, chan, msg, gps_metadata->time_usec, gps_metadata->avg_cno, gps_metadata->epoch_time_usec, gps_metadata->use_feature);
}

/**
 * @brief Send a gps_metadata message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param avg_cno [dB] Avg CN0 of overhead only satellites.
 * @param epoch_time_usec [us] GPS Epoch time
 * @param use_feature [bitmask] User feature override
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_metadata_send(mavlink_channel_t chan, uint64_t time_usec, int16_t avg_cno, uint64_t epoch_time_usec, uint8_t use_feature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_METADATA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, epoch_time_usec);
    _mav_put_int16_t(buf, 16, avg_cno);
    _mav_put_uint8_t(buf, 18, use_feature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_METADATA, buf, MAVLINK_MSG_ID_GPS_METADATA_MIN_LEN, MAVLINK_MSG_ID_GPS_METADATA_LEN, MAVLINK_MSG_ID_GPS_METADATA_CRC);
#else
    mavlink_gps_metadata_t packet;
    packet.time_usec = time_usec;
    packet.epoch_time_usec = epoch_time_usec;
    packet.avg_cno = avg_cno;
    packet.use_feature = use_feature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_METADATA, (const char *)&packet, MAVLINK_MSG_ID_GPS_METADATA_MIN_LEN, MAVLINK_MSG_ID_GPS_METADATA_LEN, MAVLINK_MSG_ID_GPS_METADATA_CRC);
#endif
}

/**
 * @brief Send a gps_metadata message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_metadata_send_struct(mavlink_channel_t chan, const mavlink_gps_metadata_t* gps_metadata)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_metadata_send(chan, gps_metadata->time_usec, gps_metadata->avg_cno, gps_metadata->epoch_time_usec, gps_metadata->use_feature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_METADATA, (const char *)gps_metadata, MAVLINK_MSG_ID_GPS_METADATA_MIN_LEN, MAVLINK_MSG_ID_GPS_METADATA_LEN, MAVLINK_MSG_ID_GPS_METADATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_METADATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_metadata_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, int16_t avg_cno, uint64_t epoch_time_usec, uint8_t use_feature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, epoch_time_usec);
    _mav_put_int16_t(buf, 16, avg_cno);
    _mav_put_uint8_t(buf, 18, use_feature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_METADATA, buf, MAVLINK_MSG_ID_GPS_METADATA_MIN_LEN, MAVLINK_MSG_ID_GPS_METADATA_LEN, MAVLINK_MSG_ID_GPS_METADATA_CRC);
#else
    mavlink_gps_metadata_t *packet = (mavlink_gps_metadata_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->epoch_time_usec = epoch_time_usec;
    packet->avg_cno = avg_cno;
    packet->use_feature = use_feature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_METADATA, (const char *)packet, MAVLINK_MSG_ID_GPS_METADATA_MIN_LEN, MAVLINK_MSG_ID_GPS_METADATA_LEN, MAVLINK_MSG_ID_GPS_METADATA_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_METADATA UNPACKING


/**
 * @brief Get field time_usec from gps_metadata message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 */
static inline uint64_t mavlink_msg_gps_metadata_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field avg_cno from gps_metadata message
 *
 * @return [dB] Avg CN0 of overhead only satellites.
 */
static inline int16_t mavlink_msg_gps_metadata_get_avg_cno(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field epoch_time_usec from gps_metadata message
 *
 * @return [us] GPS Epoch time
 */
static inline uint64_t mavlink_msg_gps_metadata_get_epoch_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field use_feature from gps_metadata message
 *
 * @return [bitmask] User feature override
 */
static inline uint8_t mavlink_msg_gps_metadata_get_use_feature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Decode a gps_metadata message into a struct
 *
 * @param msg The message to decode
 * @param gps_metadata C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_metadata_decode(const mavlink_message_t* msg, mavlink_gps_metadata_t* gps_metadata)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_metadata->time_usec = mavlink_msg_gps_metadata_get_time_usec(msg);
    gps_metadata->epoch_time_usec = mavlink_msg_gps_metadata_get_epoch_time_usec(msg);
    gps_metadata->avg_cno = mavlink_msg_gps_metadata_get_avg_cno(msg);
    gps_metadata->use_feature = mavlink_msg_gps_metadata_get_use_feature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_METADATA_LEN? msg->len : MAVLINK_MSG_ID_GPS_METADATA_LEN;
        memset(gps_metadata, 0, MAVLINK_MSG_ID_GPS_METADATA_LEN);
    memcpy(gps_metadata, _MAV_PAYLOAD(msg), len);
#endif
}
