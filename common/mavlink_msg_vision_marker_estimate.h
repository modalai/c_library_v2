#pragma once
// MESSAGE VISION_MARKER_ESTIMATE PACKING

#define MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE 97


typedef struct __mavlink_vision_marker_estimate_t {
 uint64_t usec; /*< [us] timestamp at the middle of the frame exposure in monotonic time*/
 int32_t id; /*<  id number of the tag*/
 float size_m; /*< [m] Tag size in meters*/
 float tx; /*< [m] x location of the tag with respect to camera frame in meters*/
 float ty; /*< [m] y location of the tag with respect to camera frame in meters*/
 float tz; /*< [m] z location of the tag with respect to camera frame in meters*/
 float R[9]; /*<  rotation matrix from tag frame to camera frame*/
 float float_reserved3; /*<  float reserved 1*/
 uint16_t int_reserved1; /*<  int reserved 1*/
 uint16_t int_reserved2; /*<  int reserved 2*/
} mavlink_vision_marker_estimate_t;

#define MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN 72
#define MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_MIN_LEN 72
#define MAVLINK_MSG_ID_97_LEN 72
#define MAVLINK_MSG_ID_97_MIN_LEN 72

#define MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_CRC 90
#define MAVLINK_MSG_ID_97_CRC 90

#define MAVLINK_MSG_VISION_MARKER_ESTIMATE_FIELD_R_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_MARKER_ESTIMATE { \
    97, \
    "VISION_MARKER_ESTIMATE", \
    10, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_marker_estimate_t, usec) }, \
         { "id", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_vision_marker_estimate_t, id) }, \
         { "size_m", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_marker_estimate_t, size_m) }, \
         { "tx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_marker_estimate_t, tx) }, \
         { "ty", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_vision_marker_estimate_t, ty) }, \
         { "tz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_vision_marker_estimate_t, tz) }, \
         { "R", NULL, MAVLINK_TYPE_FLOAT, 9, 28, offsetof(mavlink_vision_marker_estimate_t, R) }, \
         { "int_reserved1", NULL, MAVLINK_TYPE_UINT16_T, 0, 68, offsetof(mavlink_vision_marker_estimate_t, int_reserved1) }, \
         { "int_reserved2", NULL, MAVLINK_TYPE_UINT16_T, 0, 70, offsetof(mavlink_vision_marker_estimate_t, int_reserved2) }, \
         { "float_reserved3", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_vision_marker_estimate_t, float_reserved3) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_MARKER_ESTIMATE { \
    "VISION_MARKER_ESTIMATE", \
    10, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_marker_estimate_t, usec) }, \
         { "id", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_vision_marker_estimate_t, id) }, \
         { "size_m", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_marker_estimate_t, size_m) }, \
         { "tx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_marker_estimate_t, tx) }, \
         { "ty", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_vision_marker_estimate_t, ty) }, \
         { "tz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_vision_marker_estimate_t, tz) }, \
         { "R", NULL, MAVLINK_TYPE_FLOAT, 9, 28, offsetof(mavlink_vision_marker_estimate_t, R) }, \
         { "int_reserved1", NULL, MAVLINK_TYPE_UINT16_T, 0, 68, offsetof(mavlink_vision_marker_estimate_t, int_reserved1) }, \
         { "int_reserved2", NULL, MAVLINK_TYPE_UINT16_T, 0, 70, offsetof(mavlink_vision_marker_estimate_t, int_reserved2) }, \
         { "float_reserved3", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_vision_marker_estimate_t, float_reserved3) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_marker_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec [us] timestamp at the middle of the frame exposure in monotonic time
 * @param id  id number of the tag
 * @param size_m [m] Tag size in meters
 * @param tx [m] x location of the tag with respect to camera frame in meters
 * @param ty [m] y location of the tag with respect to camera frame in meters
 * @param tz [m] z location of the tag with respect to camera frame in meters
 * @param R  rotation matrix from tag frame to camera frame
 * @param int_reserved1  int reserved 1
 * @param int_reserved2  int reserved 2
 * @param float_reserved3  float reserved 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_marker_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t usec, int32_t id, float size_m, float tx, float ty, float tz, const float *R, uint16_t int_reserved1, uint16_t int_reserved2, float float_reserved3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_int32_t(buf, 8, id);
    _mav_put_float(buf, 12, size_m);
    _mav_put_float(buf, 16, tx);
    _mav_put_float(buf, 20, ty);
    _mav_put_float(buf, 24, tz);
    _mav_put_float(buf, 64, float_reserved3);
    _mav_put_uint16_t(buf, 68, int_reserved1);
    _mav_put_uint16_t(buf, 70, int_reserved2);
    _mav_put_float_array(buf, 28, R, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN);
#else
    mavlink_vision_marker_estimate_t packet;
    packet.usec = usec;
    packet.id = id;
    packet.size_m = size_m;
    packet.tx = tx;
    packet.ty = ty;
    packet.tz = tz;
    packet.float_reserved3 = float_reserved3;
    packet.int_reserved1 = int_reserved1;
    packet.int_reserved2 = int_reserved2;
    mav_array_memcpy(packet.R, R, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_CRC);
}

/**
 * @brief Pack a vision_marker_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec [us] timestamp at the middle of the frame exposure in monotonic time
 * @param id  id number of the tag
 * @param size_m [m] Tag size in meters
 * @param tx [m] x location of the tag with respect to camera frame in meters
 * @param ty [m] y location of the tag with respect to camera frame in meters
 * @param tz [m] z location of the tag with respect to camera frame in meters
 * @param R  rotation matrix from tag frame to camera frame
 * @param int_reserved1  int reserved 1
 * @param int_reserved2  int reserved 2
 * @param float_reserved3  float reserved 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_marker_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t usec,int32_t id,float size_m,float tx,float ty,float tz,const float *R,uint16_t int_reserved1,uint16_t int_reserved2,float float_reserved3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_int32_t(buf, 8, id);
    _mav_put_float(buf, 12, size_m);
    _mav_put_float(buf, 16, tx);
    _mav_put_float(buf, 20, ty);
    _mav_put_float(buf, 24, tz);
    _mav_put_float(buf, 64, float_reserved3);
    _mav_put_uint16_t(buf, 68, int_reserved1);
    _mav_put_uint16_t(buf, 70, int_reserved2);
    _mav_put_float_array(buf, 28, R, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN);
#else
    mavlink_vision_marker_estimate_t packet;
    packet.usec = usec;
    packet.id = id;
    packet.size_m = size_m;
    packet.tx = tx;
    packet.ty = ty;
    packet.tz = tz;
    packet.float_reserved3 = float_reserved3;
    packet.int_reserved1 = int_reserved1;
    packet.int_reserved2 = int_reserved2;
    mav_array_memcpy(packet.R, R, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_CRC);
}

/**
 * @brief Encode a vision_marker_estimate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_marker_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_marker_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_marker_estimate_t* vision_marker_estimate)
{
    return mavlink_msg_vision_marker_estimate_pack(system_id, component_id, msg, vision_marker_estimate->usec, vision_marker_estimate->id, vision_marker_estimate->size_m, vision_marker_estimate->tx, vision_marker_estimate->ty, vision_marker_estimate->tz, vision_marker_estimate->R, vision_marker_estimate->int_reserved1, vision_marker_estimate->int_reserved2, vision_marker_estimate->float_reserved3);
}

/**
 * @brief Encode a vision_marker_estimate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_marker_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_marker_estimate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_marker_estimate_t* vision_marker_estimate)
{
    return mavlink_msg_vision_marker_estimate_pack_chan(system_id, component_id, chan, msg, vision_marker_estimate->usec, vision_marker_estimate->id, vision_marker_estimate->size_m, vision_marker_estimate->tx, vision_marker_estimate->ty, vision_marker_estimate->tz, vision_marker_estimate->R, vision_marker_estimate->int_reserved1, vision_marker_estimate->int_reserved2, vision_marker_estimate->float_reserved3);
}

/**
 * @brief Send a vision_marker_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param usec [us] timestamp at the middle of the frame exposure in monotonic time
 * @param id  id number of the tag
 * @param size_m [m] Tag size in meters
 * @param tx [m] x location of the tag with respect to camera frame in meters
 * @param ty [m] y location of the tag with respect to camera frame in meters
 * @param tz [m] z location of the tag with respect to camera frame in meters
 * @param R  rotation matrix from tag frame to camera frame
 * @param int_reserved1  int reserved 1
 * @param int_reserved2  int reserved 2
 * @param float_reserved3  float reserved 1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_marker_estimate_send(mavlink_channel_t chan, uint64_t usec, int32_t id, float size_m, float tx, float ty, float tz, const float *R, uint16_t int_reserved1, uint16_t int_reserved2, float float_reserved3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_int32_t(buf, 8, id);
    _mav_put_float(buf, 12, size_m);
    _mav_put_float(buf, 16, tx);
    _mav_put_float(buf, 20, ty);
    _mav_put_float(buf, 24, tz);
    _mav_put_float(buf, 64, float_reserved3);
    _mav_put_uint16_t(buf, 68, int_reserved1);
    _mav_put_uint16_t(buf, 70, int_reserved2);
    _mav_put_float_array(buf, 28, R, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_CRC);
#else
    mavlink_vision_marker_estimate_t packet;
    packet.usec = usec;
    packet.id = id;
    packet.size_m = size_m;
    packet.tx = tx;
    packet.ty = ty;
    packet.tz = tz;
    packet.float_reserved3 = float_reserved3;
    packet.int_reserved1 = int_reserved1;
    packet.int_reserved2 = int_reserved2;
    mav_array_memcpy(packet.R, R, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE, (const char *)&packet, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_CRC);
#endif
}

/**
 * @brief Send a vision_marker_estimate message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_marker_estimate_send_struct(mavlink_channel_t chan, const mavlink_vision_marker_estimate_t* vision_marker_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_marker_estimate_send(chan, vision_marker_estimate->usec, vision_marker_estimate->id, vision_marker_estimate->size_m, vision_marker_estimate->tx, vision_marker_estimate->ty, vision_marker_estimate->tz, vision_marker_estimate->R, vision_marker_estimate->int_reserved1, vision_marker_estimate->int_reserved2, vision_marker_estimate->float_reserved3);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE, (const char *)vision_marker_estimate, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_marker_estimate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t usec, int32_t id, float size_m, float tx, float ty, float tz, const float *R, uint16_t int_reserved1, uint16_t int_reserved2, float float_reserved3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_int32_t(buf, 8, id);
    _mav_put_float(buf, 12, size_m);
    _mav_put_float(buf, 16, tx);
    _mav_put_float(buf, 20, ty);
    _mav_put_float(buf, 24, tz);
    _mav_put_float(buf, 64, float_reserved3);
    _mav_put_uint16_t(buf, 68, int_reserved1);
    _mav_put_uint16_t(buf, 70, int_reserved2);
    _mav_put_float_array(buf, 28, R, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_CRC);
#else
    mavlink_vision_marker_estimate_t *packet = (mavlink_vision_marker_estimate_t *)msgbuf;
    packet->usec = usec;
    packet->id = id;
    packet->size_m = size_m;
    packet->tx = tx;
    packet->ty = ty;
    packet->tz = tz;
    packet->float_reserved3 = float_reserved3;
    packet->int_reserved1 = int_reserved1;
    packet->int_reserved2 = int_reserved2;
    mav_array_memcpy(packet->R, R, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE, (const char *)packet, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_MARKER_ESTIMATE UNPACKING


/**
 * @brief Get field usec from vision_marker_estimate message
 *
 * @return [us] timestamp at the middle of the frame exposure in monotonic time
 */
static inline uint64_t mavlink_msg_vision_marker_estimate_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field id from vision_marker_estimate message
 *
 * @return  id number of the tag
 */
static inline int32_t mavlink_msg_vision_marker_estimate_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field size_m from vision_marker_estimate message
 *
 * @return [m] Tag size in meters
 */
static inline float mavlink_msg_vision_marker_estimate_get_size_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field tx from vision_marker_estimate message
 *
 * @return [m] x location of the tag with respect to camera frame in meters
 */
static inline float mavlink_msg_vision_marker_estimate_get_tx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ty from vision_marker_estimate message
 *
 * @return [m] y location of the tag with respect to camera frame in meters
 */
static inline float mavlink_msg_vision_marker_estimate_get_ty(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field tz from vision_marker_estimate message
 *
 * @return [m] z location of the tag with respect to camera frame in meters
 */
static inline float mavlink_msg_vision_marker_estimate_get_tz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field R from vision_marker_estimate message
 *
 * @return  rotation matrix from tag frame to camera frame
 */
static inline uint16_t mavlink_msg_vision_marker_estimate_get_R(const mavlink_message_t* msg, float *R)
{
    return _MAV_RETURN_float_array(msg, R, 9,  28);
}

/**
 * @brief Get field int_reserved1 from vision_marker_estimate message
 *
 * @return  int reserved 1
 */
static inline uint16_t mavlink_msg_vision_marker_estimate_get_int_reserved1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  68);
}

/**
 * @brief Get field int_reserved2 from vision_marker_estimate message
 *
 * @return  int reserved 2
 */
static inline uint16_t mavlink_msg_vision_marker_estimate_get_int_reserved2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  70);
}

/**
 * @brief Get field float_reserved3 from vision_marker_estimate message
 *
 * @return  float reserved 1
 */
static inline float mavlink_msg_vision_marker_estimate_get_float_reserved3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Decode a vision_marker_estimate message into a struct
 *
 * @param msg The message to decode
 * @param vision_marker_estimate C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_marker_estimate_decode(const mavlink_message_t* msg, mavlink_vision_marker_estimate_t* vision_marker_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vision_marker_estimate->usec = mavlink_msg_vision_marker_estimate_get_usec(msg);
    vision_marker_estimate->id = mavlink_msg_vision_marker_estimate_get_id(msg);
    vision_marker_estimate->size_m = mavlink_msg_vision_marker_estimate_get_size_m(msg);
    vision_marker_estimate->tx = mavlink_msg_vision_marker_estimate_get_tx(msg);
    vision_marker_estimate->ty = mavlink_msg_vision_marker_estimate_get_ty(msg);
    vision_marker_estimate->tz = mavlink_msg_vision_marker_estimate_get_tz(msg);
    mavlink_msg_vision_marker_estimate_get_R(msg, vision_marker_estimate->R);
    vision_marker_estimate->float_reserved3 = mavlink_msg_vision_marker_estimate_get_float_reserved3(msg);
    vision_marker_estimate->int_reserved1 = mavlink_msg_vision_marker_estimate_get_int_reserved1(msg);
    vision_marker_estimate->int_reserved2 = mavlink_msg_vision_marker_estimate_get_int_reserved2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN? msg->len : MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN;
        memset(vision_marker_estimate, 0, MAVLINK_MSG_ID_VISION_MARKER_ESTIMATE_LEN);
    memcpy(vision_marker_estimate, _MAV_PAYLOAD(msg), len);
#endif
}
