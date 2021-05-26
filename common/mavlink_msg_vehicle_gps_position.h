#pragma once
// MESSAGE VEHICLE_GPS_POSITION PACKING

#define MAVLINK_MSG_ID_VEHICLE_GPS_POSITION 96


typedef struct __mavlink_vehicle_gps_position_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot).*/
 uint64_t utc_time_usec; /*< [us] utc time from gps module, from epoch*/
 uint64_t time_rel_usec; /*< [us] relative time*/
 int32_t lat; /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
 int32_t lon; /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/
 int32_t alt; /*< [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.*/
 float speed_accuracy; /*< [m/s] GPS speed accuracy*/
 float cog_accuracy; /*< [m/s] GPS course accuracy*/
 float eph; /*<  GPS horizontal position accuracy*/
 float epv; /*<  GPS vertical position accuracy*/
 float hdop; /*< [m] GPS HDOP horizontal dilution of position*/
 float vdop; /*< [m] GPS VDOP vertical dilution of position*/
 int32_t noise; /*< [ms] GPS noise per millisecond*/
 int32_t jam; /*< [ms] GPS noise per millisecond*/
 float vel; /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
 float vn; /*< [m/s] GPS velocity in NORTH direction in earth-fixed NED frame*/
 float ve; /*< [m/s] GPS velocity in EAST direction in earth-fixed NED frame*/
 float vd; /*< [m/s] GPS velocity in DOWN direction in earth-fixed NED frame*/
 float cog; /*< [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint8_t fix_type; /*<  GPS fix type.*/
 uint8_t satellites_visible; /*<  Number of satellites visible. If unknown, set to 255*/
 uint8_t use_feature; /*< [bitmask] User feature override*/
} mavlink_vehicle_gps_position_t;

#define MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN 91
#define MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_MIN_LEN 91
#define MAVLINK_MSG_ID_96_LEN 91
#define MAVLINK_MSG_ID_96_MIN_LEN 91

#define MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_CRC 77
#define MAVLINK_MSG_ID_96_CRC 77



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VEHICLE_GPS_POSITION { \
    96, \
    "VEHICLE_GPS_POSITION", \
    22, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vehicle_gps_position_t, time_usec) }, \
         { "utc_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_vehicle_gps_position_t, utc_time_usec) }, \
         { "time_rel_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_vehicle_gps_position_t, time_rel_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_vehicle_gps_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_vehicle_gps_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_vehicle_gps_position_t, alt) }, \
         { "speed_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_vehicle_gps_position_t, speed_accuracy) }, \
         { "cog_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_vehicle_gps_position_t, cog_accuracy) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 88, offsetof(mavlink_vehicle_gps_position_t, fix_type) }, \
         { "eph", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_vehicle_gps_position_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_vehicle_gps_position_t, epv) }, \
         { "hdop", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_vehicle_gps_position_t, hdop) }, \
         { "vdop", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_vehicle_gps_position_t, vdop) }, \
         { "noise", NULL, MAVLINK_TYPE_INT32_T, 0, 60, offsetof(mavlink_vehicle_gps_position_t, noise) }, \
         { "jam", NULL, MAVLINK_TYPE_INT32_T, 0, 64, offsetof(mavlink_vehicle_gps_position_t, jam) }, \
         { "vel", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_vehicle_gps_position_t, vel) }, \
         { "vn", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_vehicle_gps_position_t, vn) }, \
         { "ve", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_vehicle_gps_position_t, ve) }, \
         { "vd", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_vehicle_gps_position_t, vd) }, \
         { "cog", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_vehicle_gps_position_t, cog) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 89, offsetof(mavlink_vehicle_gps_position_t, satellites_visible) }, \
         { "use_feature", NULL, MAVLINK_TYPE_UINT8_T, 0, 90, offsetof(mavlink_vehicle_gps_position_t, use_feature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VEHICLE_GPS_POSITION { \
    "VEHICLE_GPS_POSITION", \
    22, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vehicle_gps_position_t, time_usec) }, \
         { "utc_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_vehicle_gps_position_t, utc_time_usec) }, \
         { "time_rel_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_vehicle_gps_position_t, time_rel_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_vehicle_gps_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_vehicle_gps_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_vehicle_gps_position_t, alt) }, \
         { "speed_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_vehicle_gps_position_t, speed_accuracy) }, \
         { "cog_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_vehicle_gps_position_t, cog_accuracy) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 88, offsetof(mavlink_vehicle_gps_position_t, fix_type) }, \
         { "eph", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_vehicle_gps_position_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_vehicle_gps_position_t, epv) }, \
         { "hdop", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_vehicle_gps_position_t, hdop) }, \
         { "vdop", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_vehicle_gps_position_t, vdop) }, \
         { "noise", NULL, MAVLINK_TYPE_INT32_T, 0, 60, offsetof(mavlink_vehicle_gps_position_t, noise) }, \
         { "jam", NULL, MAVLINK_TYPE_INT32_T, 0, 64, offsetof(mavlink_vehicle_gps_position_t, jam) }, \
         { "vel", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_vehicle_gps_position_t, vel) }, \
         { "vn", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_vehicle_gps_position_t, vn) }, \
         { "ve", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_vehicle_gps_position_t, ve) }, \
         { "vd", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_vehicle_gps_position_t, vd) }, \
         { "cog", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_vehicle_gps_position_t, cog) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 89, offsetof(mavlink_vehicle_gps_position_t, satellites_visible) }, \
         { "use_feature", NULL, MAVLINK_TYPE_UINT8_T, 0, 90, offsetof(mavlink_vehicle_gps_position_t, use_feature) }, \
         } \
}
#endif

/**
 * @brief Pack a vehicle_gps_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot).
 * @param utc_time_usec [us] utc time from gps module, from epoch
 * @param time_rel_usec [us] relative time
 * @param lat [degE7] Latitude (WGS84, EGM96 ellipsoid)
 * @param lon [degE7] Longitude (WGS84, EGM96 ellipsoid)
 * @param alt [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
 * @param speed_accuracy [m/s] GPS speed accuracy
 * @param cog_accuracy [m/s] GPS course accuracy
 * @param fix_type  GPS fix type.
 * @param eph  GPS horizontal position accuracy
 * @param epv  GPS vertical position accuracy
 * @param hdop [m] GPS HDOP horizontal dilution of position
 * @param vdop [m] GPS VDOP vertical dilution of position
 * @param noise [ms] GPS noise per millisecond
 * @param jam [ms] GPS noise per millisecond
 * @param vel [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param vn [m/s] GPS velocity in NORTH direction in earth-fixed NED frame
 * @param ve [m/s] GPS velocity in EAST direction in earth-fixed NED frame
 * @param vd [m/s] GPS velocity in DOWN direction in earth-fixed NED frame
 * @param cog [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param use_feature [bitmask] User feature override
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_gps_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint64_t utc_time_usec, uint64_t time_rel_usec, int32_t lat, int32_t lon, int32_t alt, float speed_accuracy, float cog_accuracy, uint8_t fix_type, float eph, float epv, float hdop, float vdop, int32_t noise, int32_t jam, float vel, float vn, float ve, float vd, float cog, uint8_t satellites_visible, uint8_t use_feature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, utc_time_usec);
    _mav_put_uint64_t(buf, 16, time_rel_usec);
    _mav_put_int32_t(buf, 24, lat);
    _mav_put_int32_t(buf, 28, lon);
    _mav_put_int32_t(buf, 32, alt);
    _mav_put_float(buf, 36, speed_accuracy);
    _mav_put_float(buf, 40, cog_accuracy);
    _mav_put_float(buf, 44, eph);
    _mav_put_float(buf, 48, epv);
    _mav_put_float(buf, 52, hdop);
    _mav_put_float(buf, 56, vdop);
    _mav_put_int32_t(buf, 60, noise);
    _mav_put_int32_t(buf, 64, jam);
    _mav_put_float(buf, 68, vel);
    _mav_put_float(buf, 72, vn);
    _mav_put_float(buf, 76, ve);
    _mav_put_float(buf, 80, vd);
    _mav_put_float(buf, 84, cog);
    _mav_put_uint8_t(buf, 88, fix_type);
    _mav_put_uint8_t(buf, 89, satellites_visible);
    _mav_put_uint8_t(buf, 90, use_feature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN);
#else
    mavlink_vehicle_gps_position_t packet;
    packet.time_usec = time_usec;
    packet.utc_time_usec = utc_time_usec;
    packet.time_rel_usec = time_rel_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.speed_accuracy = speed_accuracy;
    packet.cog_accuracy = cog_accuracy;
    packet.eph = eph;
    packet.epv = epv;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.noise = noise;
    packet.jam = jam;
    packet.vel = vel;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.use_feature = use_feature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VEHICLE_GPS_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_CRC);
}

/**
 * @brief Pack a vehicle_gps_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot).
 * @param utc_time_usec [us] utc time from gps module, from epoch
 * @param time_rel_usec [us] relative time
 * @param lat [degE7] Latitude (WGS84, EGM96 ellipsoid)
 * @param lon [degE7] Longitude (WGS84, EGM96 ellipsoid)
 * @param alt [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
 * @param speed_accuracy [m/s] GPS speed accuracy
 * @param cog_accuracy [m/s] GPS course accuracy
 * @param fix_type  GPS fix type.
 * @param eph  GPS horizontal position accuracy
 * @param epv  GPS vertical position accuracy
 * @param hdop [m] GPS HDOP horizontal dilution of position
 * @param vdop [m] GPS VDOP vertical dilution of position
 * @param noise [ms] GPS noise per millisecond
 * @param jam [ms] GPS noise per millisecond
 * @param vel [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param vn [m/s] GPS velocity in NORTH direction in earth-fixed NED frame
 * @param ve [m/s] GPS velocity in EAST direction in earth-fixed NED frame
 * @param vd [m/s] GPS velocity in DOWN direction in earth-fixed NED frame
 * @param cog [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param use_feature [bitmask] User feature override
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_gps_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint64_t utc_time_usec,uint64_t time_rel_usec,int32_t lat,int32_t lon,int32_t alt,float speed_accuracy,float cog_accuracy,uint8_t fix_type,float eph,float epv,float hdop,float vdop,int32_t noise,int32_t jam,float vel,float vn,float ve,float vd,float cog,uint8_t satellites_visible,uint8_t use_feature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, utc_time_usec);
    _mav_put_uint64_t(buf, 16, time_rel_usec);
    _mav_put_int32_t(buf, 24, lat);
    _mav_put_int32_t(buf, 28, lon);
    _mav_put_int32_t(buf, 32, alt);
    _mav_put_float(buf, 36, speed_accuracy);
    _mav_put_float(buf, 40, cog_accuracy);
    _mav_put_float(buf, 44, eph);
    _mav_put_float(buf, 48, epv);
    _mav_put_float(buf, 52, hdop);
    _mav_put_float(buf, 56, vdop);
    _mav_put_int32_t(buf, 60, noise);
    _mav_put_int32_t(buf, 64, jam);
    _mav_put_float(buf, 68, vel);
    _mav_put_float(buf, 72, vn);
    _mav_put_float(buf, 76, ve);
    _mav_put_float(buf, 80, vd);
    _mav_put_float(buf, 84, cog);
    _mav_put_uint8_t(buf, 88, fix_type);
    _mav_put_uint8_t(buf, 89, satellites_visible);
    _mav_put_uint8_t(buf, 90, use_feature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN);
#else
    mavlink_vehicle_gps_position_t packet;
    packet.time_usec = time_usec;
    packet.utc_time_usec = utc_time_usec;
    packet.time_rel_usec = time_rel_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.speed_accuracy = speed_accuracy;
    packet.cog_accuracy = cog_accuracy;
    packet.eph = eph;
    packet.epv = epv;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.noise = noise;
    packet.jam = jam;
    packet.vel = vel;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.use_feature = use_feature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VEHICLE_GPS_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_CRC);
}

/**
 * @brief Encode a vehicle_gps_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_gps_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_gps_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vehicle_gps_position_t* vehicle_gps_position)
{
    return mavlink_msg_vehicle_gps_position_pack(system_id, component_id, msg, vehicle_gps_position->time_usec, vehicle_gps_position->utc_time_usec, vehicle_gps_position->time_rel_usec, vehicle_gps_position->lat, vehicle_gps_position->lon, vehicle_gps_position->alt, vehicle_gps_position->speed_accuracy, vehicle_gps_position->cog_accuracy, vehicle_gps_position->fix_type, vehicle_gps_position->eph, vehicle_gps_position->epv, vehicle_gps_position->hdop, vehicle_gps_position->vdop, vehicle_gps_position->noise, vehicle_gps_position->jam, vehicle_gps_position->vel, vehicle_gps_position->vn, vehicle_gps_position->ve, vehicle_gps_position->vd, vehicle_gps_position->cog, vehicle_gps_position->satellites_visible, vehicle_gps_position->use_feature);
}

/**
 * @brief Encode a vehicle_gps_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_gps_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_gps_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vehicle_gps_position_t* vehicle_gps_position)
{
    return mavlink_msg_vehicle_gps_position_pack_chan(system_id, component_id, chan, msg, vehicle_gps_position->time_usec, vehicle_gps_position->utc_time_usec, vehicle_gps_position->time_rel_usec, vehicle_gps_position->lat, vehicle_gps_position->lon, vehicle_gps_position->alt, vehicle_gps_position->speed_accuracy, vehicle_gps_position->cog_accuracy, vehicle_gps_position->fix_type, vehicle_gps_position->eph, vehicle_gps_position->epv, vehicle_gps_position->hdop, vehicle_gps_position->vdop, vehicle_gps_position->noise, vehicle_gps_position->jam, vehicle_gps_position->vel, vehicle_gps_position->vn, vehicle_gps_position->ve, vehicle_gps_position->vd, vehicle_gps_position->cog, vehicle_gps_position->satellites_visible, vehicle_gps_position->use_feature);
}

/**
 * @brief Send a vehicle_gps_position message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot).
 * @param utc_time_usec [us] utc time from gps module, from epoch
 * @param time_rel_usec [us] relative time
 * @param lat [degE7] Latitude (WGS84, EGM96 ellipsoid)
 * @param lon [degE7] Longitude (WGS84, EGM96 ellipsoid)
 * @param alt [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
 * @param speed_accuracy [m/s] GPS speed accuracy
 * @param cog_accuracy [m/s] GPS course accuracy
 * @param fix_type  GPS fix type.
 * @param eph  GPS horizontal position accuracy
 * @param epv  GPS vertical position accuracy
 * @param hdop [m] GPS HDOP horizontal dilution of position
 * @param vdop [m] GPS VDOP vertical dilution of position
 * @param noise [ms] GPS noise per millisecond
 * @param jam [ms] GPS noise per millisecond
 * @param vel [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param vn [m/s] GPS velocity in NORTH direction in earth-fixed NED frame
 * @param ve [m/s] GPS velocity in EAST direction in earth-fixed NED frame
 * @param vd [m/s] GPS velocity in DOWN direction in earth-fixed NED frame
 * @param cog [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param use_feature [bitmask] User feature override
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vehicle_gps_position_send(mavlink_channel_t chan, uint64_t time_usec, uint64_t utc_time_usec, uint64_t time_rel_usec, int32_t lat, int32_t lon, int32_t alt, float speed_accuracy, float cog_accuracy, uint8_t fix_type, float eph, float epv, float hdop, float vdop, int32_t noise, int32_t jam, float vel, float vn, float ve, float vd, float cog, uint8_t satellites_visible, uint8_t use_feature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, utc_time_usec);
    _mav_put_uint64_t(buf, 16, time_rel_usec);
    _mav_put_int32_t(buf, 24, lat);
    _mav_put_int32_t(buf, 28, lon);
    _mav_put_int32_t(buf, 32, alt);
    _mav_put_float(buf, 36, speed_accuracy);
    _mav_put_float(buf, 40, cog_accuracy);
    _mav_put_float(buf, 44, eph);
    _mav_put_float(buf, 48, epv);
    _mav_put_float(buf, 52, hdop);
    _mav_put_float(buf, 56, vdop);
    _mav_put_int32_t(buf, 60, noise);
    _mav_put_int32_t(buf, 64, jam);
    _mav_put_float(buf, 68, vel);
    _mav_put_float(buf, 72, vn);
    _mav_put_float(buf, 76, ve);
    _mav_put_float(buf, 80, vd);
    _mav_put_float(buf, 84, cog);
    _mav_put_uint8_t(buf, 88, fix_type);
    _mav_put_uint8_t(buf, 89, satellites_visible);
    _mav_put_uint8_t(buf, 90, use_feature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION, buf, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_CRC);
#else
    mavlink_vehicle_gps_position_t packet;
    packet.time_usec = time_usec;
    packet.utc_time_usec = utc_time_usec;
    packet.time_rel_usec = time_rel_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.speed_accuracy = speed_accuracy;
    packet.cog_accuracy = cog_accuracy;
    packet.eph = eph;
    packet.epv = epv;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.noise = noise;
    packet.jam = jam;
    packet.vel = vel;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.use_feature = use_feature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION, (const char *)&packet, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_CRC);
#endif
}

/**
 * @brief Send a vehicle_gps_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vehicle_gps_position_send_struct(mavlink_channel_t chan, const mavlink_vehicle_gps_position_t* vehicle_gps_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vehicle_gps_position_send(chan, vehicle_gps_position->time_usec, vehicle_gps_position->utc_time_usec, vehicle_gps_position->time_rel_usec, vehicle_gps_position->lat, vehicle_gps_position->lon, vehicle_gps_position->alt, vehicle_gps_position->speed_accuracy, vehicle_gps_position->cog_accuracy, vehicle_gps_position->fix_type, vehicle_gps_position->eph, vehicle_gps_position->epv, vehicle_gps_position->hdop, vehicle_gps_position->vdop, vehicle_gps_position->noise, vehicle_gps_position->jam, vehicle_gps_position->vel, vehicle_gps_position->vn, vehicle_gps_position->ve, vehicle_gps_position->vd, vehicle_gps_position->cog, vehicle_gps_position->satellites_visible, vehicle_gps_position->use_feature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION, (const char *)vehicle_gps_position, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vehicle_gps_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint64_t utc_time_usec, uint64_t time_rel_usec, int32_t lat, int32_t lon, int32_t alt, float speed_accuracy, float cog_accuracy, uint8_t fix_type, float eph, float epv, float hdop, float vdop, int32_t noise, int32_t jam, float vel, float vn, float ve, float vd, float cog, uint8_t satellites_visible, uint8_t use_feature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, utc_time_usec);
    _mav_put_uint64_t(buf, 16, time_rel_usec);
    _mav_put_int32_t(buf, 24, lat);
    _mav_put_int32_t(buf, 28, lon);
    _mav_put_int32_t(buf, 32, alt);
    _mav_put_float(buf, 36, speed_accuracy);
    _mav_put_float(buf, 40, cog_accuracy);
    _mav_put_float(buf, 44, eph);
    _mav_put_float(buf, 48, epv);
    _mav_put_float(buf, 52, hdop);
    _mav_put_float(buf, 56, vdop);
    _mav_put_int32_t(buf, 60, noise);
    _mav_put_int32_t(buf, 64, jam);
    _mav_put_float(buf, 68, vel);
    _mav_put_float(buf, 72, vn);
    _mav_put_float(buf, 76, ve);
    _mav_put_float(buf, 80, vd);
    _mav_put_float(buf, 84, cog);
    _mav_put_uint8_t(buf, 88, fix_type);
    _mav_put_uint8_t(buf, 89, satellites_visible);
    _mav_put_uint8_t(buf, 90, use_feature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION, buf, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_CRC);
#else
    mavlink_vehicle_gps_position_t *packet = (mavlink_vehicle_gps_position_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->utc_time_usec = utc_time_usec;
    packet->time_rel_usec = time_rel_usec;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->speed_accuracy = speed_accuracy;
    packet->cog_accuracy = cog_accuracy;
    packet->eph = eph;
    packet->epv = epv;
    packet->hdop = hdop;
    packet->vdop = vdop;
    packet->noise = noise;
    packet->jam = jam;
    packet->vel = vel;
    packet->vn = vn;
    packet->ve = ve;
    packet->vd = vd;
    packet->cog = cog;
    packet->fix_type = fix_type;
    packet->satellites_visible = satellites_visible;
    packet->use_feature = use_feature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION, (const char *)packet, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE VEHICLE_GPS_POSITION UNPACKING


/**
 * @brief Get field time_usec from vehicle_gps_position message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot).
 */
static inline uint64_t mavlink_msg_vehicle_gps_position_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field utc_time_usec from vehicle_gps_position message
 *
 * @return [us] utc time from gps module, from epoch
 */
static inline uint64_t mavlink_msg_vehicle_gps_position_get_utc_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field time_rel_usec from vehicle_gps_position message
 *
 * @return [us] relative time
 */
static inline uint64_t mavlink_msg_vehicle_gps_position_get_time_rel_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field lat from vehicle_gps_position message
 *
 * @return [degE7] Latitude (WGS84, EGM96 ellipsoid)
 */
static inline int32_t mavlink_msg_vehicle_gps_position_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field lon from vehicle_gps_position message
 *
 * @return [degE7] Longitude (WGS84, EGM96 ellipsoid)
 */
static inline int32_t mavlink_msg_vehicle_gps_position_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field alt from vehicle_gps_position message
 *
 * @return [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
 */
static inline int32_t mavlink_msg_vehicle_gps_position_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field speed_accuracy from vehicle_gps_position message
 *
 * @return [m/s] GPS speed accuracy
 */
static inline float mavlink_msg_vehicle_gps_position_get_speed_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field cog_accuracy from vehicle_gps_position message
 *
 * @return [m/s] GPS course accuracy
 */
static inline float mavlink_msg_vehicle_gps_position_get_cog_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field fix_type from vehicle_gps_position message
 *
 * @return  GPS fix type.
 */
static inline uint8_t mavlink_msg_vehicle_gps_position_get_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  88);
}

/**
 * @brief Get field eph from vehicle_gps_position message
 *
 * @return  GPS horizontal position accuracy
 */
static inline float mavlink_msg_vehicle_gps_position_get_eph(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field epv from vehicle_gps_position message
 *
 * @return  GPS vertical position accuracy
 */
static inline float mavlink_msg_vehicle_gps_position_get_epv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field hdop from vehicle_gps_position message
 *
 * @return [m] GPS HDOP horizontal dilution of position
 */
static inline float mavlink_msg_vehicle_gps_position_get_hdop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field vdop from vehicle_gps_position message
 *
 * @return [m] GPS VDOP vertical dilution of position
 */
static inline float mavlink_msg_vehicle_gps_position_get_vdop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field noise from vehicle_gps_position message
 *
 * @return [ms] GPS noise per millisecond
 */
static inline int32_t mavlink_msg_vehicle_gps_position_get_noise(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  60);
}

/**
 * @brief Get field jam from vehicle_gps_position message
 *
 * @return [ms] GPS noise per millisecond
 */
static inline int32_t mavlink_msg_vehicle_gps_position_get_jam(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  64);
}

/**
 * @brief Get field vel from vehicle_gps_position message
 *
 * @return [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 */
static inline float mavlink_msg_vehicle_gps_position_get_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field vn from vehicle_gps_position message
 *
 * @return [m/s] GPS velocity in NORTH direction in earth-fixed NED frame
 */
static inline float mavlink_msg_vehicle_gps_position_get_vn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field ve from vehicle_gps_position message
 *
 * @return [m/s] GPS velocity in EAST direction in earth-fixed NED frame
 */
static inline float mavlink_msg_vehicle_gps_position_get_ve(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field vd from vehicle_gps_position message
 *
 * @return [m/s] GPS velocity in DOWN direction in earth-fixed NED frame
 */
static inline float mavlink_msg_vehicle_gps_position_get_vd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field cog from vehicle_gps_position message
 *
 * @return [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline float mavlink_msg_vehicle_gps_position_get_cog(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field satellites_visible from vehicle_gps_position message
 *
 * @return  Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_vehicle_gps_position_get_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  89);
}

/**
 * @brief Get field use_feature from vehicle_gps_position message
 *
 * @return [bitmask] User feature override
 */
static inline uint8_t mavlink_msg_vehicle_gps_position_get_use_feature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  90);
}

/**
 * @brief Decode a vehicle_gps_position message into a struct
 *
 * @param msg The message to decode
 * @param vehicle_gps_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_vehicle_gps_position_decode(const mavlink_message_t* msg, mavlink_vehicle_gps_position_t* vehicle_gps_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vehicle_gps_position->time_usec = mavlink_msg_vehicle_gps_position_get_time_usec(msg);
    vehicle_gps_position->utc_time_usec = mavlink_msg_vehicle_gps_position_get_utc_time_usec(msg);
    vehicle_gps_position->time_rel_usec = mavlink_msg_vehicle_gps_position_get_time_rel_usec(msg);
    vehicle_gps_position->lat = mavlink_msg_vehicle_gps_position_get_lat(msg);
    vehicle_gps_position->lon = mavlink_msg_vehicle_gps_position_get_lon(msg);
    vehicle_gps_position->alt = mavlink_msg_vehicle_gps_position_get_alt(msg);
    vehicle_gps_position->speed_accuracy = mavlink_msg_vehicle_gps_position_get_speed_accuracy(msg);
    vehicle_gps_position->cog_accuracy = mavlink_msg_vehicle_gps_position_get_cog_accuracy(msg);
    vehicle_gps_position->eph = mavlink_msg_vehicle_gps_position_get_eph(msg);
    vehicle_gps_position->epv = mavlink_msg_vehicle_gps_position_get_epv(msg);
    vehicle_gps_position->hdop = mavlink_msg_vehicle_gps_position_get_hdop(msg);
    vehicle_gps_position->vdop = mavlink_msg_vehicle_gps_position_get_vdop(msg);
    vehicle_gps_position->noise = mavlink_msg_vehicle_gps_position_get_noise(msg);
    vehicle_gps_position->jam = mavlink_msg_vehicle_gps_position_get_jam(msg);
    vehicle_gps_position->vel = mavlink_msg_vehicle_gps_position_get_vel(msg);
    vehicle_gps_position->vn = mavlink_msg_vehicle_gps_position_get_vn(msg);
    vehicle_gps_position->ve = mavlink_msg_vehicle_gps_position_get_ve(msg);
    vehicle_gps_position->vd = mavlink_msg_vehicle_gps_position_get_vd(msg);
    vehicle_gps_position->cog = mavlink_msg_vehicle_gps_position_get_cog(msg);
    vehicle_gps_position->fix_type = mavlink_msg_vehicle_gps_position_get_fix_type(msg);
    vehicle_gps_position->satellites_visible = mavlink_msg_vehicle_gps_position_get_satellites_visible(msg);
    vehicle_gps_position->use_feature = mavlink_msg_vehicle_gps_position_get_use_feature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN? msg->len : MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN;
        memset(vehicle_gps_position, 0, MAVLINK_MSG_ID_VEHICLE_GPS_POSITION_LEN);
    memcpy(vehicle_gps_position, _MAV_PAYLOAD(msg), len);
#endif
}
