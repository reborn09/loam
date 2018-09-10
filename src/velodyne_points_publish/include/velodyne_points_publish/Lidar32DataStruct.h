#ifndef LIDAR32DATASTRUCT_H
#define LIDAR32DATASTRUCT_H

#include <sys/types.h>

#define HDL32_PACKET_SIZE 1206      // 32-laser lidar
#define HDL32_PACKET_NUM  200
#define HDL32_BUFFER_SIZE (HDL32_PACKET_NUM*HDL32_PACKET_SIZE)

static const int HDL32_NUM_LASERS = 32;                   // The number of lasers per shot
static const int HDL32_NUM_SHOTS = 12;                    // The number of shots per packet
static const u_int16_t HDL32_SHOT_START_FLAG = 0xffee;    // The byte indicating a shot begins
static const int HDL32_BEAM_NUM = 32;
static const int HDL32_BEAM_POINTSIZE = HDL32_PACKET_NUM*HDL32_NUM_SHOTS;


static const int HDL32_VALID_RADIUS = 5000/2;             // in 2 mm

//// Velodyne datastructures
//// one laser, 3 Bytes
//typedef struct hdl_laser {
//  u_int16_t distance;                   // 0-65536*2mm
//  unsigned char intensity;              // 0-255, the greater, the brighter
//} __attribute__((packed)) hdl_laser_t;

//// one shot, 100 Bytes
//typedef struct hdl_shot {
//  u_int16_t lower_upper;
//  u_int16_t rotational_angle;
//  hdl_laser_t lasers[HDL32_NUM_LASERS];
//} __attribute__((packed)) hdl_shot_t;

//// one packet 1206 Bytes
//typedef struct hdl_packet {
//    hdl_shot_t shots[HDL32_NUM_SHOTS];
//    u_int8_t GPS_time_stamp[4];
//    u_int8_t Factory[2];
//}  __attribute__((packed)) hdl_packet_t;

#endif // LIDAR32DATASTRUCT_H
