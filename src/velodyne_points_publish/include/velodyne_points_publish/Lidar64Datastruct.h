#ifndef LIDAR64DATASTRUCT_H
#define LIDAR64DATASTRUCT_H

#include <sys/types.h>

#define HDL64_PACKET_SIZE 1206      // 32-laser lidar
#define HDL64_PACKET_NUM  350
#define HDL64_BUFFER_SIZE (HDL64_PACKET_NUM*HDL64_PACKET_SIZE)

static const int HDL64_NUM_LASERS = 32;                   // The number of lasers per shot
static const int HDL64_NUM_SHOTS = 12;                    // The number of shots per packet
static const u_int16_t HDL64_SHOT_UPPER_FLAG = 0xeeff;    // The byte indicating a shot begins
static const u_int16_t HDL64_SHOT_LOWER_FLAG = 0xddff;    // The byte indicating a shot begins
static const int HDL64_BEAM_NUM = 64;
static const int HDL64_BEAM_POINTSIZE = HDL64_PACKET_NUM*HDL64_NUM_SHOTS/2;


static const int HDL64_VALID_RADIUS = 350;             // in cm

// Velodyne datastructures
// one laser, 3 Bytes
typedef struct hdl_laser {
  u_int16_t distance;                   // 0-65536*2mm
  u_int8_t intensity;              // 0-255, the greater, the brighter
} __attribute__((packed)) hdl_laser_t;

// one shot, 100 Bytes
typedef struct hdl_shot {
  u_int16_t lower_upper;
  u_int16_t rotational_angle;
  hdl_laser_t lasers[HDL64_NUM_LASERS];
} __attribute__((packed)) hdl_shot_t;

// one packet 1206 Bytes
typedef struct hdl_packet {
    hdl_shot_t shots[HDL64_NUM_SHOTS];
    u_int8_t GPS_time_stamp[4];
    u_int8_t Factory[2];
}  __attribute__((packed)) hdl_packet_t;

#endif // LIDAR64DATASTRUCT_H
