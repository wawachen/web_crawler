/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include "ros/ros.h"

namespace tsc_acs {

inline void changeByteOrder(const uint8_t *src, uint8_t *dest, int size)
{
    for(int i=0; i<size; i++)
        dest[size-i-1] = src[i];
}
template<typename T> inline void toBigEndian(T src, uint8_t * dest)
{
    changeByteOrder((uint8_t*)&src, dest, sizeof(T));
}
template<typename T> inline T fromBigEndian(const uint8_t * src)
{
    T t;
    changeByteOrder(src, (uint8_t*)&t, sizeof(T));
    return t;
}
inline ros::Time timeFromBigEndianNTP64(const uint8_t * data)
{
    // Convert fraction seconds (2^32ths) to ns.
    uint32_t ns = (uint32_t)((((uint64_t)fromBigEndian<uint32_t>(&data[4])) * 1000000000) / 0x100000000);
    return ros::Time(fromBigEndian<uint32_t>(data), ns);
}

inline ros::Duration durationFromBigEndian_us(const uint8_t * data)
{
    uint32_t us = fromBigEndian<uint32_t>(data);
    return ros::Duration(us/1000000, (us%1000000)*1000);
}

// This function included as a separate method as it is not obvious that
// big endian floats are simple byte order shifts.
inline float floatFromBigEndian(const uint8_t * data)
{
    return fromBigEndian<float>(data);
}

}
