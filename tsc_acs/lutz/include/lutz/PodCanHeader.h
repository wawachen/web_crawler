/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

namespace tsc_acs {
namespace lutz {


typedef struct
{
    uint8_t frameCount;
    uint8_t sessionId;
} __attribute__((packed)) _header;

}
}
