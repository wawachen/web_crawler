/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

// This header should be included in one and only one source file.
// Implemented as a header to allow inclusion in Arduino ino or a cpp file.

#include "PPMDriver.h"

namespace PPM
{
  ReaderBase * ReaderBase::singleton; // The one and only Reader instance.
}

