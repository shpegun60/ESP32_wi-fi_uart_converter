#include "convert.h"

static constexpr unsigned char typeLengthMappingArray[TYPE_ARRAY_LENGTH] = {
    //0     1     2     3     4     5     6	   7      8     9	 10    11    12    13    14    15
    0x00, 0x01, 0x02, 0x04, 0x08, 0x01, 0x01, 0x02, 0x04, 0x08, 0x04, 0x08, 0x10, 0x01, 0x03, 0x03};

unsigned char Convert::getTypeLen(unsigned char type)
{
    if(type > (TYPE_ARRAY_LENGTH - ((unsigned char)1))) {
        return 0;
    }
    return typeLengthMappingArray[type];
}
