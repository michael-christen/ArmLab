#include <stdio.h>
//#include <inttypes.h>
//#include <stdint.h>
//#include "common/zhash.h"
//#include <stdlib.h>
#include <stddef.h>
#include <endian.h>

int main()
{

    size_t foo = 0xffaa2211;

    int bar = be32toh(foo);

    printf("0x%x -> 0x%x\n", foo, bar);

}
