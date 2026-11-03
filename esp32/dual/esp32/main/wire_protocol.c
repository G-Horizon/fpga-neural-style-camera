#include "wire_protocol.h"
#include "crc32_table.h"
uint32_t d2_crc32(const uint8_t *p,unsigned n){uint32_t c=UINT32_MAX;while(n--)c=(c>>8)^d2_crc32_table[(c^*p++)&255];return c^UINT32_MAX;}
