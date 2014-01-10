#ifndef _PNM_H
#define _PNM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pnm pnm_t;
struct pnm
{
    int width, height;
    int format;

    uint32_t buflen;
    uint8_t *buf;
};

pnm_t *pnm_create_from_file(const char *path);
void pnm_destroy(pnm_t *pnm);

#ifdef __cplusplus
}
#endif

#endif


