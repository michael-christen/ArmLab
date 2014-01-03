#ifndef __VX_OBJECT_
#define __VX_OBJECT_

#include <assert.h>

#include "vx_types.h"
#include "vx_code_output_stream.h"
#include "common/zhash.h"

#ifdef __cplusplus
extern "C" {
#endif

struct vx_object
{
    void (*append)(vx_object_t * obj, zhash_t * resources, vx_code_output_stream_t * output);
    void * impl;

    // Reference counting:
    uint32_t refcnt; // how many places have a reference to this?
    void (*destroy)(vx_object_t * vo); // Destroy this object, and release all resources.
};

// Note: It is illegal to create a cycle of references with vx_objects. Not only will this
// break the reference counting, it would also result in infinitely long rendering codes

// Note: Subclasses must be sure to properly initialize the reference count to zero, e.g.:
//   vx_object_t * vo = calloc(1,sizeof(vx_object_t));


static inline void vx_object_dec_destroy(vx_object_t * obj)
{
    assert(obj->refcnt > 0);

    uint32_t has_ref = __sync_sub_and_fetch(&obj->refcnt, 1);

    if (!has_ref)
        obj->destroy(obj);

    /* assert(obj->rcts > 0); */
    /* obj->rcts--; */
    /* if (obj->rcts == 0) { */
    /*     // Implementer must guarantee to release all resources, including */
    /*     // decrementing any reference counts it may have been holding */
    /*     obj->destroy(obj); */
    /* } */
}

static inline void vx_object_inc_ref(vx_object_t * obj)
{
    __sync_add_and_fetch(&obj->refcnt, 1);
}

#ifdef __cplusplus
}
#endif

#endif
