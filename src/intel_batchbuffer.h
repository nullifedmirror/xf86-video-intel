#ifndef INTEL_BATCHBUFFER_H
#define INTEL_BATCHBUFFER_H

#include "dri_bufmgr.h"

struct intel_context;

#define BATCH_SZ 16384
#define EXASTATE_SZ 48000
#define BATCH_RESERVED 16

struct intelddx_batchbuffer
{
   ScrnInfoPtr pScrn;

   dri_bo *buf;
   dri_fence *last_fence;
   uint32_t flags;

   unsigned char *map;
   unsigned char *ptr;

   uint32_t size;

   GLuint dirty_state;
};

struct intelddx_batchbuffer *intelddx_batchbuffer_alloc(ScrnInfoPtr pScrn);

void intelddx_batchbuffer_free(struct intelddx_batchbuffer *batch);


void intelddx_batchbuffer_finish(struct intelddx_batchbuffer *batch);

void intelddx_batchbuffer_flush(struct intelddx_batchbuffer *batch);

void intelddx_batchbuffer_reset(struct intelddx_batchbuffer *batch);


/* Unlike bmBufferData, this currently requires the buffer be mapped.
 * Consider it a convenience function wrapping multple
 * intel_buffer_dword() calls.
 */
void intelddx_batchbuffer_data(struct intelddx_batchbuffer *batch,
                            const void *data, uint32_t bytes, uint32_t flags);

void intelddx_batchbuffer_release_space(struct intelddx_batchbuffer *batch,
                                     uint32_t bytes);

Bool intelddx_batchbuffer_emit_reloc(struct intelddx_batchbuffer *batch,
				     dri_bo *buffer,
				     uint32_t flags, uint32_t offset);

/* Inline functions - might actually be better off with these
 * non-inlined.  Certainly better off switching all command packets to
 * be passed as structs rather than dwords, but that's a little bit of
 * work...
 */
static inline uint32_t
intelddx_batchbuffer_space(struct intelddx_batchbuffer *batch)
{
   return (batch->size - BATCH_RESERVED) - (batch->ptr - batch->map);
}


static inline void
intelddx_batchbuffer_emit_dword(struct intelddx_batchbuffer *batch, uint32_t dword)
{
   assert(batch->map);
   assert(intelddx_batchbuffer_space(batch) >= 4);
   *(uint32_t *) (batch->ptr) = dword;
   batch->ptr += 4;
}

static inline void
intelddx_batchbuffer_require_space(struct intelddx_batchbuffer *batch,
                                uint32_t sz, uint32_t flags)
{
   assert(sz < batch->size - 8);
   if (intelddx_batchbuffer_space(batch) < sz ||
       (batch->flags != 0 && flags != 0 && batch->flags != flags))
      intelddx_batchbuffer_flush(batch);

   batch->flags |= flags;
}

extern uint32_t intelddx_batchbuffer_emit_pixmap(PixmapPtr pPixmap,
					     unsigned int flags,
					     dri_bo *reloc_buf,
					     unsigned int offset,
					     unsigned int delta);

/* Here are the crusty old macros, to be removed:
 */
#define BATCH_LOCALS

#define BEGIN_BATCH(n)  						\
    RING_LOCALS 							\
    if (pI830->use_ttm_batch) {						\
	intelddx_batchbuffer_require_space(pI830->batch,		\
					   (((n) + 1) & ~1) * 4, 0);	\
    } else {								\
	DO_LP_RING((((n) + 1) & ~1));					\
    }									\
    if ((n) & 1) {							\
	OUT_BATCH(MI_NOOP);						\
    }



#define OUT_BATCH(d) \
	 if (pI830->use_ttm_batch) \
		intelddx_batchbuffer_emit_dword(pI830->batch, d); \
	 else { OUT_RING(d);  }

#define OUT_BATCH_F(x) do {                     \
        union intfloat tmp;                     \
        tmp.f = (float)(x);                     \
        OUT_BATCH(tmp.ui);                      \
} while(0)

#define OUT_RELOC(buf, flags, delta) do {	\
   intelddx_batchbuffer_emit_reloc(pI830->batch, buf, flags, delta);	\
} while (0)

#define OUT_PIXMAP_RELOC(pixmap, flags, delta) if (pI830->use_ttm_batch) { \
    uint32_t _retval = intelddx_batchbuffer_emit_pixmap((pixmap), (flags),		\
                                 pI830->batch->buf, (pI830->batch->ptr - pI830->batch->map), (delta)); \
    intelddx_batchbuffer_emit_dword (pI830->batch, _retval + (delta)); \
  } else {								\
    OUT_RING(intel_get_pixmap_offset(pixmap) + delta);			\
  }

#define ADVANCE_BATCH() if (!pI830->use_ttm_batch) { ADVANCE_LP_RING(); }


#endif
