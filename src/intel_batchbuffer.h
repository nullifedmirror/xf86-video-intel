#ifndef INTEL_BATCHBUFFER_H
#define INTEL_BATCHBUFFER_H

#include "dri_bufmgr.h"

struct intel_context;

#define BATCH_SZ 16384
#define BATCH_RESERVED 16

#define INTEL_BATCH_NO_CLIPRECTS 0x1
#define INTEL_BATCH_CLIPRECTS    0x2

struct intel_batchbuffer
{
   ScrnInfoPtr pScrn;

   dri_bo *buf;
   dri_fence *last_fence;
   uint32_t flags;

   unsigned char *map;
   unsigned char *ptr;

   uint32_t size;
};

struct intel_batchbuffer *intel_batchbuffer_alloc(ScrnInfoPtr pScrn);

void intel_batchbuffer_free(struct intel_batchbuffer *batch);


void intel_batchbuffer_finish(struct intel_batchbuffer *batch);

void intel_batchbuffer_flush(struct intel_batchbuffer *batch);

void intel_batchbuffer_reset(struct intel_batchbuffer *batch);


/* Unlike bmBufferData, this currently requires the buffer be mapped.
 * Consider it a convenience function wrapping multple
 * intel_buffer_dword() calls.
 */
void intel_batchbuffer_data(struct intel_batchbuffer *batch,
                            const void *data, uint32_t bytes, uint32_t flags);

void intel_batchbuffer_release_space(struct intel_batchbuffer *batch,
                                     uint32_t bytes);

Bool intel_batchbuffer_emit_reloc(struct intel_batchbuffer *batch,
                                       dri_bo *buffer,
                                       uint32_t flags, uint32_t offset);

/* Inline functions - might actually be better off with these
 * non-inlined.  Certainly better off switching all command packets to
 * be passed as structs rather than dwords, but that's a little bit of
 * work...
 */
static inline uint32_t
intel_batchbuffer_space(struct intel_batchbuffer *batch)
{
   return (batch->size - BATCH_RESERVED) - (batch->ptr - batch->map);
}


static inline void
intel_batchbuffer_emit_dword(struct intel_batchbuffer *batch, uint32_t dword)
{
   assert(batch->map);
   assert(intel_batchbuffer_space(batch) >= 4);
   *(uint32_t *) (batch->ptr) = dword;
   batch->ptr += 4;
}

static inline void
intel_batchbuffer_require_space(struct intel_batchbuffer *batch,
                                uint32_t sz, uint32_t flags)
{
   assert(sz < batch->size - 8);
   if (intel_batchbuffer_space(batch) < sz ||
       (batch->flags != 0 && flags != 0 && batch->flags != flags))
      intel_batchbuffer_flush(batch);

   batch->flags |= flags;
}

extern Bool i830_batchbuffer_emit_pixmap(PixmapPtr pPixmap, unsigned int flags,
					 unsigned int mask,
					 dri_bo *reloc_buf,
					 unsigned int offset,
					 unsigned int delta);

/* Here are the crusty old macros, to be removed:
 */
#define BATCH_LOCALS

#define BEGIN_BATCH(n)  							\
	RING_LOCALS 								\
	if (pI830->use_ttm_batch)						\
   		intel_batchbuffer_require_space(pI830->batch, (n)*4, 0);	\
	 else \
   DO_LP_RING(n) ;

#define OUT_BATCH(d) \
	 if (pI830->use_ttm_batch) \
		intel_batchbuffer_emit_dword(pI830->batch, d); \
	 else { OUT_RING(d);  }

#define OUT_BATCH_F(x) do {                     \
        union intfloat tmp;                     \
        tmp.f = (float)(x);                     \
        OUT_BATCH(tmp.ui);                      \
} while(0)

#define OUT_RELOC(buf, flags, delta) do { 				\
   intel_batchbuffer_emit_reloc(pI830->batch, buf, flags, delta);	\
} while (0)

#define OUT_PIXMAP_RELOC(pixmap, flags, mask, delta) if (pI830->use_ttm_batch) {               \
    i830_batchbuffer_emit_pixmap((pixmap), (flags), (mask),             \
                                 pI830->batch->buf, (pI830->batch->ptr - pI830->batch->map), (delta)); \
    pI830->batch->ptr += 4;                                                 \
    } else {					   \
	OUT_RING(intel_get_pixmap_offset(pixmap)); \
    }

#define ADVANCE_BATCH() if (!pI830->use_ttm_batch) { ADVANCE_LP_RING(); }


#endif
