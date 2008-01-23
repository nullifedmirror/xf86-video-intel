/**************************************************************************
 * 
 * Copyright 2006 Tungsten Graphics, Inc., Cedar Park, Texas.
 * All Rights Reserved.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 **************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <errno.h>
#include "xf86.h"

#include "i830.h"
#include "intel_batchbuffer.h"
#include "i915_drm.h"
#include "intel_bufmgr_ttm.h"

#define MI_BATCH_BUFFER_END                  (0x0A<<23)

/* Relocations in kernel space:
 *    - pass dma buffer seperately
 *    - memory manager knows how to patch
 *    - pass list of dependent buffers
 *    - pass relocation list
 *
 * Either:
 *    - get back an offset for buffer to fire
 *    - memory manager knows how to fire buffer
 *
 * Really want the buffer to be AGP and pinned.
 *
 */

/* Cliprect fence: The highest fence protecting a dma buffer
 * containing explicit cliprect information.  Like the old drawable
 * lock but irq-driven.  X server must wait for this fence to expire
 * before changing cliprects [and then doing sw rendering?].  For
 * other dma buffers, the scheduler will grab current cliprect info
 * and mix into buffer.  X server must hold the lock while changing
 * cliprects???  Make per-drawable.  Need cliprects in shared memory
 * -- beats storing them with every cmd buffer in the queue.
 *
 * ==> X server must wait for this fence to expire before touching the
 * framebuffer with new cliprects.
 *
 * ==> Cliprect-dependent buffers associated with a
 * cliprect-timestamp.  All of the buffers associated with a timestamp
 * must go to hardware before any buffer with a newer timestamp.
 *
 * ==> Dma should be queued per-drawable for correct X/GL
 * synchronization.  Or can fences be used for this?
 *
 * Applies to: Blit operations, metaops, X server operations -- X
 * server automatically waits on its own dma to complete before
 * modifying cliprects ???
 */

void
intelddx_batchbuffer_reset(struct intelddx_batchbuffer *batch)
{
   I830Ptr pI830 = I830PTR(batch->pScrn);

   if (batch->buf != NULL) {
      dri_bo_unreference(batch->buf);
      batch->buf = NULL;
   }

   batch->buf = dri_bo_alloc(pI830->bufmgr, "batchbuffer",
			     pI830->maxBatchSize, 4096,
			     DRM_BO_FLAG_MEM_LOCAL | DRM_BO_FLAG_CACHED | DRM_BO_FLAG_CACHED_MAPPED);
   dri_bo_map(batch->buf, TRUE);
   batch->map = batch->buf->virtual;
   batch->size = pI830->maxBatchSize;
   batch->ptr = batch->map;
}

struct intelddx_batchbuffer *
intelddx_batchbuffer_alloc(ScrnInfoPtr pScrn)
{
   struct intelddx_batchbuffer *batch = calloc(sizeof(*batch), 1);

   batch->pScrn = pScrn;
   batch->last_fence = NULL;
   intelddx_batchbuffer_reset(batch);

   return batch;
}

void
intelddx_batchbuffer_free(struct intelddx_batchbuffer *batch)
{
   if (batch->last_fence) {
      dri_fence_wait(batch->last_fence);
      dri_fence_unreference(batch->last_fence);
      batch->last_fence = NULL;
   }
   if (batch->map) {
      dri_bo_unmap(batch->buf);
      batch->map = NULL;
   }
   dri_bo_unreference(batch->buf);
   batch->buf = NULL;
   free(batch);
}

static void
intel_exec_ioctl(ScrnInfoPtr pScrn,
                 GLuint used,
                 GLboolean ignore_cliprects, GLboolean allow_unlock,
                 void *start, GLuint count, dri_fence **fence)
{
   I830Ptr pI830 = I830PTR(pScrn);
   struct drm_i915_execbuffer execbuf;
   dri_fence *fo;

   assert(used);

   if (*fence) {
     dri_fence_unreference(*fence);
   }
   memset(&execbuf, 0, sizeof(execbuf));

   execbuf.num_buffers = count;
   execbuf.batch.used = used;
   execbuf.batch.cliprects = NULL;
   execbuf.batch.num_cliprects = 0;
   execbuf.batch.DR1 = 0;
   execbuf.batch.DR4 = 0;

   execbuf.ops_list = (unsigned long)start;
   execbuf.fence_arg.flags = DRM_FENCE_FLAG_SHAREABLE | DRM_I915_FENCE_FLAG_FLUSHED;

   if (drmCommandWriteRead(pI830->drmSubFD, DRM_I915_EXECBUFFER, &execbuf,
                       sizeof(execbuf))) {
      fprintf(stderr, "DRM_I830_EXECBUFFER: %d\n", -errno);
      exit(1);
   }

   fo = intel_ttm_fence_create_from_arg(pI830->bufmgr, "fence buffers",
                                        &execbuf.fence_arg);
   if (!fo) {
      fprintf(stderr, "failed to fence handle: %08x\n", execbuf.fence_arg.handle);
      exit(1);
   }
   *fence = fo;
}


static void
do_flush_locked(struct intelddx_batchbuffer *batch,
		uint32_t used,
		Bool ignore_cliprects, Bool allow_unlock)
{
   void *start;
   uint32_t count;

   dri_bo_unmap(batch->buf);
   start = dri_process_relocs(batch->buf, &count);

   batch->map = NULL;
   batch->ptr = NULL;
   batch->flags = 0;

   /* Throw away non-effective packets.  Won't work once we have
    * hardware contexts which would preserve statechanges beyond a
    * single buffer.
    */

   intel_exec_ioctl(batch->pScrn,
			  used, ignore_cliprects, allow_unlock,
			  start, count, &batch->last_fence);
      
   dri_post_submit(batch->buf, &batch->last_fence);

   i830_refresh_ring(batch->pScrn);
}

void
intelddx_batchbuffer_flush(struct intelddx_batchbuffer *batch)
{
   I830Ptr pI830 = I830PTR(batch->pScrn);
   uint32_t used = batch->ptr - batch->map;
   uint32_t flags = MI_INVALIDATE_MAP_CACHE;

   if (used == 0)
      return;

   if (IS_I965G(pI830))
	flags = 0;
   /* Add the MI_BATCH_BUFFER_END.  Always add an MI_FLUSH - this is a
    * performance drain that we would like to avoid.
    */
   if (used & 4) {
      ((int *) batch->ptr)[0] = MI_FLUSH | flags;
      ((int *) batch->ptr)[1] = 0;
      ((int *) batch->ptr)[2] = MI_BATCH_BUFFER_END;
      used += 12;
   }
   else {
      ((int *) batch->ptr)[0] = MI_FLUSH | flags;
      ((int *) batch->ptr)[1] = MI_BATCH_BUFFER_END;
      used += 8;
   }
   batch->ptr = batch->map;

   do_flush_locked(batch, used, 0, FALSE);
     
   /* Reset the buffer:
    */
   intelddx_batchbuffer_reset(batch);
}

void
intelddx_batchbuffer_finish(struct intelddx_batchbuffer *batch)
{
   intelddx_batchbuffer_flush(batch);
   if (batch->last_fence != NULL)
      dri_fence_wait(batch->last_fence);
}


/*  This is the only way buffers get added to the validate list.
 */
Bool
intelddx_batchbuffer_emit_reloc(struct intelddx_batchbuffer *batch,
                             dri_bo *buffer,
                             uint32_t flags, uint32_t delta)
{
   dri_emit_reloc(batch->buf, flags, delta, batch->ptr - batch->map, buffer);
   intelddx_batchbuffer_emit_dword (batch, buffer->offset + delta);

   return TRUE;
}

void
intelddx_batchbuffer_data(struct intelddx_batchbuffer *batch,
                       const void *data, uint32_t bytes, uint32_t flags)
{
   assert((bytes & 3) == 0);
   intelddx_batchbuffer_require_space(batch, bytes, flags);
   memcpy(batch->ptr, data, bytes);
   batch->ptr += bytes;
}

uint32_t intelddx_batchbuffer_emit_pixmap(PixmapPtr pPixmap,
					  unsigned int flags,
					  dri_bo *reloc_buf,
					  unsigned int offset,
					  unsigned int delta)
{
    ScreenPtr pScreen = pPixmap->drawable.pScreen;
    ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
    I830Ptr pI830 = I830PTR(pScrn);
    struct i830_exa_pixmap_priv *driver_priv = exaGetPixmapDriverPrivate(pPixmap);

    if (driver_priv->flags & I830_EXA_PIXMAP_IS_MAPPED) {
	dri_bo_unmap(driver_priv->bo);
	driver_priv->flags &= ~I830_EXA_PIXMAP_IS_MAPPED;
    }
    dri_emit_reloc(reloc_buf, flags, delta, offset, driver_priv->bo);
    return driver_priv->bo->offset;
}
