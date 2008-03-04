/**************************************************************************
 *
 * Copyright © 2007-2008 Red Hat Inc.
 * Copyright © 2007 Intel Corporation
 * Copyright 2006 Tungsten Graphics, Inc., Bismarck, ND., USA
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 *
 **************************************************************************/
/*
 * Authors: Thomas Hellström <thomas-at-tungstengraphics-dot-com>
 *          Keith Whitwell <keithw-at-tungstengraphics-dot-com>
 *	    Eric Anholt <eric@anholt.net>
 *	    Dave Airlie <airlied@linux.ie>
 *	    Kristian Høgsberg <krh@redhat.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <xf86drm.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>

#include "xf86.h"
#include "i830.h"
#include "errno.h"
#include "dri_bufmgr.h"
#include "string.h"

#include "i915_drm.h"

#include "intel_bufmgr_exa.h"

typedef struct _dri_bufmgr_exa {
    dri_bufmgr bufmgr;
    ScrnInfoPtr pScrn;
} dri_bufmgr_exa;

typedef struct _dri_bo_exa {
    dri_bo bo;
    i830_memory *memory;
    int refcount;
} dri_bo_exa;

static dri_bo *
dri_exa_alloc(dri_bufmgr *bufmgr, const char *name,
	      unsigned long size, unsigned int alignment,
	      uint64_t location_mask)
{
    dri_bufmgr_exa *bufmgr_exa = (dri_bufmgr_exa *)bufmgr;
    I830Ptr pI830 = I830PTR(bufmgr_exa->pScrn);
    dri_bo_exa *exa_buf;

    exa_buf = malloc(sizeof(*exa_buf));
    if (!exa_buf)
	return NULL;

    exa_buf->refcount = 1;
    exa_buf->memory = i830_allocate_memory(bufmgr_exa->pScrn,
					   name, size, alignment, 0);

    exa_buf->bo.size = exa_buf->memory->size;
    exa_buf->bo.offset = exa_buf->memory->offset;
    exa_buf->bo.bufmgr = bufmgr;
    exa_buf->bo.virtual = pI830->FbBase + exa_buf->memory->offset;

    return &exa_buf->bo;
}

static void
dri_exa_bo_reference(dri_bo *buf)
{
    dri_bo_exa *exa_buf = (dri_bo_exa *)buf;

    exa_buf->refcount++;
}

static void
dri_exa_bo_unreference(dri_bo *buf)
{
    dri_bufmgr_exa *bufmgr_exa = (dri_bufmgr_exa *)buf->bufmgr;
    dri_bo_exa *exa_buf = (dri_bo_exa *)buf;

    if (!buf)
	return;

    if (--exa_buf->refcount == 0)
	i830_free_memory(bufmgr_exa->pScrn, exa_buf->memory);
}

static int
dri_exa_bo_map(dri_bo *buf, GLboolean write_enable)
{
    return 0;
}

static int
dri_exa_bo_unmap(dri_bo *buf)
{
    return 0;
}

static void
dri_bufmgr_exa_destroy(dri_bufmgr *bufmgr)
{
    free(bufmgr);
}

static void
dri_exa_emit_reloc(dri_bo *reloc_buf, uint64_t flags, GLuint delta,
		   GLuint offset, dri_bo *target_buf)
{
}

/**
 * Initializes the EXA buffer manager, which is just a thin wrapper
 * around the EXA allocator.
 *
 * \param fd File descriptor of the opened DRM device.
 * \param fence_type Driver-specific fence type used for fences with no flush.
 * \param fence_type_flush Driver-specific fence type used for fences with a
 *	  flush.
 */
dri_bufmgr *
intel_bufmgr_exa_init(ScrnInfoPtr pScrn)
{
    dri_bufmgr_exa *bufmgr_exa;

    bufmgr_exa = calloc(1, sizeof(*bufmgr_exa));
    bufmgr_exa->pScrn = pScrn;

    bufmgr_exa->bufmgr.bo_alloc = dri_exa_alloc;
    bufmgr_exa->bufmgr.bo_reference = dri_exa_bo_reference;
    bufmgr_exa->bufmgr.bo_unreference = dri_exa_bo_unreference;
    bufmgr_exa->bufmgr.bo_map = dri_exa_bo_map;
    bufmgr_exa->bufmgr.bo_unmap = dri_exa_bo_unmap;
    bufmgr_exa->bufmgr.destroy = dri_bufmgr_exa_destroy;
    bufmgr_exa->bufmgr.emit_reloc = dri_exa_emit_reloc;

    return &bufmgr_exa->bufmgr;
}

