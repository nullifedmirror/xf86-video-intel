/*
 * Copyright © 2007 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Eric Anholt <eric@anholt.net>
 *
 */

/**
 * @file dri_bufmgr_remap.h
 *
 * This file contains renaming macros for dri_bufmgr.c and intel_bufmgr_ttm.c,
 * to prevent symbol collisions with the AIGLX-loaded 3D driver.
 */

#define dri_bo_alloc ddx_dri_bo_alloc
#define dri_bo_alloc_static ddx_dri_bo_alloc_static
#define dri_bo_reference ddx_dri_bo_reference
#define dri_bo_unreference ddx_dri_bo_unreference
#define dri_bo_map ddx_dri_bo_map
#define dri_bo_unmap ddx_dri_bo_unmap
#define dri_fence_wait ddx_dri_fence_wait
#define dri_fence_reference ddx_dri_fence_reference
#define dri_fence_unreference ddx_dri_fence_unreference
#define dri_bo_subdata ddx_dri_bo_subdata
#define dri_bo_get_subdata ddx_dri_bo_get_subdata
#define dri_bufmgr_destroy ddx_dri_bufmgr_destroy
#define dri_emit_reloc ddx_dri_emit_reloc
#define dri_process_relocs ddx_dri_process_relocs
#define dri_post_process_relocs ddx_dri_post_process_relocs
#define dri_post_submit ddx_dri_post_submit

#define intel_ttm_bo_create_from_handle ddx_intel_ttm_bo_create_from_handle
#define intel_ttm_fence_create_from_arg ddx_intel_ttm_fence_create_from_arg
#define intel_bufmgr_ttm_init ddx_intel_bufmgr_ttm_init

