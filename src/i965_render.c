/*
 * Copyright © 2006 Intel Corporation
 * Copyright © 2007 Red Hat, Inc.
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Wang Zhenyu <zhenyu.z.wang@intel.com>
 *    Eric Anholt <eric@anholt.net>
 *    Carl Worth  <cworth@redhat.com>
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>
#include "xf86.h"
#include "i830.h"
#include "i915_reg.h"

/* bring in brw structs */
#include "brw_defines.h"
#include "brw_structs.h"

#ifdef I830DEBUG
#define DEBUG_I830FALLBACK 1
#endif

#ifdef DEBUG_I830FALLBACK
#define I830FALLBACK(s, arg...)				\
do {							\
	DPRINTF(PFX, "EXA fallback: " s "\n", ##arg); 	\
	return FALSE;					\
} while(0)
#else
#define I830FALLBACK(s, arg...) 			\
do { 							\
	return FALSE;					\
} while(0)
#endif

/* Set up a default static partitioning of the URB, which is supposed to
 * allow anything we would want to do, at potentially lower performance.
 */
#define URB_CS_ENTRY_SIZE     0
#define URB_CS_ENTRIES	      0

#define URB_VS_ENTRY_SIZE     1	  // each 512-bit row
#define URB_VS_ENTRIES	      8	  // we needs at least 8 entries

#define URB_GS_ENTRY_SIZE     0
#define URB_GS_ENTRIES	      0

#define URB_CLIP_ENTRY_SIZE   0
#define URB_CLIP_ENTRIES      0

#define URB_SF_ENTRY_SIZE     2
#define URB_SF_ENTRIES	      1

struct blendinfo {
    Bool dst_alpha;
    Bool src_alpha;
    CARD32 src_blend;
    CARD32 dst_blend;
};

struct formatinfo {
    int fmt;
    CARD32 card_fmt;
};

// refer vol2, 3d rasterization 3.8.1

/* defined in brw_defines.h */
static struct blendinfo i965_blend_op[] = {
    /* Clear */
    {0, 0, BRW_BLENDFACTOR_ZERO,          BRW_BLENDFACTOR_ZERO},
    /* Src */
    {0, 0, BRW_BLENDFACTOR_ONE,           BRW_BLENDFACTOR_ZERO},
    /* Dst */
    {0, 0, BRW_BLENDFACTOR_ZERO,          BRW_BLENDFACTOR_ONE},
    /* Over */
    {0, 1, BRW_BLENDFACTOR_ONE,           BRW_BLENDFACTOR_INV_SRC_ALPHA},
    /* OverReverse */
    {1, 0, BRW_BLENDFACTOR_INV_DST_ALPHA, BRW_BLENDFACTOR_ONE},
    /* In */
    {1, 0, BRW_BLENDFACTOR_DST_ALPHA,     BRW_BLENDFACTOR_ZERO},
    /* InReverse */
    {0, 1, BRW_BLENDFACTOR_ZERO,          BRW_BLENDFACTOR_SRC_ALPHA},
    /* Out */
    {1, 0, BRW_BLENDFACTOR_INV_DST_ALPHA, BRW_BLENDFACTOR_ZERO},
    /* OutReverse */
    {0, 1, BRW_BLENDFACTOR_ZERO,          BRW_BLENDFACTOR_INV_SRC_ALPHA},
    /* Atop */
    {1, 1, BRW_BLENDFACTOR_DST_ALPHA,     BRW_BLENDFACTOR_INV_SRC_ALPHA},
    /* AtopReverse */
    {1, 1, BRW_BLENDFACTOR_INV_DST_ALPHA, BRW_BLENDFACTOR_SRC_ALPHA},
    /* Xor */
    {1, 1, BRW_BLENDFACTOR_INV_DST_ALPHA, BRW_BLENDFACTOR_INV_SRC_ALPHA},
    /* Add */
    {0, 0, BRW_BLENDFACTOR_ONE,           BRW_BLENDFACTOR_ONE},
};

/* FIXME: surface format defined in brw_defines.h, shared Sampling engine
 * 1.7.2
 */
static struct formatinfo i965_tex_formats[] = {
    {PICT_a8r8g8b8, BRW_SURFACEFORMAT_B8G8R8A8_UNORM },
    {PICT_x8r8g8b8, BRW_SURFACEFORMAT_B8G8R8X8_UNORM },
    {PICT_a8b8g8r8, BRW_SURFACEFORMAT_R8G8B8A8_UNORM },
    {PICT_x8b8g8r8, BRW_SURFACEFORMAT_R8G8B8X8_UNORM },
    {PICT_r5g6b5,   BRW_SURFACEFORMAT_B5G6R5_UNORM   },
    {PICT_a1r5g5b5, BRW_SURFACEFORMAT_B5G5R5A1_UNORM },
    {PICT_a8,       BRW_SURFACEFORMAT_A8_UNORM	 },
};

static void i965_get_blend_cntl(int op, PicturePtr pMask, CARD32 dst_format,
				CARD32 *sblend, CARD32 *dblend)
{

    *sblend = i965_blend_op[op].src_blend;
    *dblend = i965_blend_op[op].dst_blend;

    /* If there's no dst alpha channel, adjust the blend op so that we'll treat
     * it as always 1.
     */
    if (PICT_FORMAT_A(dst_format) == 0 && i965_blend_op[op].dst_alpha) {
        if (*sblend == BRW_BLENDFACTOR_DST_ALPHA)
            *sblend = BRW_BLENDFACTOR_ONE;
        else if (*sblend == BRW_BLENDFACTOR_INV_DST_ALPHA)
            *sblend = BRW_BLENDFACTOR_ZERO;
    }

    /* If the source alpha is being used, then we should only be in a case where
     * the source blend factor is 0, and the source blend value is the mask
     * channels multiplied by the source picture's alpha.
     */
    if (pMask && pMask->componentAlpha && PICT_FORMAT_RGB(pMask->format)
            && i965_blend_op[op].src_alpha) {
        if (*dblend == BRW_BLENDFACTOR_SRC_ALPHA) {
	    *dblend = BRW_BLENDFACTOR_SRC_COLOR;
        } else if (*dblend == BRW_BLENDFACTOR_INV_SRC_ALPHA) {
	    *dblend = BRW_BLENDFACTOR_INV_SRC_COLOR;
        }
    }

}

static Bool i965_get_dest_format(PicturePtr pDstPicture, CARD32 *dst_format)
{
    switch (pDstPicture->format) {
    case PICT_a8r8g8b8:
    case PICT_x8r8g8b8:
        *dst_format = BRW_SURFACEFORMAT_B8G8R8A8_UNORM;
        break;
    case PICT_r5g6b5:
        *dst_format = BRW_SURFACEFORMAT_B5G6R5_UNORM;
        break;
    case PICT_a1r5g5b5:
    	*dst_format = BRW_SURFACEFORMAT_B5G5R5A1_UNORM;
	break;
    case PICT_x1r5g5b5:
        *dst_format = BRW_SURFACEFORMAT_B5G5R5X1_UNORM;
        break;
    case PICT_a8:
        *dst_format = BRW_SURFACEFORMAT_A8_UNORM;
        break;
    case PICT_a4r4g4b4:
    case PICT_x4r4g4b4:
	*dst_format = BRW_SURFACEFORMAT_B4G4R4A4_UNORM;
	break;
    default:
        I830FALLBACK("Unsupported dest format 0x%x\n",
		     (int)pDstPicture->format);
    }

    return TRUE;
}

static Bool i965_check_composite_texture(PicturePtr pPict, int unit)
{
    int w = pPict->pDrawable->width;
    int h = pPict->pDrawable->height;
    int i;

    if ((w > 0x7ff) || (h > 0x7ff))
        I830FALLBACK("Picture w/h too large (%dx%d)\n", w, h);

    for (i = 0; i < sizeof(i965_tex_formats) / sizeof(i965_tex_formats[0]);
	 i++)
    {
        if (i965_tex_formats[i].fmt == pPict->format)
            break;
    }
    if (i == sizeof(i965_tex_formats) / sizeof(i965_tex_formats[0]))
        I830FALLBACK("Unsupported picture format 0x%x\n",
		     (int)pPict->format);

    if (pPict->repeat && pPict->repeatType != RepeatNormal)
	I830FALLBACK("extended repeat (%d) not supported\n",
		     pPict->repeatType);

    if (pPict->filter != PictFilterNearest &&
        pPict->filter != PictFilterBilinear)
    {
        I830FALLBACK("Unsupported filter 0x%x\n", pPict->filter);
    }

    return TRUE;
}

Bool
i965_check_composite(int op, PicturePtr pSrcPicture, PicturePtr pMaskPicture,
		     PicturePtr pDstPicture)
{
    CARD32 tmp1;

    /* Check for unsupported compositing operations. */
    if (op >= sizeof(i965_blend_op) / sizeof(i965_blend_op[0]))
        I830FALLBACK("Unsupported Composite op 0x%x\n", op);

    if (pMaskPicture && pMaskPicture->componentAlpha &&
            PICT_FORMAT_RGB(pMaskPicture->format)) {
        /* Check if it's component alpha that relies on a source alpha and on
         * the source value.  We can only get one of those into the single
         * source value that we get to blend with.
         */
        if (i965_blend_op[op].src_alpha &&
            (i965_blend_op[op].src_blend != BRW_BLENDFACTOR_ZERO))
	{
	    I830FALLBACK("Component alpha not supported with source "
			 "alpha and source value blending.\n");
	}
    } 

    if (!i965_check_composite_texture(pSrcPicture, 0))
        I830FALLBACK("Check Src picture texture\n");
    if (pMaskPicture != NULL && !i965_check_composite_texture(pMaskPicture, 1))
        I830FALLBACK("Check Mask picture texture\n");

    if (!i965_get_dest_format(pDstPicture, &tmp1))
	I830FALLBACK("Get Color buffer format\n");

    return TRUE;

}

#define ALIGN(i,m)    (((i) + (m) - 1) & ~((m) - 1))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define BRW_GRF_BLOCKS(nreg)    ((nreg + 15) / 16 - 1)

/* these offsets will remain the same for all buffers post allocation */
static int dest_surf_offset, src_surf_offset, mask_surf_offset;
static int vb_offset;
static int binding_table_offset;

static float *vb;
static int vb_index;

static const CARD32 sip_kernel_static[][4] = {
/*    wait (1) a0<1>UW a145<0,1,0>UW { align1 +  } */
    { 0x00000030, 0x20000108, 0x00001220, 0x00000000 },
/*    nop (4) g0<1>UD { align1 +  } */
    { 0x0040007e, 0x20000c21, 0x00690000, 0x00000000 },
/*    nop (4) g0<1>UD { align1 +  } */
    { 0x0040007e, 0x20000c21, 0x00690000, 0x00000000 },
/*    nop (4) g0<1>UD { align1 +  } */
    { 0x0040007e, 0x20000c21, 0x00690000, 0x00000000 },
/*    nop (4) g0<1>UD { align1 +  } */
    { 0x0040007e, 0x20000c21, 0x00690000, 0x00000000 },
/*    nop (4) g0<1>UD { align1 +  } */
    { 0x0040007e, 0x20000c21, 0x00690000, 0x00000000 },
/*    nop (4) g0<1>UD { align1 +  } */
    { 0x0040007e, 0x20000c21, 0x00690000, 0x00000000 },
/*    nop (4) g0<1>UD { align1 +  } */
    { 0x0040007e, 0x20000c21, 0x00690000, 0x00000000 },
/*    nop (4) g0<1>UD { align1 +  } */
    { 0x0040007e, 0x20000c21, 0x00690000, 0x00000000 },
/*    nop (4) g0<1>UD { align1 +  } */
    { 0x0040007e, 0x20000c21, 0x00690000, 0x00000000 },
};

/*
 * this program computes dA/dx and dA/dy for the texture coordinates along
 * with the base texture coordinate. It was extracted from the Mesa driver
 */

#define SF_KERNEL_NUM_GRF  16
#define SF_MAX_THREADS	   1

static const CARD32 sf_kernel_static[][4] = {
#include "exa_sf_prog.h"
};

static const CARD32 sf_kernel_mask_static[][4] = {
#include "exa_sf_mask_prog.h"
};

static const CARD32 sf_kernel_rotation_static[][4] = {
#include "exa_sf_rotation_prog.h"
};

/* ps kernels */
#define PS_KERNEL_NUM_GRF   32
#define PS_MAX_THREADS	   32

static const CARD32 ps_kernel_nomask_static [][4] = {
#include "exa_wm_nomask_prog.h"
};

static const CARD32 ps_kernel_maskca_static [][4] = {
#include "exa_wm_maskca_prog.h"
};

static const CARD32 ps_kernel_maskca_srcalpha_static [][4] = {
#include "exa_wm_maskca_srcalpha_prog.h"
};

static const CARD32 ps_kernel_masknoca_static [][4] = {
#include "exa_wm_masknoca_prog.h"
};

static const CARD32 ps_kernel_rotation_static [][4] = {
#include "exa_wm_rotation_prog.h"
};

typedef enum {
    SAMPLER_STATE_FILTER_NEAREST,
    SAMPLER_STATE_FILTER_BILINEAR,
    SAMPLER_STATE_FILTER_COUNT
} sampler_state_filter_t;

typedef enum {
    SAMPLER_STATE_EXTEND_NONE,
    SAMPLER_STATE_EXTEND_REPEAT,
    SAMPLER_STATE_EXTEND_COUNT
} sampler_state_extend_t;

typedef struct _brw_cc_unit_state_padded {
    struct brw_cc_unit_state state;
    char pad[64 - sizeof (struct brw_cc_unit_state)];
} brw_cc_unit_state_padded;

/* Many of the fields in the state structure must be aligned to a
 * 64-byte boundary, (or a 32-byte boundary, but 64 is good enough for
 * those too). */
#define PAD64_MULTI(previous, idx, factor) char previous ## _pad ## idx [(64 - (sizeof(struct previous) * (factor)) % 64) % 64]
#define PAD64(previous, idx) PAD64_MULTI(previous, idx, 1)
#define KERNEL_DECL(template) \
    CARD32 template [((sizeof (template ## _static) + 63) & ~63) / 16][4];
typedef struct _gen4_state {
    char wm_scratch[128 * PS_MAX_THREADS];

    /* Index by [src_filter][src_extend][mask_filter][mask_extend] */
    struct brw_sampler_state sampler_state[SAMPLER_STATE_FILTER_COUNT]
					  [SAMPLER_STATE_EXTEND_COUNT]
					  [SAMPLER_STATE_FILTER_COUNT]
					  [SAMPLER_STATE_EXTEND_COUNT][2];

    struct brw_sampler_default_color default_color_state;
    PAD64 (brw_sampler_default_color, 0);
    struct brw_vs_unit_state vs_state;
    PAD64 (brw_vs_unit_state, 0);

    /* Index by [src_blend][dst_blend] */
    brw_cc_unit_state_padded cc_state[BRW_BLENDFACTOR_COUNT]
				     [BRW_BLENDFACTOR_COUNT];

    struct brw_cc_viewport cc_viewport;
    PAD64 (brw_cc_viewport, 0);

    KERNEL_DECL (sip_kernel);

    /* SF kernels and corresponding states */
    KERNEL_DECL (sf_kernel);
    struct brw_sf_unit_state sf_state;
    PAD64 (brw_sf_unit_state, 0);

    KERNEL_DECL (sf_kernel_mask);
    struct brw_sf_unit_state sf_state_mask;
    PAD64 (brw_sf_unit_state, 1);

    KERNEL_DECL (sf_kernel_rotation);
    struct brw_sf_unit_state sf_state_rotation;
    PAD64 (brw_sf_unit_state, 2);

    /* PS kernels and corresponding WM states */
    KERNEL_DECL (ps_kernel_nomask);
    struct brw_wm_unit_state wm_state_nomask[SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT]
					    [SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT];

    KERNEL_DECL (ps_kernel_maskca);
    struct brw_wm_unit_state wm_state_maskca[SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT]
					    [SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT];

    KERNEL_DECL (ps_kernel_maskca_srcalpha);
    struct brw_wm_unit_state wm_state_maskca_srcalpha[SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT]
					    [SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT];

    KERNEL_DECL (ps_kernel_masknoca);
    struct brw_wm_unit_state wm_state_masknoca[SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT]
					    [SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT];

    KERNEL_DECL (ps_kernel_rotation);
    struct brw_wm_unit_state wm_state_rotation[SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT]
					    [SAMPLER_STATE_FILTER_COUNT]
					    [SAMPLER_STATE_EXTEND_COUNT];
} gen4_state_t;

/* How many composite operations will we fit in one object. */
#define GEN4_MAX_OPS			32
#define GEN4_SURFACE_STATE_PER_OP	3
#define GEN4_MAX_SURFACE_STATES		(GEN4_MAX_OPS * GEN4_SURFACE_STATE_PER_OP)
/* We only need 3, but we use 8 to get the proper alignment. */
#define GEN4_BINDING_TABLE_PER_OP	8
#define GEN4_MAX_BINDING_TABLE		(GEN4_MAX_OPS * GEN4_BINDING_TABLE_PER_OP)
#define GEN4_VERTICES_PER_OP		24
#define GEN4_MAX_VERTICES		(GEN4_MAX_OPS * GEN4_VERTICES_PER_OP)

typedef struct _brw_surface_state_padded {
    struct brw_surface_state state;
    char pad[32 - sizeof (struct brw_surface_state)];
} brw_surface_state_padded;

typedef struct _gen4_surface_state {
    brw_surface_state_padded surface_state[GEN4_MAX_SURFACE_STATES];

    CARD32 binding_table[GEN4_MAX_BINDING_TABLE];

    float vb[GEN4_MAX_VERTICES];
} gen4_surface_state_t;

static CARD32 
i965_get_card_format(PicturePtr pPict)
{
    int i;

    for (i = 0; i < sizeof(i965_tex_formats) / sizeof(i965_tex_formats[0]);
	 i++)
    {
	if (i965_tex_formats[i].fmt == pPict->format)
	    break;
    }
    return i965_tex_formats[i].card_fmt;
}

static Bool
i965_check_rotation_transform(PictTransformPtr t)
{
    /* XXX this is arbitrary */
    int a, b;
    a = xFixedToInt(t->matrix[0][1]);
    b = xFixedToInt(t->matrix[1][0]);
    if (a == -1 && b == 1)
	return TRUE;
    else if (a == 1 && b == -1)
	return TRUE;
    else
	return FALSE;
}

static void
sf_state_init (struct brw_sf_unit_state *sf_state, int kernel_offset)
{
    memset(sf_state, 0, sizeof(*sf_state));
    sf_state->thread0.grf_reg_count = BRW_GRF_BLOCKS(SF_KERNEL_NUM_GRF);
    sf_state->sf1.single_program_flow = 1;
    sf_state->sf1.binding_table_entry_count = 0;
    sf_state->sf1.thread_priority = 0;
    sf_state->sf1.floating_point_mode = 0; /* Mesa does this */
    sf_state->sf1.illegal_op_exception_enable = 1;
    sf_state->sf1.mask_stack_exception_enable = 1;
    sf_state->sf1.sw_exception_enable = 1;
    sf_state->thread2.per_thread_scratch_space = 0;
    /* scratch space is not used in our kernel */
    sf_state->thread2.scratch_space_base_pointer = 0;
    sf_state->thread3.const_urb_entry_read_length = 0; /* no const URBs */
    sf_state->thread3.const_urb_entry_read_offset = 0; /* no const URBs */
    sf_state->thread3.urb_entry_read_length = 1; /* 1 URB per vertex */
    /* don't smash vertex header, read start from dw8 */
    sf_state->thread3.urb_entry_read_offset = 1;
    sf_state->thread3.dispatch_grf_start_reg = 3;
    sf_state->thread4.max_threads = SF_MAX_THREADS - 1;
    sf_state->thread4.urb_entry_allocation_size = URB_SF_ENTRY_SIZE - 1;
    sf_state->thread4.nr_urb_entries = URB_SF_ENTRIES;
    sf_state->thread4.stats_enable = 1;
    sf_state->sf5.viewport_transform = FALSE; /* skip viewport */
    sf_state->sf6.cull_mode = BRW_CULLMODE_NONE;
    sf_state->sf6.scissor = 0;
    sf_state->sf7.trifan_pv = 2;
    sf_state->sf6.dest_org_vbias = 0x8;
    sf_state->sf6.dest_org_hbias = 0x8;

    sf_state->thread0.kernel_start_pointer = kernel_offset >> 6;
}

static void
sampler_state_init (struct brw_sampler_state *sampler_state,
		    sampler_state_filter_t filter,
		    sampler_state_extend_t extend,
		    int default_color_offset)
{
    /* PS kernel use this sampler */
    memset(sampler_state, 0, sizeof(*sampler_state));
    sampler_state->ss0.lod_preclamp = 1; /* GL mode */

    sampler_state->ss0.default_color_mode = 0; /* GL mode */

    switch(filter) {
    default:
    case SAMPLER_STATE_FILTER_NEAREST:
	sampler_state->ss0.min_filter = BRW_MAPFILTER_NEAREST;
	sampler_state->ss0.mag_filter = BRW_MAPFILTER_NEAREST;
	break;
    case SAMPLER_STATE_FILTER_BILINEAR:
	sampler_state->ss0.min_filter = BRW_MAPFILTER_LINEAR;
	sampler_state->ss0.mag_filter = BRW_MAPFILTER_LINEAR;
	break;
    }

    switch (extend) {
    default:
    case SAMPLER_STATE_EXTEND_NONE:
	sampler_state->ss1.r_wrap_mode = BRW_TEXCOORDMODE_CLAMP_BORDER;
	sampler_state->ss1.s_wrap_mode = BRW_TEXCOORDMODE_CLAMP_BORDER;
	sampler_state->ss1.t_wrap_mode = BRW_TEXCOORDMODE_CLAMP_BORDER;
	break;
    case SAMPLER_STATE_EXTEND_REPEAT:
	sampler_state->ss1.r_wrap_mode = BRW_TEXCOORDMODE_WRAP;
	sampler_state->ss1.s_wrap_mode = BRW_TEXCOORDMODE_WRAP;
	sampler_state->ss1.t_wrap_mode = BRW_TEXCOORDMODE_WRAP;
	break;
    }

    sampler_state->ss2.default_color_pointer = default_color_offset >> 5;

    sampler_state->ss3.chroma_key_enable = 0; /* disable chromakey */
}

static void
wm_state_init (struct brw_wm_unit_state *wm_state,
	       Bool has_mask,
	       int scratch_offset,
	       int kernel_offset,
	       int sampler_state_offset)
{
    memset(wm_state, 0, sizeof (*wm_state));
    wm_state->thread0.grf_reg_count = BRW_GRF_BLOCKS(PS_KERNEL_NUM_GRF);
    wm_state->thread1.single_program_flow = 1;

    wm_state->thread2.scratch_space_base_pointer = scratch_offset >> 10;

    wm_state->thread2.per_thread_scratch_space = 0;
    wm_state->thread3.const_urb_entry_read_length = 0;
    wm_state->thread3.const_urb_entry_read_offset = 0;

    wm_state->thread3.urb_entry_read_offset = 0;
    /* wm kernel use urb from 3, see wm_program in compiler module */
    wm_state->thread3.dispatch_grf_start_reg = 3; /* must match kernel */

    wm_state->wm4.stats_enable = 1;  /* statistic */
    wm_state->wm4.sampler_state_pointer = sampler_state_offset >> 5;
    wm_state->wm4.sampler_count = 1; /* 1-4 samplers used */
    wm_state->wm5.max_threads = PS_MAX_THREADS - 1;
    wm_state->wm5.thread_dispatch_enable = 1;
    /* just use 16-pixel dispatch (4 subspans), don't need to change kernel
     * start point
     */
    wm_state->wm5.enable_16_pix = 1;
    wm_state->wm5.enable_8_pix = 0;
    wm_state->wm5.early_depth_test = 1;

    wm_state->thread0.kernel_start_pointer = kernel_offset >> 6;

    /* Each pair of attributes (src/mask coords) is one URB entry */
    if (has_mask) {
	wm_state->thread1.binding_table_entry_count = 3; /* 2 tex and fb */
	wm_state->thread3.urb_entry_read_length = 2;
    } else {
	wm_state->thread1.binding_table_entry_count = 2; /* 1 tex and fb */
	wm_state->thread3.urb_entry_read_length = 1;
    }
}

static void
cc_state_init (struct brw_cc_unit_state *cc_state,
	       int src_blend,
	       int dst_blend,
	       int cc_viewport_offset)
{
    memset(cc_state, 0, sizeof(*cc_state));
    cc_state->cc0.stencil_enable = 0;   /* disable stencil */
    cc_state->cc2.depth_test = 0;       /* disable depth test */
    cc_state->cc2.logicop_enable = 0;   /* disable logic op */
    cc_state->cc3.ia_blend_enable = 1;  /* blend alpha just like colors */
    cc_state->cc3.blend_enable = 1;     /* enable color blend */
    cc_state->cc3.alpha_test = 0;       /* disable alpha test */

    cc_state->cc4.cc_viewport_state_offset = cc_viewport_offset >> 5;

    cc_state->cc5.dither_enable = 0;    /* disable dither */
    cc_state->cc5.logicop_func = 0xc;   /* COPY */
    cc_state->cc5.statistics_enable = 1;
    cc_state->cc5.ia_blend_function = BRW_BLENDFUNCTION_ADD;

    /* XXX: alpha blend factor should be same as color, but check
     * for CA case in future
     */
    cc_state->cc5.ia_src_blend_factor = src_blend;
    cc_state->cc5.ia_dest_blend_factor = dst_blend;

    cc_state->cc6.blend_function = BRW_BLENDFUNCTION_ADD;
    cc_state->cc6.clamp_post_alpha_blend = 1;
    cc_state->cc6.clamp_pre_alpha_blend = 1;
    cc_state->cc6.clamp_range = 0;  /* clamp range [0,1] */

    cc_state->cc6.src_blend_factor = src_blend;
    cc_state->cc6.dest_blend_factor = dst_blend;
}

static void
gen4_state_init (gen4_state_t *state)
{
    struct brw_cc_viewport *cc_viewport;
    struct brw_vs_unit_state *vs_state;
    int i,j, k, l;

    cc_viewport = &state->cc_viewport;
    cc_viewport->min_depth = -1.e35;
    cc_viewport->max_depth = 1.e35;

    /* Color calculator state */
    for (i = 0; i < BRW_BLENDFACTOR_COUNT; i++)
	for (j = 0; j < BRW_BLENDFACTOR_COUNT; j++)
	    cc_state_init (&state->cc_state[i][j].state, i, j,
			   offsetof (gen4_state_t, cc_viewport));

    for (i = 0; i < SAMPLER_STATE_FILTER_COUNT; i++)
	for (j = 0; j < SAMPLER_STATE_EXTEND_COUNT; j++)
	    for (k = 0; k < SAMPLER_STATE_FILTER_COUNT; k++)
		for (l = 0; l < SAMPLER_STATE_EXTEND_COUNT; l++) {
		    sampler_state_init (&state->sampler_state[i][j][k][l][0],
					i, j,
					offsetof (gen4_state_t, default_color_state));
		    sampler_state_init (&state->sampler_state[i][j][k][l][1],
					k, l,
					offsetof (gen4_state_t, default_color_state));
		}

    /* Set up the vertex shader to be disabled (passthrough) */
    vs_state = &state->vs_state;
    vs_state->thread4.nr_urb_entries = URB_VS_ENTRIES;
    vs_state->thread4.urb_entry_allocation_size = URB_VS_ENTRY_SIZE - 1;
    vs_state->vs6.vs_enable = 0;
    vs_state->vs6.vert_cache_disable = 1;

    /* Copy all SF kernels into state structure. */
    memcpy(state->sf_kernel, sf_kernel_static,
	   sizeof (sf_kernel_static));
    memcpy(state->sf_kernel_mask, sf_kernel_mask_static,
	   sizeof (sf_kernel_mask_static));
    memcpy(state->sf_kernel_rotation, sf_kernel_rotation_static,
	   sizeof (sf_kernel_rotation_static));

    sf_state_init (&state->sf_state,
		   offsetof (gen4_state_t, sf_kernel));
    sf_state_init (&state->sf_state_mask,
		   offsetof (gen4_state_t, sf_kernel_mask));
    sf_state_init (&state->sf_state_rotation,
		   offsetof (gen4_state_t, sf_kernel_rotation));

    for (i = 0; i < SAMPLER_STATE_FILTER_COUNT; i++)
	for (j = 0; j < SAMPLER_STATE_EXTEND_COUNT; j++)
	    for (k = 0; k < SAMPLER_STATE_FILTER_COUNT; k++)
		for (l = 0; l < SAMPLER_STATE_EXTEND_COUNT; l++) {
		    wm_state_init (&state->wm_state_nomask[i][j][k][l],
				   FALSE,
				   offsetof (gen4_state_t, wm_scratch),
				   offsetof (gen4_state_t, ps_kernel_nomask),
				   offsetof (gen4_state_t, sampler_state[i][j][k][l][0]));
		    wm_state_init (&state->wm_state_maskca[i][j][k][l],
				   TRUE,
				   offsetof (gen4_state_t, wm_scratch),
				   offsetof (gen4_state_t, ps_kernel_maskca),
				   offsetof (gen4_state_t, sampler_state[i][j][k][l][0]));
		    wm_state_init (&state->wm_state_maskca_srcalpha[i][j][k][l],
				   TRUE,
				   offsetof (gen4_state_t, wm_scratch),
				   offsetof (gen4_state_t, ps_kernel_maskca_srcalpha),
				   offsetof (gen4_state_t, sampler_state[i][j][k][l][0]));
		    wm_state_init (&state->wm_state_masknoca[i][j][k][l],
				   TRUE,
				   offsetof (gen4_state_t, wm_scratch),
				   offsetof (gen4_state_t, ps_kernel_masknoca),
				   offsetof (gen4_state_t, sampler_state[i][j][k][l][0]));
		    wm_state_init (&state->wm_state_rotation[i][j][k][l],
				   FALSE,
				   offsetof (gen4_state_t, wm_scratch),
				   offsetof (gen4_state_t, ps_kernel_rotation),
				   offsetof (gen4_state_t, sampler_state[i][j][k][l][0]));
		}

    /* Upload kernels */
    memcpy (state->sip_kernel, sip_kernel_static, sizeof (sip_kernel_static));

    memcpy (state->ps_kernel_nomask, ps_kernel_nomask_static,
	    sizeof (ps_kernel_nomask_static));
    memcpy (state->ps_kernel_maskca, ps_kernel_maskca_static,
	    sizeof (ps_kernel_maskca_static));
    memcpy (state->ps_kernel_maskca_srcalpha,
	    ps_kernel_maskca_srcalpha_static,
	    sizeof (ps_kernel_maskca_srcalpha_static));
    memcpy (state->ps_kernel_masknoca, ps_kernel_masknoca_static,
	    sizeof (ps_kernel_masknoca_static));
    memcpy (state->ps_kernel_rotation, ps_kernel_rotation_static,
	    sizeof (ps_kernel_rotation_static));
}

static void
gen4_surface_state_init (unsigned char *start_base,
			 struct i965_exastate_buffer *state)
{
    struct brw_surface_state *dest_surf_state, *src_surf_state, *mask_surf_state;
    unsigned int surf_state_offset = offsetof (gen4_surface_state_t,
					       surface_state);

    vb_offset = (offsetof (gen4_surface_state_t, vb) +
		 sizeof (float) * GEN4_VERTICES_PER_OP * state->num_ops);

    binding_table_offset = (offsetof (gen4_surface_state_t, binding_table) +
			    sizeof (CARD32) * GEN4_BINDING_TABLE_PER_OP *
			    state->num_ops);

    /* destination surface state */
    dest_surf_offset = (surf_state_offset +
			sizeof (brw_surface_state_padded) *
			(GEN4_SURFACE_STATE_PER_OP * state->num_ops + 0));
    dest_surf_state = (void *)(start_base + dest_surf_offset);
    dest_surf_state->ss0.surface_type = BRW_SURFACE_2D;
    dest_surf_state->ss0.data_return_format = BRW_SURFACERETURNFORMAT_FLOAT32;
    dest_surf_state->ss0.writedisable_alpha = 0;
    dest_surf_state->ss0.writedisable_red = 0;
    dest_surf_state->ss0.writedisable_green = 0;
    dest_surf_state->ss0.writedisable_blue = 0;
    dest_surf_state->ss0.color_blend = 1;
    dest_surf_state->ss0.vert_line_stride = 0;
    dest_surf_state->ss0.vert_line_stride_ofs = 0;
    dest_surf_state->ss0.mipmap_layout_mode = 0;
    dest_surf_state->ss0.render_cache_read_mode = 0;
    dest_surf_state->ss2.mip_count = 0;
    dest_surf_state->ss2.render_target_rotation = 0;

    /* source surface state */
    src_surf_offset = (surf_state_offset +
		       sizeof (brw_surface_state_padded) *
		       (GEN4_SURFACE_STATE_PER_OP * state->num_ops + 1));
    src_surf_state = (void *)(start_base + src_surf_offset);
    src_surf_state->ss0.surface_type = BRW_SURFACE_2D;
    src_surf_state->ss0.writedisable_alpha = 0;
    src_surf_state->ss0.writedisable_red = 0;
    src_surf_state->ss0.writedisable_green = 0;
    src_surf_state->ss0.writedisable_blue = 0;
    src_surf_state->ss0.color_blend = 1;
    src_surf_state->ss0.vert_line_stride = 0;
    src_surf_state->ss0.vert_line_stride_ofs = 0;
    src_surf_state->ss0.mipmap_layout_mode = 0;
    src_surf_state->ss0.render_cache_read_mode = 0;
    src_surf_state->ss2.mip_count = 0;
    src_surf_state->ss2.render_target_rotation = 0;

    /* mask surface state */
    mask_surf_offset = (surf_state_offset +
			sizeof (brw_surface_state_padded) *
			(GEN4_SURFACE_STATE_PER_OP * state->num_ops + 2));
    mask_surf_state = (void *)(start_base + mask_surf_offset);
    mask_surf_state->ss0.surface_type = BRW_SURFACE_2D;
    mask_surf_state->ss0.writedisable_alpha = 0;
    mask_surf_state->ss0.writedisable_red = 0;
    mask_surf_state->ss0.writedisable_green = 0;
    mask_surf_state->ss0.writedisable_blue = 0;
    mask_surf_state->ss0.color_blend = 1;
    mask_surf_state->ss0.vert_line_stride = 0;
    mask_surf_state->ss0.vert_line_stride_ofs = 0;
    mask_surf_state->ss0.mipmap_layout_mode = 0;
    mask_surf_state->ss0.render_cache_read_mode = 0;
    mask_surf_state->ss2.mip_count = 0;
    mask_surf_state->ss2.render_target_rotation = 0;
}

void i965_exastate_flush(struct i965_exastate_buffer *state)
{
    if (state->surface_buf) {
	ddx_bo_unmap(state->surface_buf);
	ddx_bo_unreference(state->surface_buf);
	state->surface_buf = NULL;
	state->surface_map = NULL;
    }
}

static void
i965_exastate_reset(struct i965_exastate_buffer *state)
{
    I830Ptr pI830 = I830PTR(state->pScrn);

    /* First the general state buffer. */
    if (state->buf == NULL) {
	state->buf = ddx_bo_alloc(pI830->bufmgr, "exa state buffer",
				  sizeof (gen4_state_t), 4096,
				  DRM_BO_FLAG_MEM_LOCAL | DRM_BO_FLAG_CACHED | DRM_BO_FLAG_CACHED_MAPPED);
	ddx_bo_map(state->buf, TRUE);
	state->map = state->buf->virtual;
	gen4_state_init ((void *) state->map);
	ddx_bo_unmap(state->buf);
    }

    /* Then the surface state buffer */
    if (state->surface_buf != NULL && state->num_ops >= GEN4_MAX_OPS) {
	ddx_bo_unreference(state->surface_buf);
	state->surface_buf = NULL;
    }

    if (state->surface_buf == NULL) {
	state->surface_buf = ddx_bo_alloc(pI830->bufmgr, "exa surface state buffer",
					  sizeof (gen4_surface_state_t), 4096,
					  DRM_BO_FLAG_MEM_LOCAL | DRM_BO_FLAG_CACHED | DRM_BO_FLAG_CACHED_MAPPED);
	ddx_bo_map(state->surface_buf, TRUE);
	state->num_ops = 0;

	state->surface_map = state->surface_buf->virtual;
    }
}

static sampler_state_filter_t
sampler_state_filter_from_picture (int filter)
{
    switch (filter) {
    case PictFilterNearest:
	return SAMPLER_STATE_FILTER_NEAREST;
    case PictFilterBilinear:
	return SAMPLER_STATE_FILTER_BILINEAR;
    default:
	return -1;
    }
}

static sampler_state_extend_t
sampler_state_extend_from_picture (int repeat)
{
    if (repeat)
	return SAMPLER_STATE_EXTEND_REPEAT;
    else
	return SAMPLER_STATE_EXTEND_NONE;
}

static void
gen4_emit_batch_header (ScrnInfoPtr pScrn)
{
    I830Ptr pI830 = I830PTR(pScrn);
    int sip_kernel_offset;
    int urb_vs_start, urb_vs_size;
    int urb_gs_start, urb_gs_size;
    int urb_clip_start, urb_clip_size;
    int urb_sf_start, urb_sf_size;
    int urb_cs_start, urb_cs_size;

    urb_vs_start = 0;
    urb_vs_size = URB_VS_ENTRIES * URB_VS_ENTRY_SIZE;
    urb_gs_start = urb_vs_start + urb_vs_size;
    urb_gs_size = URB_GS_ENTRIES * URB_GS_ENTRY_SIZE;
    urb_clip_start = urb_gs_start + urb_gs_size;
    urb_clip_size = URB_CLIP_ENTRIES * URB_CLIP_ENTRY_SIZE;
    urb_sf_start = urb_clip_start + urb_clip_size;
    urb_sf_size = URB_SF_ENTRIES * URB_SF_ENTRY_SIZE;
    urb_cs_start = urb_sf_start + urb_sf_size;
    urb_cs_size = URB_CS_ENTRIES * URB_CS_ENTRY_SIZE;

    IntelEmitInvarientState(pScrn);

    sip_kernel_offset = offsetof (gen4_state_t, sip_kernel);

    /* Begin the long sequence of commands needed to set up the 3D
     * rendering pipe
     */
    {
	BEGIN_BATCH(2);
	OUT_BATCH(MI_FLUSH |
		  MI_STATE_INSTRUCTION_CACHE_FLUSH |
		  BRW_MI_GLOBAL_SNAPSHOT_RESET);
	OUT_BATCH(MI_NOOP);
	ADVANCE_BATCH();
    }
    {
	BEGIN_BATCH(16);

/* Match Mesa driver setup */
	OUT_BATCH(BRW_PIPELINE_SELECT | PIPELINE_SELECT_3D);

	OUT_BATCH(BRW_CS_URB_STATE | 0);
	OUT_BATCH((0 << 4) |  /* URB Entry Allocation Size */
		  (0 << 0));  /* Number of URB Entries */

	/* Zero out the two base address registers so all offsets are
	 * absolute.
	 */
	OUT_BATCH(BRW_STATE_BASE_ADDRESS | 4);

	if (pI830->use_ttm_batch) {
	    OUT_RELOC(pI830->exa965->buf, DRM_BO_FLAG_MEM_TT, BASE_ADDRESS_MODIFY);

	    OUT_RELOC(pI830->exa965->surface_buf, DRM_BO_FLAG_MEM_TT, BASE_ADDRESS_MODIFY);
	} else {
	    OUT_BATCH(pI830->exa_965_state->offset | BASE_ADDRESS_MODIFY);
	    OUT_BATCH(pI830->exa_965_state->offset | BASE_ADDRESS_MODIFY);
	}

	OUT_BATCH(0 | BASE_ADDRESS_MODIFY);  /* media base addr, don't care */
	/* general state max addr, disabled */
	OUT_BATCH(0x10000000 | BASE_ADDRESS_MODIFY);
	/* media object state max addr, disabled */
	OUT_BATCH(0x10000000 | BASE_ADDRESS_MODIFY);

	/* Set system instruction pointer */
	OUT_BATCH(BRW_STATE_SIP | 0);
	OUT_BATCH(sip_kernel_offset);

	/* URB fence */
	OUT_BATCH(BRW_URB_FENCE |
		  UF0_CS_REALLOC |
		  UF0_SF_REALLOC |
		  UF0_CLIP_REALLOC |
		  UF0_GS_REALLOC |
		  UF0_VS_REALLOC |
		  1);
	OUT_BATCH(((urb_clip_start + urb_clip_size) << UF1_CLIP_FENCE_SHIFT) |
		  ((urb_gs_start + urb_gs_size) << UF1_GS_FENCE_SHIFT) |
		  ((urb_vs_start + urb_vs_size) << UF1_VS_FENCE_SHIFT));
	OUT_BATCH(((urb_cs_start + urb_cs_size) << UF2_CS_FENCE_SHIFT) |
		  ((urb_sf_start + urb_sf_size) << UF2_SF_FENCE_SHIFT));

	/* Constant buffer state */
	OUT_BATCH(BRW_CS_URB_STATE | 0);
	OUT_BATCH(((URB_CS_ENTRY_SIZE - 1) << 4) |
		  (URB_CS_ENTRIES << 0));

	ADVANCE_BATCH();
    }
}

Bool
i965_prepare_composite(int op, PicturePtr pSrcPicture,
		       PicturePtr pMaskPicture, PicturePtr pDstPicture,
		       PixmapPtr pSrc, PixmapPtr pMask, PixmapPtr pDst)
{
    ScrnInfoPtr pScrn = xf86Screens[pSrcPicture->pDrawable->pScreen->myNum];
    I830Ptr pI830 = I830PTR(pScrn);
    CARD32 src_pitch, src_tile_format = 0, src_tiled = 0;
    CARD32 mask_pitch = 0, mask_tile_format = 0, mask_tiled = 0;
    CARD32 dst_format, dst_pitch, dst_tile_format = 0, dst_tiled = 0;
    Bool rotation_program = FALSE;
    int wm_state_offset;
    int sf_state_offset, cc_state_offset;
    char *surface_start_base;
    void *surface_map;
    sampler_state_filter_t src_filter, mask_filter;
    sampler_state_extend_t src_extend, mask_extend;
    struct brw_surface_state *dest_surf_state, *src_surf_state, *mask_surf_state;
    CARD32 *binding_table;
    CARD32 src_blend, dst_blend;

    if (pI830->use_ttm_batch) {
	i965_exastate_reset(pI830->exa965);
	surface_map = pI830->exa965->surface_map;
	gen4_surface_state_init (surface_map, pI830->exa965);
    }else{
	surface_map = pI830->exa_965_state->offset + pI830->FbBase;
    }

    surface_start_base = surface_map;

    *pI830->last_3d = LAST_3D_RENDER;

    src_pitch = intel_get_pixmap_pitch(pSrc);
    if (i830_pixmap_tiled(pSrc)) {
        src_tiled = 1;
	src_tile_format = 0; /* Tiled X */
    }

    dst_pitch = intel_get_pixmap_pitch(pDst);
    if (i830_pixmap_tiled(pDst)) {
        dst_tiled = 1;
	dst_tile_format = 0; /* Tiled X */
    }

    if (pMask) {
	mask_pitch = intel_get_pixmap_pitch(pMask);
	if (i830_pixmap_tiled(pMask)) {
  	    mask_tiled = 1;
	    mask_tile_format = 0;
	}
    }
    pI830->scale_units[0][0] = pSrc->drawable.width;
    pI830->scale_units[0][1] = pSrc->drawable.height;

    pI830->transform[0] = pSrcPicture->transform;

    if (!pMask) {
	pI830->transform[1] = NULL;
	pI830->scale_units[1][0] = -1;
	pI830->scale_units[1][1] = -1;
	if (pI830->transform[0] && 
		i965_check_rotation_transform(pI830->transform[0]))
	    rotation_program = TRUE;
    } else {
	pI830->transform[1] = pMaskPicture->transform;
	if (pI830->transform[1])
	    I830FALLBACK("i965 mask transform not implemented!\n");
	pI830->scale_units[1][0] = pMask->drawable.width;
	pI830->scale_units[1][1] = pMask->drawable.height;
    }

    /* setup 3d pipeline state */
    if (pMask)
	sf_state_offset = offsetof (gen4_state_t, sf_state_mask);
    else if (rotation_program)
	sf_state_offset = offsetof (gen4_state_t, sf_state_rotation);
    else
	sf_state_offset = offsetof (gen4_state_t, sf_state);

    /* Because we only have a single static buffer for our state currently,
     * we have to sync before updating it every time.
     */
    vb = (void *)(surface_start_base + vb_offset);
    vb_index = 0;

    i965_get_blend_cntl(op, pMaskPicture, pDstPicture->format,
			&src_blend, &dst_blend);

    /* Set up the state buffer for the destination surface */
    dest_surf_state = (void *)(surface_start_base + dest_surf_offset);
    i965_get_dest_format(pDstPicture, &dst_format);
    dest_surf_state->ss0.surface_format = dst_format;

    if (pI830->use_ttm_batch) {
    	intelddx_batchbuffer_emit_pixmap(pDst,
				     DRM_BO_FLAG_MEM_TT | DRM_BO_FLAG_WRITE,
				     DRM_BO_MASK_MEM | DRM_BO_FLAG_WRITE,
				     pI830->exa965->surface_buf, dest_surf_offset + 4, 0);
    } else {
        dest_surf_state->ss1.base_addr = intel_get_pixmap_offset(pDst);
    }

    dest_surf_state->ss2.height = pDst->drawable.height - 1;
    dest_surf_state->ss2.width = pDst->drawable.width - 1;
    dest_surf_state->ss3.pitch = dst_pitch - 1;
    dest_surf_state->ss3.tile_walk = dst_tile_format;
    dest_surf_state->ss3.tiled_surface = dst_tiled;

    /* Set up the source surface state buffer */
    src_surf_state = (void *)(surface_start_base + src_surf_offset);
    src_surf_state->ss0.surface_format = i965_get_card_format(pSrcPicture);

    if (pI830->use_ttm_batch) {
        intelddx_batchbuffer_emit_pixmap(pSrc,
				 DRM_BO_FLAG_MEM_TT | DRM_BO_FLAG_READ,
				 DRM_BO_MASK_MEM | DRM_BO_FLAG_READ,
				 pI830->exa965->surface_buf, src_surf_offset + 4, 0);
    } else {
        src_surf_state->ss1.base_addr = intel_get_pixmap_offset(pSrc);
    }
    src_surf_state->ss2.width = pSrc->drawable.width - 1;
    src_surf_state->ss2.height = pSrc->drawable.height - 1;
    src_surf_state->ss3.pitch = src_pitch - 1;
    src_surf_state->ss3.tile_walk = src_tile_format;
    src_surf_state->ss3.tiled_surface = src_tiled;

    /* setup mask surface */
    if (pMask) {
	mask_surf_state = (void *)(surface_start_base + mask_surf_offset);
   	mask_surf_state->ss0.surface_format = i965_get_card_format(pMaskPicture);
        if (pI830->use_ttm_batch) {
	   intelddx_batchbuffer_emit_pixmap(pMask, 
				     DRM_BO_FLAG_MEM_TT | DRM_BO_FLAG_READ,
				     DRM_BO_MASK_MEM | DRM_BO_FLAG_READ,
				     pI830->exa965->surface_buf, mask_surf_offset + 4, 0);
        } else {
	    mask_surf_state->ss1.base_addr = intel_get_pixmap_offset(pMask);
	}
   	mask_surf_state->ss2.width = pMask->drawable.width - 1;
   	mask_surf_state->ss2.height = pMask->drawable.height - 1;
   	mask_surf_state->ss3.pitch = mask_pitch - 1;
	mask_surf_state->ss3.tile_walk = mask_tile_format;
	mask_surf_state->ss3.tiled_surface = mask_tiled;
    }

    binding_table = (void *)(surface_start_base +
			     binding_table_offset);
    /* Set up a binding table for our surfaces.  Only the PS will use it */
    binding_table[0] = dest_surf_offset;
    binding_table[1] = src_surf_offset;
    if (pMask)
   	binding_table[2] = mask_surf_offset;
    else
	binding_table[2] = 0;

    src_filter = sampler_state_filter_from_picture (pSrcPicture->filter);
    if (src_filter < 0)
	I830FALLBACK ("Bad filter 0x%x\n", pSrcPicture->filter);

    src_extend = sampler_state_extend_from_picture (pSrcPicture->repeat);

    if (pMaskPicture) {
	mask_filter = sampler_state_filter_from_picture (pMaskPicture->filter);
	if (mask_filter < 0)
	    I830FALLBACK ("Bad filter 0x%x\n", pMaskPicture->filter);

	mask_extend = sampler_state_extend_from_picture (pMaskPicture->repeat);
    } else {
	mask_filter = SAMPLER_STATE_FILTER_NEAREST;
	mask_extend = SAMPLER_STATE_EXTEND_NONE;
    }

    /* Set up the PS kernel (dispatched by WM) */
    if (pMask) {
	if (pMaskPicture->componentAlpha && 
	    PICT_FORMAT_RGB(pMaskPicture->format)) {
	    if (i965_blend_op[op].src_alpha) {
		wm_state_offset = offsetof (gen4_state_t,
					    wm_state_maskca_srcalpha
					    [src_filter]
					    [src_extend]
					    [mask_filter]
					    [mask_extend]);
	    } else {
		wm_state_offset = offsetof (gen4_state_t,
					    wm_state_maskca
					    [src_filter]
					    [src_extend]
					    [mask_filter]
					    [mask_extend]);
	    }
	} else {
	    wm_state_offset = offsetof (gen4_state_t,
					wm_state_masknoca
					[src_filter]
					[src_extend]
					[mask_filter]
					[mask_extend]);
	}
    } else if (rotation_program) {
	wm_state_offset = offsetof (gen4_state_t,
				    wm_state_rotation
				    [src_filter]
				    [src_extend]
				    [mask_filter]
				    [mask_extend]);
    } else {
	wm_state_offset = offsetof (gen4_state_t,
				    wm_state_nomask
				    [src_filter]
				    [src_extend]
				    [mask_filter]
				    [mask_extend]);
    }

    cc_state_offset = offsetof (gen4_state_t,
				cc_state[src_blend][dst_blend]);

    /* Any commands that don't change from one composite operation to
     * the next we simply emit once at the beginning of the entire
     * batch. */
    if (pI830->exa965->num_ops == 0)
	gen4_emit_batch_header (pScrn);

    {
	BEGIN_BATCH(22);
	/* Pipe control */
   	OUT_BATCH(BRW_PIPE_CONTROL |
		 BRW_PIPE_CONTROL_NOWRITE |
		 BRW_PIPE_CONTROL_IS_FLUSH |
		 2);
   	OUT_BATCH(0);			       /* Destination address */
   	OUT_BATCH(0);			       /* Immediate data low DW */
   	OUT_BATCH(0);			       /* Immediate data high DW */

	/* Binding table pointers */
   	OUT_BATCH(BRW_3DSTATE_BINDING_TABLE_POINTERS | 4);
   	OUT_BATCH(0); /* vs */
   	OUT_BATCH(0); /* gs */
   	OUT_BATCH(0); /* clip */
   	OUT_BATCH(0); /* sf */
	/* Only the PS uses the binding table */
   	OUT_BATCH(binding_table_offset); /* ps */

	/* The drawing rectangle clipping is always on.  Set it to values that
	 * shouldn't do any clipping.
	 */
   	OUT_BATCH(BRW_3DSTATE_DRAWING_RECTANGLE | 2); /* XXX 3 for BLC or CTG */
   	OUT_BATCH(0x00000000);	/* ymin, xmin */
	OUT_BATCH(DRAW_YMAX(pDst->drawable.height - 1) |
		 DRAW_XMAX(pDst->drawable.width - 1)); /* ymax, xmax */
   	OUT_BATCH(0x00000000);	/* yorigin, xorigin */

	/* skip the depth buffer */
	/* skip the polygon stipple */
	/* skip the polygon stipple offset */
	/* skip the line stipple */

	/* Set the pointers to the 3d pipeline state */
   	OUT_BATCH(BRW_3DSTATE_PIPELINED_POINTERS | 5);
	OUT_BATCH(offsetof (gen4_state_t, vs_state));  /* 32 byte aligned */
   	OUT_BATCH(BRW_GS_DISABLE);   /* disable GS, resulting in passthrough */
   	OUT_BATCH(BRW_CLIP_DISABLE); /* disable CLIP, resulting in passthrough */
	OUT_BATCH(sf_state_offset); /* 32 byte aligned */
	OUT_BATCH(wm_state_offset); /* 32 byte aligned */
	OUT_BATCH(cc_state_offset); /* 64 byte aligned */
	ADVANCE_BATCH();
    }
    {
        int nelem = pMask ? 3: 2;

   	BEGIN_BATCH(pMask?12:10);
	/* Set up the pointer to our vertex buffer */
   	OUT_BATCH(BRW_3DSTATE_VERTEX_BUFFERS | 3);
   	OUT_BATCH((0 << VB0_BUFFER_INDEX_SHIFT) |
	    	 VB0_VERTEXDATA |
	    	 ((4 * 2 * nelem) << VB0_BUFFER_PITCH_SHIFT));

	if (pI830->use_ttm_batch) {
	    OUT_RELOC(pI830->exa965->surface_buf, DRM_BO_FLAG_MEM_TT, vb_offset);

	} else {
	    OUT_BATCH(pI830->exa_965_state->offset + vb_offset);
	}

        OUT_BATCH(GEN4_MAX_VERTICES); // set max index
   	OUT_BATCH(0); // ignore for VERTEXDATA, but still there

	/* Set up our vertex elements, sourced from the single vertex buffer.
	 */
   	OUT_BATCH(BRW_3DSTATE_VERTEX_ELEMENTS | ((2 * nelem) - 1));
	/* vertex coordinates */
   	OUT_BATCH((0 << VE0_VERTEX_BUFFER_INDEX_SHIFT) |
		  VE0_VALID |
	    	 (BRW_SURFACEFORMAT_R32G32_FLOAT << VE0_FORMAT_SHIFT) |
	    	 (0 << VE0_OFFSET_SHIFT));
   	OUT_BATCH((BRW_VFCOMPONENT_STORE_SRC << VE1_VFCOMPONENT_0_SHIFT) |
	    	 (BRW_VFCOMPONENT_STORE_SRC << VE1_VFCOMPONENT_1_SHIFT) |
	     	 (BRW_VFCOMPONENT_STORE_1_FLT << VE1_VFCOMPONENT_2_SHIFT) |
	    	 (BRW_VFCOMPONENT_STORE_1_FLT << VE1_VFCOMPONENT_3_SHIFT) |
	    	 (4 << VE1_DESTINATION_ELEMENT_OFFSET_SHIFT));
	/* u0, v0 */
   	OUT_BATCH((0 << VE0_VERTEX_BUFFER_INDEX_SHIFT) |
	    	 VE0_VALID |
	    	 (BRW_SURFACEFORMAT_R32G32_FLOAT << VE0_FORMAT_SHIFT) |
	    	 (8 << VE0_OFFSET_SHIFT)); /* offset vb in bytes */
   	OUT_BATCH((BRW_VFCOMPONENT_STORE_SRC << VE1_VFCOMPONENT_0_SHIFT) |
	    	 (BRW_VFCOMPONENT_STORE_SRC << VE1_VFCOMPONENT_1_SHIFT) |
	    	 (BRW_VFCOMPONENT_NOSTORE << VE1_VFCOMPONENT_2_SHIFT) |
	    	 (BRW_VFCOMPONENT_NOSTORE << VE1_VFCOMPONENT_3_SHIFT) |
	    	 (8 << VE1_DESTINATION_ELEMENT_OFFSET_SHIFT)); /* VUE offset in dwords */
	/* u1, v1 */
   	if (pMask) {
	    OUT_BATCH((0 << VE0_VERTEX_BUFFER_INDEX_SHIFT) |
		     VE0_VALID |
		     (BRW_SURFACEFORMAT_R32G32_FLOAT << VE0_FORMAT_SHIFT) |
		     (16 << VE0_OFFSET_SHIFT));
	    OUT_BATCH((BRW_VFCOMPONENT_STORE_SRC << VE1_VFCOMPONENT_0_SHIFT) |
		     (BRW_VFCOMPONENT_STORE_SRC << VE1_VFCOMPONENT_1_SHIFT) |
		     (BRW_VFCOMPONENT_NOSTORE << VE1_VFCOMPONENT_2_SHIFT) |
		     (BRW_VFCOMPONENT_NOSTORE << VE1_VFCOMPONENT_3_SHIFT) |
		     (10 << VE1_DESTINATION_ELEMENT_OFFSET_SHIFT));
   	}

   	ADVANCE_BATCH();
    }

#ifdef I830DEBUG
    ErrorF("try to sync to show any errors...");
    I830Sync(pScrn);
#endif
    return TRUE;
}
		       
void
i965_composite(PixmapPtr pDst, int srcX, int srcY, int maskX, int maskY,
	       int dstX, int dstY, int w, int h)
{
    ScrnInfoPtr pScrn = xf86Screens[pDst->drawable.pScreen->myNum];
    I830Ptr pI830 = I830PTR(pScrn);
    Bool has_mask;
    float src_x[3], src_y[3], mask_x[3], mask_y[3];
    int i;

    i830_get_transformed_coordinates(srcX, srcY,
				     pI830->transform[0],
				     &src_x[0], &src_y[0]);
    i830_get_transformed_coordinates(srcX, srcY + h,
				     pI830->transform[0],
				     &src_x[1], &src_y[1]);
    i830_get_transformed_coordinates(srcX + w, srcY + h,
				     pI830->transform[0],
				     &src_x[2], &src_y[2]);

    if (pI830->scale_units[1][0] == -1 || pI830->scale_units[1][1] == -1) {
	has_mask = FALSE;
    } else {
	has_mask = TRUE;
	i830_get_transformed_coordinates(maskX, maskY,
					 pI830->transform[1],
					 &mask_x[0], &mask_y[0]);
	i830_get_transformed_coordinates(maskX, maskY + h,
					 pI830->transform[1],
					 &mask_x[1], &mask_y[1]);
	i830_get_transformed_coordinates(maskX + w, maskY + h,
					 pI830->transform[1],
					 &mask_x[2], &mask_y[2]);
    }

    /* Wait for any existing composite rectangles to land before we overwrite
     * the VB with the next one.
     */
    if ((vb_index + 18) > GEN4_MAX_VERTICES) {
      ErrorF("vb index exceeded maximum bailing...");
      return;
    }

    i = vb_index;
    /* rect (x2,y2) */
    vb[i++] = (float)(dstX + w);
    vb[i++] = (float)(dstY + h);
    vb[i++] = src_x[2] / pI830->scale_units[0][0];
    vb[i++] = src_y[2] / pI830->scale_units[0][1];
    if (has_mask) {
        vb[i++] = mask_x[2] / pI830->scale_units[1][0];
        vb[i++] = mask_y[2] / pI830->scale_units[1][1];
    }

    /* rect (x1,y2) */
    vb[i++] = (float)dstX;
    vb[i++] = (float)(dstY + h);
    vb[i++] = src_x[1] / pI830->scale_units[0][0];
    vb[i++] = src_y[1] / pI830->scale_units[0][1];
    if (has_mask) {
        vb[i++] = mask_x[1] / pI830->scale_units[1][0];
        vb[i++] = mask_y[1] / pI830->scale_units[1][1];
    }

    /* rect (x1,y1) */
    vb[i++] = (float)dstX;
    vb[i++] = (float)dstY;
    vb[i++] = src_x[0] / pI830->scale_units[0][0];
    vb[i++] = src_y[0] / pI830->scale_units[0][1];
    if (has_mask) {
        vb[i++] = mask_x[0] / pI830->scale_units[1][0];
        vb[i++] = mask_y[0] / pI830->scale_units[1][1];
    }

    {
      BEGIN_BATCH(6);
      OUT_BATCH(BRW_3DPRIMITIVE |
	       BRW_3DPRIMITIVE_VERTEX_SEQUENTIAL |
	       (_3DPRIM_RECTLIST << BRW_3DPRIMITIVE_TOPOLOGY_SHIFT) |
	       (0 << 9) |  /* CTG - indirect vertex count */
	       4);
      OUT_BATCH(3);  /* vertex count per instance */
      OUT_BATCH(vb_index); /* start vertex offset */
      OUT_BATCH(1); /* single instance - mbz in docs */
      OUT_BATCH(0); /* start instance location */
      OUT_BATCH(0); /* index buffer offset, ignored */
      ADVANCE_BATCH();
    }

    vb_index = i;

    pI830->exa965->num_ops++;

#ifdef I830DEBUG
    ErrorF("sync after 3dprimitive");
    I830Sync(pScrn);
#endif
    /* we must be sure that the pipeline is flushed before next exa draw,
       because that will be new state, binding state and instructions*/
    /* Mark sync so we can wait for it before setting up the VB on the next
     * rectangle.
     */
}

void i965_done_composite(PixmapPtr pDst)
{
    ScrnInfoPtr pScrn = xf86Screens[pDst->drawable.pScreen->myNum];
    I830Ptr pI830 = I830PTR(pScrn);

    {
	BEGIN_BATCH(4);
   	OUT_BATCH(BRW_PIPE_CONTROL |
	    BRW_PIPE_CONTROL_NOWRITE |
	    BRW_PIPE_CONTROL_WC_FLUSH |
	    BRW_PIPE_CONTROL_IS_FLUSH |
	    (1 << 10) |  /* XXX texture cache flush for BLC/CTG */
	    2);
   	OUT_BATCH(0); /* Destination address */
   	OUT_BATCH(0); /* Immediate data low DW */
   	OUT_BATCH(0); /* Immediate data high DW */
	ADVANCE_BATCH();
    }

    if (pI830->use_ttm_batch && pI830->exa965->num_ops >= GEN4_MAX_OPS) {
	intelddx_batchbuffer_flush(pI830->batch);
    }
}

static struct i965_exastate_buffer *
i965_exastate_alloc(ScrnInfoPtr pScrn)
{
    struct i965_exastate_buffer *state = calloc(sizeof(*state), 1);

    state->pScrn = pScrn;
    state->num_ops = 0;
    i965_exastate_reset(state);
    return state;

}

int
i965_init_exa_state(ScrnInfoPtr pScrn)
{
    I830Ptr pI830 = I830PTR(pScrn);

    if (pI830->use_ttm_batch) {
	pI830->exa965 = i965_exastate_alloc(pScrn);
    } else {
	void *map = pI830->FbBase + pI830->exa_965_state->offset;
	gen4_state_init ((void *) map);
    }

    return 0;
}
