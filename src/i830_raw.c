/**************************************************************************

 Copyright 2006 Dave Airlie <airlied@linux.ie>
 
 Dervied from intelfhw.c licenesd under MIT/XFree86.
 Copyright © 2002, 2003 David Dawes <dawes@xfree86.org>
 
Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
on the rights to use, copy, modify, merge, publish, distribute, sub
license, and/or sell copies of the Software, and to permit persons to whom
the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice (including the next
paragraph) shall be included in all copies or substantial portions of the
Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
THE COPYRIGHT HOLDERS AND/OR THEIR SUPPLIERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
USE OR OTHER DEALINGS IN THE SOFTWARE.

**************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "xf86.h"
#include "xf86_ansic.h"
#include "xf86_OSproc.h"
#include "xf86Resources.h"
#include "xf86RAC.h"
#include "xf86cmap.h"
#include "compiler.h"
#include "mibstore.h"
#include "vgaHW.h"
#include "mipointer.h"
#include "micmap.h"
#include "shadowfb.h"
#include <X11/extensions/randr.h>
#include "fb.h"
#include "miscstruct.h"
#include "xf86xv.h"
#include <X11/extensions/Xv.h>
#include "shadow.h"
#include "i830.h"


#ifdef XF86DRI
#include "dri.h"
#endif

#define ROUND_UP_TO(x, y)	(((x) + (y) - 1) / (y) * (y))

#define PIPE_A			0
#define PIPE_B			1

#define PLLS_I8xx 0
#define PLLS_I9xx 1
#define PLLS_MAX 2

struct pll_min_max {
	int min_m, max_m, min_m1, max_m1;
	int min_m2, max_m2, min_n, max_n;
	int min_p, max_p, min_p1, max_p1;
	int min_vco, max_vco, p_transition_clk, ref_clk;
	int p_inc_lo, p_inc_hi;
};

/* PLL parameters (these are for 852GM/855GM/865G, check earlier chips). */
/* Clock values are in units of kHz */
/* I9xx needs LVDS clocks as well and possibly tv-out */
static struct pll_min_max plls[PLLS_MAX] = {
  { 108, 140, 18, 26,
    6, 16, 3, 16,
    4, 128, 0, 31,
    930000, 1400000, 165000, 48000,
    4, 2,
  }, //I8xx

  { 75, 120, 10, 20,
    5, 9, 4, 7,
    5, 80, 1, 8,
    1250000, 2800000, 200000, 96000,
    10, 5,
  }  //I9xx
};

static CARD32 calc_vclock(CARD32 index, CARD32 m1, CARD32 m2, CARD32 n, CARD32 p1, CARD32 p2)
{
  struct pll_min_max *pll = &plls[index];
  CARD32 m;
  CARD32 vco;
  CARD32 p;

  m = ( 5 * (m1 + 2) ) + ( m2 + 2 );
  n += 2;
  vco = pll->ref_clk * m / n;

  if (index == PLLS_I8xx) {
    p = ((p1 + 2) * (1 << (p2 + 1)));
  } else {
    p = ((p1) * (p2 ? 5 : 10));
  }
  return vco / p;
}

static CARD32 calc_vclock3(CARD32 index, CARD32 m, CARD32 n, CARD32 p)
{
  struct pll_min_max *pll = &plls[index];
  
  return pll->ref_clk * m / n / p;
}

/* Split the M parameter into M1 and M2. */
static int
splitm(int index, unsigned int m, CARD32 *retm1, CARD32 *retm2)
{
	int m1, m2;
	int testm;
	struct pll_min_max *pll = &plls[index];

	/* no point optimising too much - brute force m */
	for (m1 = pll->min_m1; m1 < pll->max_m1+1; m1++)
	{
	  for (m2 = pll->min_m2; m2 < pll->max_m2+1; m2++)
	  {
	    testm  = ( 5 * ( m1 + 2 )) + (m2 + 2);
	    if (testm == m)
	    {
		*retm1 = (unsigned int)m1;
		*retm2 = (unsigned int)m2;	      
		return 0;
	    }
	  }
	}
	return 1;
}


/* Split the P parameter into P1 and P2. */
static int
splitp(int index, unsigned int p, CARD32 *retp1, CARD32 *retp2)
{
	int p1, p2;
	struct pll_min_max *pll = &plls[index];	
	/* deal with i9xx plls first - incomplete but mostly working */
	if (index == PLLS_I9xx)
	{
		p2 = (p % 10) ? 1 : 0;
		
		p1 = p / (p2 ? 5 : 10);
		
		*retp1 = (unsigned int)p1;
		*retp2 = (unsigned int)p2;
		return 0;
	}

	if (p % 4 == 0)
		p2 = 1;
	else
		p2 = 0;
	p1 = (p / (1 << (p2 + 1))) - 2;
	if (p % 4 == 0 && p1 < pll->min_p1) {
		p2 = 0;
		p1 = (p / (1 << (p2 + 1))) - 2;
	}
	if (p1  < pll->min_p1 || p1 > pll->max_p1 || (p1 + 2) * (1 << (p2 + 1)) != p) {
		return 1;
	} else {
		*retp1 = (unsigned int)p1;
		*retp2 = (unsigned int)p2;
		return 0;
	}
}

static int
calc_pll_params(int index, int clock, 
		     CARD32 *retm1, CARD32 *retm2, 
		     CARD32 *retn, CARD32 *retp1, CARD32 *retp2, 
		     CARD32 *retclock, int *flags)
{
        CARD32 m1, m2, n, p1, p2, n1, testm;
        CARD32 f_vco, p, p_best = 0, m, f_out;
	CARD32 err_max, err_target, err_best = 10000000;
	CARD32 n_best = 0, m_best = 0, f_best, f_err;
	CARD32 p_min, p_max, p_inc, div_max;
	int ret;
	struct pll_min_max *pll = &plls[index];

	/* Accept 0.5% difference, but aim for 0.1% */
	err_max = 5 * clock / 1000;
	err_target = clock / 1000;

	DPRINTF(PFX, "Clock is %d\n", clock);

 	div_max = pll->max_vco / clock;
	p_inc = (clock <= pll->p_transition_clk) ? pll->p_inc_lo : pll->p_inc_hi;
	p_min = p_inc;
	p_max = ROUND_DOWN_TO(div_max, p_inc);
	if (p_min < pll->min_p)
		p_min = pll->min_p;
	if (p_max > pll->max_p)
		p_max = pll->max_p;

	if (clock < pll->ref_clk/2)
	{
	  *flags |= I830_MFLAG_DOUBLE;
	}

	DPRINTF(PFX, "p range is %d-%d (%d)\n", p_min, p_max, p_inc);

	p = p_min;
	do {
		ret = splitp(index, p, &p1, &p2);
		if (ret) {
		  DPRINTF(PFX, "cannot split p = %d\n", p);
			p += p_inc;
			continue;
		}
		n = pll->min_n;
		f_vco = clock * p;

		do {
			m = ROUND_UP_TO(f_vco * n, pll->ref_clk) / pll->ref_clk;
			if (m < pll->min_m)
				m = pll->min_m + 1;
			if (m > pll->max_m)
				m = pll->max_m - 1;
			for (testm = m - 1; testm <= m; testm++) {
				f_out = calc_vclock3(index, testm, n, p);
				if (splitm(index, testm, &m1, &m2)) {
					DPRINTF(PFX, "cannot split m = %d\n", testm);
					n++;
					continue;
				}
				if (clock > f_out)
					f_err = clock - f_out;
				else /* slightly bias the error for bigger clocks */
					f_err = f_out - clock + 1;

				if (f_err < err_best) {
					m_best = testm;
					n_best = n;
					p_best = p;
					f_best = f_out;
					err_best = f_err;
				}
			}
			n++;
		} while ((n <= pll->max_n) && (f_out >= clock));
		p += p_inc;
	} while (p <= p_max);

	if (!m_best) {
	  DPRINTF(PFX,"cannot find parameters for clock %d\n", clock);
		return 1;
	}
	m = m_best;
	n = n_best;
	p = p_best;
	splitm(index, m, &m1, &m2);
	splitp(index, p, &p1, &p2);
	n1 = n - 2;

	DPRINTF(PFX, "m, n, p: %d (%d,%d), %d (%d), %d (%d,%d), "
		"f: %d (%d), VCO: %d\n",
		m, m1, m2, n, n1, p, p1, p2,
		calc_vclock3(index, m, n, p), calc_vclock(index, m1, m2, n1, p1, p2),
		calc_vclock3(index, m, n, p) * p);
	*retm1 = m1;
	*retm2 = m2;
	*retn = n1;
	*retp1 = p1;
	*retp2 = p2;
	*retclock = calc_vclock(index, m1, m2, n1, p1, p2);

	return 0;
}

static int
check_overflow(ScrnInfoPtr pScrn, CARD32 value, CARD32 limit, const char *description)
{
  if (value > limit) {
    xf86DrvMsg(pScrn->scrnIndex, X_WARNING,
	       "%s value %d exceeds limit %d\n",
	       description, value, limit);
    return 1;
  }
  return 0;
}

/* It is assumed that hw is filled in with the initial state information. */
int
I830RawSetHw(ScrnInfoPtr pScrn, DisplayModePtr pMode)
{
  I830Ptr pI830 = I830PTR(pScrn);
  I830RegPtr hw = &pI830->ModeReg;
  int pipe = PIPE_A;
  unsigned long Start;
  CARD32 *dpll, *fp0, *fp1;
  CARD32 m1, m2, n, p1, p2, clock_target, clock;
  CARD32 hsync_start, hsync_end, hblank_start, hblank_end, htotal, hactive;
  CARD32 vsync_start, vsync_end, vblank_start, vblank_end, vtotal, vactive;
  CARD32 vsync_pol, hsync_pol;
  CARD32 *vs, *vb, *vt, *hs, *hb, *ht, *ss, *pipe_conf;
  int index;
  int displays = pI830->operatingDevices;
  int ret, i;
  
  index = IS_I9XX(pI830) ? PLLS_I9xx : PLLS_I8xx;

  /* Disable VGA */
  hw->vgacntrl |= VGA_CNTRL_DISABLE;

  if (pI830->pipeEnabled[1])
    pipe = PIPE_B;
  
  /* Set which pipe's registers will be set. */
  if (pipe == PIPE_B) {
    dpll = &hw->dpll_b;
    fp0 = &hw->fpb0;
    fp1 = &hw->fpb1;
    hs = &hw->hsync_b;
    hb = &hw->hblank_b;
    ht = &hw->htotal_b;
    vs = &hw->vsync_b;
    vb = &hw->vblank_b;
    vt = &hw->vtotal_b;
    ss = &hw->pipe_src_b;
    pipe_conf = &hw->pipe_b_conf;
  } else {
    dpll = &hw->dpll_a;
    fp0 = &hw->fpa0;
    fp1 = &hw->fpa1;
    hs = &hw->hsync_a;
    hb = &hw->hblank_a;
    ht = &hw->htotal_a;
    vs = &hw->vsync_a;
    vb = &hw->vblank_a;
    vt = &hw->vtotal_a;
    ss = &hw->pipe_src_a;
    pipe_conf = &hw->pipe_a_conf;
  }
  
  /* Use ADPA register for sync control. */
  hw->adpa &= ~ADPA_USE_VGA_HVPOLARITY;
  
  /* sync polarity */
  hsync_pol = (pMode->Flags & V_PHSYNC) ?
    ADPA_HSYNC_ACTIVE_HIGH : ADPA_HSYNC_ACTIVE_LOW;
  vsync_pol = (pMode->Flags & V_PVSYNC) ?
    ADPA_VSYNC_ACTIVE_HIGH : ADPA_VSYNC_ACTIVE_LOW;
  hw->adpa &= ~(ADPA_HSYNC_ACTIVE_HIGH | ADPA_VSYNC_ACTIVE_HIGH);
  hw->adpa |= (hsync_pol | vsync_pol);
  
  /* Connect correct pipe to the analog port DAC */
  hw->adpa &= ~(ADPA_PIPE_SELECT_MASK);
  hw->adpa |= (pipe==PIPE_B) ? ADPA_PIPE_B_SELECT : 0;
  
  /* Set DPMS state to D0 (on) */
  hw->adpa &= ~(ADPA_VSYNC_CNTL_DISABLE | ADPA_HSYNC_CNTL_DISABLE);
  //  hw->adpa |= ADPA_DPMS_D0;
  
  hw->adpa |= ADPA_DAC_ENABLE;
  
  *dpll |= (DPLL_VCO_ENABLE | DPLL_VGA_MODE_DISABLE);
  *dpll &= ~(DPLL_RATE_SELECT_MASK | DPLL_REFERENCE_SELECT_MASK);
  *dpll |= (DPLL_REFERENCE_DEFAULT | DPLL_RATE_SELECT_FP0);

  /* Desired clock in kHz */
  clock_target = pMode->Clock;

  ret=calc_pll_params(index, clock_target, &m1, &m2, &n, &p1, &p2, &clock, &pMode->PrivFlags);
  if (ret) {
	  ErrorF("calc_pll_params failed\n");
	  return FALSE;
  }
  
  /* Check for overflow. */
  if (check_overflow(pScrn, p1, DPLL_P1_MASK, "PLL P1 parameter"))
    return FALSE;
  if (check_overflow(pScrn, p2, DPLL_P2_MASK, "PLL P2 parameter"))
    return FALSE;
  if (check_overflow(pScrn, m1, FP_DIVISOR_MASK, "PLL M1 parameter"))
    return FALSE;
  if (check_overflow(pScrn, m2, FP_DIVISOR_MASK, "PLL M2 parameter"))
    return FALSE;
  if (check_overflow(pScrn, n, FP_DIVISOR_MASK, "PLL N parameter"))
    return FALSE;
  
  *dpll &= ~DPLL_P1_FORCE_DIV2;
  *dpll &= ~((DPLL_P2_MASK << DPLL_P2_SHIFT) |
	     (DPLL_P1_MASK << DPLL_P1_SHIFT));


  if (IS_I9XX(pI830))
  {
	  CARD32 tmpp1;

	  *dpll |= 0x4000000;
	  *dpll |= p2 << DPLL_I9XX_P2_SHIFT;
	  
	  tmpp1 = (1 << (p1 - 1));
	  *dpll |= tmpp1 << 16;
  }
  else
  {
	  *dpll |= (p2 << DPLL_P2_SHIFT) | (p1 << DPLL_P1_SHIFT);
  }

  *fp0 = (n << FP_N_DIVISOR_SHIFT) |
    (m1 << FP_M1_DIVISOR_SHIFT) |
    (m2 << FP_M2_DIVISOR_SHIFT);
  //  *fp1 = *fp0;
  
  *dpll &= ~DPLL_2X_CLOCK_ENABLE;
  for (i=0; i<pI830->num_outputs; i++)
  {
    if (pI830->output[i].sdvo_drv)
      *dpll |= DPLL_2X_CLOCK_ENABLE;
  }
  /* leave these alone for now */

  //  hw->dvob &= ~DVO_ENABLE;
  //hw->dvoc &= ~DVO_ENABLE;
  //  hw->dvoc |= 0x4084 | DVO_ENABLE;
  //hw->dvob |= DVO_ENABLE;
  if (pMode->PrivFlags & I830_MFLAG_DOUBLE)
    hw->dvoc = 0x81c80080;
  else
    hw->dvoc = 0x80480080;

  /* Use display plane A. */
  hw->disp_a_ctrl |= DISPLAY_PLANE_ENABLE;
  hw->disp_a_ctrl &= ~DISPPLANE_GAMMA_ENABLE;
  hw->disp_a_ctrl &= ~DISPPLANE_PIXFORMAT_MASK;
  switch (pScrn->bitsPerPixel) {
  case 8:
    hw->disp_a_ctrl |= DISPPLANE_8BPP | DISPPLANE_GAMMA_ENABLE;
    break;
  case 15:
    hw->disp_a_ctrl |= DISPPLANE_15_16BPP;
    break;
  case 16:
    hw->disp_a_ctrl |= DISPPLANE_16BPP;
    break;
  case 24:
  case 32:
    hw->disp_a_ctrl |= DISPPLANE_32BPP_NO_ALPHA;
    break;
  }
  hw->disp_a_ctrl &= ~(DISPPLANE_SEL_PIPE_MASK);
  hw->disp_a_ctrl |= (pipe == PIPE_B ? DISPPLANE_SEL_PIPE_B : DISPPLANE_SEL_PIPE_A);
  
  /* Set CRTC registers. */
  hactive = pMode->CrtcHDisplay;
  hsync_start = pMode->CrtcHSyncStart;
  hsync_end = pMode->CrtcHSyncEnd;
  htotal = pMode->CrtcHTotal;
  hblank_start = pMode->CrtcHBlankStart;
  hblank_end = pMode->CrtcHBlankEnd;
  
  DPRINTF(PFX, "H: act %d, ss %d, se %d, tot %d bs %d, be %d\n",
	  hactive, hsync_start, hsync_end, htotal, hblank_start,
	  hblank_end);
  
  vactive = pMode->CrtcVDisplay;
  vsync_start = pMode->CrtcVSyncStart;
  vsync_end = pMode->CrtcVSyncEnd;
  vtotal = pMode->CrtcVTotal;
  vblank_start = pMode->CrtcVBlankStart;
  vblank_end = pMode->CrtcVBlankEnd;
  
  DPRINTF(PFX, "V: act %d, ss %d, se %d, tot %d bs %d, be %d\n",
	  vactive, vsync_start, vsync_end, vtotal, vblank_start,
	  vblank_end);

  /* Adjust for register values, and check for overflow. */
  hactive--;
  if (check_overflow(pScrn, hactive, HACTIVE_MASK, "CRTC hactive"))
    return FALSE;
  hsync_start--;
  if (check_overflow(pScrn, hsync_start, HSYNCSTART_MASK, "CRTC hsync_start"))
    return FALSE;
  hsync_end--;
  if (check_overflow(pScrn, hsync_end, HSYNCEND_MASK, "CRTC hsync_end"))
    return FALSE;
  htotal--;
  if (check_overflow(pScrn, htotal, HTOTAL_MASK, "CRTC htotal"))
    return FALSE;
  hblank_start--;
  if (check_overflow(pScrn, hblank_start, HBLANKSTART_MASK, "CRTC hblank_start"))
    return FALSE;
  hblank_end--;
  if (check_overflow(pScrn, hblank_end, HBLANKEND_MASK, "CRTC hblank_end"))
    return FALSE;
  
  vactive--;
  if (check_overflow(pScrn, vactive, VACTIVE_MASK, "CRTC vactive"))
    return FALSE;
  vsync_start--;
  if (check_overflow(pScrn, vsync_start, VSYNCSTART_MASK, "CRTC vsync_start"))
    return FALSE;
  vsync_end--;
  if (check_overflow(pScrn, vsync_end, VSYNCEND_MASK, "CRTC vsync_end"))
    return FALSE;
  vtotal--;
  if (check_overflow(pScrn, vtotal, VTOTAL_MASK, "CRTC vtotal"))
    return FALSE;
  vblank_start--;
  if (check_overflow(pScrn, vblank_start, VBLANKSTART_MASK, "CRTC vblank_start"))
    return FALSE;
  vblank_end--;
  if (check_overflow(pScrn, vblank_end, VBLANKEND_MASK, "CRTC vblank_end"))
    return FALSE;
  
  *ht = (htotal << HTOTAL_SHIFT) | (hactive << HACTIVE_SHIFT);
  *hb = (hblank_start << HBLANKSTART_SHIFT) |
    (hblank_end << HSYNCEND_SHIFT);
  *hs = (hsync_start << HSYNCSTART_SHIFT) | (hsync_end << HSYNCEND_SHIFT);
  
  *vt = (vtotal << VTOTAL_SHIFT) | (vactive << VACTIVE_SHIFT);
  *vb = (vblank_start << VBLANKSTART_SHIFT) |
    (vblank_end << VSYNCEND_SHIFT);
  *vs = (vsync_start << VSYNCSTART_SHIFT) | (vsync_end << VSYNCEND_SHIFT);
  *ss = (hactive << SRC_SIZE_HORIZ_SHIFT) |
    (vactive << SRC_SIZE_VERT_SHIFT);

  hw->swf1x[3] = 0x58580000;
  *pipe_conf |= PIPEACONF_ENABLE;

  /* Set the palette to 8-bit mode. */
  //  *pipe_conf &= ~PIPEACONF_GAMMA;
  return TRUE;
}

/* Program a (non-VGA) video mode. */
int
I830ProgramModeReg(ScrnInfoPtr pScrn, DisplayModePtr pMode)
{
  I830Ptr pI830 = I830PTR(pScrn);
  I830RegPtr hw = &pI830->ModeReg;
	int pipe = PIPE_A;
	CARD32 tmp, tmp_val[3];
	const CARD32 *dpll, *fp0, *fp1, *pipe_conf;
	const CARD32 *hs, *ht, *hb, *vs, *vt, *vb, *ss;
	CARD32 dpll_reg, fp0_reg, fp1_reg, pipe_conf_reg;
	CARD32 hsync_reg, htotal_reg, hblank_reg;
	CARD32 vsync_reg, vtotal_reg, vblank_reg;
	CARD32 pipe_src_reg;
	int count, i;

	/* Assume single pipe, display plane A, analog CRT. */

	/* Disable VGA */
	OUTREG(VGACNTRL, hw->vgacntrl);

	/* Check whether pipe A or pipe B is enabled. */
	if (hw->pipe_a_conf & PIPEACONF_ENABLE)
	  pipe = PIPE_A;
	else if (hw->pipe_b_conf & PIPEBCONF_ENABLE)
	  pipe = PIPE_B;

	if (pipe == PIPE_B) {
		dpll = &hw->dpll_b;
		fp0 = &hw->fpb0;
		fp1 = &hw->fpb1;
		pipe_conf = &hw->pipe_b_conf;
		hs = &hw->hsync_b;
		hb = &hw->hblank_b;
		ht = &hw->htotal_b;
		vs = &hw->vsync_b;
		vb = &hw->vblank_b;
		vt = &hw->vtotal_b;
		ss = &hw->pipe_src_b;
		dpll_reg = DPLL_B;
		fp0_reg = FPB0;
		fp1_reg = FPB1;
		pipe_conf_reg = PIPEBCONF;
		hsync_reg = HSYNC_B;
		htotal_reg = HTOTAL_B;
		hblank_reg = HBLANK_B;
		vsync_reg = VSYNC_B;
		vtotal_reg = VTOTAL_B;
		vblank_reg = VBLANK_B;
		pipe_src_reg = PIPEBSRC;
	} else {
		dpll = &hw->dpll_a;
		fp0 = &hw->fpa0;
		fp1 = &hw->fpa1;
		pipe_conf = &hw->pipe_a_conf;
		hs = &hw->hsync_a;
		hb = &hw->hblank_a;
		ht = &hw->htotal_a;
		vs = &hw->vsync_a;
		vb = &hw->vblank_a;
		vt = &hw->vtotal_a;
		ss = &hw->pipe_src_a;
		dpll_reg = DPLL_A;
		fp0_reg = FPA0;
		fp1_reg = FPA1;
		pipe_conf_reg = PIPEACONF;
		hsync_reg = HSYNC_A;
		htotal_reg = HTOTAL_A;
		hblank_reg = HBLANK_A;
		vsync_reg = VSYNC_A;
		vtotal_reg = VTOTAL_A;
		vblank_reg = VBLANK_A;
		pipe_src_reg = PIPEASRC;
	}

	/* turn off pipe */
	tmp = INREG(pipe_conf_reg);
	tmp &= ~PIPEACONF_ENABLE;
	OUTREG(pipe_conf_reg, tmp);
	
	count = 0;
	do{
	  tmp_val[count%3] = INREG(0x70000);
	  if ((tmp_val[0] == tmp_val[1]) && (tmp_val[1]==tmp_val[2]))
	    break;
	  count++;
	  usleep(1);
	  if (count % 200 == 0)
	  {
	    tmp = INREG(pipe_conf_reg);
	    tmp &= ~PIPEACONF_ENABLE;
	    OUTREG(pipe_conf_reg, tmp);
	  }
	} while(count < 2000);

	/* inreg dvoc */
	OUTREG(DVOC, INREG(DVOC) & ~DVO_ENABLE);
	
	OUTREG(ADPA, INREG(ADPA) & ~ADPA_DAC_ENABLE);

	/* Disable Sync */
	tmp = INREG(ADPA);
	tmp &= ~(ADPA_VSYNC_CNTL_DISABLE | ADPA_HSYNC_CNTL_DISABLE);
	tmp |= ADPA_VSYNC_CNTL_DISABLE | ADPA_HSYNC_CNTL_DISABLE;
	OUTREG(ADPA, tmp);

	/* do some funky magic - xyzzy */
	OUTREG(0x61204, 0xabcd0000);

	/* turn off PLL */
	tmp = INREG(dpll_reg);
	tmp &= ~DPLL_VCO_ENABLE;
	OUTREG(dpll_reg, tmp);

	/* Set PLL parameters */
	OUTREG(fp0_reg, *fp0);
	OUTREG(fp1_reg, *fp1);

	/* Enable PLL */
	OUTREG(dpll_reg, *dpll);

	/* Set DVOs B/C */
	OUTREG(DVOB, hw->dvob);
	OUTREG(DVOC, hw->dvoc);

	/* undo funky magic */
	OUTREG(0x61204, 0x00000000);

	OUTREG(ADPA, INREG(ADPA) | ADPA_DAC_ENABLE);

	/* Set ADPA */
	OUTREG(ADPA, (hw->adpa & ~(ADPA_VSYNC_CNTL_DISABLE|ADPA_HSYNC_CNTL_DISABLE)) | (ADPA_VSYNC_CNTL_DISABLE|ADPA_HSYNC_CNTL_DISABLE));

	/* Set pipe parameters */
	OUTREG(hsync_reg, *hs);
	OUTREG(hblank_reg, *hb);
	OUTREG(htotal_reg, *ht);
	OUTREG(vsync_reg, *vs);
	OUTREG(vblank_reg, *vb);
	OUTREG(vtotal_reg, *vt);
	OUTREG(pipe_src_reg, *ss);

	OUTREG(DSPASIZE, (pMode->HDisplay - 1) | ((pMode->VDisplay - 1) << 16));
	for (count = 0, i = SWF10; i<SWF16; i+=4)
	  OUTREG(i, hw->swf1x[count++]);

	/* Enable pipe */
	OUTREG(pipe_conf_reg, *pipe_conf | PIPEACONF_ENABLE);

	/* Enable sync */
	tmp = INREG(ADPA);
	tmp &= ~(ADPA_VSYNC_CNTL_DISABLE|ADPA_HSYNC_CNTL_DISABLE);
	//	tmp |= ADPA_DPMS_D0;
	OUTREG(ADPA, tmp);

	OUTREG(DSPACNTR, hw->disp_a_ctrl);
	OUTREG(DSPBCNTR, hw->disp_b_ctrl);
	I830SetupDSPRegisters(pScrn, pMode);

	return TRUE;
}

Bool
I830RawSaveState(ScrnInfoPtr pScrn, I830RegPtr hw)
{
  I830Ptr pI830 = I830PTR(pScrn);
  int i, count;
  hw->vga0_divisor = INREG(VCLK_DIVISOR_VGA0);
  hw->vga1_divisor = INREG(VCLK_DIVISOR_VGA1);
  hw->vga_pd = INREG(VCLK_POST_DIV);

  hw->dpll_a = INREG(DPLL_A);
  hw->dpll_b = INREG(DPLL_B);
  hw->fpa0 = INREG(FPA0);
  hw->fpa1 = INREG(FPA1);
  hw->fpb0 = INREG(FPB0);
  hw->fpb1 = INREG(FPB1);

  hw->htotal_a = INREG(HTOTAL_A);
  hw->hblank_a = INREG(HBLANK_A);
  hw->hsync_a = INREG(HSYNC_A);
  hw->vtotal_a = INREG(VTOTAL_A);
  hw->vblank_a = INREG(VBLANK_A);
  hw->vsync_a = INREG(VSYNC_A);
  hw->pipe_src_a = INREG(PIPEASRC);
  hw->bclrpat_a = INREG(BCLRPAT_A);
  hw->htotal_b = INREG(HTOTAL_B);
  hw->hblank_b = INREG(HBLANK_B);
  hw->hsync_b = INREG(HSYNC_B);
  hw->vtotal_b = INREG(VTOTAL_B);
  hw->vblank_b = INREG(VBLANK_B);
  hw->vsync_b = INREG(VSYNC_B);
  hw->pipe_src_b = INREG(PIPEBSRC);
  hw->bclrpat_b = INREG(BCLRPAT_B);

  hw->dvoa = INREG(DVOA);
  hw->dvob = INREG(DVOB);
  hw->dvoc = INREG(DVOC);
  hw->dvoa_srcdim = INREG(DVOA_SRCDIM);
  hw->dvob_srcdim = INREG(DVOB_SRCDIM);
  hw->dvoc_srcdim = INREG(DVOC_SRCDIM);
  hw->lvds = INREG(LVDS);

  
  hw->pipe_a_conf = INREG(PIPEACONF);
  hw->pipe_b_conf = INREG(PIPEBCONF);
  hw->disp_arb = INREG(DISPLAY_ARB);


  hw->disp_a_ctrl = INREG(DSPACNTR);
  hw->disp_b_ctrl = INREG(DSPBCNTR);
  hw->disp_a_base = INREG(DSPABASE);
  hw->disp_b_base = INREG(DSPBBASE);
  hw->disp_a_stride = INREG(DSPASTRIDE);
  hw->disp_b_stride = INREG(DSPBSTRIDE);

  hw->add_id = INREG(ADD_ID);
  
  hw->vgacntrl = INREG(VGACNTRL);  
  
  hw->fw_blc_0 = INREG(FWATER_BLC);
  hw->fw_blc_1 = INREG(FWATER_BLC2);

  for (count = 0, i = SWF10; i<SWF16; i+=4)
    hw->swf1x[count++] = INREG(i);

  return TRUE;
}

Bool
I830RawRestoreState(ScrnInfoPtr pScrn, I830RegPtr hw)
{
  I830Ptr pI830 = I830PTR(pScrn);

  OUTREG(VCLK_DIVISOR_VGA0, hw->vga0_divisor);
  OUTREG(VCLK_DIVISOR_VGA1, hw->vga1_divisor);
  OUTREG(VCLK_POST_DIV, hw->vga_pd);

  OUTREG(HTOTAL_A, hw->htotal_a);
  OUTREG(HBLANK_A, hw->hblank_a);
  OUTREG(HSYNC_A, hw->hsync_a);

  OUTREG(VTOTAL_A, hw->vtotal_a);
  OUTREG(VBLANK_A, hw->vblank_a);
  OUTREG(VSYNC_A, hw->vsync_a);

  OUTREG(PIPEASRC, hw->pipe_src_a);
	 
  OUTREG(DPLL_A, hw->dpll_a);
  OUTREG(DPLL_B, hw->dpll_b);
  OUTREG(FPA0, hw->fpa0);
  OUTREG(FPA1, hw->fpa1);
  OUTREG(FPB0, hw->fpa0);
  OUTREG(FPB1, hw->fpa1);

  OUTREG(PIPEACONF, hw->pipe_a_conf);
  OUTREG(PIPEBCONF, hw->pipe_b_conf);
  OUTREG(DISPLAY_ARB, hw->disp_arb);

  OUTREG(DSPACNTR, hw->disp_a_ctrl);
  OUTREG(DSPBCNTR, hw->disp_b_ctrl);
  OUTREG(DSPABASE, hw->disp_a_base);
  OUTREG(DSPBBASE, hw->disp_b_base);
  OUTREG(DSPASTRIDE, hw->disp_a_stride);
  OUTREG(DSPBSTRIDE, hw->disp_b_stride);

  OUTREG(VGACNTRL, hw->vgacntrl);
  
  return TRUE;
}

Bool
I830RawSwitchMode(int scrnIndex, DisplayModePtr mode, int flags)
{
  ScrnInfoPtr pScrn = xf86Screens[scrnIndex];

  xf86DrvMsg(scrnIndex, X_ERROR,
	     "called raw switch mode\n");

  if (I830RawSetHw(pScrn, mode))
    I830ProgramModeReg(pScrn, mode);


  return 0;

}

Bool
I830RawSetMode(ScrnInfoPtr pScrn, DisplayModePtr mode)
{
  int ret;
  I830Ptr pI830 = I830PTR(pScrn);
  Bool didLock = FALSE;
  int retry_count = 0;
  int i;

  DPRINTF(PFX, "RawSetMode");

  didLock = I830DRILock(pScrn);
  
  if (pI830->Clone) {
    pI830->CloneHDisplay = mode->HDisplay;
    pI830->CloneVDisplay = mode->VDisplay;
  }

  I830RawSetHw(pScrn, mode);

 retry:
  /* do SDVO setup */
  for (i = 0; i < pI830->num_outputs; i++)
  {
    if (pI830->output[i].sdvo_drv)
    {
      I830SDVOPreSetMode(pI830->output[i].sdvo_drv, mode);
    }
  }
  
  ret=I830ProgramModeReg(pScrn, mode);

  for (i = 0; i < pI830->num_outputs; i++)
  {
    if (pI830->output[i].sdvo_drv)
    {
      ret = I830SDVOPostSetMode(pI830->output[i].sdvo_drv, mode);
      /* if it didn't enable the DFP on the output */
      if ((ret==FALSE) && ((pI830->MonType1 & PIPE_DFP) == PIPE_DFP) && (retry_count<3))
      {
	retry_count++;
	goto retry;
      }
      else
	      ret = TRUE;
    }
  }
  if (didLock)
    I830DRIUnlock(pScrn);

  pScrn->vtSema = TRUE;

  return ret;

}


void I830DumpPLLRegisters(ScrnInfoPtr pScrn)
{
	I830Ptr pI830 = I830PTR(pScrn);
	struct pll_min_max *pll;
	int p1, p2, m1, m2, n, p, m, clock;
	CARD32 temp;

	if (IS_I9XX(pI830))
		pll = &plls[PLLS_I9xx];
	else
		pll = &plls[PLLS_I8xx];
	
	temp = INREG(DPLL_A);
	
	if (IS_I9XX(pI830)) {
		int tmpp1;

		tmpp1 = (temp >> DPLL_P1_SHIFT) & 0xff;

		switch (tmpp1)
		{
		case 0x1: p1 = 1; break;
		case 0x2: p1 = 2; break;
		case 0x4: p1 = 3; break;
		case 0x8: p1 = 4; break;
		case 0x10: p1 = 5; break;
		case 0x20: p1 = 6; break;
		case 0x40: p1 = 7; break;
		case 0x80: p1 = 8; break;
		default: break;
		}
		p2 = (temp >> DPLL_I9XX_P2_SHIFT) & DPLL_P2_MASK;
		p = ((p1) * (p2 ? 5 : 10));
	} else {
		p1 = (temp >> DPLL_P1_SHIFT) & DPLL_P1_MASK;
		p2 = (temp >> DPLL_P2_SHIFT) & DPLL_P2_MASK;
		p = (p1+2) * ( 1<< (p2 + 1));
	}
	
	ErrorF("DPLL A is %08X: p1 is %d p2 is %d p is %d\n", temp, p1, p2, p);
	temp = INREG(FPA0);
	n = (temp >> FP_N_DIVISOR_SHIFT) & FP_DIVISOR_MASK;
	m1 = (temp >> FP_M1_DIVISOR_SHIFT) & FP_DIVISOR_MASK;
	m2 = (temp >> FP_M2_DIVISOR_SHIFT) & FP_DIVISOR_MASK;
	m = (5 * ((m1) + 2) + ((m2) + 2));
	n += 2;
	clock = (((pll->ref_clk * m) / n) / p);
	
	ErrorF("FPA0 is %08X N is %d m1 is %d m2 is %d\n", temp, n, m1, m2);
	ErrorF("m %d n %d p %d clock %d\n", m, n, p, clock);
	
	temp = INREG(FPA1);
	n = (temp >> FP_N_DIVISOR_SHIFT) & FP_DIVISOR_MASK;
	m1 = (temp >> FP_M1_DIVISOR_SHIFT) & FP_DIVISOR_MASK;
	m2 = (temp >> FP_M2_DIVISOR_SHIFT) & FP_DIVISOR_MASK;
	m = (5 * ((m1) + 2) + ((m2) + 2));
	n += 2;
	clock = ((pll->ref_clk * m / n) / p);
	
	return;
}
