/*
 * Copyright © 2006 Intel Corporation
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
 *    Eric Anholt <eric@anholt.net>
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "xf86.h"
#include "i830.h"
#include "i830_modes.h"

/** @file
 * This file deals with creating lists of valid modes for an output device.
 *
 * It attempts to always have valid modes for the output in the list.  This is
 * unlike xf86Mode.c, which dumps all the modes it can find into the list and
 * then validates them and then prunes them out later.
 */

static int
i830GetModeListLen(DisplayModePtr first)
{
    DisplayModePtr mode;
    int i = 0;

    for (mode = first; mode != NULL && mode != first; mode = mode->next)
	i++;
    return i;
}

/**
 * i830SetModeToPanelParameters() fills a mode pointer with timing information
 * from the panel.
 *
 * Note that the blanking periods will be very strange for lower resolution
 * modes, but it is assumed that the driver will force the panel's fixed mode
 * anyway.
 */
static void
i830SetModeToPanelParameters(ScrnInfoPtr pScrn, DisplayModePtr pMode)
{
    I830Ptr pI830 = I830PTR(pScrn);

    pMode->HTotal     = pI830->panel_fixed_hactive;
    pMode->HSyncStart = pI830->panel_fixed_hactive +
			pI830->panel_fixed_hsyncoff;
    pMode->HSyncEnd   = pMode->HSyncStart + pI830->panel_fixed_hsyncwidth;
    pMode->VTotal     = pI830->panel_fixed_vactive;
    pMode->VSyncStart = pI830->panel_fixed_vactive +
			pI830->panel_fixed_vsyncoff;
    pMode->VSyncEnd   = pMode->VSyncStart + pI830->panel_fixed_vsyncwidth;
    pMode->Clock      = pI830->panel_fixed_clock;
}

#define ADD_NEW_TO_TAIL() do {			\
    new->next = NULL;				\
    new->prev = last;				\
						\
    if (last)					\
	last->next = new;			\
    last = new;					\
    if (!first)					\
	first = new;				\
} while (0)

/**
 * Returns true if the singly-linked list/circleq beginning at first contains
 * a mode displaying a resolution of x by y.
 */
static Bool
i830ModeListContainsSize(DisplayModePtr first, int x, int y)
{
    DisplayModePtr mode;

    for (mode = first; mode != NULL && mode != first; mode = mode->next) {
	if (mode->HDisplay == x && mode->VDisplay == y)
	    return TRUE;
    }

    return FALSE;
}

/**
 * FP mode validation routine for using panel fitting.
 *
 * Modes in the list will be in the order of user-selected modes, followed by
 * the panel's native mode and then defaut VESA mode sizes less than the panel
 * size.
 *
 * Returns a newly-allocated doubly-linked circular list of modes if any were
 * found, or NULL otherwise.
 */
DisplayModePtr
i830ValidateFPModes(ScrnInfoPtr pScrn, char **ppModeName)
{
    I830Ptr pI830 = I830PTR(pScrn);
    DisplayModePtr last = NULL;
    DisplayModePtr new = NULL;
    DisplayModePtr first = NULL;
    DisplayModePtr p;
    int i, width, height;

    /* We have a flat panel connected to the primary display, and we
     * don't have any DDC info.
     */
    for (i = 0; ppModeName[i] != NULL; i++) {
	if (sscanf(ppModeName[i], "%dx%d", &width, &height) != 2)
	    continue;

	/* Allow all non-standard modes as long as they do not exceed the
	 * native resolution of the panel.
	 */
	if (width < 320 || width > pI830->PanelXRes ||
	    height < 200 || height > pI830->PanelYRes) {
	    xf86DrvMsg(pScrn->scrnIndex, X_WARNING, "Mode %s is out of range.\n",
		       ppModeName[i]);
	    xf86DrvMsg(pScrn->scrnIndex, X_WARNING,
		       "Valid modes must be between 320x200-%dx%d\n",
		       pI830->PanelXRes, pI830->PanelYRes);
	    continue;
	}

	new = xnfcalloc(1, sizeof(DisplayModeRec));
	new->name = xnfalloc(strlen(ppModeName[i]) + 1);
	strcpy(new->name, ppModeName[i]);
	new->HDisplay = width;
	new->VDisplay = height;
	new->type |= M_T_USERDEF;

	i830SetModeToPanelParameters(pScrn, new);

	ADD_NEW_TO_TAIL();

	xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		   "Valid mode using panel fitting: %s\n", new->name);
    }

    /* Always add in the panel's native mode, for randr purposes. */
    if (pI830->PanelXRes != 0 && pI830->PanelYRes != 0 &&
	!i830ModeListContainsSize(first, pI830->PanelXRes, pI830->PanelYRes))
    {
	char stmp[32];

	new = xnfcalloc(1, sizeof (DisplayModeRec));
	snprintf(stmp, 32, "%dx%d", pI830->PanelXRes, pI830->PanelYRes);
	new->name = xnfalloc(strlen(stmp) + 1);
	strcpy(new->name, stmp);
	new->HDisplay = pI830->PanelXRes;
	new->VDisplay = pI830->PanelYRes;
	i830SetModeToPanelParameters(pScrn, new);
	new->type = M_T_DEFAULT;

	ADD_NEW_TO_TAIL();
    }

    /* Add in all unique default vesa mode sizes smaller than panel size.
     * Used for randr */
    for (p = pScrn->monitor->Modes; p && p->next; p = p->next->next) {
	if (p->HDisplay > pI830->PanelXRes || p->VDisplay > pI830->PanelYRes)
	    continue;

	if (i830ModeListContainsSize(first, p->HDisplay, p->VDisplay))
	    continue;

	new = xnfcalloc(1, sizeof(DisplayModeRec));
	new->name = xnfalloc(strlen(p->name) + 1);
	strcpy(new->name, p->name);
	new->HDisplay = p->HDisplay;
	new->VDisplay = p->VDisplay;
	i830SetModeToPanelParameters(pScrn, new);
	new->type |= M_T_DEFAULT;

	ADD_NEW_TO_TAIL();
    }

    /* Close the doubly-linked mode list */
    if (last) {
	last->next = first;
	first->prev = last;
    }

    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
	       "Total number of valid FP mode(s) found: %d\n",
	       i830GetModeListLen(first));

    return first;
}

/* Pulled from xf86Mode.c since it was static there.  Used for finding
 * the established modes from the mode list.
 */
static double
ModeVRefresh(DisplayModePtr mode)
{
    double refresh = 0.0;

    if (mode->VRefresh > 0.0)
	refresh = mode->VRefresh;
    else if (mode->HTotal > 0 && mode->VTotal > 0) {
	refresh = mode->Clock * 1000.0 / mode->HTotal / mode->VTotal;
	if (mode->Flags & V_INTERLACE)
	    refresh *= 2.0;
	if (mode->Flags & V_DBLSCAN)
	    refresh /= 2.0;
	if (mode->VScan > 1)
	    refresh /= (float)(mode->VScan);
    }
    return refresh;
}

/* From xf86DefMode{s,Set}.c */
extern DisplayModeRec xf86DefaultModes[];

static void
i830AddEstablishedMode(ScrnInfoPtr pScrn, int x, int y, int refresh,
		       DisplayModePtr *first, DisplayModePtr *last)
{
    DisplayModePtr mode, new;

    for (mode = xf86DefaultModes; mode->name != NULL; mode++) {
	float mode_refresh;

	if (mode->HDisplay != x || mode->VDisplay != y)
	    continue;

	mode_refresh = ModeVRefresh(mode);
	/* Select the default mode that's within 5% of the refresh rate we're
	 * looking for.  Should get the right one every time.
	 */
	if (fabs(mode_refresh - refresh) >= (refresh * .05))
	    continue;

	new = xnfcalloc(1, sizeof(DisplayModeRec));
	memcpy(new, mode, sizeof(DisplayModeRec));
	new->name = strdup(mode->name);

	new->next = NULL;
	new->prev = *last;

	if (*last)
	    (*last)->next = new;
	*last = new;
	if (!first)
	    *first = new;
    }
}

/**
 * EDID mode validation routine for using panel fitting.
 *
 * Modes in the list will be in the order of user-selected modes, followed by
 * EDID modes from the given monitor.  For now, it's only pulling out one set
 * of EDID modes.
 *
 * Returns a newly-allocated doubly-linked circular list of modes if any were
 * found, or NULL otherwise.
 */
DisplayModePtr
i830ValidateEDIDModes(ScrnInfoPtr pScrn, xf86MonPtr MonInfo)
{
    DisplayModePtr last = NULL;
    DisplayModePtr new = NULL;
    DisplayModePtr first = NULL;
    int i;
    char stmp[32];
    struct detailed_timings *dt;

    /* XXX: Need to insert user-requested modes first here if we can.  It's
     * straight EDID at the moment.
     */

    for (i = 0; i < DET_TIMINGS; i++) {
	switch (MonInfo->det_mon[i].type) {
	case DT:
	    dt = &MonInfo->det_mon[i].section.d_timings;

	    new = xnfcalloc(1, sizeof(DisplayModeRec));
	    snprintf(stmp, 32, "%dx%d", dt->h_active, dt->v_active);
	    new->name = xnfalloc(strlen(stmp) + 1);
	    new->HDisplay   = dt->h_active;
	    new->VDisplay   = dt->v_active;
	    new->HTotal     = dt->h_active + dt->h_blanking;
	    new->HSyncStart = dt->h_active + dt->h_sync_off;
	    new->HSyncEnd   = new->HSyncStart + dt->h_sync_width;
	    new->VTotal     = dt->v_active + dt->v_blanking;
	    new->VSyncStart = dt->v_active + dt->v_sync_off;
	    new->VSyncEnd   = new->VSyncStart + dt->v_sync_width;
	    new->Clock      = dt->clock;
	    /* XXX: Deal with syncing */
	
	    new->type |= M_T_DEFAULT;

	    ADD_NEW_TO_TAIL();

	    break;
	}
    }

    for (i = 0; i < STD_TIMINGS; i++) {
	if (MonInfo->timings2[i].hsize < 256)	/* sanity check */
	    continue;

	new = i830GetGTF(MonInfo->timings2[i].hsize, MonInfo->timings2[i].vsize,
			 MonInfo->timings2[i].refresh, FALSE, 0);
	ADD_NEW_TO_TAIL();
    }

    if (MonInfo->timings1.t1 & 0x80)
	i830AddEstablishedMode(pScrn, 720, 400, 70, &first, &last);
    if (MonInfo->timings1.t1 & 0x40)
	i830AddEstablishedMode(pScrn, 720, 400, 88, &first, &last);
    if (MonInfo->timings1.t1 & 0x20)
	i830AddEstablishedMode(pScrn, 640, 480, 60, &first, &last);
    if (MonInfo->timings1.t1 & 0x10)
	i830AddEstablishedMode(pScrn, 640, 480, 67, &first, &last);
    if (MonInfo->timings1.t1 & 0x08)
	i830AddEstablishedMode(pScrn, 640, 480, 72, &first, &last);
    if (MonInfo->timings1.t1 & 0x04)
	i830AddEstablishedMode(pScrn, 640, 480, 75, &first, &last);
    if (MonInfo->timings1.t1 & 0x02)
	i830AddEstablishedMode(pScrn, 800, 600, 56, &first, &last);
    if (MonInfo->timings1.t1 & 0x01)
	i830AddEstablishedMode(pScrn, 800, 600, 60, &first, &last);
    if (MonInfo->timings1.t2 & 0x80)
	i830AddEstablishedMode(pScrn, 800, 600, 72, &first, &last);
    if (MonInfo->timings1.t2 & 0x40)
	i830AddEstablishedMode(pScrn, 800, 600, 75, &first, &last);
    if (MonInfo->timings1.t2 & 0x20)
	i830AddEstablishedMode(pScrn, 832, 624, 75, &first, &last);
    if (MonInfo->timings1.t2 & 0x10)
	i830AddEstablishedMode(pScrn, 1024, 768, 87, &first, &last);
    if (MonInfo->timings1.t2 & 0x08)
	i830AddEstablishedMode(pScrn, 1024, 768, 60, &first, &last);
    if (MonInfo->timings1.t2 & 0x04)
	i830AddEstablishedMode(pScrn, 1024, 768, 70, &first, &last);
    if (MonInfo->timings1.t2 & 0x02)
	i830AddEstablishedMode(pScrn, 1024, 768, 75, &first, &last);
    if (MonInfo->timings1.t2 & 0x01)
	i830AddEstablishedMode(pScrn, 1280, 1024, 75, &first, &last);
    if (MonInfo->timings1.t_manu & 0x80)
	i830AddEstablishedMode(pScrn, 1152, 870, 75, &first, &last);

    /* Close the doubly-linked mode list */
    if (last) {
	last->next = first;
	first->prev = last;
    }

    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
	       "Total number of valid EDID mode(s) found: %d\n",
	       i830GetModeListLen(first));

    return first;
}

/**
 * Takes the current known information about the output device, and sets up
 * output[i].modes containing the list of modes that can be programmed for it.
 */
void
i830UpdateOutputModeList(ScrnInfoPtr pScrn, int i)
{
    I830Ptr pI830 = I830PTR(pScrn);
    DisplayModePtr modes = NULL;

    switch (pI830->output[i].type) {
    case I830_OUTPUT_LVDS:
	modes = i830ValidateFPModes(pScrn, pScrn->display->modes);
	break;
    default:
	if (pI830->output[i].MonInfo != NULL) {
	    modes = i830ValidateEDIDModes(pScrn, pI830->output[i].MonInfo);
	}
	break;
    }

    while (pI830->output[i].modes != NULL)
	xf86DeleteMode(&pI830->output[i].modes, pI830->output[i].modes);

    pI830->output[i].modes = modes;
}

#define MAX(a,b) ((a) > (b) ? (a) : (b))

/**
 * Sets the size of the root window by simply expanding to cover at least the
 * modes given.
 *
 * Note that this means we won't allocate enough for doing mergedfb.
 */
void
i830SetDefaultRootWindowSize(ScrnInfoPtr pScrn, DisplayModePtr first)
{
    DisplayModePtr mode;

    for (mode = first; mode != NULL && mode != first; mode = mode->next) {
	int displayWidth;

	/* XXX: Need to check if we've got enough memory to do this. */

	pScrn->displayWidth = MAX(pScrn->displayWidth, displayWidth);
	pScrn->virtualX = MAX(pScrn->virtualX, mode->HDisplay);
	pScrn->virtualY = MAX(pScrn->virtualY, mode->VDisplay);
	pScrn->display->virtualX = pScrn->virtualX;
	pScrn->display->virtualY = pScrn->virtualY;
    }
}
