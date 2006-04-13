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

#include "xf86.h"
#include "i830.h"
#include "i830_modes.h"

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
 * the panel's native mode
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
    int count = 0;
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

	count++;
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
	       "Total number of valid FP mode(s) found: %d\n", count);

    return first;
}

#define MAX(a,b) ((a) > (b) ? (a) : (b))

void
i830FitScreenVirtualForModes(ScrnInfoPtr pScrn, DisplayModePtr first)
{
    DisplayModePtr mode;

    for (mode = first; mode != NULL && mode != first; mode = mode->next) {
	pScrn->virtualX = MAX(pScrn->virtualX, mode->HDisplay);
	pScrn->virtualY = MAX(pScrn->virtualY, mode->VDisplay);
	pScrn->display->virtualX = pScrn->virtualX;
	pScrn->display->virtualY = pScrn->virtualY;
    }
}
