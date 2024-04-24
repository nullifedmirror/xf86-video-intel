#include "vbe.h"
#include "vgaHW.h"

void vbeFree(vbeInforPtr pVbe) __attribute__((weak));
void vgaHWProtectProc(ScrnInfoPtr pScrn, Bool on) __attribute__((weak));

void _vbeFree(vbeInfoPtr pVbe) {
	if (vbeFree) {
		vbeFree(pVbe);
	} else {
		abort();
	}
}

void _vgaHWProtectProc(ScrnInfoPtr pScrn, Bool on) {
	if (vgaHWProtectProc) {
		vgaHWProtectProc(pScrn, on);
	} else {
		abort();
	}
}
