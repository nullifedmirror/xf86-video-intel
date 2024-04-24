#include "vbe.h"
#include "vgaHW.h"

void _vbeFree(vbeInfoPtr pVbe) {
#ifdef HAS_VBE_FREE
		vbeFree(pVbe);
#else
		abort();
#endif
}

void _vgaHWProtect(ScrnInfoPtr pScrn, Bool on) {
#ifdef HAS_VGA_HW_PROTECT
		vgaHWProtect(pScrn, on);
#else
		abort();
#endif
}
