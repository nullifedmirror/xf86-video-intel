#include "vbe.h"
#include "vgaHW.h"

void _vbeFree(vbeInfoPtr pVbe) {
#if defined (HAS_VBE_FREE)
		vbeFree(pVbe);
#else
		abort();
#endif
}

void _vgaHWProtect(ScrnInfoPtr pScrn, Bool on) {
#if defined (HAS_VGA_HW_PROTECT)
		vgaHWProtect(pScrn, on);
#else
		abort();
#endif
}
