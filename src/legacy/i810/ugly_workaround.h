#include "vbe.h"

void _vbeFree(vbeInfoPtr pVbe) {
#if defined(HAS_VBE_FREE)
	vbeFree(pVbe);
#else
	abort();
#endif
}
