
#ifndef INTEL_BUFMGR_TTM_H
#define INTEL_BUFMGR_TTM_H

#include "dri_bufmgr.h"

extern ddx_bo *intelddx_ttm_bo_create_from_handle(ddx_bufmgr *bufmgr, const char *name,
					       unsigned int handle);

dri_fence *intelddx_ttm_fence_create_from_arg(ddx_bufmgr *bufmgr, const char *name,
					   drm_fence_arg_t *arg);


ddx_bufmgr *intelddx_bufmgr_ttm_init(int fd, unsigned int fence_type,
				  unsigned int fence_type_flush, int batch_size);

#endif
