#define EXASTATE_SZ 48000

struct i965_exastate_buffer {
    dri_bo *buf;
    unsigned char *map;

    dri_bo *vbo;
    unsigned int vbo_prim_start;
    unsigned int vbo_used;
    unsigned int element_size;

    Bool no_flush;

    dri_bo *surface_buf;
    unsigned char *surface_map;
    int num_ops;

    dri_fence *last_fence;
    ScrnInfoPtr pScrn;
};
