/**************************************************************************

Copyright 1998-1999 Precision Insight, Inc., Cedar Park, Texas.
Copyright © 2002 David Dawes

All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sub license, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice (including the
next paragraph) shall be included in all copies or substantial portions
of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**************************************************************************/
/* $XFree86: xc/programs/Xserver/hw/xfree86/drivers/i810/i830.h,v 1.12 2004/01/07 03:43:19 dawes Exp $ */

/*
 * Authors:
 *   Keith Whitwell <keith@tungstengraphics.com>
 *   David Dawes <dawes@xfree86.org>
 *
 */

#if 1
#define I830DEBUG
#endif

#ifndef REMAP_RESERVED
#define REMAP_RESERVED 0
#endif

#ifndef _I830_H_
#define _I830_H_

#include "compiler.h"
#include "xf86PciInfo.h"
#include "xf86Pci.h"
#include "i810_reg.h"
#include "xaa.h"
#include "xf86Cursor.h"
#include "xf86xv.h"
#include "xf86int10.h"
#include "vbe.h"
#include "vgaHW.h"
#include "randrstr.h"

#ifdef XF86DRI
#include "xf86drm.h"
#include "sarea.h"
#define _XF86DRI_SERVER_
#include "dri.h"
#include "GL/glxint.h"
#include "i830_dri.h"
#endif

#include "common.h"

/* I830 Video BIOS support */

/*
 * The mode handling is based upon the VESA driver written by
 * Paulo César Pereira de Andrade <pcpa@conectiva.com.br>.
 */

#define PIPE_NONE	0<<0
#define PIPE_CRT	1<<0
#define PIPE_TV		1<<1
#define PIPE_DFP	1<<2
#define PIPE_LFP	1<<3
#define PIPE_CRT2	1<<4
#define PIPE_TV2	1<<5
#define PIPE_DFP2	1<<6
#define PIPE_LFP2	1<<7

typedef struct _VESARec {
   /* SVGA state */
   pointer state, pstate;
   int statePage, stateSize, stateMode, stateRefresh;
   CARD32 *savedPal;
   int savedScanlinePitch;
   xf86MonPtr monitor;
   /* Don't try to set the refresh rate for any modes. */
   Bool useDefaultRefresh;
   /* display start */
   int x, y;
} VESARec, *VESAPtr;


typedef struct _I830Rec *I830Ptr;

typedef void (*I830WriteIndexedByteFunc)(I830Ptr pI830, IOADDRESS addr,
                                         CARD8 index, CARD8 value);
typedef CARD8(*I830ReadIndexedByteFunc)(I830Ptr pI830, IOADDRESS addr,
                                        CARD8 index);
typedef void (*I830WriteByteFunc)(I830Ptr pI830, IOADDRESS addr, CARD8 value);
typedef CARD8(*I830ReadByteFunc)(I830Ptr pI830, IOADDRESS addr);

/* Linear region allocated in framebuffer. */
typedef struct _I830MemPool *I830MemPoolPtr;
typedef struct _I830MemRange *I830MemRangePtr;
typedef struct _I830MemRange {
   long Start;
   long End;
   long Size;
   unsigned long Physical;
   unsigned long Offset;		/* Offset of AGP-allocated portion */
   unsigned long Alignment;
   int Key;
   I830MemPoolPtr Pool;
} I830MemRange;

typedef struct _I830MemPool {
   I830MemRange Total;
   I830MemRange Free;
   I830MemRange Fixed;
   I830MemRange Allocated;
} I830MemPool;

typedef struct {
   int tail_mask;
   I830MemRange mem;
   unsigned char *virtual_start;
   int head;
   int tail;
   int space;
} I830RingBuffer;

typedef struct {
   int            lastInstance;
   int            refCount;
   ScrnInfoPtr    pScrn_1;
   ScrnInfoPtr    pScrn_2;
   int            RingRunning;
#ifdef I830_XV
   int            XvInUse;
#endif
} I830EntRec, *I830EntPtr;

typedef struct _I830SaveRec {
  unsigned int Fence[8];
  
  CARD32 vga0_divisor;
  CARD32 vga1_divisor;
  CARD32 vga_pd;
  CARD32 dpll_a;
  CARD32 dpll_b;
  CARD32 fpa0;
  CARD32 fpa1;
  CARD32 fpb0;
  CARD32 fpb1;
  //	CARD32 palette_a[PALETTE_8_ENTRIES];
  //	CARD32 palette_b[PALETTE_8_ENTRIES];
  CARD32 htotal_a;
  CARD32 hblank_a;
  CARD32 hsync_a;
  CARD32 vtotal_a;
  CARD32 vblank_a;
  CARD32 vsync_a;
  CARD32 pipe_src_a;
  CARD32 bclrpat_a;
  CARD32 htotal_b;
  CARD32 hblank_b;
  CARD32 hsync_b;
  CARD32 vtotal_b;
  CARD32 vblank_b;
  CARD32 vsync_b;
  CARD32 pipe_src_b;
  CARD32 bclrpat_b;
  CARD32 adpa;
  CARD32 dvoa;
  CARD32 dvob;
  CARD32 dvoc;
  CARD32 dvoa_srcdim;
  CARD32 dvob_srcdim;
  CARD32 dvoc_srcdim;
  CARD32 lvds;
  CARD32 pipe_a_conf;
  CARD32 pipe_b_conf;
  CARD32 disp_arb;
  CARD32 cursor_a_control;
  CARD32 cursor_b_control;
  CARD32 cursor_a_base;
  CARD32 cursor_b_base;
  CARD32 cursor_size;
  CARD32 disp_a_ctrl;
  CARD32 disp_b_ctrl;
  CARD32 disp_a_base;
  CARD32 disp_b_base;
  CARD32 cursor_a_palette[4];
  CARD32 cursor_b_palette[4];
  CARD32 disp_a_stride;
  CARD32 disp_b_stride;
  CARD32 vgacntrl;
  CARD32 add_id;
  CARD32 swf0x[7];
  CARD32 swf1x[7];
  CARD32 swf3x[3];
  CARD32 fence[8];
  CARD32 instpm;
  CARD32 mem_mode;
  CARD32 fw_blc_0;
  CARD32 fw_blc_1;
} I830RegRec, *I830RegPtr;

/* store information about an Ixxx DVO */
/* The i830->i865 use multiple DVOs with multiple i2cs */
/* the i915, i945 have a single sDVO i2c bus - which is different */
#define MAX_OUTPUTS 6

/* these are outputs from the chip - integrated only 
   external chips are via DVO or SDVO output */
#define I830_OUTPUT_UNUSED 0
#define I830_OUTPUT_ANALOG 1
#define I830_OUTPUT_DVO 2
#define I830_OUTPUT_SDVO 3
#define I830_OUTPUT_LVDS 4
#define I830_OUTPUT_TVOUT 5

#define I830_I2C_CHIP_NONE 0
#define I830_I2C_CHIP_LVDS 1
#define I830_I2C_CHIP_TMDS 2
#define I830_I2C_CHIP_TVOUT 4

#define I830_OUTPUT_PIPE_NC  0
#define I830_OUTPUT_PIPE_A   1
#define I830_OUTPUT_PIPE_B   2

struct _I830DVODriver {
  int type;
  char *modulename;
  char *fntablename;
  int address;
  const char **symbols;
  I2CVidOutputRec *vid_rec;
  void *devpriv;
  pointer modhandle;
};

typedef struct _I830SDVODriver {
  int found;
  I2CDevRec d;
  unsigned char sdvo_regs[20];
} I830SDVORec, *I830SDVOPtr;

struct _I830OutputRec {
  int type;
  int pipe;
  int flags;
  xf86MonPtr MonInfo;
  I2CBusPtr pI2CBus;
  I2CBusPtr pDDCBus;
  struct _I830DVODriver *i2c_drv;
  struct _I830SDVODriver *sdvo_drv;
};

typedef struct _I830Rec {
   unsigned char *MMIOBase;
   unsigned char *FbBase;
   int cpp;

   unsigned int bios_version;

   Bool newPipeSwitch;

   Bool fakeSwitch;
   
   int fixedPipe;

   DisplayModePtr currentMode;

   Bool Clone;
   int CloneRefresh;
   int CloneHDisplay;
   int CloneVDisplay;

   I830EntPtr entityPrivate;	
   int pipe, origPipe;
   int init;

   unsigned int bufferOffset;		/* for I830SelectBuffer */
   BoxRec FbMemBox;
   BoxRec FbMemBox2;
   int CacheLines;

   /* These are set in PreInit and never changed. */
   long FbMapSize;
   long TotalVideoRam;
   I830MemRange StolenMemory;		/* pre-allocated memory */
   long BIOSMemorySize;			/* min stolen pool size */
   int BIOSMemSizeLoc;

   /* These change according to what has been allocated. */
   long FreeMemory;
   I830MemRange MemoryAperture;
   I830MemPool StolenPool;
   long allocatedMemory;

   /* Regions allocated either from the above pools, or from agpgart. */
   /* for single and dual head configurations */
   I830MemRange FrontBuffer;
   I830MemRange FrontBuffer2;
   I830MemRange Scratch;
   I830MemRange Scratch2;

   /* Regions allocated either from the above pools, or from agpgart. */
   I830MemRange	*CursorMem;
   I830MemRange	*CursorMemARGB;
   I830RingBuffer *LpRing;

#if REMAP_RESERVED
   I830MemRange Dummy;
#endif

#ifdef I830_XV
   /* For Xvideo */
   I830MemRange *OverlayMem;
   I830MemRange LinearMem;
#endif
   unsigned int LinearAlloc;
  
   XF86ModReqInfo shadowReq; /* to test for later libshadow */
   I830MemRange RotatedMem;
   I830MemRange RotatedMem2;
   Rotation rotation;
   int InitialRotation;
   int displayWidth;
   void (*PointerMoved)(int, int, int);
   CreateScreenResourcesProcPtr    CreateScreenResources;
   int *used3D;

#ifdef XF86DRI
   I830MemRange BackBuffer;
   I830MemRange DepthBuffer;
   I830MemRange TexMem;
   int TexGranularity;
   I830MemRange ContextMem;
   int drmMinor;
   Bool have3DWindows;
#endif

   Bool NeedRingBufferLow;
   Bool allowPageFlip;
   Bool disableTiling;

   int backPitch;

   Bool CursorNeedsPhysical;
   Bool CursorIsARGB;
   CursorPtr pCurs;

   int MonType1;
   int MonType2;
   Bool specifiedMonitor;

   DGAModePtr DGAModes;
   int numDGAModes;
   Bool DGAactive;
   int DGAViewportStatus;

   int Chipset;
   unsigned long LinearAddr;
   unsigned long MMIOAddr;
   IOADDRESS ioBase;
   EntityInfoPtr pEnt;
   pciVideoPtr PciInfo;
   PCITAG PciTag;
   CARD8 variant;

   unsigned int BR[20];

   int GttBound;

   unsigned char **ScanlineColorExpandBuffers;
   int NumScanlineColorExpandBuffers;
   int nextColorExpandBuf;

   I830RegRec SavedReg; /* original (text) mode */
   I830RegRec ModeReg;  /* current mode */

   Bool noAccel;
   Bool SWCursor;
   Bool cursorOn;
   XAAInfoRecPtr AccelInfoRec;
   xf86CursorInfoPtr CursorInfoRec;
   CloseScreenProcPtr CloseScreen;

   I830WriteIndexedByteFunc writeControl;
   I830ReadIndexedByteFunc readControl;
   I830WriteByteFunc writeStandard;
   I830ReadByteFunc readStandard;

   Bool XvDisabled;			/* Xv disabled in PreInit. */
   Bool XvEnabled;			/* Xv enabled for this generation. */

#ifdef I830_XV
   int colorKey;
   XF86VideoAdaptorPtr adaptor;
   ScreenBlockHandlerProcPtr BlockHandler;
   Bool *overlayOn;
#endif

   Bool directRenderingDisabled;	/* DRI disabled in PreInit. */
   Bool directRenderingEnabled;		/* DRI enabled this generation. */

#ifdef XF86DRI
   Bool directRenderingOpen;
   int LockHeld;
   DRIInfoPtr pDRIInfo;
   int drmSubFD;
   int numVisualConfigs;
   __GLXvisualConfig *pVisualConfigs;
   I830ConfigPrivPtr pVisualConfigsPriv;
   drm_handle_t buffer_map;
   drm_handle_t ring_map;
#endif

   /* Broken-out options. */
   OptionInfoPtr Options;

   /* Stolen memory support */
   Bool StolenOnly;

   /* Video BIOS support. */
   vbeInfoPtr pVbe;
   VbeInfoBlock *vbeInfo;
   VESAPtr vesa;

   Bool overrideBIOSMemSize;
   int saveBIOSMemSize;
   int newBIOSMemSize;
   Bool useSWF1;
   int saveSWF1;

   Bool swfSaved;
   CARD32 saveSWF0;
   CARD32 saveSWF4;

   /* Use BIOS call 0x5f05 to set the refresh rate. */
   Bool useExtendedRefresh;

   Bool checkDevices;
   int monitorSwitch;
   int operatingDevices;
   int toggleDevices;
   int savedDevices;
   int lastDevice0, lastDevice1, lastDevice2;

   /* These are indexed by the display types */
   Bool displayAttached[NumDisplayTypes];
   Bool displayPresent[NumDisplayTypes];
   BoxRec displaySize[NumDisplayTypes];

   /* [0] is Pipe A, [1] is Pipe B. */
   int availablePipes;
   int pipeDevices[MAX_DISPLAY_PIPES];
   /* [0] is display plane A, [1] is display plane B. */
   Bool pipeEnabled[MAX_DISPLAY_PIPES];
   BoxRec pipeDisplaySize[MAX_DISPLAY_PIPES];
   int planeEnabled[MAX_DISPLAY_PIPES];

   /* Driver phase/state information */
   Bool preinit;
   Bool starting;
   Bool closing;
   Bool suspended;
   Bool leaving;

   /* fbOffset converted to (x, y). */
   int xoffset;
   int yoffset;

   unsigned int SaveGeneration;
   Bool vbeRestoreWorkaround;
   Bool displayInfo;
   Bool devicePresence;

   OsTimerPtr devicesTimer;
   Bool rawmode;
   int MaxClock;
  
   int ddc2;
   int num_outputs;

   struct _I830OutputRec output[MAX_OUTPUTS];
  //   I830SDVOPtr sdvo;
} I830Rec;

#define I830PTR(p) ((I830Ptr)((p)->driverPrivate))
#define I830REGPTR(p) (&(I830PTR(p)->ModeReg))

#define I830_SELECT_FRONT	0
#define I830_SELECT_BACK	1
#define I830_SELECT_DEPTH	2

/* I830 specific functions */
extern int I830WaitLpRing(ScrnInfoPtr pScrn, int n, int timeout_millis);
extern void I830SetPIOAccess(I830Ptr pI830);
extern void I830SetMMIOAccess(I830Ptr pI830);
extern void I830PrintErrorState(ScrnInfoPtr pScrn);
extern void I830Sync(ScrnInfoPtr pScrn);
extern void I830InitHWCursor(ScrnInfoPtr pScrn);
extern Bool I830CursorInit(ScreenPtr pScreen);
extern void I830EmitInvarientState(ScrnInfoPtr pScrn);
extern void I830SelectBuffer(ScrnInfoPtr pScrn, int buffer);

extern void I830RefreshRing(ScrnInfoPtr pScrn);
extern void I830EmitFlush(ScrnInfoPtr pScrn);

extern Bool I830DGAInit(ScreenPtr pScreen);

#ifdef I830_XV
extern void I830InitVideo(ScreenPtr pScreen);
extern void I830VideoSwitchModeBefore(ScrnInfoPtr pScrn, DisplayModePtr mode);
extern void I830VideoSwitchModeAfter(ScrnInfoPtr pScrn, DisplayModePtr mode);
#endif

extern Bool I830AllocateRotatedBuffer(ScrnInfoPtr pScrn, const int flags);
extern Bool I830AllocateRotated2Buffer(ScrnInfoPtr pScrn, const int flags);
#ifdef XF86DRI
extern Bool I830Allocate3DMemory(ScrnInfoPtr pScrn, const int flags);
extern Bool I830AllocateBackBuffer(ScrnInfoPtr pScrn, const int flags);
extern Bool I830AllocateDepthBuffer(ScrnInfoPtr pScrn, const int flags);
extern Bool I830AllocateTextureMemory(ScrnInfoPtr pScrn, const int flags);
extern void I830SetupMemoryTiling(ScrnInfoPtr pScrn);
extern Bool I830DRIScreenInit(ScreenPtr pScreen);
extern Bool I830CheckDRIAvailable(ScrnInfoPtr pScrn);
extern Bool I830DRIDoMappings(ScreenPtr pScreen);
extern Bool I830DRIResume(ScreenPtr pScreen);
extern void I830DRICloseScreen(ScreenPtr pScreen);
extern Bool I830DRIFinishScreenInit(ScreenPtr pScreen);
extern Bool I830UpdateDRIBuffers(ScrnInfoPtr pScrn, drmI830Sarea *sarea);
extern void I830DRIUnmapScreenRegions(ScrnInfoPtr pScrn, drmI830Sarea *sarea);
extern Bool I830DRIMapScreenRegions(ScrnInfoPtr pScrn, drmI830Sarea *sarea);
extern void I830DRIUnlock(ScrnInfoPtr pScrn);
extern Bool I830DRILock(ScrnInfoPtr pScrn);
#endif

extern Bool I830AccelInit(ScreenPtr pScreen);
extern void I830SetupForScreenToScreenCopy(ScrnInfoPtr pScrn, int xdir,
					   int ydir, int rop,
					   unsigned int planemask,
					   int trans_color);
extern void I830SubsequentScreenToScreenCopy(ScrnInfoPtr pScrn, int srcX,
					     int srcY, int dstX, int dstY,
					     int w, int h);
extern void I830SetupForSolidFill(ScrnInfoPtr pScrn, int color, int rop,
				  unsigned int planemask);
extern void I830SubsequentSolidFillRect(ScrnInfoPtr pScrn, int x, int y,
					int w, int h);

extern void I830ResetAllocations(ScrnInfoPtr pScrn, const int flags);
extern long I830CheckAvailableMemory(ScrnInfoPtr pScrn);
extern long I830GetExcessMemoryAllocations(ScrnInfoPtr pScrn);
extern Bool I830Allocate2DMemory(ScrnInfoPtr pScrn, const int flags);
extern Bool I830DoPoolAllocation(ScrnInfoPtr pScrn, I830MemPool *pool);
extern Bool I830FixupOffsets(ScrnInfoPtr pScrn);
extern Bool I830BindGARTMemory(ScrnInfoPtr pScrn);
extern Bool I830UnbindGARTMemory(ScrnInfoPtr pScrn);
extern unsigned long I830AllocVidMem(ScrnInfoPtr pScrn, I830MemRange *result,
				     I830MemPool *pool, long size,
				     unsigned long alignment, int flags);
extern void I830FreeVidMem(ScrnInfoPtr pScrn, I830MemRange *range);

extern void I830ChangeFrontbuffer(ScrnInfoPtr pScrn,int buffer);
extern Bool I830IsPrimary(ScrnInfoPtr pScrn);

extern DisplayModePtr I830GetModePool(ScrnInfoPtr pScrn, vbeInfoPtr pVbe,
					VbeInfoBlock *vbe);
extern void I830SetModeParameters(ScrnInfoPtr pScrn, vbeInfoPtr pVbe);
extern void I830UnsetModeParameters(ScrnInfoPtr pScrn, vbeInfoPtr pVbe);
extern void I830PrintModes(ScrnInfoPtr pScrn);
extern int I830GetBestRefresh(ScrnInfoPtr pScrn, int refresh);
extern Bool I830CheckModeSupport(ScrnInfoPtr pScrn, int x, int y, int mode);
extern Bool I830Rotate(ScrnInfoPtr pScrn, DisplayModePtr mode);
extern Bool I830FixOffset(ScrnInfoPtr pScrn, I830MemRange *mem);

extern Bool I830I2CDetectSDVOController(ScrnInfoPtr pScrn, int output_index);
extern Bool I830I2CDetectDVOControllers(ScrnInfoPtr pScrn, I2CBusPtr pI2CBus, struct _I830DVODriver **retdrv);
extern Bool I830I2CInit(ScrnInfoPtr pScrn, I2CBusPtr *bus_ptr, int i2c_reg, char *name);
extern Bool I830SDVOPreSetMode(I830SDVOPtr s, DisplayModePtr mode);
extern Bool I830SDVOPostSetMode(I830SDVOPtr s, DisplayModePtr mode);

extern Bool I830RawSetMode(ScrnInfoPtr pScrn, DisplayModePtr mode);
extern Bool I830RawSwitchMode(int scrnIndex, DisplayModePtr mode, int flags);
extern Bool I830RawSaveState(ScrnInfoPtr pScrn, I830RegPtr hw);
extern Bool I830RawRestoreState(ScrnInfoPtr pScrn, I830RegPtr hw);

/*
 * 12288 is set as the maximum, chosen because it is enough for
 * 1920x1440@32bpp with a 2048 pixel line pitch with some to spare.
 */
#define I830_MAXIMUM_VBIOS_MEM		12288
#define I830_DEFAULT_VIDEOMEM_2D	(MB(32) / 1024)
#define I830_DEFAULT_VIDEOMEM_3D	(MB(64) / 1024)

/* Flags for memory allocation function */
#define FROM_ANYWHERE			0x00000000
#define FROM_POOL_ONLY			0x00000001
#define FROM_NEW_ONLY			0x00000002
#define FROM_MASK			0x0000000f

#define ALLOCATE_AT_TOP			0x00000010
#define ALLOCATE_AT_BOTTOM		0x00000020
#define FORCE_GAPS			0x00000040

#define NEED_PHYSICAL_ADDR		0x00000100
#define ALIGN_BOTH_ENDS			0x00000200
#define FORCE_LOW			0x00000400

#define ALLOC_NO_TILING			0x00001000
#define ALLOC_INITIAL			0x00002000

#define ALLOCATE_DRY_RUN		0x80000000

/* Chipset registers for VIDEO BIOS memory RW access */
#define _855_DRAM_RW_CONTROL 0x58
#define _845_DRAM_RW_CONTROL 0x90
#define DRAM_WRITE    0x33330000

#endif /* _I830_H_ */
