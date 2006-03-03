/**************************************************************************

 Copyright 2006 Dave Airlie <airlied@linux.ie>
 
All Rights Reserved.

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

#include "sil164/sil164.h"

static const char *SIL164Symbols[] = {
  SIL164_SYMBOL_LIST
};

/* driver list */
struct _I830RegI2CDriver i830_i2c_drivers[] =
  { I830_I2C_TMDS, "sil164", "SIL164VidOutput", (SIL164_ADDR_1<<1), SIL164Symbols, NULL , NULL, NULL};



#define I830_NUM_I2C_DRIVERS (sizeof(i830_i2c_drivers)/sizeof(struct _I830RegI2CDriver))

/* bit locations in the registers */
#define SCL_DIR_MASK		0x0001
#define SCL_DIR			0x0002
#define SCL_VAL_MASK		0x0004
#define SCL_VAL_OUT		0x0008
#define SCL_VAL_IN		0x0010
#define SDA_DIR_MASK		0x0100
#define SDA_DIR			0x0200
#define SDA_VAL_MASK		0x0400
#define SDA_VAL_OUT		0x0800
#define SDA_VAL_IN		0x1000

#define I2C_TIMEOUT(x)	/*(x)*/  /* Report timeouts */
#define I2C_TRACE(x)    /*(x)*/  /* Report progress */

static void i830_setscl(I2CBusPtr b, int state)
{
  ScrnInfoPtr    pScrn      = xf86Screens[b->scrnIndex];
  I830Ptr pI830 = I830PTR(pScrn);
  CARD32 val;

  OUTREG(b->DriverPrivate.uval, (state ? SCL_VAL_OUT : 0) | SCL_DIR | SCL_DIR_MASK | SCL_VAL_MASK);
  val=INREG(b->DriverPrivate.uval);
}

static void i830_setsda(I2CBusPtr b, int state)
{
  ScrnInfoPtr    pScrn      = xf86Screens[b->scrnIndex];
  I830Ptr pI830 = I830PTR(pScrn);
  CARD32 val;

  OUTREG(b->DriverPrivate.uval, (state ? SDA_VAL_OUT : 0) | SDA_DIR | SDA_DIR_MASK | SDA_VAL_MASK);
  val=INREG(b->DriverPrivate.uval);
}

static void i830_getscl(I2CBusPtr b, int *state)
{
  ScrnInfoPtr    pScrn      = xf86Screens[b->scrnIndex];
  I830Ptr pI830 = I830PTR(pScrn);
  CARD32 val;

  OUTREG(b->DriverPrivate.uval, SCL_DIR_MASK);
  OUTREG(b->DriverPrivate.uval, 0);
  val=INREG(b->DriverPrivate.uval);
  *state = ((val & SCL_VAL_IN) != 0);
}

static int i830_getsda(I2CBusPtr b)
{
  ScrnInfoPtr    pScrn      = xf86Screens[b->scrnIndex];
  I830Ptr pI830 = I830PTR(pScrn);
  CARD32 val;

  OUTREG(b->DriverPrivate.uval, SDA_DIR_MASK);
  OUTREG(b->DriverPrivate.uval, 0);
  val=INREG(b->DriverPrivate.uval);
  return ((val & SDA_VAL_IN) != 0);
}

static inline void sdalo(I2CBusPtr b)
{
  i830_setsda(b, 0);
  b->I2CUDelay(b, b->RiseFallTime);
}

static inline void sdahi(I2CBusPtr b)
{
  i830_setsda(b, 1);
  b->I2CUDelay(b, b->RiseFallTime);
}

static inline void scllo(I2CBusPtr b)
{
  i830_setscl(b, 0);
  b->I2CUDelay(b, b->RiseFallTime);
}

static inline int sclhi(I2CBusPtr b, int timeout)
{
  int scl = 0;
  int i;

  i830_setscl(b, 1);
  b->I2CUDelay(b, b->RiseFallTime);
  
  for (i = timeout; i > 0; i -= b->RiseFallTime) {
    i830_getscl(b, &scl);
    if (scl) break;
    b->I2CUDelay(b, b->RiseFallTime);
  }

  if (i <= 0) {
    I2C_TIMEOUT(ErrorF("[I2CRaiseSCL(<%s>, %d) timeout]", b->BusName, timeout));    
    return FALSE;
  }
  return TRUE;
}

static Bool
I830I2CGetByte(I2CDevPtr d, I2CByte *data, Bool last)
{
    I2CBusPtr b = d->pI2CBus;
    int i, sda;
    unsigned char indata = 0;

    sdahi(b);
    
    for (i=0; i<8; i++) {
      if (sclhi(b, d->BitTimeout)==FALSE) {
	I2C_TRACE(ErrorF("timeout at bit #%d\n", 7-i));
	return FALSE;
      };
      indata*=2;
      if ( i830_getsda (b) )
      {
	indata |= 0x01;
      }
      scllo(b);
    }

    if (last) {
      sdahi(b);
    }
    else
      sdalo(b);

    if (sclhi(b, d->BitTimeout) == FALSE) {
      sdahi(b);
      return FALSE;
    };

    scllo(b);
    sdahi(b);

    *data = indata & 0xff;
    I2C_TRACE(ErrorF("R%02x ", (int) *data));
    
    return TRUE;
}

static Bool
I830I2CPutByte(I2CDevPtr d, I2CByte c)
{
    Bool r;
    int i, scl, sda;
    int sb, ack;
    I2CBusPtr b = d->pI2CBus;

    for ( i = 7; i>=0; i--)
    {
      sb = c & (1 << i);
      i830_setsda(b, sb);
      b->I2CUDelay(b, b->RiseFallTime);

      if (sclhi(b, d->ByteTimeout) == FALSE) {
	sdahi(b);
	return FALSE;
      }

      i830_setscl(b, 0);
      b->I2CUDelay(b, b->RiseFallTime);
    }
    sdahi(b);
    if (sclhi(b, d->ByteTimeout) == FALSE) {
      I2C_TIMEOUT(ErrorF("[I2CPutByte(<%s>, 0x%02x, %d, %d, %d) timeout]", 
			 b->BusName, c, d->BitTimeout, 
			 d->ByteTimeout, d->AcknTimeout));
      return FALSE;
    }
    ack = i830_getsda(b);
    I2C_TRACE(ErrorF("Put byte 0x%02x , getsda() = %d\n", c & 0xff, ack));

    scllo(b);
    return 0 == ack;
}

static Bool
I830I2CStart(I2CBusPtr b, int timeout)
{
  if (sclhi(b, timeout) == FALSE)
    return FALSE;

  
  sdalo(b);
  scllo(b);
    
  return TRUE;
}

static void
I830I2CStop(I2CDevPtr d)
{
  I2CBusPtr b = d->pI2CBus;
  
  sdalo(b);
  sclhi(b, d->ByteTimeout);
  sdahi(b);
}

static Bool
I830I2CAddress(I2CDevPtr d, I2CSlaveAddr addr)
{
    if (I830I2CStart(d->pI2CBus, d->StartTimeout)) {
	if (I830I2CPutByte(d, addr & 0xFF)) {
	    if ((addr & 0xF8) != 0xF0 &&
		(addr & 0xFE) != 0x00)
		return TRUE;

	    if (I830I2CPutByte(d, (addr >> 8) & 0xFF))
		return TRUE;
	}

	I830I2CStop(d);
    }

    return FALSE;
}


/* the i830 has a number of I2C Buses */
Bool
I830I2CInit(ScrnInfoPtr pScrn, I2CBusPtr *bus_ptr, int i2c_reg, char *name)
{
  I830Ptr pI830 = I830PTR(pScrn);
  I2CBusPtr pI2CBus;
  char namestr[25];

  pI2CBus = xf86CreateI2CBusRec();

  if (!pI2CBus) return FALSE;

  pI2CBus->BusName = name;
  pI2CBus->scrnIndex = pScrn->scrnIndex;
  pI2CBus->I2CGetByte = I830I2CGetByte;
  pI2CBus->I2CPutByte = I830I2CPutByte;
  pI2CBus->I2CStart = I830I2CStart;
  pI2CBus->I2CStop = I830I2CStop;
  pI2CBus->I2CAddress = I830I2CAddress;
  pI2CBus->DriverPrivate.uval = i2c_reg;

  if (!xf86I2CBusInit(pI2CBus)) return FALSE;

  *bus_ptr = pI2CBus;
  return TRUE;

}

Bool
I830I2CDetectControllers(ScrnInfoPtr pScrn, I2CBusPtr pI2CBus, struct _I830RegI2CDriver **retdrv)
{
  I830Ptr pI830 = I830PTR(pScrn);  
  int i;
  void *ret_ptr;
  struct _I830RegI2CDriver *drv;

  for (i=0; i<I830_NUM_I2C_DRIVERS; i++)
  {
    drv = &i830_i2c_drivers[i];
    drv->modhandle = xf86LoadSubModule(pScrn, drv->modulename);

    if (!drv->modhandle)
      continue;
    
    xf86LoaderReqSymLists(drv->symbols, NULL);
    
    ret_ptr = NULL;
    drv->vid_rec = LoaderSymbol(drv->fntablename);
    if (drv->vid_rec)
       ret_ptr = drv->vid_rec->Detect(pI2CBus, drv->address);
    
    if (ret_ptr) {
      drv->devpriv = ret_ptr;
      *retdrv = drv;
      return TRUE;
    }
    xf86UnloadModule(drv->modhandle);
  }
  return FALSE;
}

