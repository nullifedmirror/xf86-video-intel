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

static void I830I2CGetBits(I2CBusPtr b, int *clock, int *data)
{
    ScrnInfoPtr    pScrn      = xf86Screens[b->scrnIndex];
    I830Ptr pI830 = I830PTR(pScrn);
    unsigned long  val;

    OUTREG(pI830->DDCReg, (SCL_DIR_MASK));
    OUTREG(pI830->DDCReg, 0);
    val = INREG(pI830->DDCReg);

    *clock = (val & SCL_VAL_IN) != 0;

    OUTREG(pI830->DDCReg, (SDA_DIR_MASK));
    OUTREG(pI830->DDCReg, 0);
    val = INREG(pI830->DDCReg);

    *data = (val & SDA_VAL_IN) != 0;
}

static void I830I2CPutBits(I2CBusPtr b, int clock, int data)
{
    ScrnInfoPtr    pScrn      = xf86Screens[b->scrnIndex];
    I830Ptr pI830 = I830PTR(pScrn);
    unsigned long  val;

    /* do this in two passes */
    OUTREG(pI830->DDCReg, (clock ? SCL_VAL_OUT : 0) | SCL_DIR | SCL_DIR_MASK | SCL_VAL_MASK);
    val = INREG(pI830->DDCReg);

    OUTREG(pI830->DDCReg, (data ? SDA_VAL_OUT : 0) | SDA_DIR | SDA_DIR_MASK | SDA_VAL_MASK);
    val = INREG(pI830->DDCReg);
}

Bool
I830I2cInit(ScrnInfoPtr pScrn)
{
  I830Ptr pI830 = I830PTR(pScrn);
  
  pI830->pI2CBus = xf86CreateI2CBusRec();

  if (!pI830->pI2CBus) return FALSE;

  pI830->pI2CBus->BusName = "DDC";
  pI830->pI2CBus->scrnIndex = pScrn->scrnIndex;
  pI830->pI2CBus->I2CPutBits = I830I2CPutBits;
  pI830->pI2CBus->I2CGetBits = I830I2CGetBits;
  pI830->pI2CBus->AcknTimeout = 10;

  if (!xf86I2CBusInit(pI830->pI2CBus)) return FALSE;

  return TRUE;

}
