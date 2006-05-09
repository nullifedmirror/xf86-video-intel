/**************************************************************************

Copyright © 2006 Dave Airlie

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
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**************************************************************************/
#include "xf86.h"
#include "xf86_OSproc.h"
#include "xf86Resources.h"
#include "compiler.h"
#include "miscstruct.h"
#include "xf86i2c.h"

#include "ch7xxx.h"

static Bool ch7xxxReadByte(CH7xxxPtr sil, int addr, unsigned char *ch)
{
  if (!xf86I2CReadByte(&(sil->d), addr, ch)) {
    xf86DrvMsg(sil->d.pI2CBus->scrnIndex, X_ERROR, "Unable to read from %s Slave %d.\n", sil->d.pI2CBus->BusName, sil->d.SlaveAddr);
    return FALSE;
  }
  return TRUE;
}

static Bool ch7xxxWriteByte(CH7xxxPtr sil, int addr, unsigned char ch)
{
  if (!xf86I2CWriteByte(&(sil->d), addr, ch)) {
    xf86DrvMsg(sil->d.pI2CBus->scrnIndex, X_ERROR, "Unable to write to %s Slave %d.\n", sil->d.pI2CBus->BusName, sil->d.SlaveAddr);
    return FALSE;
  }
  return TRUE;
}

/* Silicon Image 164 driver for chip on i2c bus */
static void *ch7xxxDetect(I2CBusPtr b, I2CSlaveAddr addr)
{
  /* this will detect the CH7xxx chip on the specified i2c bus */
  CH7xxxPtr sil;
  unsigned char ch;

  xf86DrvMsg(b->scrnIndex, X_ERROR, "detecting ch7xxx\n");
  
  sil = xcalloc(1, sizeof(CH7xxxRec));
  if (sil == NULL)
    return NULL;

  sil->d.DevName = "CH7xxx TMDS Controller";
  sil->d.SlaveAddr = addr;
  sil->d.pI2CBus = b;
  sil->d.StartTimeout = b->StartTimeout;
  sil->d.BitTimeout = b->BitTimeout;
  sil->d.AcknTimeout = b->AcknTimeout;
  sil->d.ByteTimeout = b->ByteTimeout;
  sil->d.DriverPrivate.ptr = sil;

  if (!ch7xxxReadByte(sil, CH7xxx_REG_VID, &ch))
    goto out;

  if (ch!=(CH7xxx_VID & 0xFF))
  {
    xf86DrvMsg(sil->d.pI2CBus->scrnIndex, X_ERROR, "ch7xxx not detected got %d: from %s Slave %d.\n", ch, sil->d.pI2CBus->BusName, sil->d.SlaveAddr);
    goto out;
  }


  if (!ch7xxxReadByte(sil, CH7xxx_REG_DID, &ch))
    goto out;

  if (ch!=(CH7xxx_DID & 0xFF))
  {
    xf86DrvMsg(sil->d.pI2CBus->scrnIndex, X_ERROR, "ch7xxx not detected got %d: from %s Slave %d.\n", ch, sil->d.pI2CBus->BusName, sil->d.SlaveAddr);
    goto out;
  }


  if (!xf86I2CDevInit(&(sil->d)))
  {
    goto out;
  }

  return sil;
  
 out:
  xfree(sil);
  return NULL;
}


static Bool ch7xxxInit(I2CDevPtr d)
{
  CH7xxxPtr sil = CH7PTR(d);

  /* not much to do */
  return TRUE;
}

static ModeStatus ch7xxxModeValid(I2CDevPtr d, DisplayModePtr mode)
{
  CH7xxxPtr sil = CH7PTR(d);
  
  return MODE_OK;
}

static void ch7xxxMode(I2CDevPtr d, DisplayModePtr mode)
{
  CH7xxxPtr sil = CH7PTR(d);

  /* don't do much */
  return;
}

/* set the CH7xxx power state */
static void ch7xxxPower(I2CDevPtr d, Bool On)
{
  CH7xxxPtr sil = CH7PTR(d);
  int ret;
  unsigned char ch;
  
  ret = ch7xxxReadByte(sil, CH7xxx_REG8, &ch);
  if (ret)
    return;

  if (On)
    ch |= CH7xxx_8_PD;
  else
    ch &= ~CH7xxx_8_PD;

  ch7xxxWriteByte(sil, CH7xxx_REG8, ch);
  return;
}

static void ch7xxxPrintRegs(I2CDevPtr d)
{
  CH7xxxPtr sil = CH7PTR(d);
}

static void ch7xxxSaveRegs(I2CDevPtr d)
{
  CH7xxxPtr sil = CH7PTR(d);
  
  if (!ch7xxxReadByte(sil, CH7xxx_FREQ_LO, &sil->SavedReg.freq_lo))
      return;

  if (!ch7xxxReadByte(sil, CH7xxx_FREQ_HI, &sil->SavedReg.freq_hi))
      return;

  if (!ch7xxxReadByte(sil, CH7xxx_REG8, &sil->SavedReg.reg8))
    return;
  
  if (!ch7xxxReadByte(sil, CH7xxx_REG9, &sil->SavedReg.reg9))
    return;

  if (!ch7xxxReadByte(sil, CH7xxx_REGC, &sil->SavedReg.regc))
    return;
  
  return;

}

I830I2CVidOutputRec CH7xxxVidOutput = {
  ch7xxxDetect,
  ch7xxxInit,
  ch7xxxModeValid,
  ch7xxxMode,
  ch7xxxPower,
  ch7xxxPrintRegs,
  ch7xxxSaveRegs,
  NULL,
};
