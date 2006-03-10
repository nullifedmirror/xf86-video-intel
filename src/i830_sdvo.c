
#include "xf86.h"
#include "xf86_ansic.h"
#include "xf86_OSproc.h"
#include "compiler.h"
#include "i830.h"

/* SDVO support for i9xx chipsets */
static Bool sReadByte(I830SDVOPtr s, int addr, unsigned char *ch)
{
  if (!xf86I2CReadByte(&(s->d), addr, ch)) {
    xf86DrvMsg(s->d.pI2CBus->scrnIndex, X_ERROR, "Unable to read from %s Slave %d.\n", s->d.pI2CBus->BusName, s->d.SlaveAddr);
    return FALSE;
  }
  return TRUE;
}

static Bool sWriteByte(I830SDVOPtr s, int addr, unsigned char ch)
{
  if (!xf86I2CWriteByte(&(s->d), addr, ch)) {
    xf86DrvMsg(s->d.pI2CBus->scrnIndex, X_ERROR, "Unable to write to %s Slave %d.\n", s->d.pI2CBus->BusName, s->d.SlaveAddr);
    return FALSE;
  }
  return TRUE;
}

void *
I830SDVOInit(I2CBusPtr b)
{
  /**/
  I830SDVOPtr sdvo;
  
  sdvo = xcalloc(1, sizeof(I830SDVORec));
  if (sdvo==NULL)
    return NULL;

  sdvo->d.DevName = "SDVO Controller";
  sdvo->d.SlaveAddr = 0x39 << 1;
  sdvo->d.pI2CBus = b;
  sdvo->d.StartTimeout = b->StartTimeout;
  sdvo->d.BitTimeout = b->BitTimeout;
  sdvo->d.AcknTimeout = b->AcknTimeout;
  sdvo->d.ByteTimeout = b->ByteTimeout;
  sdvo->d.DriverPrivate.ptr = sdvo;

  if (!xf86I2CDevInit(&(sdvo->d)))
      goto out;
  return sdvo;

 out:
  xfree(sdvo);
  return NULL;
}

/* following on from tracing the intel BIOS i2c routines */
Bool
I830SDVOWriteOutputRegs(I830SDVOPtr s)
{
  int i;

  /* blast the output regs */
  for (i = 7; i >= 0; i--)
  {
    sWriteByte(s, i, s->sdvo_regs[i]);
  }
  
  /* blast the command reg */
  sWriteByte(s, 8, s->sdvo_regs[8]);
}

/* the INTEL BIOS seems to always read back a bunch of regs
   from 0x09 to 0x11 in reverse order */
Bool
I830SDVOReadInputRegs(I830SDVOPtr s)
{
  int i;

  for (i=0x11; i>=0x09; i--)
  {
    sReadByte(s, i, &s->sdvo_regs[i]);
  }
}

Bool
I830SDVOGetStatus(I830SDVOPtr s)
{
  /* write out 0x10 */

  s->sdvo_regs[0x08] = 0x10;

  I830SDVOWriteOutputRegs(s);
  
  I830SDVOReadInputRegs(s);

  ErrorF("status is %d\n", s->sdvo_regs[0x09]);

  return TRUE;
}
