
#include "xf86.h"
#include "xf86_ansic.h"
#include "xf86_OSproc.h"
#include "compiler.h"
#include "i830.h"

/* THINGS I KNOW ABOUT SDVO 
   0x02 GET DEVICE CAPS

   0x03 GET TRAINED INPUTS

   0x04 GET ACTIVE OUTPUTS
   0x05 SET ACTIVE OUTPUTS

   0x06 GET IN OUT MAP
   0x0b GET ATTACHED DISPLAYS
   
   0x10 SET TARGET INPUT  - 
   0x11 SET TARGET OUTPUT - 

   0x14 SET INPUT TIMINGS
   0x15 SET INPUT TIMINGS

   0x16 SET OUTPUT TIMINGS
   0x17 SET OUTPUT TIMINGS 2

   0x18 GET OUTPUT TIMINGS
   0x19 GET OUTPUT TIMINGS 2

   0x1A GET PREFERRED INPUT TIMINGS
   0x1B GET PREFERRED INPUT TIMINGS
       response packet 01 30 2A 00 98 40 00
   0x1C GET PREFERRED INPUT TIMINGS

   0x1D GET INPUT PIXEL CLOCK RANGE COMMAND

   0x20 GET CLOCK RATE MULTIPLIER
   0x21 SET CLOCK RATE MULTIPLIER

   0x2B GET POWER STATE

   0x7d SET POWER STATE	
   
*/


unsigned short magic_table[][6] =
  { 
    { 0x402A, 0x0051, 0x9800, 0x7030, 0x0013, 0x001e }, // 1280
    { 0x3026, 0x0041, 0x4000, 0x8818, 0x0036, 0x0018 }, // 1024
    { 0x201c, 0x5831, 0x0020, 0x8028, 0x0014, 0x001e }, // 800
    { 0x102d, 0xe020, 0xa080, 0x6010, 0x00a2, 0x0018 }, //640
  };

unsigned short curr_table[6];
unsigned short out_timings[6];

unsigned char c16a[8];
unsigned char c17a[8];


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
I830SDVOWriteOutputs(I830SDVOPtr s)
{
  int i;

  ErrorF("SDVO: W: ");
  for (i = 0; i<9; i++)
    ErrorF("%02X ", s->sdvo_regs[i]);
  ErrorF("\n");
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
  
  /* follow BIOS ordering */
  sReadByte(s, 0x09, &s->sdvo_regs[0x09]);
  
  sReadByte(s, 0x0D, &s->sdvo_regs[0x0d]);
  sReadByte(s, 0x0C, &s->sdvo_regs[0x0c]);
  sReadByte(s, 0x0B, &s->sdvo_regs[0x0b]);
  sReadByte(s, 0x0A, &s->sdvo_regs[0x0a]);
  sReadByte(s, 0x11, &s->sdvo_regs[0x11]);
  sReadByte(s, 0x10, &s->sdvo_regs[0x10]);
  sReadByte(s, 0x0f, &s->sdvo_regs[0x0f]);
  sReadByte(s, 0x0E, &s->sdvo_regs[0x0e]);

  ErrorF("SDVO: R: ");
  for (i=0x09; i< 0x12; i++)
    ErrorF("%02X ", s->sdvo_regs[i]);
  ErrorF("\n");
}

Bool
I830SDVOWriteCommand10(I830SDVOPtr s)
{
  /* write out 0x10 */
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x10;

  I830SDVOWriteOutputs(s);
  
  I830SDVOReadInputRegs(s);

  return TRUE;
}

Bool
I830DoSDVOTrans(I830SDVOPtr s, char cmd)
{
  //  memcpy(&s->sdvo_regs[0], cmdargs, 8);
  s->sdvo_regs[0x08] = cmd;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);

  return TRUE;
}

Bool
I830SDVOWriteCommand03(I830SDVOPtr s, int on)
{
  memset(s->sdvo_regs, 0, 9);

  s->sdvo_regs[0x08] = 0x3;
  

  s->sdvo_regs[0x07] = on ? 0x80 : 0x00;
  s->sdvo_regs[0x04] = on ? 0x80 : 0x00;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}

Bool
I830SDVOWriteCommand04(I830SDVOPtr s, int on)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x4;

  s->sdvo_regs[0x07] = on ? 0x01 : 0x00;
  s->sdvo_regs[0x03] = 0x1;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}


Bool
I830SDVOWriteCommand05(I830SDVOPtr s, int on)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x5;

  s->sdvo_regs[0x07] = on ? 0x01 : 0x00;
  s->sdvo_regs[0x03] = on ? 0x01 : 0x00;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}


Bool
I830SDVOWriteCommand1D(I830SDVOPtr s, unsigned short clock, unsigned short height)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x1d;
  
  /* set clock regs */
  s->sdvo_regs[0x06] = (clock >> 8) & 0xff;
  s->sdvo_regs[0x07] = clock & 0xff;

  /* set height regs */
  s->sdvo_regs[0x02] = (height >> 8) & 0xff;
  s->sdvo_regs[0x03] = height & 0xff;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}


Bool
I830SDVOWriteCommand11(I830SDVOPtr s)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x11;
  
  s->sdvo_regs[0x07] = 0x1;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}


Bool
I830SDVOWriteCommand18(I830SDVOPtr s)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x18;
  
  s->sdvo_regs[0x07] = 0x0;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}

Bool
I830SDVOWriteCommand19(I830SDVOPtr s)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x19;
  
  s->sdvo_regs[0x07] = 0x0;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}

Bool
I830SDVOWriteCommand161C14(I830SDVOPtr s, char cmd, unsigned short clock, unsigned short magic1, unsigned short magic2, unsigned short magic3)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = cmd;
  
  /* set clock regs */
  s->sdvo_regs[0x06] = (clock >> 8) & 0xff;
  s->sdvo_regs[0x07] = clock & 0xff;

  s->sdvo_regs[0x00] = (magic1 >> 8) & 0xff;
  s->sdvo_regs[0x01] = magic1 & 0xff;

  s->sdvo_regs[0x02] = (magic2 >> 8) & 0xff;
  s->sdvo_regs[0x03] = magic2 & 0xff;  

  s->sdvo_regs[0x04] = (magic3 >> 8) & 0xff;
  s->sdvo_regs[0x05] = magic3 & 0xff;  

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}

Bool
I830SDVOWriteCommand14(I830SDVOPtr s, unsigned short clock, unsigned short magic1, unsigned short magic2, unsigned short magic3)
{
  return I830SDVOWriteCommand161C14(s, 0x14, clock, magic1, magic2, magic3);
}

Bool
I830SDVOWriteCommand16(I830SDVOPtr s, unsigned short clock, unsigned short magic1, unsigned short magic2, unsigned short magic3)
{
  return I830SDVOWriteCommand161C14(s, 0x16, clock, magic1, magic2, magic3);
}

Bool
I830SDVOWriteCommand1C(I830SDVOPtr s, unsigned short clock, unsigned short magic1, unsigned short magic2, unsigned short magic3)
{
  return I830SDVOWriteCommand161C14(s, 0x1C, clock, magic1, magic2, magic3);
}

Bool
I830SDVOWriteCommand1517(I830SDVOPtr s, char cmd, unsigned short magic4, unsigned short magic5, unsigned short magic6)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = cmd;
 
  /* set clock regs */
  s->sdvo_regs[0x06] = (magic4 >> 8) & 0xff;
  s->sdvo_regs[0x07] = magic4 & 0xff;

  s->sdvo_regs[0x04] = (magic5 >> 8) & 0xff;
  s->sdvo_regs[0x05] = magic5 & 0xff;

  s->sdvo_regs[0x02] = (magic6 >> 8) & 0xff;
  s->sdvo_regs[0x03] = magic6 & 0xff;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}

Bool
I830SDVOWriteCommand15(I830SDVOPtr s, unsigned short magic4, unsigned short magic5, unsigned short magic6)
{
  return I830SDVOWriteCommand1517(s, 0x15, magic4, magic5, magic6);
}

Bool
I830SDVOWriteCommand17(I830SDVOPtr s, unsigned short magic4, unsigned short magic5, unsigned short magic6)
{
  return I830SDVOWriteCommand1517(s, 0x17, magic4, magic5, magic6);
}

Bool
I830SDVOWriteCommand1A(I830SDVOPtr s, unsigned short clock, unsigned short width, unsigned short height)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x1A;
  
  /* set clock regs */
  s->sdvo_regs[0x06] = (clock >> 8) & 0xff;
  s->sdvo_regs[0x07] = clock & 0xff;

  s->sdvo_regs[0x02] = (height >> 8) & 0xff;
  s->sdvo_regs[0x03] = height & 0xff;  

  s->sdvo_regs[0x04] = (width >> 8) & 0xff;
  s->sdvo_regs[0x05] = width & 0xff;

  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}

Bool
I830SDVOWriteCommand1B(I830SDVOPtr s)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x1B;
  
  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}

Bool
I830SDVOParseResponse1B(I830SDVOPtr s)
{
  /* 1B response */
  curr_table[0] = (s->sdvo_regs[0x10] | (s->sdvo_regs[0x11] << 8));

  curr_table[1] = (s->sdvo_regs[0x0e] | (s->sdvo_regs[0x0f] << 8));

  curr_table[2] = (s->sdvo_regs[0x0c] | (s->sdvo_regs[0x0d] << 8));

}

Bool
I830SDVOParseResponse1C(I830SDVOPtr s)
{
  curr_table[3] = (s->sdvo_regs[0x0a] | (s->sdvo_regs[0x0b] << 8));
  curr_table[4] = (s->sdvo_regs[0x0c] | (s->sdvo_regs[0x0d] << 8));

  curr_table[5] = 0x1e;

}
Bool
I830SDVOWriteCommand21(I830SDVOPtr s, unsigned char val)
{
  memset(s->sdvo_regs, 0, 9);
  
  s->sdvo_regs[0x08] = 0x21;
  
  s->sdvo_regs[0x07] = val;
  I830SDVOWriteOutputs(s);
  I830SDVOReadInputRegs(s);
  
  return TRUE;
}

Bool
I830SDVOPreSetMode(I830SDVOPtr s, DisplayModePtr mode)
{
  unsigned short clock = mode->Clock/10, width=mode->CrtcHDisplay, height=mode->CrtcVDisplay;
  unsigned short *m_p;
  unsigned short h_blank_len, h_sync_len, v_blank_len, v_sync_len, h_sync_offset, v_sync_offset;
  unsigned short sync_flags;

  /* do some mode translations */
  h_blank_len = mode->CrtcHBlankEnd - mode->CrtcHBlankStart;
  h_sync_len = mode->CrtcHSyncEnd - mode->CrtcHSyncStart;
  
  v_blank_len = mode->CrtcVBlankEnd - mode->CrtcVBlankStart;
  v_sync_len = mode->CrtcVSyncEnd - mode->CrtcVSyncStart;
  
  h_sync_offset = mode->CrtcHSyncStart - mode->CrtcHBlankStart;
  v_sync_offset = mode->CrtcVSyncStart - mode->CrtcVBlankStart;

  sync_flags = 0x18;
  if (mode->Flags & V_PHSYNC)
    sync_flags |= 0x2;
  if (mode->Flags & V_PVSYNC)
    sync_flags |= 0x4;
  /* high bits of 0 */
  c16a[7] = clock & 0xff;
  c16a[6] = (clock >> 8) & 0xff;
  c16a[5] = (width & 0xff);
  c16a[4] = (h_blank_len & 0xff);
  c16a[3] = (((width >> 8) & 0xf) << 4) | ((h_blank_len >> 8) & 0xf);
  c16a[2] = (height & 0xff);
  c16a[1] = (v_blank_len & 0xff);
  c16a[0] = (((height >> 8) & 0xf) << 4) | ((v_blank_len >> 8) & 0xf);

  c17a[7] = h_sync_offset;
  c17a[6] = h_sync_len & 0xff;
  c17a[5] = (v_sync_offset & 0xf) << 4 | (v_sync_len & 0xf);
  c17a[4] = 0;
  c17a[3] = sync_flags;
  c17a[2] = 0;
  out_timings[0] = c16a[1] | ((short)c16a[0] << 8);
  out_timings[1] = c16a[3] | ((short)c16a[2] << 8);
  out_timings[2] = c16a[5] | ((short)c16a[4] << 8);
  out_timings[3] = c17a[7] | ((short)c17a[6] << 8);
  out_timings[4] = c17a[5] | ((short)c17a[4] << 8);
  out_timings[5] = c17a[3] | ((short)c17a[2] << 8);
  
  I830SDVOWriteCommand10(s);
  I830SDVOWriteCommand1D(s, clock, height);

  I830SDVOWriteCommand04(s, 0);
  I830SDVOWriteCommand05(s, 0);

  I830SDVOWriteCommand11(s);
  I830SDVOWriteCommand16(s, clock, out_timings[0], out_timings[1], out_timings[2]);

  I830SDVOWriteCommand11(s);
  I830SDVOWriteCommand17(s, out_timings[3], out_timings[4],out_timings[5]);

  I830SDVOWriteCommand10(s);

  I830SDVOWriteCommand1A(s, clock, width, height);
  I830SDVOWriteCommand10(s);

  I830SDVOWriteCommand1B(s);
  I830SDVOParseResponse1B(s);
  I830SDVOWriteCommand10(s);


  I830SDVOWriteCommand1C(s, clock, out_timings[0], out_timings[1], out_timings[2]);
  I830SDVOParseResponse1C(s);
  I830SDVOWriteCommand10(s);


  I830SDVOWriteCommand14(s, clock, curr_table[0], curr_table[1], curr_table[2]);

  I830SDVOWriteCommand10(s);
  I830SDVOWriteCommand15(s, curr_table[3], curr_table[4], out_timings[5]);

  I830SDVOWriteCommand10(s);
  I830SDVOWriteCommand21(s, 0x01);
  
}


Bool
I830SDVOPostSetMode(I830SDVOPtr s, DisplayModePtr mode)
{
  int clock = mode->Clock/10, height=mode->CrtcVDisplay;

  /* the BIOS writes out 6 commands post mode set */
  /* two 03s, 04 05, 10, 1d */
  /* these contain the height and mode clock / 10 by the looks of it */

  I830SDVOWriteCommand03(s, 1);
  I830SDVOWriteCommand03(s, 0);
  I830SDVOWriteCommand04(s, 1);
  I830SDVOWriteCommand05(s, 1);

  I830SDVOWriteCommand10(s);
  I830SDVOWriteCommand1D(s, clock, height);

}

char sdvo_cmds_1024[][9] = 
  { { 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 },
    { 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x19, 0x64, 0x1D }, // 2A30 = 10800

    { 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04 }, // same
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05 }, // same

    { 0x30, 0x26, 0x00, 0x41, 0x40, 0x00, 0x00, 0x01, 0x11 }, // 5198 = 20888
    { 0x30, 0x26, 0x00, 0x41, 0x40, 0x00, 0x19, 0x64, 0x16 }, // 402A = 16426
    { 0x00, 0x00, 0x00, 0x18, 0x00, 0x36, 0x00, 0x01, 0x11 },
    { 0x00, 0x00, 0x00, 0x18, 0x00, 0x36, 0x88, 0x18, 0x17 },

    { 0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x00, 0x00, 0x10 },

    { 0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x19, 0x64, 0x1a },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 },

    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B },
    { 0x30, 0x26, 0x00, 0x41, 0x40, 0x00, 0x00, 0x00, 0x10 },

    { 0x30, 0x26, 0x00, 0x41, 0x40, 0x00, 0x19, 0x64, 0x1C },
    { 0x30, 0x26, 0x00, 0x41, 0x40, 0x00, 0x00, 0x00, 0x10 },

    { 0x30, 0x26, 0x00, 0x41, 0x40, 0x00, 0x19, 0x64, 0x14 },
    { 0x00, 0x00, 0x00, 0x18, 0x00, 0x36, 0x00, 0x00, 0x10 },

    { 0x00, 0x00, 0x00, 0x18, 0x00, 0x36, 0x88, 0x18, 0x15 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 },

    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x21 },

    // MODE SETTING HAPPENS IN HERE

    { 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x03 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 },
    { 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x04 },
    { 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x05 },

    { 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 }, // same as 1
    { 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x19, 0x64, 0x1D }, // same as 2
  };

char sdvo_cmds_1280[][9] =
  { { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 },
    { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x2A, 0x30, 0x1D }, // 2A30 = 10800

    { 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05 },

    { 0x40, 0x2A, 0x00, 0x51, 0x98, 0x00, 0x00, 0x01, 0x11 }, // 5198 = 20888
    { 0x40, 0x2A, 0x00, 0x51, 0x98, 0x00, 0x2a, 0x30, 0x16 }, // 402A = 16426
    { 0x00, 0x00, 0x00, 0x1E, 0x00, 0x13, 0x00, 0x01, 0x11 },
    { 0x00, 0x00, 0x00, 0x1E, 0x00, 0x13, 0x70, 0x30, 0x17 },

    { 0x00, 0x00, 0x04, 0x00, 0x05, 0x00, 0x00, 0x00, 0x10 },

    { 0x00, 0x00, 0x04, 0x00, 0x05, 0x00, 0x2a, 0x30, 0x1a },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 },

    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B },
    { 0x40, 0x2A, 0x00, 0x51, 0x98, 0x00, 0x00, 0x00, 0x10 },

    { 0x40, 0x2A, 0x00, 0x51, 0x98, 0x00, 0x2a, 0x30, 0x1C },
    { 0x40, 0x2A, 0x00, 0x51, 0x98, 0x00, 0x00, 0x00, 0x10 },

    { 0x40, 0x2A, 0x00, 0x51, 0x98, 0x00, 0x2a, 0x30, 0x14 },
    { 0x00, 0x00, 0x00, 0x1E, 0x00, 0x13, 0x00, 0x00, 0x10 },

    { 0x00, 0x00, 0x00, 0x1E, 0x00, 0x13, 0x70, 0x30, 0x15 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 },

    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x21 },

    // MODE SETTING HAPPENS IN HERE

    { 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x03 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 },
    { 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x04 },
    { 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x05 },

    { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 }, // same as 1
    { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x2A, 0x30, 0x1D }, // same as 2
  };
    

Bool
I830SDVOModeSetup(I830SDVOPtr s)
{

}

