/**************************************************************************

 Copyright 2006 Dave Airlie <airlied@linux.ie>
 
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

#ifndef CH7xxx_REG_H
#define CH7xxx_REG_H

#define CH7xxx_REG_VID 0x4a
#define CH7xxx_REG_DID 0x4b

#define CH7xxx_VID 0x84
#define CH7xxx_DID 0x17

typedef struct _CH7xxxSaveRec {
  CARD8 freq_lo;
  CARD8 freq_hi;
  CARD8 reg8;
  CARD8 reg9;
  CARD8 regc;
} CH7xxxSaveRec;

typedef struct {
  I2CDevRec d;
  CH7xxxSaveRec SavedReg;
  CH7xxxSaveRec ModeReg;
} CH7xxxRec, *CH7xxxPtr;

#define CH7PTR(d) ((CH7xxxPtr)(d->DriverPrivate.ptr))

#endif
