#ifndef SIL164_H
#define SIL164_H

#define SIL164_VID 0x0001
#define SIL164_DID 0x0006

#define SIL164_VID_LO 0x00
#define SIL164_VID_HI 0x01
#define SIL164_DID_LO 0x02
#define SIL164_DID_HI 0x03
#define SIL164_REV    0x04
#define SIL164_RSVD   0x05
#define SIL164_FREQ_LO 0x06
#define SIL164_FREQ_HI 0x07

#define SIL164_REG8 0x08
#define SIL164_8_VEN (1<<5)
#define SIL164_8_HEN (1<<4)
#define SIL164_8_DSEL (1<<3)
#define SIL164_8_BSEL (1<<2)
#define SIL164_8_EDGE (1<<1)
#define SIL164_8_PD   (1<<0)

#define SIL164_REG9 0x09
#define SIL164_9_VLOW (1<<7)
#define SIL164_9_MSEL_MASK (0x7<<4)
#define SIL164_9_TSEL (1<<3)
#define SIL164_9_RSEN (1<<2)
#define SIL164_9_HTPLG (1<<1)
#define SIL164_9_MDI (1<<0)

#define SIL164_REGC 0x0c

typedef struct _Sil164SaveRec {
  CARD8 freq_lo;
  CARD8 freq_hi;
  CARD8 reg8;
  CARD8 reg9;
  CARD8 regc;
} SIL164SaveRec;

typedef struct {
  I2CDevRec d;
  SIL164SaveRec SavedReg;
  SIL164SaveRec ModeReg;
} SIL164Rec, *SIL164Ptr;

#define SIL164_ADDR_1 0x38

#define SIL164_SYMBOL_LIST "SIL164VidOutput"

#define SILPTR(d) ((SIL164Ptr)(d->DriverPrivate.ptr))
#endif
