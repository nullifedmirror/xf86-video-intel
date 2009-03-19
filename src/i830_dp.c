/*
 * Copyright © 2008 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Keith Packard <keithp@keithp.com>
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "xf86.h"
#include "i830.h"
#include "xf86Modes.h"
#include "i830_display.h"
#include "i830_dp.h"
#include <unistd.h>

struct i830_dp_priv {
    uint32_t output_reg;
    uint32_t save_DP;
    uint8_t  save_link_configuration[0x9];
    Bool has_audio;
    uint16_t i2c_address;
    Bool i2c_running;
};

static void
i830_dp_link_train(xf86OutputPtr output, uint32_t DP);

static int
i830_dp_mode_valid(xf86OutputPtr output, DisplayModePtr mode)
{
    if (mode->Clock > 250000)
	return MODE_CLOCK_HIGH;

    if (mode->Clock < 20000)
	return MODE_CLOCK_LOW;

    return MODE_OK;
}

static int
i830_dp_get_pixel_multiplier(DisplayModePtr pMode)
{
    if (pMode->Clock >= 100000)
	return 1;
    else if (pMode->Clock >= 50000)
	return 2;
    else
	return 4;
}

static Bool
i830_dp_mode_fixup(xf86OutputPtr output, DisplayModePtr mode,
		     DisplayModePtr adjusted_mode)
{
    /* Make the CRTC code factor in the SDVO pixel multiplier.  The SDVO
     * device will be told of the multiplier during mode_set.
     */
    adjusted_mode->Clock *= i830_dp_get_pixel_multiplier(mode);
    return TRUE;
}

static uint32_t
pack_aux(uint8_t *src, int src_bytes)
{
    int	i;
    uint32_t v = 0;

    if (src_bytes > 4)
	src_bytes = 4;
    for (i = 0; i < src_bytes; i++)
	v |= ((uint32_t) src[i]) << ((3-i) * 8);
    return v;
}

static void
unpack_aux(uint32_t src, uint8_t *dst, int dst_bytes)
{
    int i;
    if (dst_bytes > 4)
	dst_bytes = 4;
    for (i = 0; i < dst_bytes; i++)
	dst[i] = src >> ((3-i) * 8);
}

static int
i830_dp_aux_ch(ScrnInfoPtr pScrn, uint32_t output_reg,
	       uint8_t *send, int send_bytes,
	       uint8_t *recv, int recv_size)
{
    I830Ptr pI830 = I830PTR(pScrn);
    uint32_t	ch_ctl = output_reg + 0x10;
    uint32_t	ch_data = ch_ctl + 4;
    int		i;
    int		recv_bytes;
    uint32_t	ctl;
    uint32_t	status;

    /* Load the send data into the aux channel data registers */
    for (i = 0; i < send_bytes; i += 4) {
	uint32_t    d = pack_aux(send + i, send_bytes - i);;

//	xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
//		   "aux_ch send[%d] = %08x\n", i/4, d);
	OUTREG(ch_data + i, d);
    }

    /* The clock divider is based off the hrawclk,
     * and would like to run at 2MHz. The 133 below assumes
     * a 266MHz hrawclk; need to figure out how we're supposed
     * to know what hrawclk is...
     */
    ctl = (DP_AUX_CH_CTL_SEND_BUSY |
	   DP_AUX_CH_CTL_TIME_OUT_1600us |
	   (send_bytes << DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT) |
	   (5 << DP_AUX_CH_CTL_PRECHARGE_2US_SHIFT) |
	   (133 << DP_AUX_CH_CTL_BIT_CLOCK_2X_SHIFT) |
	   DP_AUX_CH_CTL_TIME_OUT_ERROR |
	   DP_AUX_CH_CTL_RECEIVE_ERROR);

    /* Send the command and wait for it to complete */
    OUTREG(ch_ctl, ctl);
    for (;;) {
	status = INREG(ch_ctl);
	if ((status & DP_AUX_CH_CTL_SEND_BUSY) == 0)
	    break;
	usleep(100);
    }

    /* Clear done status and any errors */
    OUTREG(ch_ctl, (ctl |
		    DP_AUX_CH_CTL_DONE |
		    DP_AUX_CH_CTL_TIME_OUT_ERROR |
		    DP_AUX_CH_CTL_RECEIVE_ERROR));

    if ((status & DP_AUX_CH_CTL_DONE) == 0)
    {
	xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		   "DisplayPort AUX CH failed to complete: 0x%08x\n",
		   status);
	return -1;
    }

    /* Unload any bytes sent back from the other side */
    recv_bytes = ((status & DP_AUX_CH_CTL_MESSAGE_SIZE_MASK) >>
		  DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT);

    if (recv_bytes > recv_size)
	recv_bytes = recv_size;
    for (i = 0; i < recv_bytes; i += 4) {
	uint32_t    d = INREG(ch_data + i);

//	xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
//		   "aux_ch recv[%d] = %08x\n", i/4, d);
	unpack_aux(d, recv + i, recv_bytes - i);
    }

    return recv_bytes;
}

/* Write data to the aux channel in native mode */
static int
i830_dp_aux_native_write(ScrnInfoPtr pScrn, uint32_t output_reg,
			 uint16_t address, uint8_t *send, int send_bytes)
{
    int		ret;
    uint8_t	msg[20];
    int		msg_bytes;
    uint8_t	ack;

    assert(send_bytes <= 16);
    msg[0] = AUX_NATIVE_WRITE << 4;
    msg[1] = address >> 8;
    msg[2] = address;
    msg[3] = send_bytes - 1;
    memcpy(&msg[4], send, send_bytes);
    msg_bytes = send_bytes + 4;
    for (;;) {
	ret = i830_dp_aux_ch(pScrn, output_reg, msg, msg_bytes, &ack, 1);
	if (ret < 0)
	    return ret;
        if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_ACK)
	    break;
	else if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_DEFER)
	    usleep(100);
	else {
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "aux ch native write returns %08x\n", ack);
	    return -1;
	}
    }
    return send_bytes;
}

/* Write a single byte to the aux channel in native mode */
static int
i830_dp_aux_native_write_1(ScrnInfoPtr pScrn, uint32_t output_reg,
			   uint16_t address, uint8_t byte)
{
    return i830_dp_aux_native_write(pScrn, output_reg, address, &byte, 1);
}

/* read bytes from a native aux channel */
static int
i830_dp_aux_native_read(ScrnInfoPtr pScrn, uint32_t output_reg,
			uint16_t address, uint8_t *recv, int recv_bytes)
{
    uint8_t msg[4];
    int msg_bytes;
    uint8_t reply[20];
    int reply_bytes;
    uint8_t ack;
    int ret;

    msg[0] = AUX_NATIVE_READ << 4;
    msg[1] = address >> 8;
    msg[2] = address & 0xff;
    msg[3] = recv_bytes - 1;

    msg_bytes = 4;
    reply_bytes = recv_bytes + 1;

    for (;;) {
	ret = i830_dp_aux_ch(pScrn, output_reg, msg, msg_bytes,
			     reply, reply_bytes);
	if (ret <= 0)
	    return ret;
	ack = reply[0];
        if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_ACK) {
	    memcpy(recv, reply + 1, ret - 1);
	    return ret - 1;
	}
	else if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_DEFER)
	    usleep(100);
	else {
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "aux ch native write returns %08x\n", ack);
	    return -1;
	}
    }
}

/* Run a single AUX_CH I2C transaction, writing/reading data as necessary */

enum dp_aux_i2c_mode {
    aux_i2c_start,
    aux_i2c_write,
    aux_i2c_read,
    aux_i2c_stop
};

static int
i830_dp_aux_i2c_transaction(ScrnInfoPtr pScrn, uint32_t output_reg,
			    uint16_t address,
			    enum dp_aux_i2c_mode mode,
			    uint8_t write_byte, uint8_t *read_byte)
{
    uint8_t msg[5];
    uint8_t reply[2];
    int msg_bytes;
    int reply_bytes;
    int ret;

    /* Set up the command byte */
    if (address & 1)
	msg[0] = AUX_I2C_READ << 4;
    else
	msg[0] = AUX_I2C_WRITE << 4;

    if (mode != aux_i2c_stop)
	msg[0] |= AUX_I2C_MOT << 4;

    /* Note that the AUX_CH I2C stuff wants the read/write
     * bit stripped off
     */
    msg[1] = address >> 9;
    msg[2] = address >> 1;

    switch (mode) {
    case aux_i2c_start:
    case aux_i2c_stop:
    default:
        msg_bytes = 3;
        reply_bytes = 1;
	break;
    case aux_i2c_write:
	msg[3] = 0;
	msg[4] = write_byte;
	msg_bytes = 5;
	reply_bytes = 1;
	break;
    case aux_i2c_read:
	msg[3] = 0;
	msg_bytes = 4;
	reply_bytes = 2;
	break;
    }

    for (;;) {
	ret = i830_dp_aux_ch(pScrn, output_reg, msg, msg_bytes,
			     reply, reply_bytes);
	if (ret <= 0) {
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "i2c_read: aux_ch error %d\n", ret);
	    return -1;
	}
	if ((reply[0] & AUX_I2C_REPLY_MASK) == AUX_I2C_REPLY_ACK) {
	    if (mode == aux_i2c_read)
		*read_byte = reply[1];
	    return reply_bytes - 1;
	}
	else if ((reply[0] & AUX_I2C_REPLY_MASK) == AUX_I2C_REPLY_DEFER)
	    usleep(100);
	else {
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "aux ch i2c write returns %02x\n", reply[0]);
	    return -1;
	}
    }
}

static void
i830_dp_mode_set(xf86OutputPtr output, DisplayModePtr mode,
		   DisplayModePtr adjusted_mode)
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);
    xf86CrtcPtr crtc = output->crtc;
    I830CrtcPrivatePtr intel_crtc = crtc->driver_private;
    uint32_t dp;

    dp = (DP_LINK_TRAIN_OFF |
	  DP_VOLTAGE_0_4 |
	  DP_PRE_EMPHASIS_0 |
	  DP_PORT_WIDTH_4 |
	  DP_ENHANCED_FRAMING |
	  DP_SYNC_VS_HIGH |
	  DP_SYNC_HS_HIGH);

    if (dev_priv->has_audio)
	    dp |= DP_AUDIO_OUTPUT_ENABLE;

    if (intel_crtc->pipe == 1)
	dp |= DP_PIPEB_SELECT;

    OUTREG(dev_priv->output_reg, dp);
    POSTING_READ(dev_priv->output_reg);
}

#include "i830_debug.h"

static void
i830_dp_dpms(xf86OutputPtr output, int mode)
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);
    uint32_t  temp;

    if (mode == DPMSModeOff) {
	temp = INREG(dev_priv->output_reg);
	OUTREG(dev_priv->output_reg, temp & ~DP_PORT_EN);
    } else {
	temp = INREG(dev_priv->output_reg);
	temp |= DP_PORT_EN;

	i830_dp_aux_native_write(pScrn, dev_priv->output_reg, 0x100,
				 dev_priv->save_link_configuration, sizeof (dev_priv->save_link_configuration));
	xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		   "%s: i830_dp_link_train\n", __func__);
	i830DumpRegs(pScrn);
	i830_dp_link_train(output, temp);
    }
}

static int
i830_dp_lane_status(xf86OutputPtr output, uint8_t lane_status[3])
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    int ret;

    ret = i830_dp_aux_native_read(pScrn,
				  dev_priv->output_reg, DP_LANE0_1_STATUS,
				  lane_status, 3);
    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
	       "lane status(%d) %02x %02x %02x\n",
	       ret, lane_status[0], lane_status[1], lane_status[2]);
    return ret;
}

static void
i830_dp_save(xf86OutputPtr output)
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);
    uint8_t lane_status[3];

    dev_priv->save_DP = INREG(dev_priv->output_reg);
    i830_dp_aux_native_read(pScrn, dev_priv->output_reg, 0x100,
		     dev_priv->save_link_configuration, sizeof (dev_priv->save_link_configuration));
    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
	       "link configuration: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	       dev_priv->save_link_configuration[0],
	       dev_priv->save_link_configuration[1],
	       dev_priv->save_link_configuration[2],
	       dev_priv->save_link_configuration[3],
	       dev_priv->save_link_configuration[4],
	       dev_priv->save_link_configuration[5],
	       dev_priv->save_link_configuration[6],
	       dev_priv->save_link_configuration[7],
	       dev_priv->save_link_configuration[8]);

    i830_dp_lane_status(output, lane_status);
}

static void
i830_dp_link_train(xf86OutputPtr output, uint32_t DP)
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);
    uint8_t	training[10];
    uint8_t	train_set[4];
    uint8_t lane_status[3];
    uint8_t adjust_request[10];
    int ret;
    int i;
    int voltage;
    int pre_emphasis;
    Bool clock_recovery = FALSE;

    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
	       "i830_dp_link_train dp 0x%08x\n", DP);
    OUTREG(dev_priv->output_reg, DP);
    POSTING_READ(dev_priv->output_reg);

    if ((DP & DP_PORT_EN) == 0) {
	return;
    }
    DP &= ~DP_LINK_TRAIN_MASK;

    usleep (20*1000);

    ret = i830_dp_aux_native_read(pScrn, dev_priv->output_reg,
			   0x206, adjust_request, 2);
    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
	       "adjust_request(%d): %02x %02x\n",
	       ret, adjust_request[0], adjust_request[1]);
    /* main link disabled */
    OUTREG(dev_priv->output_reg, DP | DP_LINK_TRAIN_PAT_1);
    POSTING_READ(dev_priv->output_reg);
    usleep (15*1000);
    i830_dp_aux_native_write_1(pScrn, dev_priv->output_reg,
			DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_1);
    usleep (15*1000);
    ret = i830_dp_aux_native_read(pScrn, dev_priv->output_reg,
			   DP_TRAINING_PATTERN_SET,
			   training, 1);
    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
	       "training pattern 1: %02x\n", training[0]);
    /* clock recovery pattern */
    for (voltage = 0; voltage < 4; voltage++) {
	for (pre_emphasis = 0; pre_emphasis < 4; pre_emphasis++) {
	    memset(train_set, (voltage << 0) | (pre_emphasis << 3), 4);
	    ret = i830_dp_aux_native_write(pScrn, dev_priv->output_reg,
					   DP_TRAINING_LANE0_SET, train_set, 4);
	    if (ret != 4) {
		xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
			   "voltage %d pre_emphasis %d set failed\n",
			   voltage, pre_emphasis);
		continue;
	    }
	    for (i = 0; i < 4; i++) {
		usleep (15*1000);
		ret = i830_dp_lane_status(output, lane_status);
		if (ret != 3)
		    continue;
		if ((lane_status[0] & DP_LANE_CR_DONE) &&
		    (lane_status[0] & (DP_LANE_CR_DONE << 4)) &&
		    (lane_status[1] & DP_LANE_CR_DONE) &&
		    (lane_status[1] & (DP_LANE_CR_DONE << 4)))
		{
		    clock_recovery = TRUE;
		}
		if (clock_recovery)
		    break;
	    }
	    if (clock_recovery)
		break;
	}
	if (clock_recovery)
	    break;
    }
    if (!clock_recovery) {
	xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		   "clock recovery failed\n");
    } else {
	xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		   "clock recovery at voltage %d pre_emphasis %d\n",
		   voltage, pre_emphasis);
    }
    /* channel eq pattern */
    OUTREG(dev_priv->output_reg, DP | DP_LINK_TRAIN_PAT_2);
    POSTING_READ(dev_priv->output_reg);
    usleep (15*1000);
    i830_dp_aux_native_write_1(pScrn, dev_priv->output_reg,
			DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_2);
    usleep(15*1000);
    ret = i830_dp_aux_native_read(pScrn, dev_priv->output_reg,
			   DP_TRAINING_PATTERN_SET,
			   training, 1);
    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
	       "training pattern 2: %02x\n", training[0]);
    for (i = 0; i < 4; i++) {
	int ret;

	ret = i830_dp_aux_native_write(pScrn, dev_priv->output_reg,
				DP_TRAINING_LANE0_SET, train_set, 4);
	if (ret != 4) {
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "voltage swing set failed\n");
	    break;
	}

	usleep(15*1000);
	ret = i830_dp_lane_status(output, lane_status);
	if (ret != 3)
	    break;
	if ((lane_status[0] & DP_LANE_CHANNEL_EQ_DONE) &&
	    (lane_status[0] & (DP_LANE_CHANNEL_EQ_DONE<<4)) &&
	    (lane_status[1] & DP_LANE_CHANNEL_EQ_DONE) &&
	    (lane_status[1] & (DP_LANE_CHANNEL_EQ_DONE<<4)))
	    break;
	if (i == 3) {
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "channel eq failed\n");
	    break;
	}
    }
    OUTREG(dev_priv->output_reg, DP | DP_LINK_TRAIN_OFF);
    POSTING_READ(dev_priv->output_reg);
    usleep (15*1000);
    i830_dp_aux_native_write_1(pScrn, dev_priv->output_reg,
			DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_DISABLE);
    usleep (15*1000);
}

/*
 * I2C over AUX CH
 */

/*
 * Send the address. If the I2C link is running, this 'restarts'
 * the connection with the new address, this is used for doing
 * a write followed by a read (as needed for DDC)
 */
static Bool
i830_dp_i2c_address(I2CDevPtr dev, I2CSlaveAddr addr)
{
    I2CBusPtr bus = dev->pI2CBus;
    xf86OutputPtr output = bus->DriverPrivate.ptr;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    ScrnInfoPtr scrn = output->scrn;

    dev_priv->i2c_address = addr;
    dev_priv->i2c_running = TRUE;
    return i830_dp_aux_i2c_transaction(scrn, dev_priv->output_reg, addr,
				       aux_i2c_start, 0, NULL) >= 0;
}

/* DIX never even calls this function, so it better not be necessary */
static Bool
i830_dp_i2c_start(I2CBusPtr bus, int timeout)
{
    return TRUE;
}

/*
 * Stop the I2C transaction. This closes out the link, sending
 * a bare address packet with the MOT bit turned off
 */
static void
i830_dp_i2c_stop(I2CDevPtr dev)
{
    I2CBusPtr bus = dev->pI2CBus;
    xf86OutputPtr output = bus->DriverPrivate.ptr;
    ScrnInfoPtr scrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;

    if (dev_priv->i2c_running)
	(void) i830_dp_aux_i2c_transaction(scrn, dev_priv->output_reg,
					   dev_priv->i2c_address,
					   aux_i2c_stop, 0, NULL);
    dev_priv->i2c_running = FALSE;
}

/*
 * Write a single byte to the current I2C address, this assumes
 * that the I2C link is running (or presumably it won't work).
 */
static Bool
i830_dp_i2c_put_byte(I2CDevPtr dev, I2CByte byte)
{
    I2CBusPtr bus = dev->pI2CBus;
    xf86OutputPtr output = bus->DriverPrivate.ptr;
    ScrnInfoPtr scrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;

    return i830_dp_aux_i2c_transaction(scrn, dev_priv->output_reg,
				       dev_priv->i2c_address,
				       aux_i2c_write, byte, NULL) >= 0;
}

static Bool
i830_dp_i2c_get_byte(I2CDevPtr dev, I2CByte *byte_ret, Bool last)
{
    I2CBusPtr bus = dev->pI2CBus;
    xf86OutputPtr output = bus->DriverPrivate.ptr;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    ScrnInfoPtr scrn = output->scrn;

    return i830_dp_aux_i2c_transaction(scrn, dev_priv->output_reg,
				       dev_priv->i2c_address,
				       aux_i2c_read, 0, byte_ret) == 1;
}

static Bool
i830_dp_i2c_init(ScrnInfoPtr pScrn, I2CBusPtr *bus_ptr,
		 xf86OutputPtr output, char *name)
{
    I2CBusPtr pI2CBus;

    pI2CBus = xf86CreateI2CBusRec();

    if (!pI2CBus)
	return FALSE;

    pI2CBus->BusName = name;
    pI2CBus->scrnIndex = pScrn->scrnIndex;
    pI2CBus->I2CGetByte = i830_dp_i2c_get_byte;
    pI2CBus->I2CPutByte = i830_dp_i2c_put_byte;
    pI2CBus->I2CAddress = i830_dp_i2c_address;
    pI2CBus->I2CStart = i830_dp_i2c_start;
    pI2CBus->I2CStop = i830_dp_i2c_stop;
    pI2CBus->DriverPrivate.ptr = output;

    /* Assume all busses are used for DDCish stuff */

    /*
     * These were set incorrectly in the server pre-1.3, Having
     * duplicate settings is sub-optimal, but this lets the driver
     * work with older servers
     */
    pI2CBus->ByteTimeout = 2200; /* VESA DDC spec 3 p. 43 (+10 %) */
    pI2CBus->StartTimeout = 550;
    pI2CBus->BitTimeout = 40;
    pI2CBus->AcknTimeout = 40;
    pI2CBus->RiseFallTime = 20;

    if (!xf86I2CBusInit(pI2CBus))
	return FALSE;

    *bus_ptr = pI2CBus;
    return TRUE;
}

static void
i830_dp_restore(xf86OutputPtr output)
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;

    i830_dp_aux_native_write(pScrn, dev_priv->output_reg, 0x100,
		     dev_priv->save_link_configuration, sizeof (dev_priv->save_link_configuration));
    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
	       "%s: i830_dp_link_train\n", __func__);
    i830DumpRegs(pScrn);
    i830_dp_link_train(output, dev_priv->save_DP);
}

/*
 * According to DP spec
 * 5.1.2:
 *  1. Read DPCD
 *  2. Configure link according to Receiver Capabilities
 *  3. Use Link Training from 2.5.3.3 and 3.5.1.3
 *  4. Check link status on receipt of hot-plug interrupt
*/

/**
 * Uses CRT_HOTPLUG_EN and CRT_HOTPLUG_STAT to detect DP connection.
 *
 * \return TRUE if DP port is connected.
 * \return FALSE if DP port is disconnected.
 */
static xf86OutputStatus
i830_dp_detect(xf86OutputPtr output)
{
    ScrnInfoPtr	pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);
    uint32_t temp, bit;

    dev_priv->has_audio = FALSE;

    /* For G4X desktop chip, PEG_BAND_GAP_DATA 3:0 must first be written 0xd.
     * Failure to do so will result in spurious interrupts being
     * generated on the port when a cable is not attached.
     */
    if (IS_G4X(pI830) && !IS_GM45(pI830)) {
	temp = INREG(PEG_BAND_GAP_DATA);
	OUTREG(PEG_BAND_GAP_DATA, (temp & ~0xf) | 0xd);
    }

    temp = INREG(PORT_HOTPLUG_EN);

    OUTREG(PORT_HOTPLUG_EN,
	   temp |
	   DPB_HOTPLUG_INT_EN |
	   DPC_HOTPLUG_INT_EN |
	   DPD_HOTPLUG_INT_EN);

    POSTING_READ(PORT_HOTPLUG_EN);

    switch (dev_priv->output_reg) {
    case DP_B:
	bit = DPB_HOTPLUG_INT_STATUS;
	break;
    case DP_C:
	bit = DPC_HOTPLUG_INT_STATUS;
	break;
    case DP_D:
	bit = DPD_HOTPLUG_INT_STATUS;
	break;
    default:
	return XF86OutputStatusUnknown;
    }

    temp = INREG(PORT_HOTPLUG_STAT);

    if ((temp & bit) == 0)
	return XF86OutputStatusDisconnected;

    if (pI830->debug_modes)
	xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		   "DisplayPort monitor detected on DP-%d\n",
		   (dev_priv->output_reg == DP_B) ? 1 :
		   (dev_priv->output_reg == DP_C) ? 2 : 3);

    return XF86OutputStatusConnected;
}

static void
i830_dp_destroy (xf86OutputPtr output)
{
    I830OutputPrivatePtr intel_output = output->driver_private;

    if (intel_output != NULL) {
	xf86DestroyI2CBusRec(intel_output->pDDCBus, FALSE, FALSE);
	xfree(intel_output);
    }
}

static const xf86OutputFuncsRec i830_dp_output_funcs = {
    .dpms = i830_dp_dpms,
    .save = i830_dp_save,
    .restore = i830_dp_restore,
    .mode_valid = i830_dp_mode_valid,
    .mode_fixup = i830_dp_mode_fixup,
    .prepare = i830_output_prepare,
    .mode_set = i830_dp_mode_set,
    .commit = i830_output_commit,
    .detect = i830_dp_detect,
    .get_modes = i830_ddc_get_modes,
    .destroy = i830_dp_destroy
};

/*
 * Returns whether the SDVO/HDMI/DP port is used
 * by DP, this prevents it from detection attempts
 * for SDVO or HDMI
 */
Bool
i830_dp_init(ScrnInfoPtr pScrn, int output_reg)
{
    xf86OutputPtr output;
    I830Ptr pI830 = I830PTR(pScrn);
    I830OutputPrivatePtr intel_output;
    struct i830_dp_priv *dev_priv;
    uint32_t dp;
    uint8_t dpcd[4];
#if 0
    uint8_t edid[0x100];
    int i;
    int ret;
#endif

    dp = INREG(output_reg);
    if ((dp & DP_DETECTED) == 0)
	return FALSE;

    /* The DP_DETECTED bits are not reliable; see if there's anyone
     * home
     */
    if (i830_dp_aux_native_read(pScrn, output_reg,
			 0, dpcd, sizeof (dpcd)) != sizeof (dpcd) ||
	(dpcd[0] == 0))
    {
	return TRUE;
    }
#if 0
    ret = i830_dp_aux_i2c_address(pScrn, output_reg, 0xa1);
    xf86DrvMsg(pScrn->scrnIndex, X_ERROR, "send EDID address %d\n", ret);
    for (i = 0; i < 16; i++) {
	ret = i830_dp_aux_i2c_read_1(pScrn, output_reg,
				     0xa1, &edid[i], i == 15 ? TRUE : FALSE);
	xf86DrvMsg(pScrn->scrnIndex, X_ERROR, "get(%d) EDID[%d]=%02x\n", ret, i, edid[i]);
    }
    return TRUE;
#endif

    output = xf86OutputCreate(pScrn, &i830_dp_output_funcs,
			      (output_reg == DP_B) ? "DP-1" :
			      (output_reg == DP_C) ? "DP-2" : "DP-3");
    if (!output)
	return TRUE;
    intel_output = xnfcalloc(sizeof (I830OutputPrivateRec) +
			     sizeof (struct i830_dp_priv), 1);
    if (intel_output == NULL) {
	xf86OutputDestroy(output);
	return TRUE;
    }
    output->driver_private = intel_output;
    output->interlaceAllowed = FALSE;
    output->doubleScanAllowed = FALSE;

    dev_priv = (struct i830_dp_priv *)(intel_output + 1);
    dev_priv->output_reg = output_reg;
    dev_priv->has_audio = FALSE;

    intel_output->dev_priv = dev_priv;
    intel_output->type = I830_OUTPUT_DISPLAYPORT;
    intel_output->pipe_mask = ((1 << 0) | (1 << 1));
    intel_output->clone_mask = (1 << I830_OUTPUT_DISPLAYPORT);

    /* Set up the DDC bus. */
    i830_dp_i2c_init(pScrn, &intel_output->pDDCBus, output,
		     (output_reg == DP_B) ? "DPDDC-B" :
		     (output_reg == DP_C) ? "DPDDC-C" : "DPDDC_D");


    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
	       "DP output %d detected\n",
	       (output_reg == DP_B) ? 1 :
	       (output_reg == DP_C) ? 2 : 3);
    return TRUE;
}
