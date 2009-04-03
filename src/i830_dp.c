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

#define DP_LINK_STATUS_SIZE	6
#define DP_LINK_CHECK_TIMEOUT	(10 * 1000)

#define DP_LINK_CONFIGURATION_SIZE	9

struct i830_dp_priv {
    uint32_t output_reg;
    uint32_t DP;
    uint8_t  link_configuration[DP_LINK_CONFIGURATION_SIZE];
    uint32_t save_DP;
    uint8_t  save_link_configuration[DP_LINK_CONFIGURATION_SIZE];
    Bool has_audio;
    uint16_t i2c_address;
    uint8_t link_bw;
    uint8_t lane_count;
    Bool i2c_running;
    uint8_t dpcd[4];
    OsTimerPtr link_check_timer;
};

static void
i830_dp_link_train(xf86OutputPtr output, uint32_t DP,
		   uint8_t link_configuration[DP_LINK_CONFIGURATION_SIZE]);

static void
i830_dp_link_down(xf86OutputPtr output, uint32_t DP);

static int
i830_dp_max_lane_count(xf86OutputPtr output)
{
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    int max_lane_count = 4;

    if (dev_priv->dpcd[0] >= 0x11) {
	max_lane_count = dev_priv->dpcd[2] & 0x1f;
	switch (max_lane_count) {
	case 1: case 2: case 4:
	    break;
	default:
	    max_lane_count = 4;
	}
    }
    return max_lane_count;
}

static int
i830_dp_max_link_bw(xf86OutputPtr output)
{
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    int max_link_bw = dev_priv->dpcd[1];

    switch (max_link_bw) {
    case DP_LINK_BW_1_62:
    case DP_LINK_BW_2_7:
	break;
    default:
	max_link_bw = DP_LINK_BW_1_62;
	break;
    }
    return max_link_bw;
}

static int
i830_dp_link_clock(uint8_t link_bw)
{
    if (link_bw == DP_LINK_BW_2_7)
	return 270000;
    else
	return 162000;
}

static int
i830_dp_link_required(int pixel_clock)
{
    return pixel_clock * 3;
}

static int
i830_dp_mode_valid(xf86OutputPtr output, DisplayModePtr mode)
{
    int max_link_clock = i830_dp_link_clock(i830_dp_max_link_bw(output));
    int max_lanes = i830_dp_max_lane_count(output);

    if (i830_dp_link_required(mode->Clock) > max_link_clock * max_lanes)
	return MODE_CLOCK_HIGH;

    if (mode->Clock < 10000)
	return MODE_CLOCK_LOW;

    return MODE_OK;
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

    /* Check for timeout or receive error.
     * Timeouts occur when the sink is not connected
     */
    if (status & (DP_AUX_CH_CTL_TIME_OUT_ERROR | DP_AUX_CH_CTL_RECEIVE_ERROR))
	return -1;

    /* Unload any bytes sent back from the other side */
    recv_bytes = ((status & DP_AUX_CH_CTL_MESSAGE_SIZE_MASK) >>
		  DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT);

    if (recv_bytes > recv_size)
	recv_bytes = recv_size;
    for (i = 0; i < recv_bytes; i += 4) {
	uint32_t    d = INREG(ch_data + i);

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

static Bool
i830_dp_mode_fixup(xf86OutputPtr output, DisplayModePtr mode,
		     DisplayModePtr adjusted_mode)
{
    ScrnInfoPtr	pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);
    int	lane_count, clock;
    int max_lane_count = i830_dp_max_lane_count(output);
    int max_clock = i830_dp_max_link_bw(output) == DP_LINK_BW_2_7 ? 1 : 0;;
    static int bws[2] = { DP_LINK_BW_1_62, DP_LINK_BW_2_7 };

    for (lane_count = 1; lane_count <= max_lane_count; lane_count <<= 1)
    {
	for (clock = 0; clock <= max_clock; clock++)
	{
	    int link_avail = i830_dp_link_clock(bws[clock]) * lane_count;

	    if (i830_dp_link_required(mode->Clock) <= link_avail)
	    {
		dev_priv->link_bw = bws[clock];
		dev_priv->lane_count = lane_count;
		adjusted_mode->Clock = i830_dp_link_clock(dev_priv->link_bw);
		if (pI830->debug_modes)
			xf86DrvMsg(0, X_INFO,
				   "link_bw %d lane_count %d clock %d\n",
				   dev_priv->link_bw, dev_priv->lane_count,
				   adjusted_mode->Clock);
		return TRUE;
	    }
	}
    }
    return FALSE;
}

struct i830_dp_m_n {
	uint32_t	tu;
	uint32_t	gmch_m;
	uint32_t	gmch_n;
	uint32_t	link_m;
	uint32_t	link_n;
};

static void
i830_reduce_ratio(uint32_t *num, uint32_t *den)
{
	while (*num > 0xffffff || *den > 0xffffff) {
		*num >>= 1;
		*den >>= 1;
	}
}

static void
i830_dp_compute_m_n(int bytes_per_pixel,
		    int nlanes,
		    int pixel_clock,
		    int link_clock,
		    struct i830_dp_m_n *m_n)
{
	m_n->tu = 64;
	m_n->gmch_m = pixel_clock * bytes_per_pixel;
	m_n->gmch_n = link_clock * nlanes;
	i830_reduce_ratio(&m_n->gmch_m, &m_n->gmch_n);
	m_n->link_m = pixel_clock;
	m_n->link_n = link_clock;
	i830_reduce_ratio(&m_n->link_m, &m_n->link_n);
}

void
i830_dp_set_m_n(xf86CrtcPtr crtc, DisplayModePtr mode,
		   DisplayModePtr adjusted_mode)
{
    ScrnInfoPtr pScrn = crtc->scrn;
    xf86CrtcConfigPtr xf86_config = XF86_CRTC_CONFIG_PTR(pScrn);
    I830CrtcPrivatePtr intel_crtc = crtc->driver_private;
    I830Ptr pI830 = I830PTR(pScrn);
    int lane_count = 4;
    int i;
    struct i830_dp_m_n m_n;

    /*
     * Find the lane count in the output private
     */
    for (i = 0; i < xf86_config->num_output; i++) {
	xf86OutputPtr output = xf86_config->output[i];
	I830OutputPrivatePtr intel_output = output->driver_private;
	struct i830_dp_priv *dev_priv = intel_output->dev_priv;

	if (output->crtc == crtc && intel_output->type == I830_OUTPUT_DISPLAYPORT) {
	    lane_count = dev_priv->lane_count;
	    break;
	}
    }

    /*
     * Compute the GMCH and Link ratios. The '3' here is
     * the number of bytes_per_pixel post-LUT, which we always
     * set up for 8-bits of R/G/B, or 3 bytes total.
     */
    i830_dp_compute_m_n(3, lane_count,
			mode->Clock, adjusted_mode->Clock, &m_n);

    if (intel_crtc->pipe == 0) {
	OUTREG(PIPEA_GMCH_DATA_M,
	       ((m_n.tu - 1) << PIPE_GMCH_DATA_M_TU_SIZE_SHIFT) |
	       m_n.gmch_m);
	OUTREG(PIPEA_GMCH_DATA_N,
	       m_n.gmch_n);
	OUTREG(PIPEA_DP_LINK_M, m_n.link_m);
	OUTREG(PIPEA_DP_LINK_N, m_n.link_n);
    } else {
	OUTREG(PIPEB_GMCH_DATA_M,
	       ((m_n.tu - 1) << PIPE_GMCH_DATA_M_TU_SIZE_SHIFT) |
	       m_n.gmch_m);
	OUTREG(PIPEB_GMCH_DATA_N,
	       m_n.gmch_n);
	OUTREG(PIPEB_DP_LINK_M, m_n.link_m);
	OUTREG(PIPEB_DP_LINK_N, m_n.link_n);
    }
}

static void
i830_dp_mode_set(xf86OutputPtr output, DisplayModePtr mode,
		   DisplayModePtr adjusted_mode)
{
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    xf86CrtcPtr crtc = output->crtc;
    I830CrtcPrivatePtr intel_crtc = crtc->driver_private;

    dev_priv->DP = (DP_LINK_TRAIN_OFF |
		    DP_VOLTAGE_0_4 |
		    DP_PRE_EMPHASIS_0 |
		    DP_SYNC_VS_HIGH |
		    DP_SYNC_HS_HIGH);

    switch (dev_priv->lane_count) {
    case 1:
	dev_priv->DP |= DP_PORT_WIDTH_1;
	break;
    case 2:
	dev_priv->DP |= DP_PORT_WIDTH_2;
	break;
    case 4:
	dev_priv->DP |= DP_PORT_WIDTH_4;
	break;
    }
    if (dev_priv->has_audio)
	dev_priv->DP |= DP_AUDIO_OUTPUT_ENABLE;

    memset(dev_priv->link_configuration, 0, DP_LINK_CONFIGURATION_SIZE);
    dev_priv->link_configuration[0] = dev_priv->link_bw;
    dev_priv->link_configuration[1] = dev_priv->lane_count;

    /*
     * Check for DPCD version > 1.1,
     * enable enahanced frame stuff in that case
     */
    if (dev_priv->dpcd[0] >= 0x11) {
	dev_priv->link_configuration[1] |= DP_LANE_COUNT_ENHANCED_FRAME_EN;
	dev_priv->DP |= DP_ENHANCED_FRAMING;
    }

    if (intel_crtc->pipe == 1)
	dev_priv->DP |= DP_PIPEB_SELECT;
}

#include "i830_debug.h"

static CARD32
i830_dp_link_check_timer(OsTimerPtr timer, CARD32 now, pointer arg);

static void
i830_dp_dpms(xf86OutputPtr output, int mode)
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);
    uint32_t  dp_reg = INREG(dev_priv->output_reg);

    if (mode == DPMSModeOff) {
	if (dp_reg & DP_PORT_EN)
	    i830_dp_link_down(output, dev_priv->DP);
	if (dev_priv->link_check_timer) {
	    TimerFree(dev_priv->link_check_timer);
	    dev_priv->link_check_timer = NULL;
	}
    } else {
	if (!(dp_reg & DP_PORT_EN)) {
	    uint32_t	pipestat;
	    uint32_t	pipestat_reg;
	    xf86CrtcPtr crtc = output->crtc;
	    I830CrtcPrivatePtr intel_crtc = crtc->driver_private;

	    if (intel_crtc->pipe == 1)
		pipestat_reg = PIPEBSTAT;
	    else
		pipestat_reg = PIPEASTAT;
	    OUTREG(pipestat_reg, INREG(pipestat_reg));	/* clear interrupt status */
	    do {
		pipestat = INREG(pipestat_reg);
	    } while (!(pipestat & VSYNC_INT_STATUS));
	    xf86DrvMsg(pScrn->scrnIndex, X_INFO, "Registers before link training\n");
	    i830DumpRegs(pScrn);
	    i830_dp_link_train(output, dev_priv->DP, dev_priv->link_configuration);

	    i830WaitForVblank(pScrn);
	    i830_dp_link_down(output, dev_priv->DP);
	    i830WaitForVblank(pScrn);
	    crtc->funcs->dpms(crtc, DPMSModeOff);
	    i830WaitForVblank(pScrn);
	    crtc->funcs->dpms(crtc, DPMSModeOn);
	    i830WaitForVblank(pScrn);
	    i830_dp_link_train(output, dev_priv->DP, dev_priv->link_configuration);
	    i830WaitForVblank(pScrn);

	    if (!dev_priv->link_check_timer)
		dev_priv->link_check_timer = TimerSet(NULL, 0, DP_LINK_CHECK_TIMEOUT,
						      i830_dp_link_check_timer, output);
	}
    }
}

/*
 * Fetch AUX CH registers 0x202 - 0x207 which contain
 * link status information
 */
static Bool
i830_dp_get_link_status(xf86OutputPtr output,
			uint8_t link_status[DP_LINK_STATUS_SIZE])
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    int ret;

    ret = i830_dp_aux_native_read(pScrn,
				  dev_priv->output_reg, DP_LANE0_1_STATUS,
				  link_status, DP_LINK_STATUS_SIZE);
    if (ret != DP_LINK_STATUS_SIZE) {
	xf86DrvMsg(pScrn->scrnIndex, X_INFO, "dp link status failed\n");
	return FALSE;
    }
    return TRUE;
}

static uint8_t
i830_dp_link_status(uint8_t link_status[DP_LINK_STATUS_SIZE],
		    int r)
{
	return link_status[r - DP_LANE0_1_STATUS];
}

static void
i830_dp_save(xf86OutputPtr output)
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);

    dev_priv->save_DP = INREG(dev_priv->output_reg);
    i830_dp_aux_native_read(pScrn, dev_priv->output_reg, 0x100,
		     dev_priv->save_link_configuration, sizeof (dev_priv->save_link_configuration));
    if (pI830->debug_modes)
	xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		   "link configuration: %02x %02x %02x %02x "
		   "%02x %02x %02x %02x %02x\n",
		   dev_priv->save_link_configuration[0],
		   dev_priv->save_link_configuration[1],
		   dev_priv->save_link_configuration[2],
		   dev_priv->save_link_configuration[3],
		   dev_priv->save_link_configuration[4],
		   dev_priv->save_link_configuration[5],
		   dev_priv->save_link_configuration[6],
		   dev_priv->save_link_configuration[7],
		   dev_priv->save_link_configuration[8]);
}

static uint8_t
i830_get_adjust_request_voltage(uint8_t link_status[DP_LINK_STATUS_SIZE],
				int lane)
{
    int	    i = DP_ADJUST_REQUEST_LANE0_1 + (lane >> 1);
    int	    s = ((lane & 1) ?
		 DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT :
		 DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT);
    uint8_t l = i830_dp_link_status(link_status, i);

    return ((l >> s) & 3) << DP_TRAIN_VOLTAGE_SWING_SHIFT;
}

static uint8_t
i830_get_adjust_request_pre_emphasis(uint8_t link_status[DP_LINK_STATUS_SIZE],
				     int lane)
{
    int	    i = DP_ADJUST_REQUEST_LANE0_1 + (lane >> 1);
    int	    s = ((lane & 1) ?
		 DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT :
		 DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT);
    uint8_t l = i830_dp_link_status(link_status, i);

    return ((l >> s) & 3) << DP_TRAIN_PRE_EMPHASIS_SHIFT;
}

/*
 * These are source-specific values; current Intel hardware supports
 * a maximum voltage of 800mV and a maximum pre-emphasis of 6dB
 */
#define I830_DP_VOLTAGE_MAX	    DP_TRAIN_VOLTAGE_SWING_800

static uint8_t
i830_dp_pre_emphasis_max(uint8_t voltage_swing)
{
    switch (voltage_swing & DP_TRAIN_VOLTAGE_SWING_MASK) {
    case DP_TRAIN_VOLTAGE_SWING_400:
	return DP_TRAIN_PRE_EMPHASIS_6;
    case DP_TRAIN_VOLTAGE_SWING_600:
	return DP_TRAIN_PRE_EMPHASIS_3_5;
    case DP_TRAIN_VOLTAGE_SWING_800:
    case DP_TRAIN_VOLTAGE_SWING_1200:
    default:
	return DP_TRAIN_PRE_EMPHASIS_0;
    }
}

static void
i830_get_adjust_train(uint8_t link_status[DP_LINK_STATUS_SIZE],
		      int lane_count,
		      uint8_t train_set[4])
{
    uint8_t v = 0;
    uint8_t p = 0;
    int lane;

    for (lane = 0; lane < lane_count; lane++) {
	uint8_t this_v = i830_get_adjust_request_voltage(link_status, lane);
	uint8_t this_p = i830_get_adjust_request_pre_emphasis(link_status, lane);

	if (this_v > v)
	    v = this_v;
	if (this_p > p)
	    p = this_p;
    }

    if (v >= I830_DP_VOLTAGE_MAX)
	v = I830_DP_VOLTAGE_MAX | DP_TRAIN_MAX_SWING_REACHED;

    if (p >= i830_dp_pre_emphasis_max(v))
	p = i830_dp_pre_emphasis_max(v) | DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

    for (lane = 0; lane < 4; lane++)
	train_set[lane] = v | p;
}

static uint32_t
i830_dp_signal_levels(uint8_t train_set, int lane_count)
{
    uint32_t	signal_levels = 0;

    switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
    case DP_TRAIN_VOLTAGE_SWING_400:
    default:
	signal_levels |= DP_VOLTAGE_0_4;
	break;
    case DP_TRAIN_VOLTAGE_SWING_600:
	signal_levels |= DP_VOLTAGE_0_6;
	break;
    case DP_TRAIN_VOLTAGE_SWING_800:
	signal_levels |= DP_VOLTAGE_0_8;
	break;
    case DP_TRAIN_VOLTAGE_SWING_1200:
	signal_levels |= DP_VOLTAGE_1_2;
	break;
    }
    switch (train_set & DP_TRAIN_PRE_EMPHASIS_MASK) {
    case DP_TRAIN_PRE_EMPHASIS_0:
    default:
	signal_levels |= DP_PRE_EMPHASIS_0;
	break;
    case DP_TRAIN_PRE_EMPHASIS_3_5:
	signal_levels |= DP_PRE_EMPHASIS_3_5;
	break;
    case DP_TRAIN_PRE_EMPHASIS_6:
	signal_levels |= DP_PRE_EMPHASIS_6;
	break;
    case DP_TRAIN_PRE_EMPHASIS_9_5:
	signal_levels |= DP_PRE_EMPHASIS_9_5;
	break;
    }
    return signal_levels;
}

static uint8_t
i830_get_lane_status(uint8_t link_status[DP_LINK_STATUS_SIZE],
		     int lane)
{
    int i = DP_LANE0_1_STATUS + (lane >> 1);
    int s = (lane & 1) * 4;
    uint8_t l = i830_dp_link_status(link_status, i);

    return (l >> s) & 0xf;
}

/* Check for clock recovery is done on all channels */
static Bool
i830_clock_recovery_ok(uint8_t link_status[DP_LINK_STATUS_SIZE], int lane_count)
{
	int lane;
	uint8_t lane_status;

	for (lane = 0; lane < lane_count; lane++) {
		lane_status = i830_get_lane_status(link_status, lane);
		if ((lane_status & DP_LANE_CR_DONE) == 0)
			return FALSE;
	}
	return TRUE;
}

/* Check to see if channel eq is done on all channels */
#define CHANNEL_EQ_BITS (DP_LANE_CR_DONE|\
			 DP_LANE_CHANNEL_EQ_DONE|\
			 DP_LANE_SYMBOL_LOCKED)
static Bool
i830_channel_eq_ok(uint8_t link_status[DP_LINK_STATUS_SIZE], int lane_count)
{
	uint8_t lane_align;
	uint8_t lane_status;
	int lane;

	lane_align = i830_dp_link_status(link_status,
					 DP_LANE_ALIGN_STATUS_UPDATED);
	if ((lane_align & DP_INTERLANE_ALIGN_DONE) == 0)
		return FALSE;
	for (lane = 0; lane < lane_count; lane++) {
		lane_status = i830_get_lane_status(link_status, lane);
		if ((lane_status & CHANNEL_EQ_BITS) != CHANNEL_EQ_BITS)
			return FALSE;
	}
	return TRUE;
}

static Bool
i830_dp_set_link_train(xf86OutputPtr output,
		       uint32_t dp_reg_value,
		       uint8_t dp_train_pat,
		       uint8_t train_set[4],
		       Bool first)
{
	ScrnInfoPtr pScrn = output->scrn;
	I830OutputPrivatePtr intel_output = output->driver_private;
	struct i830_dp_priv *dev_priv = intel_output->dev_priv;
	I830Ptr pI830 = I830PTR(pScrn);
	int ret;

	OUTREG(dev_priv->output_reg, dp_reg_value);
	POSTING_READ(dev_priv->output_reg);
//	if (first)
		i830WaitForVblank(pScrn);

	i830_dp_aux_native_write_1(pScrn, dev_priv->output_reg,
				   DP_TRAINING_PATTERN_SET,
				   dp_train_pat);

	ret = i830_dp_aux_native_write(pScrn, dev_priv->output_reg,
				       DP_TRAINING_LANE0_SET, train_set, 4);
	if (ret != 4) {
		xf86DrvMsg(pScrn->scrnIndex, X_INFO,
			   "dp could't write train set\n");
		return FALSE;
	}
	return TRUE;
}

static void
i830_dp_link_train(xf86OutputPtr output, uint32_t DP,
		   uint8_t link_configuration[DP_LINK_CONFIGURATION_SIZE])
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);
    uint8_t	train_set[4];
    uint8_t link_status[DP_LINK_STATUS_SIZE];
    int i;
    uint8_t voltage;
    Bool clock_recovery = FALSE;
    Bool channel_eq = FALSE;
    Bool first = TRUE;
    int tries;

    /* Write the link configuration data */
    i830_dp_aux_native_write(pScrn, dev_priv->output_reg, 0x100,
			     link_configuration, DP_LINK_CONFIGURATION_SIZE);

    DP |= DP_PORT_EN;
    DP &= ~DP_LINK_TRAIN_MASK;
    memset(train_set, 0, 4);
    voltage = 0xff;
    tries = 0;
    clock_recovery = FALSE;
    for (;;) {
	/* Use train_set[0] to set the voltage and pre emphasis values */
	uint32_t    signal_levels = i830_dp_signal_levels(train_set[0], dev_priv->lane_count);
	DP = (DP & ~(DP_VOLTAGE_MASK|DP_PRE_EMPHASIS_MASK)) | signal_levels;

	if (!i830_dp_set_link_train(output, DP | DP_LINK_TRAIN_PAT_1,
				    DP_TRAINING_PATTERN_1, train_set, first))
		break;
	first = FALSE;
	/* Set training pattern 1 */

	usleep(100);
        if (!i830_dp_get_link_status(output, link_status))
	    break;

	if (i830_clock_recovery_ok(link_status, dev_priv->lane_count)) {
	    clock_recovery = TRUE;
	    break;
	}

	/* Check to see if we've tried the max voltage */
	for (i = 0; i < dev_priv->lane_count; i++)
	    if ((train_set[i] & DP_TRAIN_MAX_SWING_REACHED) == 0)
		break;
	if (i == dev_priv->lane_count) {
	    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		       "clock recovery reached max voltage\n");
	    break;
	}

	/* Check to see if we've tried the same voltage 5 times */
	if ((train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK) == voltage) {
	    ++tries;
	    if (tries == 5) {
		xf86DrvMsg(pScrn->scrnIndex, X_INFO,
			   "clock recovery tried 5 times\n");
		break;
	    }
	} else
	    tries = 0;
	voltage = train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK;

	/* Compute new train_set as requested by target */
        i830_get_adjust_train(link_status, dev_priv->lane_count, train_set);
    }
    if (!clock_recovery)
	xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		   "clock recovery failed\n");
    else if (pI830->debug_modes)
	xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		   "clock recovery at voltage %d pre-emphasis %d\n",
		   train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK,
		   (train_set[0] & DP_TRAIN_PRE_EMPHASIS_MASK) >>
		   DP_TRAIN_PRE_EMPHASIS_SHIFT);

    /* channel equalization */
    tries = 0;
    channel_eq = FALSE;
    for (;;) {
	/* Use train_set[0] to set the voltage and pre emphasis values */
	uint32_t    signal_levels = i830_dp_signal_levels(train_set[0], dev_priv->lane_count);
	DP = (DP & ~(DP_VOLTAGE_MASK|DP_PRE_EMPHASIS_MASK)) | signal_levels;

	/* channel eq pattern */
	if (!i830_dp_set_link_train(output, DP | DP_LINK_TRAIN_PAT_2,
				    DP_TRAINING_PATTERN_2, train_set,
				    FALSE))
		break;

	usleep(400);
        if (!i830_dp_get_link_status(output, link_status))
	    break;

	if (i830_channel_eq_ok(link_status, dev_priv->lane_count)) {
	    channel_eq = TRUE;
	    break;
	}

	/* Try 5 times */
	if (tries > 5) {
	    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		       "channel eq failed: 5 tries\n");
	    break;
	}

	/* Compute new train_set as requested by target */
        i830_get_adjust_train(link_status, dev_priv->lane_count, train_set);
	++tries;
    }
    if (!channel_eq)
	xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		   "channel eq failed\n");
    else if (pI830->debug_modes)
	xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		   "channel eq at voltage %d pre-emphasis %d\n",
		   train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK,
		   (train_set[0] & DP_TRAIN_PRE_EMPHASIS_MASK)
		   >> DP_TRAIN_PRE_EMPHASIS_SHIFT);

    OUTREG(dev_priv->output_reg, DP | DP_LINK_TRAIN_OFF);
    POSTING_READ(dev_priv->output_reg);
    i830_dp_aux_native_write_1(pScrn, dev_priv->output_reg,
			DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_DISABLE);
}

static void
i830_dp_link_down(xf86OutputPtr output, uint32_t DP)
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);

    OUTREG(dev_priv->output_reg, DP & ~DP_PORT_EN);
    POSTING_READ(dev_priv->output_reg);
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
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;

    if (dev_priv->save_DP & DP_PORT_EN)
	i830_dp_link_train(output, dev_priv->save_DP, dev_priv->save_link_configuration);
    else
	i830_dp_link_down(output,  dev_priv->save_DP);
}

/*
 * According to DP spec
 * 5.1.2:
 *  1. Read DPCD
 *  2. Configure link according to Receiver Capabilities
 *  3. Use Link Training from 2.5.3.3 and 3.5.1.3
 *  4. Check link status on receipt of hot-plug interrupt
*/

static void
i830_dp_check_link_status(xf86OutputPtr output)
{
	I830OutputPrivatePtr intel_output = output->driver_private;
	struct i830_dp_priv *dev_priv = intel_output->dev_priv;
	uint8_t link_status[DP_LINK_STATUS_SIZE];

	if (!output->crtc)
		return;

	if (!i830_dp_get_link_status(output, link_status)) {
		i830_dp_link_down(output, dev_priv->DP);
		return;
	}

	if (!i830_channel_eq_ok(link_status, dev_priv->lane_count))
		i830_dp_link_train(output, dev_priv->DP, dev_priv->link_configuration);
}

static CARD32
i830_dp_link_check_timer(OsTimerPtr timer, CARD32 now, pointer arg)
{
	xf86OutputPtr output = (xf86OutputPtr) arg;

	i830_dp_check_link_status(output);
	return DP_LINK_CHECK_TIMEOUT;
}

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
    xf86OutputStatus status;

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

    status = XF86OutputStatusDisconnected;
    if (i830_dp_aux_native_read(pScrn, dev_priv->output_reg,
				0, dev_priv->dpcd,
				sizeof (dev_priv->dpcd)) == sizeof (dev_priv->dpcd))
    {
	if (dev_priv->dpcd[0] != 0)
	    status = XF86OutputStatusConnected;
    }
    if (pI830->debug_modes)
	xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		   "DisplayPort monitor detected on DP-%d\n",
		   (dev_priv->output_reg == DP_B) ? 1 :
		   (dev_priv->output_reg == DP_C) ? 2 : 3);

    return status;
}

static void
i830_dp_destroy (xf86OutputPtr output)
{
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;

    if (intel_output != NULL) {
	xf86DestroyI2CBusRec(intel_output->pDDCBus, FALSE, FALSE);
	if (dev_priv->link_check_timer)
	    TimerFree(dev_priv->link_check_timer);
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

    dp = INREG(output_reg);
    if ((dp & DP_DETECTED) == 0)
	return FALSE;

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


    if (pI830->debug_modes) {
	    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		       "DP output %d detected\n",
		       (output_reg == DP_B) ? 1 :
		       (output_reg == DP_C) ? 2 : 3);
    }
    return TRUE;
}
