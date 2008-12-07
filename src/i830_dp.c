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
    uint8_t  save_link_configuration[0x10];
    Bool has_audio;
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

static Bool
i830_dp_mode_fixup(xf86OutputPtr output, DisplayModePtr mode,
		     DisplayModePtr adjusted_mode)
{
    /* The DP output doesn't need the pixel multiplication that SDVO does,
     * so no fixup.
     */
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
i830_dp_aux_write(ScrnInfoPtr pScrn, uint32_t output_reg,
		  uint32_t address, uint8_t *send, int send_bytes)
{
    I830Ptr pI830 = I830PTR(pScrn);
    uint32_t	ch_ctl = output_reg + 0x10;
    uint32_t	ch_cmd = ch_ctl + 4;
    uint32_t	ch_data = ch_ctl + 8;
    int		i;
    uint32_t	ctl;
    uint32_t	cmd;
    uint32_t	status;

    cmd = ((0x8 << 28) |
	   (address << 8) |
	   (send_bytes));
    OUTREG(ch_cmd, cmd);
    for (i = 0; i < send_bytes; i += 4)
	OUTREG(ch_data + i, pack_aux(send + i, send_bytes - i));

    ctl = (DP_AUX_CH_CTL_SEND_BUSY |
	   DP_AUX_CH_CTL_TIME_OUT_400us |
	   ((send_bytes + 4) << DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT) |
	   (5 << DP_AUX_CH_CTL_PRECHARGE_2US_SHIFT) |
	   (133 << DP_AUX_CH_CTL_BIT_CLOCK_2X_SHIFT));
    OUTREG(ch_ctl, ctl);
    for (;;) {
	status = INREG(ch_ctl);
	if ((status & DP_AUX_CH_CTL_SEND_BUSY) == 0)
	    break;
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
    return send_bytes;
}

static int
i830_dp_aux_write_1(ScrnInfoPtr pScrn, uint32_t output_reg,
		    uint32_t address, uint8_t byte)
{
    return i830_dp_aux_write(pScrn, output_reg, address, &byte, 1);
}

static int
i830_dp_aux_read(ScrnInfoPtr pScrn, uint32_t output_reg,
		 uint32_t address, uint8_t *recv, int recv_bytes)
{
    I830Ptr pI830 = I830PTR(pScrn);
    uint32_t	ch_ctl = output_reg + 0x10;
    uint32_t	ch_cmd = ch_ctl + 4;
    int		i;
    uint8_t	header[4];
    uint8_t	header_bytes;
    uint32_t	ctl;
    uint32_t	cmd;
    uint32_t	status;
    int		got_bytes;
    int		tries = 0;

    cmd = ((0x9 << 28) |
	   (address << 8) |
	   (recv_bytes-1));
    ctl = (DP_AUX_CH_CTL_SEND_BUSY |
	   DP_AUX_CH_CTL_TIME_OUT_400us |
	   (4 << DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT) |
	   (5 << DP_AUX_CH_CTL_PRECHARGE_2US_SHIFT) |
	   (133 << DP_AUX_CH_CTL_BIT_CLOCK_2X_SHIFT));
    for (;;) {
	OUTREG(ch_cmd, cmd);

	OUTREG(ch_ctl, ctl);
	for (;;) {
	    status = INREG(ch_ctl);
	    if ((status & DP_AUX_CH_CTL_SEND_BUSY) == 0)
		break;
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

	got_bytes = ((status & DP_AUX_CH_CTL_MESSAGE_SIZE_MASK) >>
		     DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT);
	if (got_bytes > 20)
	    got_bytes = 20;
	if (got_bytes - 1 < recv_bytes)
	    recv_bytes = got_bytes - 1;
	if (got_bytes - 1 > recv_bytes)
	{
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "DisplayPort AUX CH read returns too much: %d > %d\n",
		       got_bytes, recv_bytes);
	}
	header_bytes = 4;
	if (recv_bytes + 1 < 4)
	    header_bytes = recv_bytes + 1;
	unpack_aux(INREG(ch_cmd), header, header_bytes);
	switch (header[0] & 0x3) {
	case 0:
	    break;
	case 1:
	case 3:
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "DisplayPort AUX CH %d read fails\n", address);
	    return -1;
	case 2:
	    tries++;
	    if (tries > 10) {
		xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
			   "DisplayPort AUX CH read %d gave up after %d retries\n",
			   address, tries);
		return -1;
	    }
	    usleep(1000);
	    continue;
	}
	break;
    }
    memcpy(recv, header + 1, header_bytes-1);
    for (i = 4; i < recv_bytes + 1; i += 4)
	unpack_aux(INREG(ch_cmd + i), recv + (i-1), recv_bytes - (i-1));
    return recv_bytes;
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

    dp = (DP_PORT_EN |
	  DP_LINK_TRAIN_OFF |
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

    ret = i830_dp_aux_read(pScrn, dev_priv->output_reg, DP_LANE0_1_STATUS,
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
    i830_dp_aux_read(pScrn, dev_priv->output_reg, 0x100,
		     dev_priv->save_link_configuration, sizeof (dev_priv->save_link_configuration));
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
    int tries;
    int i;

    if ((DP & DP_PORT_EN) == 0) {
	OUTREG(dev_priv->output_reg, DP);
	return;
    }
    DP &= ~DP_LINK_TRAIN_MASK;

    ret = i830_dp_aux_read(pScrn, dev_priv->output_reg,
			   0x206, adjust_request, 2);
    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
	       "adjust_request(%d): %02x %02x\n",
	       ret, adjust_request[0], adjust_request[1]);
    /* main link disabled */
    OUTREG(dev_priv->output_reg, DP | DP_LINK_TRAIN_PAT_1);
    POSTING_READ(dev_priv->output_reg);
    i830_dp_aux_write_1(pScrn, dev_priv->output_reg,
			DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_1);
    usleep (15*1000);
    ret = i830_dp_aux_read(pScrn, dev_priv->output_reg,
			   DP_TRAINING_PATTERN_SET,
			   training, 1);
    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
	       "training pattern 1: %02x\n", training[0]);
    memset(train_set, '\0', sizeof (train_set));
    train_set[0] = (3 << 3) | (0 << 0);
    train_set[1] = (3 << 3) | (0 << 0);
    train_set[2] = (3 << 3) | (0 << 0);
    train_set[3] = (3 << 3) | (0 << 0);
    /* clock recovery pattern */
    tries = 0;
    for (i = 0; i < 4; i++) {

	ret = i830_dp_aux_write(pScrn, dev_priv->output_reg,
				DP_TRAINING_LANE0_SET, train_set, 4);
	if (ret != 4) {
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "voltage swing set failed\n");
	    break;
	}

	usleep (15*1000);
	ret = i830_dp_lane_status(output, lane_status);
	if (ret != 3)
	    break;
	if ((lane_status[0] & DP_LANE_CR_DONE) &&
	    (lane_status[0] & (DP_LANE_CR_DONE << 4)) &&
	    (lane_status[1] & DP_LANE_CR_DONE) &&
	    (lane_status[1] & (DP_LANE_CR_DONE << 4)))
	    break;
	if (i == 3) {
	    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		       "clock recovery failed\n");
	    break;
	}
    }
    /* channel eq pattern */
    OUTREG(dev_priv->output_reg, DP | DP_LINK_TRAIN_PAT_2);
    POSTING_READ(dev_priv->output_reg);
    i830_dp_aux_write_1(pScrn, dev_priv->output_reg,
			DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_2);
    usleep(15*1000);
    ret = i830_dp_aux_read(pScrn, dev_priv->output_reg,
			   DP_TRAINING_PATTERN_SET,
			   training, 1);
    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
	       "training pattern 2: %02x\n", training[0]);
    for (i = 0; i < 4; i++) {
	int ret;

	ret = i830_dp_aux_write(pScrn, dev_priv->output_reg,
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
    i830_dp_aux_write_1(pScrn, dev_priv->output_reg,
			DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_DISABLE);
    usleep (15*1000);
}

static void
i830_dp_restore(xf86OutputPtr output)
{
    ScrnInfoPtr pScrn = output->scrn;
    I830OutputPrivatePtr intel_output = output->driver_private;
    struct i830_dp_priv *dev_priv = intel_output->dev_priv;
    I830Ptr pI830 = I830PTR(pScrn);

    OUTREG(dev_priv->output_reg, dev_priv->save_DP);
    i830_dp_aux_write(pScrn, dev_priv->output_reg, 0x100,
		     dev_priv->save_link_configuration, sizeof (dev_priv->save_link_configuration));
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

    dp = INREG(output_reg);
    if ((dp & DP_DETECTED) == 0)
	return FALSE;

    /* The DP_DETECTED bits are not reliable; see if there's anyone
     * home
     */
    if (i830_dp_aux_read(pScrn, output_reg,
			 0, dpcd, sizeof (dpcd)) != sizeof (dpcd) ||
	(dpcd[0] == 0))
    {
	return TRUE;
    }
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
    switch (output_reg) {
    case DP_B:
	I830I2CInit(pScrn, &intel_output->pDDCBus, GPIOE, "DPDDC_B");
	break;
    case DP_C:
	I830I2CInit(pScrn, &intel_output->pDDCBus, GPIOF, "DPDDC_C");
	break;
    case DP_D:
	I830I2CInit(pScrn, &intel_output->pDDCBus, GPIOG, "DPDDC_D");
	break;
    }

    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
	       "DP output %d detected\n",
	       (output_reg == DP_B) ? 1 :
	       (output_reg == DP_C) ? 2 : 3);
    return TRUE;
}
