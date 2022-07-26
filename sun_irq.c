/*
 * Handles FPGA interrupt packets
 *
 * Copyright (C) IMAGO Technologies GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include "imago_fpga.h"


// Process a SUN interrupt packet
void imago_sun_interrupt(DEVICE_DATA *pDevData, u32 *sun_packet)
{
	u8 wordCount = sun_packet[1] & 0xFF;
	u8 deviceID = (sun_packet[1] >> 20) & (MAX_IRQDEVICECOUNT-1);
	u8 serialId = (sun_packet[1] >> 26) & 1;
	struct SUN_DEVICE_DATA *pSunDevice = &pDevData->SunDeviceData[deviceID];
	unsigned long flags;

	dev_dbg(pDevData->dev, "imago_sun_interrupt() > packet word count: %d\n", wordCount);

	if (wordCount != 1) {
		dev_err(pDevData->dev, "imago_sun_interrupt() > received payload size is incorrect: %u words\n", wordCount);
		return;
	}

	raw_spin_lock_irqsave(&pDevData->lock, flags);

	if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA && pSunDevice->serialID == serialId) {
		pSunDevice->requestState = SUN_REQ_STATE_RESULT;

		raw_spin_unlock_irqrestore(&pDevData->lock, flags);

		dev_dbg(pDevData->dev, "completing request for DeviceID %u\n", deviceID);

		memcpy(pSunDevice->packet, sun_packet, 3*4);
		complete(&pSunDevice->result_complete);
	}
	else if (pSunDevice->requestState == SUN_REQ_STATE_ABORT) {
		// we received the answer from an aborted request => complete silently
		pSunDevice->requestState = SUN_REQ_STATE_IDLE;
		raw_spin_unlock_irqrestore(&pDevData->lock, flags);
	}
	else {
		raw_spin_unlock_irqrestore(&pDevData->lock, flags);
		if (pSunDevice->requestState != SUN_REQ_STATE_INFPGA)
			dev_warn(pDevData->dev, "imago_sun_interrupt() > Unexpected state (%u) for DeviceID=%u, dropping packet\n", pSunDevice->requestState, deviceID);
		else
			dev_warn(pDevData->dev, "imago_sun_interrupt() > serial ID doesn't match for DeviceID=%u, dropping packet\n", deviceID);
	}
}

