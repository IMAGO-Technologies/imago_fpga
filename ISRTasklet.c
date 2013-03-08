/*
 * ISRTasklet.c
 *
 * handel the PCI(e) IRQs/MSI and the tasklet
 *
 * Copyright (C) 201x IMAGO Technologies GmbH
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

#include "AGEXDrv.h"


DECLARE_TASKLET(_AGEXDrv_tasklet,AGEXDrv_tasklet,0);


//<====================================>
//	IRQ fns
//
//<====================================>

//Schaltet im PCI-Gerät die Ints ab/zu
void AGEXDrv_SwitchInterruptOn(const bool boTurnOn)
{
	u32 regVal;

	//alles gut?
	if(_PCI_IOMEM_StartAdr == NULL)
		return;

	//was schreiben
	if(boTurnOn)
		regVal = 0x1;
	else
		regVal = 0x0;

	iowrite32(regVal, _PCI_IOMEM_StartAdr + ISR_ONOFF_OFFSET);
}



//< HWIRQ > nur prüfen ob von uns, ja tasklet starten
irqreturn_t AGEXDrv_interrupt(int irq, void *args)
{
	u32 regVal;

	/* war der int von uns? reg lesen  */
	if(_PCI_IOMEM_StartAdr == NULL)
		return IRQ_NONE;

	regVal = ioread32(_PCI_IOMEM_StartAdr + ISR_AVAILABLE_OFFSET);
	if(regVal & 0x1)
	{
		//INTs abschalten
		AGEXDrv_SwitchInterruptOn(FALSE);

		// trigger the tasklet
		tasklet_schedule(&_AGEXDrv_tasklet);

		return IRQ_HANDLED;
	}
	else
		return IRQ_NONE;
}



//<====================================>
//	tasklet fns
//
//<====================================>

//SWI bzw. DPC Achtung hat keinen process context
void AGEXDrv_tasklet (unsigned long unused)
{
	u32 regVal, fifoLevel, header0, header1;
	u16 deviceID, wordCount, index;
	bool boPacketDataDone = FALSE;

	pr_devel("agexdrv: AGEXDrv_tasklet\n");


	//Alles gut?
	if(_PCI_IOMEM_StartAdr == NULL)
		return;


	/* vom FPGA den PaketKopf einlesen/auswerten */
	// FIFO-Fuellstand einlesen
	regVal = ioread32(_PCI_IOMEM_StartAdr + ISR_AVAILABLE_OFFSET);

	// im Bit 0 steht das Interrupt-Flag, danach der Fuellstand
	fifoLevel = (regVal >> 1) & 0x1FF;

	if (fifoLevel < 3){
		// sollte nicht auftreten
		printk(KERN_ERR "agexdrv: SUN Error: FIFO level to small: %d\n", fifoLevel);
		return;
	}
	pr_devel("agexdrv: SUN FIFO level: %d\n", fifoLevel);

	// Header0 und Header1 einlesen
	header0 = ioread32(_PCI_IOMEM_StartAdr);
	header1 = ioread32(_PCI_IOMEM_StartAdr);
	wordCount = header1 & 0xFF;
	deviceID = (header1>>20) & (MAX_IRQDEVICECOUNT-1);
	pr_devel("agexdrv: SUN WordCount: %d\n", wordCount);




	/* läuft noch eine Anfrage für die ID? */
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		// DeviceID auswerten (warten wir auf das SUNPaket?)
		// 	-> DeviceID muss passen und wir haben die daten noch nicht reingefüllt
		if(		( _LongTermRequestList[index].boIsInFPGA == TRUE )
			&&	(_LongTermRequestList[index].DeviceID == deviceID) )
		{
			//Wartet (noch) ein Request aufs Paket?
			if(_LongTermRequestList[index].boIsInProcessUse == TRUE)
			{
				pr_devel("agexdrv: Completing request %d, id %d\n",index,deviceID);

				//wenns in den Buffer passt, Daten einlesen
				if(MAX_SUNPACKETSIZE >= 4*wordCount+8) // Paket mit Header0/1
				{
					_LongTermRequestList[index].IRQBuffer[0] = header0;
					_LongTermRequestList[index].IRQBuffer[1] = header1;

					ioread32_rep(_PCI_IOMEM_StartAdr,_LongTermRequestList[index].IRQBuffer+2,wordCount);

					boPacketDataDone	= TRUE;
					_LongTermRequestList[index].IRQBuffer_anzBytes = 4*wordCount+8;
				}
				else
				{
					_LongTermRequestList[index].IRQBuffer_anzBytes = 0;
					printk(KERN_WARNING "agexdrv: LongTerm buffer to small\n");
				}

				//den process aufwecken
				up(&_LongTermRequestList[index].WaitSem);

			}//end if boIsInProcessUse == true


			//Eintrag freigeben
			//	-> egal ob die DLL noch auf eine Anfrage wartet oder nicht
			//	das Paket für diese ID ist gekommen
			_LongTermRequestList[index].boIsInFPGA = FALSE;

		}//end if DeviceID == [index].DeviceID
	}//end for MAX_LONG_TERM_IO_REQUEST



	/* Immer! egal ob sie einer will oder nicht die daten weglesen */
	if (!boPacketDataDone)
	{
		u16 i;
		volatile u32 dummy;
		for (i=0; i<wordCount; i++)
			dummy = ioread32(_PCI_IOMEM_StartAdr);
	}


	/* Re-enable the interrupt */
	AGEXDrv_SwitchInterruptOn(TRUE);

	return;
}

