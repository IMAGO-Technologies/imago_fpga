/*
 * LockedOps.c
 *
 * Handle the the read/write/IO Ops 
 * (only one thread is inside this file at the same time, every arg should be valid)
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

//fÃ¼hrt die sucht einen freien eintrag und belegt diesen, gibt >= 0 fÃ¼r OK sonst fehler code zurÃ¼ck,
// -> Fehler, wenn eine Anfrage zur ID offen ist
// -> Fehler, wenn kein Platz mehr ist
// dass die DeviceID geÃ¶ffnet wird nicht Ã¼berprÃ¼ft, ist fÃ¼r den Treiber Egal
long Locked_startlongtermread(PDEVICE_DATA pDevData, const u32 DeviceID)
{
	u32 index;

	/* Es darf zu dieser Zeit keine Anfrage mit der Ã¼bergebenen ID offen sein */
	//(Ã¼ber die ID erfolgt die Zuordnung IRQ <-> Request)
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		if(		( (pDevData->LongTermRequestList[index].boIsInFPGA == TRUE) || (pDevData->LongTermRequestList[index].boIsInProcessUse == TRUE) )
			&&	(pDevData->LongTermRequestList[index].DeviceID == DeviceID) )
		{
			printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_startlongtermread> DeviceID is in use (index: %u, devId: %u, %d:%d)!\n", index, DeviceID,
							pDevData->LongTermRequestList[index].boIsInFPGA, pDevData->LongTermRequestList[index].boIsInProcessUse);
			return -EBUSY;
		}
	}

	/* Ist noch Platz fÃ¼r noch eine Anfrage? */
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		if(		(pDevData->LongTermRequestList[index].boIsInFPGA == FALSE)
			&& 	(pDevData->LongTermRequestList[index].boIsInProcessUse == FALSE))
		{
			//init des eintrags
			pDevData->LongTermRequestList[index].IRQBuffer_anzBytes = 0;
			pDevData->LongTermRequestList[index].DeviceID			= DeviceID;
			pDevData->LongTermRequestList[index].boAbortWaiting		= FALSE;
			while( down_trylock(&pDevData->LongTermRequestList[index].WaitSem) == 0){}		//die sem runter zÃ¤hlen bis sie blocked, SWI & Abort setzen es

			//Eintrag wird genutzt
			pDevData->LongTermRequestList[index].boIsInProcessUse	= TRUE;
			pDevData->LongTermRequestList[index].boIsInFPGA			= TRUE;		//vor dem Write setzen, kann ja sein das gleich nach dem write ein IRQ kommt

			return index;
		}

		pr_devel(MODDEBUGOUTTEXT" Locked_startlongtermread> Item is in use (index: %u, %d:%d)!\n", index,
							pDevData->LongTermRequestList[index].boIsInFPGA, pDevData->LongTermRequestList[index].boIsInProcessUse);

	}


	return -EBUSY;
}

//fÃ¼hrt die WriteOp aus, gibt >= 0 fÃ¼r OK sonst fehler code zurÃ¼ck,
long Locked_write(PDEVICE_DATA pDevData, const u8 __user * pToUserMem, const size_t BytesToWrite)
{
	u8 TempBuffer[4*(3+1)];		//damit bei AGEX2 immer 64Bit sind

	/* User Data -> Kernel */
	if(BytesToWrite > sizeof(TempBuffer) )
		return -EFBIG;
	if( copy_from_user (TempBuffer, pToUserMem, BytesToWrite) != 0){
		printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_write> copy_from_user faild\n"); return -EFAULT;
	}



	/* Kernel -> PCI */
	//Notes:
	// -fÃ¼r die AGEX muss sich die Adr nicht Ã¤ndern
	// -bei der AGEX2 mÃ¼ssen es 32Bit mit steigender Adr sein
	//
	// aus include/asm-generic/iomap.h fÃ¼r ioread/writeX_rep
	// "...They do _not_ update the port address. If you
	//	want MMIO that copies stuff laid out in MMIO
	//	memory across multiple ports, use "memcpy_toio()..."
	//
	// aber auch memcpy_toio() macht nicht immer 32Bit
	// "http://www.gossamer-threads.com/lists/linux/kernel/650995?do=post_view_threaded#650995"
	if (BytesToWrite % 4)
	{
		u32 ByteIndex =0;
		for(; ByteIndex < BytesToWrite; ByteIndex++)
			iowrite8(TempBuffer[ByteIndex], pDevData->pVABAR0+ByteIndex);
	}
	else
	{
		u32 WordsToCopy = BytesToWrite/4;
		u32 WordIndex =0;
		for(; WordIndex < WordsToCopy; WordIndex++)
			iowrite32( ((u32*)TempBuffer)[WordIndex], pDevData->pVABAR0 + WordIndex*4 );
	}

	return BytesToWrite;
}

//fÃ¼hrt eine IOOp durch, gibt >= 0 fÃ¼r OK sonst fehler code zurÃ¼ck,
long Locked_ioctl(PDEVICE_DATA pDevData, const u32 cmd, u8 __user * pToUserMem, const u32 BufferSizeBytes)
{
	long result=0;

	switch(cmd)
	{
		/* Gibt die Version als String zurÃ¼ck */
		/**********************************************************************/
		case AGEXDRV_IOC_GET_VERSION:
			if( sizeof(MODVERSION) > BufferSizeBytes){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else
			{
				if( copy_to_user(pToUserMem,MODVERSION,sizeof(MODVERSION)) !=0 ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> copy_to_user faild\n"); result = -EFAULT;
				}
				else
					result = sizeof(MODVERSION);
			}

			break;


		/* Gibt das Build date/time als String zurÃ¼ck */
		/**********************************************************************/
		case AGEXDRV_IOC_GET_BUILD_DATE:
			if( sizeof(MODDATECODE) > BufferSizeBytes){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else
			{
				if( copy_to_user(pToUserMem,MODDATECODE,sizeof(MODDATECODE)) !=0 ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> copy_to_user faild\n"); result = -EFAULT;
				}
				else
					result = sizeof(MODDATECODE);
			}

		break;


		/* Gibt SubType zurÃ¼ck */
		/**********************************************************************/
		case AGEXDRV_IOC_GET_SUBTYPE:
			if( sizeof(pDevData->DeviceSubType) > BufferSizeBytes){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else
			{
				if( copy_to_user(pToUserMem,&pDevData->DeviceSubType,sizeof(pDevData->DeviceSubType)) !=0 ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> copy_to_user faild\n"); result = -EFAULT;
				}
				else
					result = sizeof(pDevData->DeviceSubType);
			}

		break;



		/* Markiert die DeviceID als frei*/
		/**********************************************************************/
		//	-> kein Fehler wenn ungenutzt oder noch im LongTermRequest noch OutOfRange
		case AGEXDRV_IOC_RELEASE_DEVICEID:
			if( BufferSizeBytes != 1){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else
			{
				//lesen
				u8 DeviceID;
				if( get_user(DeviceID,pToUserMem) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT;
					break;
				}

				//gÃ¼ltig?
				if(DeviceID >= MAX_IRQDEVICECOUNT)
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DeviceID out of range!\n");
				else
				{
					//nur zum Tracen
					pr_devel(MODDEBUGOUTTEXT" Locked_ioctl> Release DeviceID = %d, was %s\n",
							   DeviceID, (pDevData->boIsDeviceIDUsed[DeviceID])?("true"):("false") );

					//freigeben, immer egal was fÃ¼r ein Zustand war
					pDevData->boIsDeviceIDUsed[DeviceID] = FALSE;
				}
			}

			break;


		/* Gibt eine neue DeviceID zurÃ¼ck */
		/**********************************************************************/
		// -> nur wenn sie zur Zeit als frei markiert ist boIsDeviceIDUsed[]==false
		// -> nur wenn sie nicht im LongTermRequestList[] ist
		// 		dass kann passieren wenn die DLL eine LongTermAnfrage startet und dann das device zu macht
		// 		die Anfrage ist dann noch unterwegs, ja der Request wurde abgebrochen aber die DeviceID ist
		// 		noch genutzt
		case AGEXDRV_IOC_CREATE_DEVICEID:
			if( BufferSizeBytes != 1){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else
			{
				s16 NewDeviceID = -1;
				u8 iLongTerm;
				u16 index;

				//alle mÃ¶glichen IDs testen
				for(index=0; index < MAX_IRQDEVICECOUNT; index++)
				{
					//benutzt?
					if(pDevData->boIsDeviceIDUsed[index])
						continue;

					//lÃ¤uf noch ein LongTempRequest mit der ID?
					NewDeviceID = index;
					for(iLongTerm=0; iLongTerm<MAX_LONG_TERM_IO_REQUEST; iLongTerm++)
					{
						if(		( (pDevData->LongTermRequestList[iLongTerm].boIsInFPGA == TRUE) || (pDevData->LongTermRequestList[iLongTerm].boIsInProcessUse == TRUE) )
							&&	(pDevData->LongTermRequestList[iLongTerm].DeviceID == NewDeviceID) )
						{
							//wenn noch offen war rÃ¼cksetzen, weiter suchen
							NewDeviceID = -1;
						}
					}

					//neue ID gefunden?
					if(NewDeviceID != -1)
					{
						//speichern
						if( put_user(NewDeviceID,pToUserMem) != 0){
							printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> put_user faild\n");
							result = -EFAULT;
							break;
						}
						result = 1;

						//marken
						pDevData->boIsDeviceIDUsed[NewDeviceID] = TRUE;

						break;
					}
				}

				//Fehler?
				if( NewDeviceID == -1){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No free DeviceID\n"); result = -EMFILE;
				}
				else
					pr_devel(MODDEBUGOUTTEXT" Locked_ioctl> NewDeviceID = %d\n", NewDeviceID );
			}

			break;


		/* Wenn zur DeviceID ein LONGTERM_IOREQUEST lÃ¤uft, sem posten. flag setzen*/
		/**********************************************************************/
		case AGEXDRV_IOC_ABORT_LONGTERM_READ:
			if( BufferSizeBytes != 1){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else
			{
				u8 DeviceID;
				u32 index;

				//DevId lesen
				if( get_user(DeviceID,pToUserMem) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
					result = -EFAULT;
					break;
				}

				//jetzt laufende EintrÃ¤ge zur ID suchen
				for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
				{
					if(		(pDevData->LongTermRequestList[index].boIsInFPGA == TRUE) && (pDevData->LongTermRequestList[index].boIsInProcessUse == TRUE) 
						&&	(pDevData->LongTermRequestList[index].DeviceID == DeviceID) )
					{
						//Process aufwecken
						pr_devel(MODDEBUGOUTTEXT" Locked_ioctl> Abort LongRead = %d, DeviceID = %d\n", index, DeviceID);
						pDevData->LongTermRequestList[index].boAbortWaiting = TRUE;
						smp_mb();	//sicher ist sicher
						up(&pDevData->LongTermRequestList[index].WaitSem);
					}
				}				
			}

			break;


			

		/******** DMA ********/
		/*********************/
		/* übergibt die anzDMAs/TCs/SGs, kann nur geändert werden wenn noch auf 0 bzw. wenn keine Änderung*/
		/**********************************************************************/
		case AGEXDRV_IOC_DMAREAD_CONFIG:
			if( BufferSizeBytes < (3) ){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else if( pDevData->DeviceSubType!=SubType_AGEX2_CL)
			{				
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");	result = -EFAULT;
			}
			else
			{
				u16 tmpDMAs, tmpTCs, tmpSGs;
				if( get_user(tmpDMAs,pToUserMem+0) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
				if( get_user(tmpTCs,pToUserMem+2) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
				if( get_user(tmpSGs,pToUserMem+4) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}

				//nur größer setzen, bzw nur wenn noch nicht gesetzt
				// - in AGEXDrv_init()/AGEXDrv_PCI_probe() > AGEXDrv_InitDrvData() wurde alle max möglichen geinit
				// - nicht ändern wegen RaceCond, da wir davon augehen das die IRQFlags fortlaufend sind
				if( 	!((pDevData->DMARead_anzChannels == 0) || (pDevData->DMARead_anzChannels == tmpDMAs) ) 
					|| 	!((pDevData->DMARead_anzTCs == 0) || (pDevData->DMARead_anzTCs == tmpTCs)) 
					|| 	!((pDevData->DMARead_anzSGs == 0) || (pDevData->DMARead_anzSGs == tmpSGs)) ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Can't configure used DMA/TCs!");	result = -EFAULT;
				}
				//nicht über max setzen
				else if( (tmpDMAs > MAX_DMA_READ_DMACHANNELS) || (tmpTCs > MAX_DMA_READ_CHANNELTCS) || (tmpSGs > MAX_DMA_READ_TCSGS) ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> ConfigDMARead anzDMA/TC/SG too large!");	result = -EFAULT;
				}
				else
				{
					pDevData->DMARead_anzChannels	= tmpDMAs;
					pDevData->DMARead_anzTCs		= tmpTCs;
					pDevData->DMARead_anzSGs		= tmpSGs;
					pr_devel(MODDEBUGOUTTEXT" Locked_ioctl> ConfigDMARead anzDMAs: %d, anzTCs: %d, anzSGs: %d",
							pDevData->DMARead_anzChannels, pDevData->DMARead_anzTCs, pDevData->DMARead_anzSGs);
				}
		    }

			break;



		/* fügt, wenn Platz, UserBuffer dem .Jobs_ToDo FIFO hinzu  */
		/**********************************************************************/
		case AGEXDRV_IOC_DMAREAD_ADD_BUFFER:
			if( BufferSizeBytes < (1+2*sizeof(u64) )){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else if( pDevData->DeviceSubType!=SubType_AGEX2_CL)
			{				
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");	result = -EFAULT;
			}
			else					
			{
				u8 	iDMAChannel;
			   	u64 UserPTR, anzBytesToDo;

				//> Args vom User lesen und testen
				if( get_user(iDMAChannel,pToUserMem+0) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
				if( get_user(UserPTR,pToUserMem+1) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
				if( get_user(anzBytesToDo,pToUserMem+sizeof(u64)+1) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
			
				//wenn es kein DMA gibt ist anz=0
				if( iDMAChannel >= pDevData->DMARead_anzChannels ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!"); result = -EFAULT; break;
				}
				if( (pToUserMem==0) || (anzBytesToDo==0) ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> USER ptr can't be zero or zero transfer!");	result = -EFAULT; break;
				}


				if( down_killable(&pDevData->DMARead_SpinLock) != 0){
					result = -EINTR; break;}
//----------------------------->
				//noch Platz? (.Jobs_ToDo)
				if( kfifo_avail( &pDevData->DMARead_Channels[iDMAChannel].Jobs_ToDo) < 1){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> JobToDoBuffer is full\n"); result = -ENOMEM;}
				else
				{
					//noch Platz? (.Jobs_Done), es müssen passen
					// - alle (möglichen) laufenden
					// - da beim AbortBuffer alle Jobs aus .Jobs_ToDo nach .Jobs_Done verschoben werden +1
					// - da beim AbortWaiter ein dummyBuffer hinzugefügt wird +1
					//
					u64 SizeNeeded = kfifo_len( &pDevData->DMARead_Channels[iDMAChannel].Jobs_ToDo);
					SizeNeeded += pDevData->DMARead_anzTCs + 1 + 1;
					if( kfifo_avail( &pDevData->DMARead_Channels[iDMAChannel].Jobs_Done) < SizeNeeded){
						printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> JobDoneBuffer is too full\n"); result = -ENOMEM;}
					else
					{
						//> Job adden
						DMA_READ_JOB tmpJob;
						tmpJob.pVMUser 				= (uintptr_t) UserPTR;
						tmpJob.anzBytesToTransfer 	= anzBytesToDo;
						tmpJob.boIsOk				= false;/* don't care, da nicht in .Jobs_Done */
						tmpJob.BufferCounter		= 0;	/* don't care, da nicht in .Jobs_Done */

						if( kfifo_put(&pDevData->DMARead_Channels[iDMAChannel].Jobs_ToDo, tmpJob) == 0){
							printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> can't add into JobToDoBuffer\n"); result = -ENOMEM;}
					}
				}
//<-----------------------------
				up(&pDevData->DMARead_SpinLock);

				
				//> Versucht neue DMAs(über alle Channels) zu starten
				AGEXDrv_DMARead_StartDMA(pDevData);


		    }//else Type & Args Ok

			break;




		/* wartet(mit TimeOut) auf einen fertigen DMABuffer, gibt den (Status/Counter/VAPtr) zurück  */
		/**********************************************************************/
		case AGEXDRV_IOC_DMAREAD_WAIT_FOR_BUFFER:
			if( BufferSizeBytes < ( 1 + sizeof(u16) +  sizeof(u64))){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else if( pDevData->DeviceSubType!=SubType_AGEX2_CL)
			{				
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");	result = -EFAULT;
			}
			else
			{
				u8 	iDMAChannel;
				u32 TimeOut_ms;
				DMA_READ_JOB tmpJob; /* ={}; geht aber auch nicht, gibt warning */
				memset(&tmpJob, 0, sizeof(tmpJob));	/* wegen warning */

				//args vom USER
				if( get_user(iDMAChannel,pToUserMem+0) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
				if( get_user(TimeOut_ms,pToUserMem+1) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
				if( iDMAChannel >= pDevData->DMARead_anzChannels ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!"); result = -EFAULT; break;
				}


				//> warten (mit timeout)
				//IOCTRL lock temoprär lösen
				up(&pDevData->DeviceSem);
//<......................................................................
				// aufwachen durch Signal, up vom SWI, oder User [Abort], bzw TimeOut
				if( TimeOut_ms == 0xFFFFFFFF )
				{
					if( down_interruptible(&pDevData->DMARead_Channels[iDMAChannel].WaitSem) != 0)
						{result  = -EINTR;}
				}
				else
				{
					unsigned long jiffiesTimeOut = msecs_to_jiffies(TimeOut_ms);
					if( down_timeout(&pDevData->DMARead_Channels[iDMAChannel].WaitSem,jiffiesTimeOut) != 0)
						{result = -EINTR;}
				}
				down(&pDevData->DeviceSem);
//......................................................................>

				
				//> Buffer aus FIFO entnehmen (wenn warten erfolgreich war [könnte aber auch ein abort sein])
				if(result == 0 )
				{
					if( down_killable(&pDevData->DMARead_SpinLock) != 0)
						{result = -EINTR; break;}
//---------------------------------------------------------->

					if( kfifo_len( &pDevData->DMARead_Channels[iDMAChannel].Jobs_Done)>=1 )
					{
						if( kfifo_get(&pDevData->DMARead_Channels[iDMAChannel].Jobs_Done, &tmpJob) != 1) /*sicher ist sicher*/
							result = -EINTR;
					}
					else
						pr_devel(MODDEBUGOUTTEXT" - DMARead wake up, without buffer!\n");

//<----------------------------------------------------------
					up(&pDevData->DMARead_SpinLock);	
				}

		
				//> an User schicken (wenn Ok [der DMA kann natürlich fehler haben, bzw. ein dummyBuffer vom Abort sein])
				if( result == 0 )
				{
					if( put_user( (u8) tmpJob.boIsOk, pToUserMem) != 0){
						printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
					if( put_user( tmpJob.BufferCounter, pToUserMem+1) != 0){
						printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
					if( put_user( tmpJob.pVMUser, pToUserMem+1+sizeof(u16)) != 0){
						printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}

						pr_devel(MODDEBUGOUTTEXT" Locked_ioctl> DMARead return buffer iDMA: %d, res: %d, Seq: %d, VMPtr: %p\n",
						   	iDMAChannel, tmpJob.boIsOk, tmpJob.BufferCounter, (void*)tmpJob.pVMUser);
				}

			
		    }

			break;



		/* versucht die laufende DMAs abzubrechen, async */
		/* versucht die UserThreads abbzubrechen, async */
		/**********************************************************************/
		case AGEXDRV_IOC_DMAREAD_ABORT_DMA:
		case AGEXDRV_IOC_DMAREAD_ABORT_WAITER:
			if( BufferSizeBytes < (1) ){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n"); result = -EFBIG;
			}
			else if( pDevData->DeviceSubType!=SubType_AGEX2_CL)
			{				
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");	result = -EFAULT;
			}
			else
			{
				//> Args vom User lesen und testen
				u8 	iDMAChannel;
				if( get_user(iDMAChannel,pToUserMem+0) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n"); result = -EFAULT; break;	}
				if( iDMAChannel >= pDevData->DMARead_anzChannels ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!"); result = -EFAULT; break;
				}


				//> DMA/UserThreads abbrechen
				if(cmd == AGEXDRV_IOC_DMAREAD_ABORT_DMA)
					AGEXDrv_DMARead_Abort_DMAChannel(pDevData, iDMAChannel);
				else
					AGEXDrv_DMARead_Abort_DMAWaiter(pDevData, iDMAChannel);
		    }

			break;





		//sollte nie sein (siehe oben bzw. Ebene höher)
		default:
			return -ENOTTY;
	}


	return result;
}
