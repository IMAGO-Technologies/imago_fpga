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


//führt die sucht einen freien eintrag und belegt diesen, gibt >= 0 für OK sonst fehler code zurück,
// -> Fehler, wenn eine Anfrage zur ID offen ist
// -> Fehler, wenn kein Platz mehr ist
// dass die DeviceID geöffnet wird nicht überprüft, ist für den Treiber Egal
long Locked_startlongtermread(PDEVICE_DATA pDevData, const u32 DeviceID)
{
	u32 index;

	/* Es darf zu dieser Zeit keine Anfrage mit der übergebenen ID offen sein */
	//(über die ID erfolgt die Zuordnung IRQ <-> Request)
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		if(		( (pDevData->LongTermRequestList[index].boIsInFPGA == TRUE) || (pDevData->LongTermRequestList[index].boIsInProcessUse == TRUE) )
			&&	(pDevData->LongTermRequestList[index].DeviceID == DeviceID) )
		{
			return -EBUSY;
		}
	}

	/* Ist noch Platz für noch eine Anfrage? */
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		if(		(pDevData->LongTermRequestList[index].boIsInFPGA == FALSE)
			&& 	(pDevData->LongTermRequestList[index].boIsInProcessUse == FALSE))
		{
			//init des eintrags
			pDevData->LongTermRequestList[index].IRQBuffer_anzBytes =0;
			pDevData->LongTermRequestList[index].DeviceID = DeviceID;
			while( down_trylock(&pDevData->LongTermRequestList[index].WaitSem) == 0){}		//die sem runter zählen bis sie blocked

			//Eintrag wird genutzt
			pDevData->LongTermRequestList[index].boIsInProcessUse= TRUE;
			pDevData->LongTermRequestList[index].boIsInFPGA		= TRUE;		//vor dem Write setzen, kann ja sein das gleich nach dem write ein IRQ kommt

			return index;
		}
	}


	return -EBUSY;
}

//führt die WriteOp aus, gibt >= 0 für OK sonst fehler code zurück,
long Locked_write(PDEVICE_DATA pDevData, const u8 __user * pToUserMem, const size_t BytesToWrite)
{
	u8 TempBuffer[4*(3+1)];		//damit bei AGEX2 immer 64Bit sind

	/* User Data -> Kernel */
	if(BytesToWrite > sizeof(TempBuffer) )
		return -EFBIG;
	if( copy_from_user (TempBuffer, pToUserMem, BytesToWrite) != 0){
		printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_write> copy_from_user faild\n");
		return -EFAULT;
	}



	/* Kernel -> PCI */
	//Notes:
	// -für die AGEX muss sich die Adr nicht ändern
	// -bei der AGEX2 müssen es 32Bit mit steigender Adr sein
	//
	// aus include/asm-generic/iomap.h für ioread/writeX_rep
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

//führt eine IOOp durch, gibt >= 0 für OK sonst fehler code zurück,
long Locked_ioctl(PDEVICE_DATA pDevData, const u32 cmd, u8 __user * pToUserMem, const u32 BufferSizeBytes)
{
	long result=0;

	switch(cmd)
	{
		/* Gibt die Version als String zurück */
		/**********************************************************************/
		case AGEXDRV_IOC_GET_VERSION:
			if( sizeof(MODVERSION) > BufferSizeBytes){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				result = -EFBIG;
			}
			else
			{
				if( copy_to_user(pToUserMem,MODVERSION,sizeof(MODVERSION)) !=0 ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> copy_to_user faild\n");
					result = -EFAULT;
				}
				else
					result = sizeof(MODVERSION);
			}

			break;


		/* Gibt das Build date/time als String zurück */
		/**********************************************************************/
		case AGEXDRV_IOC_GET_BUILD_DATE:
			if( sizeof(MODDATECODE) > BufferSizeBytes){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				result = -EFBIG;
			}
			else
			{
				if( copy_to_user(pToUserMem,MODDATECODE,sizeof(MODDATECODE)) !=0 ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> copy_to_user faild\n");
					result = -EFAULT;
				}
				else
					result = sizeof(MODDATECODE);
			}

		break;


		/* Gibt SubType zurück */
		/**********************************************************************/
		case AGEXDRV_IOC_GET_SUBTYPE:
			if( sizeof(pDevData->DeviceSubType) > BufferSizeBytes){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				result = -EFBIG;
			}
			else
			{
				if( copy_to_user(pToUserMem,&pDevData->DeviceSubType,sizeof(pDevData->DeviceSubType)) !=0 ){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> copy_to_user faild\n");
					result = -EFAULT;
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
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				result = -EFBIG;
			}
			else
			{
				//lesen
				u8 DeviceID;
				if( get_user(DeviceID,pToUserMem) != 0){
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
					result = -EFAULT;
					break;
				}

				//gültig?
				if(DeviceID >= MAX_IRQDEVICECOUNT)
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DeviceID out of range!\n");
				else
				{
					//nur zum Tracen
					pr_debug(MODDEBUGOUTTEXT" Locked_ioctl> Release DeviceID = %d, was %s\n",
							   DeviceID, (pDevData->boIsDeviceIDUsed[DeviceID])?("true"):("false") );

					//freigeben, immer egal was für ein Zustand war
					pDevData->boIsDeviceIDUsed[DeviceID] = FALSE;
				}
			}

			break;


		/* Gibt eine neue DeviceID zurück */
		/**********************************************************************/
		// -> nur wenn sie zur Zeit als frei markiert ist boIsDeviceIDUsed[]==false
		// -> nur wenn sie nicht im LongTermRequestList[] ist
		// 		dass kann passieren wenn die DLL eine LongTermAnfrage startet und dann das device zu macht
		// 		die Anfrage ist dann noch unterwegs, ja der Request wurde abgebrochen aber die DeviceID ist
		// 		noch genutzt
		case AGEXDRV_IOC_CREATE_DEVICEID:
			if( BufferSizeBytes != 1){
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				result = -EFBIG;
			}
			else
			{
				s16 NewDeviceID = -1;
				u8 iLongTerm;
				u16 index;

				//alle möglichen IDs testen
				for(index=0; index < MAX_IRQDEVICECOUNT; index++)
				{
					//benutzt?
					if(pDevData->boIsDeviceIDUsed[index])
						continue;

					//läuf noch ein LongTempRequest mit der ID?
					NewDeviceID = index;
					for(iLongTerm=0; iLongTerm<MAX_LONG_TERM_IO_REQUEST; iLongTerm++)
					{
						if(		( (pDevData->LongTermRequestList[iLongTerm].boIsInFPGA == TRUE) || (pDevData->LongTermRequestList[iLongTerm].boIsInProcessUse == TRUE) )
							&&	(pDevData->LongTermRequestList[iLongTerm].DeviceID == NewDeviceID) )
						{
							//wenn noch offen war rücksetzen, weiter suchen
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
					printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No free DeviceID\n");
					result = -EMFILE;
				}
				else
					pr_debug(MODDEBUGOUTTEXT" Locked_ioctl> NewDeviceID = %d\n", NewDeviceID );
			}

			break;




		//sollte nie sein (siehe oben bzw. ebene)
		default:
			return -ENOTTY;
	}


	return result;
}
