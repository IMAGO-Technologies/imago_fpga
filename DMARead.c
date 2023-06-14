/*
 * FPGA-DMA functions
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
 *
 */

#include "imago_fpga.h"
#include <linux/io.h>	// for writel_relaxed (used for armhf only)

// flags for SG elements
#define DMA_READ_TC_SG_FLAG_START_TRANSACTION 	(0x09) // transaction start: SG descriptor FIFO reset + DMA start flag
#define DMA_READ_TC_SG_FLAG_START_TRANSFER 		(0x00) // intermediate transfer: no additional flags required
#define DMA_READ_TC_SG_FLAG_END_TRANSACTION		(0x06) // end of transaction: discriptor link + interrupt flag
#define DMA_READ_TC_SG_FLAG_END_TRANSFER 		(0x04) // end of intermediate transfer: discriptor link + interrupt flag 
#define DMA_READ_TC_SG_FLAG_ERROR 				(0x10 | DMA_READ_TC_SG_FLAG_END_TRANSACTION)

#define HOST_BUFFER_DMAREAD_COUNTER_OFFSET (4*(2+2+128))

/****************************************
 * 
 * Helpers
 *
****************************************/

//Init und mapped/pinned den "Job<>UserBuffer", struct ist beim return(min die Flags gültig) wickelt daher beim Fehler nichts rück ab (kann nicht als DPC laufen)
int imago_DMARead_MapUserBuffer(PDEVICE_DATA pDevData, DMA_READ_CHANNEL *pDMAChannel, uintptr_t pVMUser,
		u64 bufferSize, u8 reversePages, DMA_READ_JOB **ppJob)
{
	DMA_READ_JOB	*pJob			= NULL;
	u32				pagesToMap		= (bufferSize + PAGE_SIZE-1) / PAGE_SIZE;
	int 			pagesPinned		= -1;
	int 			mappedSGs		= -1;
	unsigned int	i;
	int result;

	dev_dbg(pDevData->dev, "MappUserBuffer: (%d[Bytes], %d[Pages] @ 0x%p)\n", (int)bufferSize, pagesToMap, (void*)pVMUser);

	// search free job entry
	for (i = 0; i < _ModuleData.max_dma_buffers; i++) {
		if (pDMAChannel->jobBuffers[i].pVMUser == 0)
		{
			pJob = &pDMAChannel->jobBuffers[i];
			break;
		}
	}
	*ppJob = pJob;
	if (pJob == NULL) {
		dev_warn(pDevData->dev, "MappUserBuffer: No space for more DMA buffers, see module parameter 'max_dma_buffers' for allocation");
		return -ENOMEM;
	}

	//init der Flags(da hier kein cleanUp gemacht wird)
	pJob->pVMUser 				= (uintptr_t) pVMUser;
	pJob->bufferSize		 	= bufferSize;

	pJob->boIsOk				= false;/* don't care, da nur gültig wenn in .Jobs_Done */
	pJob->BufferCounter			= 0;	/* don't care, da nur gültig wenn in .Jobs_Done */

	pJob->boIsPageListValid 	= false;
	pJob->boIsPinned			= false;
	pJob->boIsSGValid			= false;
	pJob->boIsSGMapped			= false;

	pJob->SGItemsLeft			= 0;


	//Valid? (hier kein Test ob "Offset=0 & size n * PAGES_SIZE" nur wichtig fürs alloc
	if (pDevData == NULL || pJob == NULL) {
		dev_err(pDevData->dev, "MappUserBuffer: invalid arguments");
		return -EINVAL;
	}
	if ((pJob->bufferSize & 0x3) != 0 || pJob->bufferSize <= 4) {
		dev_err(pDevData->dev, "MappUserBuffer: byte count is invalid");
		return -EINVAL;
	}
	if ((pJob->pVMUser & (PAGE_SIZE-1)) != 0) {
		dev_err(pDevData->dev, "MappUserBuffer: page pointer is not aligned");
		return -EINVAL;
	}


	//> User Buffer Pinnen
	/**********************************************************************/
	//speicher für die PageList
	pJob->ppPageList = kmalloc(pagesToMap*sizeof(struct page*), GFP_KERNEL);
	if (pJob->ppPageList == NULL) {
		dev_err(pDevData->dev, "MappUserBuffer: too many pages");
		return -ENOMEM;
	}
	pJob->boIsPageListValid = true;

	//pinnen
	//muss die SEM, für die VMAs für den aufrufenden conntext, halten
	// 'for read or write' ist eine 'rw_semaphore'
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,8,0)
	down_read(&current->mm->mmap_sem);
#else
	mmap_read_lock(current->mm);
#endif
//----------------------------->

// pin_user_pages() and related calls:
// https://www.kernel.org/doc/html/latest/core-api/pin_user_pages.html

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
// 4.8.17 >>> 4.9.0 (Oct 2016)
//	Linux 4.9-rc2(https://lwn.net/Articles/704380/): Lorenzo Stoakes (10): 			mm: replace get_user_pages() write/force parameters with gup_flags
//					http://lists.openwall.net/netdev/2016/10/13/1
//					https://marc.info/?l=linux-mm&m=147585445805166
//		bei 4.8.17 get_user_pages() 	> 	__get_user_pages_locked()  da wurde dann aus if(write) flags |= FOLL_WRITE
//
	pagesPinned = get_user_pages(pJob->pVMUser, pagesToMap, FOLL_WRITE, pJob->ppPageList, NULL);	
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
// 4.5.7 >>> 4.6.0 (Feb 2016)
//	https://github.com/torvalds/linux/commit/d4edcf0d56958db0aca0196314ca38a5e730ea92#diff-c098b65a8bd8c7db23377b90578a62c1  
//		Ingo Molnar  mm/gup: Switch all callers of get_user_pages() to not pass tsk/mm 
//
//		/mm/gup.c get_user_pages()
//			"This is the same as get_user_pages_remote(), just with a
//			 less-flexible calling convention where we assume that the task
//			 and mm being operated on are the current task's."
//	
	pagesPinned = get_user_pages(pJob->pVMUser, pagesToMap, 1, 0, pJob->ppPageList, NULL);
#else
	pagesPinned = get_user_pages(
		current, 		/* task_struct, wo sollen die 'page faults' hin */
		current->mm,	/* mm_struct, in welcher VMA der virtuelle Speicher zu finden ist */
		pJob->pVMUser,	/* UserMode Pointer, muss page-aligned sein */
		pagesToMap,
		1,			/* 1<>write&read, 0<> readOnly (für den fn caller [module]) */
		0,			/* kein force, daher aus ein ReadOnly wird kein RW, 'LDD3 driver should always 0 here' */
		pJob->ppPageList, /* NULL, oder PointerFeld zu den Pages welches anzPages/Pointer halten kann, gefüllte anz ist result */
		NULL);			/* NULL, oder PointerFeld zu den VMAs welche anzPages/Pointer haltern kann */
#endif	
//<-----------------------------
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,8,0)
	up_read(&current->mm->mmap_sem);
#else
	mmap_read_unlock(current->mm);
#endif

	if (pagesPinned < 0) {
		dev_err(pDevData->dev, "MappUserBuffer: get_user_pages() failed");
		return pagesPinned;
	}
	pJob->boIsPinned 		= true;
	pJob->pagesPinned 	= pagesPinned;
	if (((u32)pagesPinned) != pagesToMap) {
		dev_err(pDevData->dev, "MappUserBuffer: get_user_pages() %d failed from %d pinned",
			pagesPinned, pagesToMap);
		return -EFAULT;
	}



	//> SG(scatter/gather) Liste erzeugen
	/**********************************************************************/
	//Note:
	// bis 1.1.9.0 (Anfang 2018) wurde sg_alloc_table_from_pages() [wenn möglich] benutzt 
	//  * das zeitliche Verhalten ist nicht vorhersagbar (von alles 4k bis eine SG mit voller BildSize)
	//  * wir haben aber nur 20Bit im FPGA für den DWordCount eines SGElemnts [bei der Zeile kann es knapp werden]
	//  * unter WIN32 kein Problem, da max 1MByte pro Transfer (n Transfers pro Bild) siehe WDF_DMA_ENABLER_CONFIG_INIT()
	//
	//https://www.kernel.org/doc/Documentation/DMA-API.txt
	// "... The implementation is free to merge several consecutive sglist entries ..."
	//http://www.gossamer-threads.com/lists/linux/kernel/977965
	// "scatterlist.c or dma_map_sg() should coalesce?"
	// "No, it's not the job of dma_map_sg."
	//
	//aber ab =>3.6 gibt es "sg_alloc_table_from_pages()" was es macht
	//http://lists.freedesktop.org/archives/dri-devel/2012-April/021962.html	
	// "...All contiguous chunks of the pages are merged into a single sg nodes."
	//
	//http://askubuntu.com/questions/318315/how-can-i-temporarily-disable-aslr-address-space-layout-randomization
	//"/proc/sys/kernel/randomize_va_space" kann eingestellt werden ob Speicher zusammenhängen darf (0=off,2=on), bei 3.16.7 (max 16k[on], ?64[off])
	//
	//- init des "Headers" <> sg_table, und alloced n pages für je m scatterlists 
	// wird mehr als eine page gebraucht SG_MAX_SINGLE_ALLOC wird in struct scatterlist.page_link
	// das bit 0 gesetzt, dann ist es eine Pointer auf die nächste page 
	// daher nicht selbst durchlaufen
	//siehe: http://lwn.net/Articles/256368/ (The chained scatterlist API)
#if 0 //LINUX_VERSION_CODE > KERNEL_VERSION(3,6,0)
	result = sg_alloc_table_from_pages(	&pJob->SGTable,				/*header*/
										pJob->ppPageList,			/*pointer to page array*/
										pagesPinned,				/*number of pages in page array*/
										0, 							/*buffer offset*/
										pJob->bufferSize			/*buffer size [bytes]*/,
										GFP_KERNEL);				/*alloc type*/
	if (result < 0) {			
		dev_err(pDevData->dev, "MappUserBuffer: sg_alloc_table_from_pages() failed");
		return result;
	}
	pJob->boIsSGValid 	= true;
#else
{
	struct scatterlist *pSGList	= NULL;
	u32 bytesRemaining = pJob->bufferSize;
	s32	iSG							= 0;
	result = sg_alloc_table(&pJob->SGTable	/*header*/, 
							pagesPinned 	/*für wie viele Einträge*/, 
							GFP_KERNEL);	/*wie wird die page gealloc*/
	if (result < 0) {			
		dev_err(pDevData->dev, "MappUserBuffer: sg_alloc_table() failed");
		return result;
	}
	pJob->boIsSGValid 	= true;

	// sg_set_page(): set sg entry to point at given page
	pSGList = pJob->SGTable.sgl;
	for (iSG = 0; iSG < pagesPinned; iSG++) {
		u32 bytes = bytesRemaining;
		if (bytes > PAGE_SIZE)
			bytes = PAGE_SIZE;
		bytesRemaining -= bytes;
		if (reversePages)
			sg_set_page(pSGList,
						pJob->ppPageList[pagesPinned - iSG - 1],
						bytes,
						PAGE_SIZE - bytes);				/*Offset*/
		else
			sg_set_page(pSGList,				/*an diese Stelle wird die Page eingetragen*/
						pJob->ppPageList[iSG]	/*Pointer zur page struct*/,
						bytes,
						0);	/*Offset*/
		pSGList = sg_next(pSGList);
	}
}
#endif


	//> in den PCI/BUS AdrRaum mappen
	/**********************************************************************/
	//macht: 	kmmemcheck_mark_initialiued() > mark_shadow() touch der Page
	//			get_dma_ops(struct device)->map_sg()
	//*_map_sg 
	//	lxr.free-electrons.com/source/drivers/iommu/amd_iommu.c 			2865 (3.8)
	//  lxr.free-electrons.com/source/drivers/iommu/intel-iommu.c			3109 (3.8)
	//	lxr.free-electrons.com/source/arch/arm/mm/dma-mapping.c?v=3.8;a=arm 1418 (3.8)
	// geben (ohne IOMMU) einfach die phys adr zurück, ARM kümmert sich um CACHE (x86 nicht notwendig)
	// bei fehlern machen sie alles rückgängig
	//
	// Achtung!
	//  beim arm64 v4.14.16 (noch nicht bei v4.1.8) fasst die fn SG elemente zusammen
	// https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
	// 	"The implementation is free to merge several consecutive sglist entries into one..."
	//
	// Aber das vorzeitige "Ende" markieren mit sg_mark_end() geht nicht! 
	//" The chained scatterlist API"
	//https://lwn.net/Articles/256368/
	// "...Should the I/O operation not use all of the entries which were allocated in the list, though, the driver should mark the final segment with... 
	//
	// sg_mark_end()
	//https://elixir.bootlin.com/linux/v4.16/source/include/linux/scatterlist.h
	// ".. Marks the passed in sg entry as the termination point for the sg 
	//  table. A call to sg_next() on this entry will return NULL."
	//
	//  >> weil in https://elixir.bootlin.com/linux/v4.16/source/drivers/iommu/dma-iommu.c#L792
	//   for_each_sg(sg_next(sg), tmp, nents - 1, i) ... sg_next() wird dann mit NULL aufgerufen da nents die volle länge ist 
	//
	mappedSGs = dma_map_sg(pDevData->dev,	/*struct device pointer*/	
								pJob->SGTable.sgl,		/*struct scatterlist (Anfang)*/
								pJob->SGTable.nents,	/*anz der Buffers*/
								DMA_FROM_DEVICE); 		/*die Richtung wichtig für cache & bounce buffer*/
	if (mappedSGs <= 0) {
		dev_err(pDevData->dev, "MappUserBuffer: dma_map_sg() failed");
		return -EFAULT;
	}
	pJob->boIsSGMapped = true;
	pJob->SGcount = mappedSGs;

	dev_dbg(pDevData->dev, "dma_map_sg(): %d SG list entries mapped to %d regions\n", pJob->SGTable.nents, mappedSGs);
	dev_dbg(pDevData->dev, "first sg element: addr=0x%08x, len=%u\n", (unsigned int)sg_dma_address(pJob->SGTable.sgl), (unsigned int)sg_dma_len(pJob->SGTable.sgl));

	// for module parameter dma_update_in_hwi in auto mode (-1):
	// disable update of DMA in HWI if many SG elements are used
#ifdef __ARM_ARCH_7A__
	// VisionCam XM: do not turn off update of DMA in HWI to avoid dropped sensor frames
	// (because the FPGA has no big DDR RAM FIFO for buffering data)
#else
	if (pDevData->setupTcInHWI && _ModuleData.dma_update_in_hwi == -1) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
		// There should only be one or two mapped regions if an IOMMU is present.
		// We could evaluate mappedSGs instead, but this value may be non-deterministic depending
		// on memory fragmentation if no IOMMU is present. We always want the same behavior.
		if (pDevData->dev->iommu_group == NULL)
#endif
		{
			if (pJob->SGTable.nents > 16) {
				pDevData->setupTcInHWI = 0;
				dev_dbg(pDevData->dev, "dma_map_sg(): using threaded interrupt for DMA update\n");
			}
		}
	}
#endif

	return 0;
}



// Macht einen CleanUp des Jobs
// Achtung! 
// 	> es können nur Teile eines Jobs gültig sein
//  > darf nicht in einem DPC laufen! (set_page_dirty_lock())
// - unmapping/pinnen 
// - TC/Job freigeben
void imago_DMARead_UnMapUserBuffer(PDEVICE_DATA pDevData, PDMA_READ_JOB pJob)
{
	u32 			iPage			= 0;

	dev_dbg(pDevData->dev, "UnMapUserBuffer\n");

	if (pDevData == NULL) {
		dev_err(pDevData->dev, "MappUserBuffer:invalid arguments");
		return;
	}
	if (pJob == NULL || pJob->pVMUser == 0)
		return;

	//unmapping/pinnen 
	/**********************************************************************/
	//- unmappen
	//https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
	// "PLEASE NOTE:The 'nents' argument to the dma_unmap_sg call must be
	//       		the _same_ one you passed into the dma_map_sg call,
	//	      		it should _NOT_ be the 'count' value _returned_ from the dma_map_sg call."
	//
	// " After the last DMA transfer call one of the DMA unmap routines
	//	 dma_unmap_{single,sg}. If you don't touch the data from the first dma_map_*
	//	 call till dma_unmap_*, then you don't have to call the dma_sync_* routines at all."
	//
	//lxr.free-electrons.com/source/arch/arm/mm/dma-mapping.c?v=3.8;a=arm 1500 (3.8)
	// 	__iommu_unmap_sg() kümmert sich um den cache
	//https://www.kernel.org/doc/Documentation/DMA-API.txt
	// "All the parameters must be the same as those and passed in to the scatter/gather mapping API."
	if (pJob->boIsSGMapped) {
		dma_unmap_sg(pDevData->dev,				/*struct device pointer*/
				pJob->SGTable.sgl,				/*struct scatterlist (Anfang)*/
				pJob->SGTable.nents,			/*nents, items of the list*/
				DMA_FROM_DEVICE);				/*Richtung der DMA*/
	}
	pJob->boIsSGMapped = false;

	//- freen der scatterlisten
	if (pJob->boIsSGValid)
		sg_free_table(&pJob->SGTable);
	pJob->boIsSGValid = false;

	//- unpinn Pages
	if (pJob->boIsPinned) {
		//über alle Pages gehen	
		for (iPage = 0; iPage < pJob->pagesPinned; iPage++) {
			//die Page als verändert marken
			//https://www.kernel.org/doc/htmldocs/kernel-api/API-get-user-pages.html
			// "..If the page is written to, set_page_dirty* must be called after the page is
			// finished with, and before put_page is called.."
			//
			//http://www.gossamer-threads.com/lists/linux/kernel/596302?do=post_view_threaded#596302
			//http://comments.gmane.org/gmane.linux.kernel/275722
			// ".. If you don't have a reference on the page's inode, yes, you should use
			// set_page_dirty_lock(). If the page came from get_user_pages() then surely
			// you don't have a ref on the inode.."
			//
			//LDDv3 S.437 
			// "...Most code that performs this operation checks first to ensure that the
			// page is not in the reserved part of the memory map, 
			// which is never swapped out. Since user-space memory is not normally marked reserved,
			// this check should not strictly be necessary.."
			//
			// "... Regardless of whether the pages have been changed, they must be freed from the
			//	page cache, or they stay there forever. ..."
			//
			if (!PageReserved(pJob->ppPageList[iPage])) {
				//macht lock_page, set_page_dirty, unlock_page
				set_page_dirty_lock(pJob->ppPageList[iPage] );
			}	

			//page_cache_release() ist ein define auf put_page()
			put_page( pJob->ppPageList[iPage] );		
		}
	}
	pJob->boIsPinned = false;

	//- freen der page liste
	if (pJob->boIsPageListValid)
		kfree( pJob->ppPageList );
	pJob->boIsPageListValid = false;
	
	pJob->pVMUser = 0;
}

// beendet eine DMA, egal ob mit oder ohne Fehler, ob gelaufen oder nicht (läuft auch aus DPC)
static void imago_DMARead_EndDMA(PDEVICE_DATA pDevData, const u32 iDMA, const u32 iTC, const bool isOk, const u16 BufferCounter)
{	
	PDMA_READ_CHANNEL pDMAChannel = &pDevData->DMARead_Channel[iDMA];
	PDMA_READ_TC pTC = &pDMAChannel->TCs[iTC];
	unsigned long flags;

	dev_dbg(pDevData->dev, "imago_DMARead_EndDMA > DMA: %d, TC: %d, Res: %d, Seq: %d\n",
				   	iDMA, iTC, isOk, BufferCounter);
	if (!pTC->boIsUsed) {
		dev_warn(pDevData->dev, "imago_DMARead_EndDMA(): invalid TC, DMA: %d, TC: %d, Res: %d, Seq: %d\n",
						iDMA, iTC, isOk, BufferCounter);
		return;
	}

	flags = imago_DMARead_Lock(pDevData);

	// check if current job has completed all SG elements
	if (isOk && pTC->pJob->SGItemsLeft != 0) {
		// start next transfer for this job
		imago_DMARead_StartNextTransfer_Locked(pDevData, iDMA, iTC);
		imago_DMARead_Unlock(pDevData, flags);
		return;
	}

	// copy job data from finished TC
	pTC->pJob->boIsOk			= isOk;
	pTC->pJob->BufferCounter	= BufferCounter;
	pTC->pJob->timestamp		= div_u64(ktime_get_ns(), NSEC_PER_USEC);

	// add job to Jobs_Done FIFO
	if (kfifo_put(&pDMAChannel->Jobs_Done, pTC->pJob) == 0) {
		//sollte nie sein weil Test ob Platz ist, ist im ioctl 'IOC_DMAREAD_ADD_BUFFER'
		dev_warn(pDevData->dev, "imago_DMARead_EndDMA > kfifo_put failed!\n");
	}

	// start next job in Jobs_ToDo FIFO
	if (kfifo_get(&pDMAChannel->Jobs_ToDo, &pTC->pJob) == 1) {
		imago_DMARead_StartNextTransfer_Locked(pDevData, iDMA, iTC);
	}
	else {
		// TC freigeben, unter lock setzen wegen race mit imago_DMARead_Reset_DMAChannel()
		pTC->boIsUsed = false;
	}

	imago_DMARead_Unlock(pDevData, flags);

	// notify thread
	complete(&pDMAChannel->job_complete);
}


/****************************************
 * 
 * DMARead fns
 *
****************************************/

// add job to FIFO and start transfer if idle
int imago_DMARead_AddJob(PDEVICE_DATA pDevData, u32 iDMA, DMA_READ_JOB *pJob)
{
	DMA_READ_CHANNEL *pDMAChannel = &pDevData->DMARead_Channel[iDMA];
	unsigned int iTC;
	unsigned long flags;

	pJob->SGItemsLeft = pJob->SGcount;
	pJob->pSGNext = pJob->SGTable.sgl;

	flags = imago_DMARead_Lock(pDevData);

	// start transfer if a transfer channel is idle
	for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
		if (!pDMAChannel->TCs[iTC].boIsUsed) {
			pDMAChannel->TCs[iTC].pJob = pJob;
			pDMAChannel->TCs[iTC].boIsUsed = true;

			imago_DMARead_StartNextTransfer_Locked(pDevData, iDMA, iTC);
			imago_DMARead_Unlock(pDevData, flags);

			return 0;
		}
	}

	// add job to Jobs_ToDo FIFO
	if (kfifo_put(&pDMAChannel->Jobs_ToDo, pJob) == 0) {
		imago_DMARead_Unlock(pDevData, flags);
		dev_warn(pDevData->dev, "DMARead_AddJob > Error adding buffer into JobToDo FIFO\n");
		return -ENOMEM;
	}
	
	imago_DMARead_Unlock(pDevData, flags);

	return 0;
}


//startet den nächste Transfer(SGs),
// - erkennt Fehler wenn möglich DMA abbrechen
// - nächste kann aber auch 1. bzw. letzte Transfer sein
// - kommt auch mit last+1(was nie sein sollte) zurecht(ist aber ein Fehler),
// - wird (auch) im DPC aufgerufen 
// - wird mit DMALock aufgerufen
void imago_DMARead_StartNextTransfer_Locked(PDEVICE_DATA pDevData, const u32 iDMA, const u32 iTC)
{
	s32 			iSG 	= 0;
	DMA_READ_TC 	*pTC 	= pDevData->DMARead_Channel[iDMA].TCs + iTC;
	DMA_READ_JOB 	*pJob	= pTC->pJob;
	u32				tempSG[DMA_READ_TC_TC2TC_SETPBYTES/4];

	dev_dbg(pDevData->dev, "imago_DMARead_StartNextTransfer_Locked > DMA: %d, TC: %d\n",	iDMA, iTC);

	//> alles gut (sicher ist sicher) kann aber nichts machen
	if ((iDMA >= pDevData->DMARead_channels) || (iTC >= pDevData->DMARead_TCs) ||
			!pTC->boIsUsed || !pJob->boIsSGValid || !pJob->boIsSGMapped || (pJob->SGItemsLeft==0) || (pJob->pSGNext==NULL)) {
		dev_err(pDevData->dev, "imago_DMARead_StartNextTransfer_Locked > Invalid context!\n");
		return;
	}

	//Flag 	(1DWord)
	tempSG[0] = DMA_READ_TC_SG_FLAG_START_TRANSFER; 
	if(pJob->pSGNext == pJob->SGTable.sgl)	//1. SG Element über alles
		tempSG[0] |= DMA_READ_TC_SG_FLAG_START_TRANSACTION; 

	//> die SGs ins FPGA schreiben
	/***********************************************************************/
	//pro Transfer können max ... SGs übertragen werden
	for (iSG=0; ((iSG < pDevData->DMARead_SGs) && (pJob->pSGNext!=NULL) && (pJob->SGItemsLeft>0) ); iSG++)  {
		u8			word;
		u32			sg_length	= sg_dma_len(pJob->pSGNext);
		dma_addr_t  sg_address	= sg_dma_address(pJob->pSGNext);
			
		//> SG zusammenbauen

		if (pJob->SGItemsLeft == 1)				//letzte SG Element über alles (somit auch IRQ auslösen)
			tempSG[0] |= DMA_READ_TC_SG_FLAG_END_TRANSACTION;

		if (iSG == (pDevData->DMARead_SGs-1))	//letzte SG Element, mehr geht nicht ins FPGA FIFO (mit dem SG darf es dann ein IRQ geben)
			tempSG[0] |= DMA_READ_TC_SG_FLAG_END_TRANSFER; 

		//Size	(1DWord)
		tempSG[1] = sg_length / 4;			// in DWORDs

		//Adr	(2DWord)
		tempSG[2] = sg_address & 0xFFFFFFFF;//LowPart;
#ifdef CONFIG_64BIT			
		tempSG[3] = sg_address >> 32;		//HighPart;
#else
		tempSG[3] = 0;						//HighPart;				
#endif		


		//> stimmt die Ausrichtung und die size? (adr und size) 
		if(		((sg_length & 0x3) != 0)
		 	||  (sg_length > DMA_READ_TC_SG_MAX_BYTECOUNT ) )
		{
			dev_err(pDevData->dev, "imago_DMARead_StartNextTransfer_Locked > Invalid alignment or size!\n");

			//sollt nie vorkommen und wenn ist es das letzte SG Element (size geht nicht auf)
			iSG = pDevData->DMARead_SGs;				//das Kaputte Element soll das letzte sein
			tempSG[0] |= DMA_READ_TC_SG_FLAG_ERROR; 	//Flag fürs FPGA
		}
	

		//> nächstes SGElement
		pJob->pSGNext =  sg_next(pJob->pSGNext);	//wenn es sg_is_last() dann gibt die fn NULL zurück, es müssen aber nicht alle Element einer SGListe benutzt sein.
		pJob->SGItemsLeft--;
	

		//> ins FPGA schreiben
		dev_dbg(pDevData->dev, "DMA SGs > i: %d > 0x%llx, Bytes %d\n", iSG, (u64) sg_address, sg_length);

		for (word = 0; word < (DMA_READ_TC_TC2TC_SETPBYTES/4); word++)
#ifdef __ARM_ARCH_7A__
			writel_relaxed(tempSG[word], pTC->pDesriptorFifo + word);
#else
			iowrite32(tempSG[word], pTC->pDesriptorFifo + word);
#endif

		tempSG[0] = 0;
	}//for max mögliche SGs pro Transfer
}


//Beendet alle DMAs die durch sind (bzw. deren DMATransfer), und versucht neue zu starte
void imago_DMARead_DPC(PDEVICE_DATA pDevData)
{
	u8 iDMA, iTC, BitShift;
	u32 IRQReg_A, IRQReg_B;
	u32 isDoneReg, isOkReg;
	u32 DMAMask;

	IRQReg_A = ((u32*)pDevData->pVACommonBuffer)[0];
	IRQReg_B = ((u32*)pDevData->pVACommonBuffer)[1];

	//Bit [3-0] sind reserved, Rest können für ReadDMAs sein
	DMAMask = pDevData->DMARead_channels * pDevData->DMARead_TCs;
	DMAMask = (1 << DMAMask)-1;

	isDoneReg = (IRQReg_A >> 4)	 & DMAMask;
	isOkReg = (~(IRQReg_B >> 4)) & DMAMask & isDoneReg; 

	dev_dbg(pDevData->dev, "imago_DMARead_DPC > isDoneReg: 0x%08X, isOkReg: 0x%08X\n", isDoneReg, isOkReg);

	/* über alle DMAs/TCs gehen, wir gehen davon aus das es keine lücken gibt!*/
	/**************************************************************************************/
	BitShift = 0;
	for (iDMA = 0; iDMA < pDevData->DMARead_channels; iDMA++) {
		for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
			bool isDone, isOk;
			u16 bufferCounter;
		
			//> Bits sammeln (damit es besser zum lesen ist)
			isDone 		= (isDoneReg >> BitShift) & 0x1;
			isOk 		= (isOkReg   >> BitShift) & 0x1;

			//> FPGA sagt die DMA ist durch
			if (isDone) {
				bufferCounter = *(u16 *)(pDevData->pVACommonBuffer + HOST_BUFFER_DMAREAD_COUNTER_OFFSET + 8 * BitShift);
				imago_DMARead_EndDMA(pDevData, iDMA, iTC, isOk, bufferCounter);
			}

			//> nächster TC ist beim nächsten Bit (auch wenn es die nächste DMA ist)
			BitShift++;
		}//for TC
	}//for DMA
}


//bricht laufende DMAs ab sowie ungenutzte buffer
int imago_DMARead_Abort_DMAChannel(PDEVICE_DATA pDevData, const u32 iDMA)
{
	PDMA_READ_CHANNEL pDMAChannel = &pDevData->DMARead_Channel[iDMA];
	DMA_READ_JOB *pJob = NULL;
	u8 iTC;
	unsigned long flags;

	dev_dbg(pDevData->dev, "imago_DMARead_Abort_DMAChannel> DMA: %d\n", iDMA);

	flags = imago_DMARead_Lock(pDevData);

	// move jobs from Jobs_ToDo to Jobs_Done queue and signal completion for each job
	while (kfifo_get(&pDMAChannel->Jobs_ToDo, &pJob) == 1) {
		pJob->boIsOk = false;

		if (kfifo_put(&pDMAChannel->Jobs_Done, pJob) == 0) {
			dev_warn(pDevData->dev, "Abort_DMAChannel: can't add Buffer to Jobs_Done, buffer lost\n");
			break;
		}

		complete(&pDMAChannel->job_complete);

		dev_dbg(pDevData->dev, "moving job Jobs_ToDo -> Jobs_Done\n");
	}

	//> für alle gültigen/laufenden DMAs, bei allen TCs.boIsUsed==true, 
	//> dann ein „SG“ mit Bit4 (Error) schicken (wird vor dem FIFO im FPGA abgefangen)
	//> Job/Request offen lassen	
	for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
		//gültiger Eintrag mit dem gesuchten Request? (wenn boIsUsed, dann ist auch der Request gültig)
		if (pDMAChannel->TCs[iTC].boIsUsed) {

			dev_dbg(pDevData->dev, "abort DMA iTC: %d \n", iTC);

			// cancel DMA in FPGA by setting the error flag
			iowrite32(DMA_READ_TC_SG_FLAG_ERROR, pDMAChannel->TCs[iTC].pDesriptorFifo + 0);
			iowrite32(0, pDMAChannel->TCs[iTC].pDesriptorFifo + 1);
			iowrite32(0, pDMAChannel->TCs[iTC].pDesriptorFifo + 2);
			iowrite32(0, pDMAChannel->TCs[iTC].pDesriptorFifo + 3);
		}//if boIsUsed					
	}//for iTC

	imago_DMARead_Unlock(pDevData, flags);
	return 0;
}


// Aborts all threads which are waiting for DMA job completion
int imago_DMARead_Abort_DMAWaiter(PDEVICE_DATA pDevData, const u32 iDMA)
{
	PDMA_READ_CHANNEL pDMAChannel = &pDevData->DMARead_Channel[iDMA];
	int i, threads;

	dev_dbg(pDevData->dev, "imago_DMARead_Abort_DMAWaiter> DMA: %u, threads: %u\n", iDMA, pDMAChannel->dmaWaitCount);

	if (pDMAChannel->abortWait) {
		dev_warn(pDevData->dev, "imago_DMARead_Abort_DMAWaiter(): abort DMA operation is already in progress\n");
		return -EALREADY;
	}

	// set the abortWait flag: signal the abort operation to waiting threads (IOCTL IOC_DMAREAD_WAIT_FOR_BUFFER).
	// important: the number of waiting threads (dmaWaitCount) is not allowed to increase after this point.
	pDMAChannel->abortWait = 1;
	threads = pDMAChannel->dmaWaitCount;

	up(&pDevData->DeviceSem);

	// signal semaphore for all threads
	for (i = 0; i < threads; i++)
		complete(&pDMAChannel->job_complete);

	// wait for all threads to wake up and take note of the abortWait flag (dmaWaitCount decreases to 0)
	while (pDMAChannel->dmaWaitCount != 0) {
		usleep_range(1*1000, 2*1000);
	}

	down(&pDevData->DeviceSem);

	// abort operation has finished, new waiting threads are allowed again
	pDMAChannel->abortWait = 0;
	
	return 0;
}


int imago_DMARead_Reset_DMAChannel(PDEVICE_DATA pDevData, unsigned int dma_channel)
{
	PDMA_READ_CHANNEL pDMAChannel = &pDevData->DMARead_Channel[dma_channel];
	unsigned int iTC;
	unsigned int i;
	unsigned long flags;

	// abort running DMA transfers in FPGA and move jobs from Jobs_ToDo to Jobs_Done FIFO
	imago_DMARead_Abort_DMAChannel(pDevData, dma_channel);

	// Wait for completion of pending transfers from FPGA
	for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
		flags = imago_DMARead_Lock(pDevData);
		if (pDMAChannel->TCs[iTC].boIsUsed) {
			// reset completion, removing count associated with jobs in Jobs_Done FIFO,
			// because we only wait once for report of the aborted transfer
			reinit_completion(&pDMAChannel->job_complete);
			imago_DMARead_Unlock(pDevData, flags);
			if (wait_for_completion_timeout(&pDMAChannel->job_complete, msecs_to_jiffies(100)) == 0) {
				dev_err(pDevData->dev, "imago_DMARead_Reset_DMAChannel(): DMA timeout waiting for lost job\n");
				return -EFAULT;
			}
		}
		else
			imago_DMARead_Unlock(pDevData, flags);
	}

	// unmap used job buffers
	for (i = 0; i < _ModuleData.max_dma_buffers; i++) {
		if (pDMAChannel->jobBuffers[i].pVMUser != 0) {
			dev_info(pDevData->dev, "imago_DMARead_Reset_DMAChannel(): unmapping lost buffer 0x%lx\n", pDMAChannel->jobBuffers[i].pVMUser);
			if (pDMAChannel->doManualMap)
				dma_sync_sg_for_cpu(pDevData->dev, pDMAChannel->jobBuffers[i].SGTable.sgl, pDMAChannel->jobBuffers[i].SGTable.nents, DMA_FROM_DEVICE);
			imago_DMARead_UnMapUserBuffer(pDevData, &pDMAChannel->jobBuffers[i]);
		}
	}
	// reset FIFOs
	kfifo_reset(&pDMAChannel->Jobs_ToDo);
	kfifo_reset(&pDMAChannel->Jobs_Done);

	// Reset completion
	reinit_completion(&pDMAChannel->job_complete);
	pDMAChannel->dmaWaitCount = 0;
	pDMAChannel->abortWait = 0;
	pDMAChannel->doManualMap = false;

	return 0;
}
