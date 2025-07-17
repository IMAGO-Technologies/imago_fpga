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

#ifdef DEBUG
static bool list_has_entry(struct list_head *entry, struct list_head *head)
{
	struct list_head *list_tmp;
	list_for_each(list_tmp, head) {
		if (list_tmp == entry)
			return true;
	}
	return false;
}
#endif


//Init und mapped/pinned den "Job<>UserBuffer", struct ist beim return(min die Flags gültig) wickelt daher beim Fehler nichts rück ab (kann nicht als DPC laufen)
int imago_dma_map(struct DEVICE_DATA *pDevData, struct DMA_READ_CHANNEL *pDMAChannel, uintptr_t pVMUser,
		u32 bufferSize, u8 reversePages, struct DMA_READ_JOB **ppJob)
{
	struct DMA_READ_JOB *pJob = NULL;
	u32	pagesToMap = (bufferSize + PAGE_SIZE-1) / PAGE_SIZE;
	int pagesPinned = -1;
	int mappedSGs = -1;
	unsigned long flags;
	int result;

	if (pDevData == NULL)
		return -EINVAL;

	dev_dbg(pDevData->dev, "MappUserBuffer: (%u bytes, %d pages @ 0x%p)\n", bufferSize, pagesToMap, (void*)pVMUser);

	if ((bufferSize & 0x3) != 0 || bufferSize <= 4) {
		dev_err(pDevData->dev, "MappUserBuffer: byte count is invalid");
		return -EINVAL;
	}
	if ((pVMUser & (PAGE_SIZE-1)) != 0) {
		dev_err(pDevData->dev, "MappUserBuffer: page pointer is not aligned");
		return -EINVAL;
	}

	// allocate job data from slab cache
	pJob = kmem_cache_zalloc(pDevData->dma_job_cache, GFP_KERNEL);
	*ppJob = pJob;
	if (pJob == NULL) {
		dev_err(pDevData->dev, "MappUserBuffer: error allocating DMA job");
		return -ENOMEM;
	}

	flags = imago_dma_lock(pDevData);
	list_add_tail(&pJob->list, &pDMAChannel->job_list_allocated);
	imago_dma_unlock(pDevData, flags);

	pJob->pVMUser = (uintptr_t) pVMUser;

	//> User Buffer Pinnen
	/**********************************************************************/
	//speicher für die PageList
	pJob->ppPageList = kmalloc(pagesToMap * sizeof(struct page*), GFP_KERNEL);
	if (pJob->ppPageList == NULL) {
		dev_err(pDevData->dev, "MappUserBuffer: too many pages");
		imago_dma_unmap(pDevData, pDMAChannel, pJob);
		return -ENOMEM;
	}

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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 5, 0)
	pagesPinned = get_user_pages(pJob->pVMUser, pagesToMap, FOLL_WRITE, pJob->ppPageList);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
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
		imago_dma_unmap(pDevData, pDMAChannel, pJob);
		return pagesPinned;
	}
	pJob->pagesPinned 	= pagesPinned;
	if (((u32)pagesPinned) != pagesToMap) {
		dev_err(pDevData->dev, "MappUserBuffer: get_user_pages() %d failed from %d pinned",
			pagesPinned, pagesToMap);
		imago_dma_unmap(pDevData, pDMAChannel, pJob);
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
										bufferSize			/*buffer size [bytes]*/,
										GFP_KERNEL);				/*alloc type*/
	if (result < 0) {			
		dev_err(pDevData->dev, "MappUserBuffer: sg_alloc_table_from_pages() failed");
		imago_dma_unmap(pDevData, pDMAChannel, pJob);
		return result;
	}
#else
{
	struct scatterlist *pSGList	= NULL;
	u32 bytesRemaining = bufferSize;
	s32	iSG = 0;

	result = sg_alloc_table(&pJob->SGTable	/*header*/, 
							pagesPinned 	/*für wie viele Einträge*/, 
							GFP_KERNEL);	/*wie wird die page gealloc*/
	if (result < 0) {
		dev_err(pDevData->dev, "MappUserBuffer: sg_alloc_table() failed");
		imago_dma_unmap(pDevData, pDMAChannel, pJob);
		return result;
	}

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
	pJob->SGTable.nents = 0;
	mappedSGs = dma_map_sg(	pDevData->dev,			/*struct device pointer*/	
							pJob->SGTable.sgl,		/*struct scatterlist (Anfang)*/
							pJob->SGTable.orig_nents,	/*anz der Buffers*/
							DMA_FROM_DEVICE); 		/*die Richtung wichtig für cache & bounce buffer*/
	if (mappedSGs <= 0) {
		dev_err(pDevData->dev, "MappUserBuffer: dma_map_sg() failed");
		return -EFAULT;
	}
	if (mappedSGs > 0xffff) {
		dev_err(pDevData->dev, "MappUserBuffer: dma_map_sg(): too many scatter elements");
		return -EFAULT;
	}
	pJob->SGTable.nents = mappedSGs;

	dev_dbg(pDevData->dev, "dma_map_sg(): %d SG list entries mapped to %d regions\n", pJob->SGTable.orig_nents, pJob->SGTable.nents);
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
		// We could evaluate nents instead, but this value may be non-deterministic depending
		// on memory fragmentation if no IOMMU is present. We want deterministic behavior for interrupt handling.
		if (pDevData->dev->iommu_group == NULL)
#endif
		{
			if (pJob->SGTable.orig_nents > 16) {
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
void imago_dma_unmap(struct DEVICE_DATA *pDevData, struct DMA_READ_CHANNEL *pDMAChannel, struct DMA_READ_JOB *pJob)
{
	unsigned long flags;

	dev_dbg(pDevData->dev, "UnMapUserBuffer\n");

	if (pDevData == NULL) {
		dev_err(pDevData->dev, "MappUserBuffer:invalid arguments");
		return;
	}
	if (pJob == NULL)
		return;

	// remove job from job_list_allocated
#ifdef DEBUG
	if (!list_has_entry(&pJob->list, &pDMAChannel->job_list_allocated))
		dev_err(pDevData->dev, "imago_dma_unmap: job not found in job_list_allocated\n");
#endif
	flags = imago_dma_lock(pDevData);
	list_del(&pJob->list);
	imago_dma_unlock(pDevData, flags);

	/**********************************************************************/
	// unmapp
	// https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
	//
	// " After the last DMA transfer call one of the DMA unmap routines
	//	 dma_unmap_{single,sg}. If you don't touch the data from the first dma_map_*
	//	 call till dma_unmap_*, then you don't have to call the dma_sync_* routines at all."
	//
	// 	__iommu_unmap_sg() kümmert sich um den cache
	// https://www.kernel.org/doc/Documentation/DMA-API.txt
	// "All the parameters must be the same as those and passed in to the scatter/gather mapping API."
	if (pJob->SGTable.nents > 0) {
		dma_unmap_sg(pDevData->dev,
				pJob->SGTable.sgl,
				pJob->SGTable.orig_nents,	/* same as used with dma_map_sg() call */
				DMA_FROM_DEVICE);
	}

	if (pJob->SGTable.sgl != NULL)
		sg_free_table(&pJob->SGTable);

	if (pJob->ppPageList != NULL) {
		// unpinn Pages
		int i;
		for (i = 0; i < pJob->pagesPinned; i++) {
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
			if (!PageReserved(pJob->ppPageList[i])) {
				//macht lock_page, set_page_dirty, unlock_page
				set_page_dirty_lock(pJob->ppPageList[i] );
			}

			//page_cache_release() ist ein define auf put_page()
			put_page( pJob->ppPageList[i] );
		}

		kfree( pJob->ppPageList );
		pJob->ppPageList = NULL;
	}
		
	kmem_cache_free(pDevData->dma_job_cache, pJob);
}


static void imago_dma_start(struct DEVICE_DATA *pDevData, const u32 iDMA, const u32 iTC)
{
	int iSG = 0;
	struct DMA_READ_TC *pTC = pDevData->DMARead_Channel[iDMA].TCs + iTC;
	struct DMA_READ_JOB *pJob = pTC->pJob;
	u32 dma_flags = DMA_READ_TC_SG_FLAG_START_TRANSFER;

	dev_dbg(pDevData->dev, "imago_dma_start > DMA: %d, TC: %d\n",	iDMA, iTC);

	if (iDMA >= pDevData->DMARead_channels || iTC >= pDevData->DMARead_TCs ||
			pJob == NULL || pTC->sg_remaining == 0 || pTC->sg_list == NULL) {
		dev_err(pDevData->dev, "imago_dma_start > Invalid context\n");
		return;
	}

	if (pTC->sg_list == pJob->SGTable.sgl)	// first element
		dma_flags |= DMA_READ_TC_SG_FLAG_START_TRANSACTION; 

	for (iSG=0; ((iSG < pDevData->DMARead_SGs) && (pTC->sg_list != NULL) && (pTC->sg_remaining > 0) ); iSG++) {
		u32			sg_length	= sg_dma_len(pTC->sg_list);
		dma_addr_t  sg_address	= sg_dma_address(pTC->sg_list);

		// check size
		if(		((sg_length & 0x3) != 0)
		 	||  (sg_length > DMA_READ_TC_SG_MAX_BYTECOUNT ) ) {
			dev_err(pDevData->dev, "imago_dma_start > Invalid DMA size!\n");
			dma_flags |= DMA_READ_TC_SG_FLAG_ERROR;		// abort transfer
			iSG = pDevData->DMARead_SGs;				// last element
		}
		else if (pTC->sg_remaining == 1) {				// last DMA element
			dma_flags |= DMA_READ_TC_SG_FLAG_END_TRANSACTION;
		}
		else if (iSG == (pDevData->DMARead_SGs-1)) {	// reached max. element count of FPGA
			dma_flags |= DMA_READ_TC_SG_FLAG_END_TRANSFER;
		}

		dev_dbg(pDevData->dev, "DMA SGs > i: %d > 0x%llx, Bytes %d\n", iSG, (u64) sg_address, sg_length);

		writel_relaxed(dma_flags,		pTC->pDesriptorFifo + 0);
		writel_relaxed(sg_length / 4,	pTC->pDesriptorFifo + 1);
#ifdef CONFIG_64BIT
		writeq_relaxed(sg_address,		pTC->pDesriptorFifo + 2);
#else
		writel_relaxed(sg_address,		pTC->pDesriptorFifo + 2);
		writel_relaxed(0,				pTC->pDesriptorFifo + 3);
#endif

		pTC->sg_list = sg_next(pTC->sg_list);
		pTC->sg_remaining--;
		dma_flags = 0;
	}
}


static void imago_dma_finish(struct DEVICE_DATA *pDevData, const u32 iDMA, const u32 iTC, const bool success, const u16 BufferCounter)
{	
	struct DMA_READ_CHANNEL *pDMAChannel = &pDevData->DMARead_Channel[iDMA];
	struct DMA_READ_TC *pTC = &pDMAChannel->TCs[iTC];
	unsigned long flags;

	dev_dbg(pDevData->dev, "imago_dma_finish > DMA: %d, TC: %d, Res: %d, Seq: %d\n",
				   	iDMA, iTC, success, BufferCounter);
	if (pTC->pJob == NULL) {
		dev_warn(pDevData->dev, "imago_dma_finish(): invalid TC, DMA: %d, TC: %d, Res: %d, Seq: %d\n",
						iDMA, iTC, success, BufferCounter);
		return;
	}

	flags = imago_dma_lock(pDevData);

	// check if current job has completed all SG elements
	if (success && pTC->sg_remaining != 0) {
		// start next transfer for this job
		imago_dma_start(pDevData, iDMA, iTC);
		imago_dma_unlock(pDevData, flags);
		return;
	}

	// copy job data from finished TC
	pTC->pJob->success			= success;
	pTC->pJob->BufferCounter	= BufferCounter;
	pTC->pJob->timestamp		= div_u64(ktime_get_ns(), NSEC_PER_USEC);

#ifdef DEBUG
	if (list_has_entry(&pTC->pJob->list, &pDMAChannel->job_list_allocated))
		dev_err(pDevData->dev, "imago_dma_finish: job already found in job_list_allocated\n");
	if (list_has_entry(&pTC->pJob->list, &pDMAChannel->job_list_pending))
		dev_err(pDevData->dev, "imago_dma_finish: job already found in job_list_pending\n");
	if (list_has_entry(&pTC->pJob->list, &pDMAChannel->job_list_complete))
		dev_err(pDevData->dev, "imago_dma_finish: job already found in job_list_complete\n");
#endif
	// move job to job_list_complete
	list_add_tail(&pTC->pJob->list, &pDMAChannel->job_list_complete);

	// start next job in job_list_pending
	if (!list_empty(&pDMAChannel->job_list_pending)) {
		pTC->pJob = list_first_entry(&pDMAChannel->job_list_pending, struct DMA_READ_JOB, list);
		list_del(&pTC->pJob->list);
		pTC->sg_remaining = pTC->pJob->SGTable.nents;
		pTC->sg_list = pTC->pJob->SGTable.sgl;
		imago_dma_start(pDevData, iDMA, iTC);
	}
	else {
		// TC freigeben, unter lock setzen wegen race mit imago_dma_reset()
		pTC->pJob = NULL;
	}

	imago_dma_unlock(pDevData, flags);

	// notify thread
	complete(&pDMAChannel->job_complete);
}


// start dma if idle, else add the job to job_list_pending
int imago_dma_addjob(struct DEVICE_DATA *pDevData, u32 iDMA, struct DMA_READ_JOB *pJob)
{
	struct DMA_READ_CHANNEL *pDMAChannel = &pDevData->DMARead_Channel[iDMA];
	int iTC;
	unsigned long flags;

	flags = imago_dma_lock(pDevData);

#ifdef DEBUG
	if (!list_has_entry(&pJob->list, &pDMAChannel->job_list_allocated))
		dev_err(pDevData->dev, "imago_dma_addjob: job not found in job_list_allocated\n");
#endif

	// start transfer if a transfer channel is idle
	for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
		if (pDMAChannel->TCs[iTC].pJob == NULL) {
			// remove from job_list_allocated
			list_del(&pJob->list);
			pDMAChannel->TCs[iTC].pJob = pJob;
			pDMAChannel->TCs[iTC].sg_remaining = pJob->SGTable.nents;
			pDMAChannel->TCs[iTC].sg_list = pJob->SGTable.sgl;

			imago_dma_start(pDevData, iDMA, iTC);
			imago_dma_unlock(pDevData, flags);

			return 0;
		}
	}

	// move job from job_list_allocated to job_list_pending
	list_move_tail(&pJob->list, &pDMAChannel->job_list_pending);
	
	imago_dma_unlock(pDevData, flags);

	return 0;
}


// DMA event handler
void imago_dma_event(struct DEVICE_DATA *pDevData)
{
	int iDMA, iTC;
	u8 shift;
	u32 IRQReg_A, IRQReg_B;
	u32 completeReg, successReg;
	u32 DMAMask;

	IRQReg_A = ((u32*)pDevData->pVACommonBuffer)[0];
	IRQReg_B = ((u32*)pDevData->pVACommonBuffer)[1];

	//Bit [3-0] sind reserved, Rest können für ReadDMAs sein
	DMAMask = pDevData->DMARead_channels * pDevData->DMARead_TCs;
	DMAMask = (1 << DMAMask)-1;

	completeReg = (IRQReg_A >> 4)	& DMAMask;
	successReg = (~(IRQReg_B >> 4)) & DMAMask & completeReg; 

	dev_dbg(pDevData->dev, "imago_dma_event > completeReg: 0x%08X, successReg: 0x%08X\n", completeReg, successReg);

	shift = 0;
	for (iDMA = 0; iDMA < pDevData->DMARead_channels; iDMA++) {
		for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
			bool complete, success;
			u16 bufferCounter;
		
			complete	= (completeReg >> shift) & 0x1;
			success 	= (successReg  >> shift) & 0x1;

			if (complete) {
				bufferCounter = *(u16 *)(pDevData->pVACommonBuffer + HOST_BUFFER_DMAREAD_COUNTER_OFFSET + 8 * shift);
				imago_dma_finish(pDevData, iDMA, iTC, success, bufferCounter);
			}

			// bit shift increases across TC and DMA channels
			shift++;
		}
	}
}


// aborts DMA channel transfers and moves pending jobs to job_list_complete
int imago_dma_abort(struct DEVICE_DATA *pDevData, const u32 iDMA)
{
	struct DMA_READ_CHANNEL *pDMAChannel = &pDevData->DMARead_Channel[iDMA];
	int iTC;
	unsigned long flags;
	struct list_head *list_tmp, *list_next;

	dev_dbg(pDevData->dev, "imago_dma_abort> DMA: %d\n", iDMA);

	flags = imago_dma_lock(pDevData);

	// move jobs from job_list_pending to job_list_complete and signal completion for each job
	list_for_each_safe(list_tmp, list_next, &pDMAChannel->job_list_pending) {
		struct DMA_READ_JOB *pJob = list_entry(list_tmp, struct DMA_READ_JOB, list);
		pJob->success = false;

		dev_dbg(pDevData->dev, "moving job from job_list_pending to job_list_complete\n");

		list_move_tail(list_tmp, &pDMAChannel->job_list_complete);

		complete(&pDMAChannel->job_complete);
	}

	//> für alle gültigen/laufenden DMAs, bei allen TCs.boIsUsed==true, 
	//> dann ein „SG“ mit Bit4 (Error) schicken (wird vor dem FIFO im FPGA abgefangen)
	//> Job/Request offen lassen	
	for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
		//gültiger Eintrag mit dem gesuchten Request? (wenn boIsUsed, dann ist auch der Request gültig)
		if (pDMAChannel->TCs[iTC].pJob != NULL) {

			dev_dbg(pDevData->dev, "abort DMA iTC: %d \n", iTC);

			// cancel DMA in FPGA by setting the error flag
			iowrite32(DMA_READ_TC_SG_FLAG_ERROR, pDMAChannel->TCs[iTC].pDesriptorFifo + 0);
			iowrite32(0, pDMAChannel->TCs[iTC].pDesriptorFifo + 1);
			iowrite32(0, pDMAChannel->TCs[iTC].pDesriptorFifo + 2);
			iowrite32(0, pDMAChannel->TCs[iTC].pDesriptorFifo + 3);
		}					
	}

	imago_dma_unlock(pDevData, flags);
	return 0;
}


// aborts all threads which are waiting for DMA job completion
int imago_dma_abort_threads(struct DEVICE_DATA *pDevData, const u32 iDMA)
{
	struct DMA_READ_CHANNEL *pDMAChannel = &pDevData->DMARead_Channel[iDMA];
	int i, threads;

	dev_dbg(pDevData->dev, "imago_dma_abort_threads> DMA: %u, threads: %u\n", iDMA, pDMAChannel->dmaWaitCount);

	if (pDMAChannel->abortWait) {
		dev_warn(pDevData->dev, "imago_dma_abort_threads(): abort DMA operation is already in progress\n");
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


// This function is called by the library during channel initialization (CameraLinkIn::ConfigureCLInput)
// in case the DMA channel was not shutdown properly before.
int imago_dma_reset(struct DEVICE_DATA *pDevData, unsigned int dma_channel)
{
	struct DMA_READ_CHANNEL *pDMAChannel = &pDevData->DMARead_Channel[dma_channel];
	int i;
	unsigned long flags;
	struct list_head *list_tmp, *list_next;

	// abort running DMA transfers in FPGA and move jobs from job_list_pending to job_list_complete
	imago_dma_abort(pDevData, dma_channel);

	// wait for completion of pending transfers by FPGA
	for (i = 0; i < pDevData->DMARead_TCs; i++) {
		flags = imago_dma_lock(pDevData);
		if (pDMAChannel->TCs[i].pJob != NULL) {
			// reset completion, removing count associated with jobs in Jobs_Done FIFO,
			// because we only wait once for report of the aborted transfer
			reinit_completion(&pDMAChannel->job_complete);
			imago_dma_unlock(pDevData, flags);
			if (wait_for_completion_timeout(&pDMAChannel->job_complete, msecs_to_jiffies(100)) == 0) {
				dev_err(pDevData->dev, "imago_dma_reset(): DMA timeout waiting for lost job\n");
				return -EFAULT;
			}
		}
		else
			imago_dma_unlock(pDevData, flags);
	}

	// move entries from job_list_complete to job_list_allocated
	list_splice_init(&pDMAChannel->job_list_complete, &pDMAChannel->job_list_allocated);

	// unmap job buffers and remove them from job_list_allocated
	list_for_each_safe(list_tmp, list_next, &pDMAChannel->job_list_allocated) {
		struct DMA_READ_JOB *pJob = list_entry(list_tmp, struct DMA_READ_JOB, list);
		dev_info(pDevData->dev, "imago_dma_reset(): unmapping lost buffer 0x%lx\n", pJob->pVMUser);
		if (pDMAChannel->doManualMap)
			dma_sync_sg_for_cpu(pDevData->dev, pJob->SGTable.sgl, pJob->SGTable.orig_nents, DMA_FROM_DEVICE);
		imago_dma_unmap(pDevData, pDMAChannel, pJob);
	}

#ifdef DEBUG
	if (!list_empty(&pDMAChannel->job_list_allocated))
		dev_err(pDevData->dev, "imago_dma_reset: job_list_allocated is not empty\n");
	if (!list_empty(&pDMAChannel->job_list_pending))
		dev_err(pDevData->dev, "imago_dma_reset: job_list_pending is not empty\n");
	if (!list_empty(&pDMAChannel->job_list_complete))
		dev_err(pDevData->dev, "imago_dma_reset: job_list_complete is not empty\n");
#endif

	// Reset completion
	reinit_completion(&pDMAChannel->job_complete);
	pDMAChannel->dmaWaitCount = 0;
	pDMAChannel->abortWait = 0;
	pDMAChannel->doManualMap = false;

	return 0;
}
