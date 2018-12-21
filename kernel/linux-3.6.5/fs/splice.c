/*
 * "splice": joining two ropes together by interweaving their strands.
 *
 * This is the "extended pipe" functionality, where a pipe is used as
 * an arbitrary in-memory buffer. Think of a pipe as a small kernel
 * buffer that you can use to transfer data from one end to the other.
 *
 * The traditional unix read/write is extended with a "splice()" operation
 * that transfers data buffers to or from a pipe buffer.
 *
 * Named by Larry McVoy, original implementation from Linus, extended by
 * Jens to support splicing to files, network, direct splicing, etc and
 * fixing lots of bugs.
 *
 * Copyright (C) 2005-2006 Jens Axboe <axboe@kernel.dk>
 * Copyright (C) 2005-2006 Linus Torvalds <torvalds@osdl.org>
 * Copyright (C) 2006 Ingo Molnar <mingo@elte.hu>
 *
 */
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/pagemap.h>
#include <linux/splice.h>
#include <linux/memcontrol.h>
#include <linux/mm_inline.h>
#include <linux/swap.h>
#include <linux/writeback.h>
#include <linux/export.h>
#include <linux/syscalls.h>
#include <linux/uio.h>
#include <linux/security.h>
#include <linux/gfp.h>
#ifdef CONFIG_BCM_CUSTOM_RECVFILE
#include <net/sock.h>
#endif
#include <linux/socket.h>
#ifdef CONFIG_BROADCOM_CUSTOM_SENDFILE
#include <linux/vmalloc.h>      
#include <linux/workqueue.h>   
#include <linux/semaphore.h> 
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/blkdev.h>
#include <linux/cpumask.h>
#include <asm/smp.h>
#endif /* CONFIG_BROADCOM_CUSTOM_SENDFILE */

/*
 * Attempt to steal a page from a pipe buffer. This should perhaps go into
 * a vm helper function, it's already simplified quite a bit by the
 * addition of remove_mapping(). If success is returned, the caller may
 * attempt to reuse this page for another destination.
 */
static int page_cache_pipe_buf_steal(struct pipe_inode_info *pipe,
				     struct pipe_buffer *buf)
{
	struct page *page = buf->page;
	struct address_space *mapping;

	lock_page(page);

	mapping = page_mapping(page);
	if (mapping) {
		WARN_ON(!PageUptodate(page));

		/*
		 * At least for ext2 with nobh option, we need to wait on
		 * writeback completing on this page, since we'll remove it
		 * from the pagecache.  Otherwise truncate wont wait on the
		 * page, allowing the disk blocks to be reused by someone else
		 * before we actually wrote our data to them. fs corruption
		 * ensues.
		 */
		wait_on_page_writeback(page);

		if (page_has_private(page) &&
		    !try_to_release_page(page, GFP_KERNEL))
			goto out_unlock;

		/*
		 * If we succeeded in removing the mapping, set LRU flag
		 * and return good.
		 */
		if (remove_mapping(mapping, page)) {
			buf->flags |= PIPE_BUF_FLAG_LRU;
			return 0;
		}
	}

	/*
	 * Raced with truncate or failed to remove page from current
	 * address space, unlock and return failure.
	 */
out_unlock:
	unlock_page(page);
	return 1;
}

static void page_cache_pipe_buf_release(struct pipe_inode_info *pipe,
					struct pipe_buffer *buf)
{
	page_cache_release(buf->page);
	buf->flags &= ~PIPE_BUF_FLAG_LRU;
}

/*
 * Check whether the contents of buf is OK to access. Since the content
 * is a page cache page, IO may be in flight.
 */
static int page_cache_pipe_buf_confirm(struct pipe_inode_info *pipe,
				       struct pipe_buffer *buf)
{
	struct page *page = buf->page;
	int err;

	if (!PageUptodate(page)) {
		lock_page(page);

		/*
		 * Page got truncated/unhashed. This will cause a 0-byte
		 * splice, if this is the first page.
		 */
		if (!page->mapping) {
			err = -ENODATA;
			goto error;
		}

		/*
		 * Uh oh, read-error from disk.
		 */
		if (!PageUptodate(page)) {
			err = -EIO;
			goto error;
		}

		/*
		 * Page is ok afterall, we are done.
		 */
		unlock_page(page);
	}

	return 0;
error:
	unlock_page(page);
	return err;
}

const struct pipe_buf_operations page_cache_pipe_buf_ops = {
	.can_merge = 0,
	.map = generic_pipe_buf_map,
	.unmap = generic_pipe_buf_unmap,
	.confirm = page_cache_pipe_buf_confirm,
	.release = page_cache_pipe_buf_release,
	.steal = page_cache_pipe_buf_steal,
	.get = generic_pipe_buf_get,
};

static int user_page_pipe_buf_steal(struct pipe_inode_info *pipe,
				    struct pipe_buffer *buf)
{
	if (!(buf->flags & PIPE_BUF_FLAG_GIFT))
		return 1;

	buf->flags |= PIPE_BUF_FLAG_LRU;
	return generic_pipe_buf_steal(pipe, buf);
}

static const struct pipe_buf_operations user_page_pipe_buf_ops = {
	.can_merge = 0,
	.map = generic_pipe_buf_map,
	.unmap = generic_pipe_buf_unmap,
	.confirm = generic_pipe_buf_confirm,
	.release = page_cache_pipe_buf_release,
	.steal = user_page_pipe_buf_steal,
	.get = generic_pipe_buf_get,
};

static void wakeup_pipe_readers(struct pipe_inode_info *pipe)
{
	smp_mb();
	if (waitqueue_active(&pipe->wait))
		wake_up_interruptible(&pipe->wait);
	kill_fasync(&pipe->fasync_readers, SIGIO, POLL_IN);
}

/**
 * splice_to_pipe - fill passed data into a pipe
 * @pipe:	pipe to fill
 * @spd:	data to fill
 *
 * Description:
 *    @spd contains a map of pages and len/offset tuples, along with
 *    the struct pipe_buf_operations associated with these pages. This
 *    function will link that data to the pipe.
 *
 */
ssize_t splice_to_pipe(struct pipe_inode_info *pipe,
		       struct splice_pipe_desc *spd)
{
	unsigned int spd_pages = spd->nr_pages;
	int ret, do_wakeup, page_nr;

	ret = 0;
	do_wakeup = 0;
	page_nr = 0;

	pipe_lock(pipe);

	for (;;) {
		if (!pipe->readers) {
			send_sig(SIGPIPE, current, 0);
			if (!ret)
				ret = -EPIPE;
			break;
		}

		if (pipe->nrbufs < pipe->buffers) {
			int newbuf = (pipe->curbuf + pipe->nrbufs) & (pipe->buffers - 1);
			struct pipe_buffer *buf = pipe->bufs + newbuf;

			buf->page = spd->pages[page_nr];
			buf->offset = spd->partial[page_nr].offset;
			buf->len = spd->partial[page_nr].len;
			buf->private = spd->partial[page_nr].private;
			buf->ops = spd->ops;
			if (spd->flags & SPLICE_F_GIFT)
				buf->flags |= PIPE_BUF_FLAG_GIFT;

			pipe->nrbufs++;
			page_nr++;
			ret += buf->len;

			if (pipe->inode)
				do_wakeup = 1;

			if (!--spd->nr_pages)
				break;
			if (pipe->nrbufs < pipe->buffers)
				continue;

			break;
		}

		if (spd->flags & SPLICE_F_NONBLOCK) {
			if (!ret)
				ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			if (!ret)
				ret = -ERESTARTSYS;
			break;
		}

		if (do_wakeup) {
			smp_mb();
			if (waitqueue_active(&pipe->wait))
				wake_up_interruptible_sync(&pipe->wait);
			kill_fasync(&pipe->fasync_readers, SIGIO, POLL_IN);
			do_wakeup = 0;
		}

		pipe->waiting_writers++;
		pipe_wait(pipe);
		pipe->waiting_writers--;
	}

	pipe_unlock(pipe);

	if (do_wakeup)
		wakeup_pipe_readers(pipe);

	while (page_nr < spd_pages)
		spd->spd_release(spd, page_nr++);

	return ret;
}

void spd_release_page(struct splice_pipe_desc *spd, unsigned int i)
{
	page_cache_release(spd->pages[i]);
}

/*
 * Check if we need to grow the arrays holding pages and partial page
 * descriptions.
 */
int splice_grow_spd(const struct pipe_inode_info *pipe, struct splice_pipe_desc *spd)
{
	unsigned int buffers = ACCESS_ONCE(pipe->buffers);

	spd->nr_pages_max = buffers;
	if (buffers <= PIPE_DEF_BUFFERS)
		return 0;

	spd->pages = kmalloc(buffers * sizeof(struct page *), GFP_KERNEL);
	spd->partial = kmalloc(buffers * sizeof(struct partial_page), GFP_KERNEL);

	if (spd->pages && spd->partial)
		return 0;

	kfree(spd->pages);
	kfree(spd->partial);
	return -ENOMEM;
}

void splice_shrink_spd(struct splice_pipe_desc *spd)
{
	if (spd->nr_pages_max <= PIPE_DEF_BUFFERS)
		return;

	kfree(spd->pages);
	kfree(spd->partial);
}

static ssize_t kernel_readv(struct file *file, const struct iovec *vec,
			    unsigned long vlen, loff_t offset)
{
	mm_segment_t old_fs;
	loff_t pos = offset;
	ssize_t res;

	old_fs = get_fs();
	set_fs(get_ds());
	/* The cast to a user pointer is valid due to the set_fs() */
	res = vfs_readv(file, (const struct iovec __user *)vec, vlen, &pos);
	set_fs(old_fs);

	return res;
}

static ssize_t kernel_write(struct file *file, const char *buf, size_t count,
			    loff_t pos)
{
	mm_segment_t old_fs;
	ssize_t res;

	old_fs = get_fs();
	set_fs(get_ds());
	/* The cast to a user pointer is valid due to the set_fs() */
	res = vfs_write(file, (const char __user *)buf, count, &pos);
	set_fs(old_fs);

	return res;
}

#if defined(CONFIG_BROADCOM_CUSTOM_SENDFILE) && (PAGE_SIZE == 4096)

/* 
 * custom implementation of sendfile 
 *
 * the intent of the custom sendfile (csf) implementation is to
 * improve performance of single file read operations (especially
 * benchmarks) on the Broadcom NorthStar+ (NS+) SOC
 *
 * the primary application of interest is Samba
 *
 * the approach for improving performance is based on better utilizing
 * both cores of the NS+ 
 *
 * one core is dedicated to Samba
 *
 * the other core is utilized for interrupt processing and read ahead
 *
 * Samba calls sendfile to read from disk and transmit to a socket
 *
 * the sendfile implementation communicates with a read ahead
 * function using a kernel work queue
 *
 * the read ahead function predictively populates a read ahead list
 *
 * the read ahead function implements a state machine to detect
 * sequential vs. non-sequential access patterns, different types of 
 * read ahead are performed in the different states:
 *
 *   - in sequential read ahead state, the reads are adjacent to, 
 *     immediately succeeding, the most recent read request
 *
 *   - in non-sequential read ahead state, sequential reads are performed
 *     but at a delta further into the file, the start index for these 
 *     read aheads is based on saved state info and not the indices most 
 *     recently read by the application, this is necessary to perform 
 *     sequential read aheads when the application access pattern is not 
 *     sequential, doing sequential reads is important to maximize 
 *     performance, the read ahead size is the same as the most
 *     recent read request, which is important for balancing the
 *     the work load on both processor cores

 * before doing the predictive i/o, the read ahead functions makes
 * the following checks:
 *   - are any of the pages to be read already in the read ahead list
 *   - have any of the pages to be read been consumed by the application
 *     (via read) recently
 * if either criteria is met, the predictive i/o is not performed
 * in order to minimize unproductive i/o since:
 *   - pages that are already in the read ahead list obviously
 *     don't need to be read again, and
 *   - we assume that the application is not reading the same
 *     data multiple times 
 *
 * the read ahead list is a doubly-linked list, and each entry
 * in the linked list can cover a range of pages
 *
 * the read ahead function will perform non-agressive pruning of the
 * list when traversing the list to perform normal searches
 *
 * the number of entries on the read ahead list is constrained to a 
 * configured maximum value
 *
 * the read ahead function will perform aggressive pruning of the
 * list when the size reaches the maximum capacity
 * 
 * a most recently used (mru) data structure is utilized to track
 * the pages that have been most recently consumed by the application
 * 
 * a semaphore is used to coordinate execution of sendfile system calls
 * and the read ahead function
 *
 * sendfile system calls and the read ahead function operate in lock
 * step fashion
 *
 * sendfile waits for the previous read ahead function to complete execution
 *
 * sendfile checks whether the requested pages are in the read ahead list
 *
 * sendfile then kicks off the next execution of the read ahead function
 *
 * the semaphore protects the read ahead list, no other locking mechanisms
 * are used to protect accesses to the read ahead list
 *
 * sendfile will initiate i/o for requested pages that are not in the
 * read ahead list
 *
 * a private pool of pages is allocated for use by the csf implementation
 *
 * these pages are independent of the Linux page cache
 *
 * free pages are maintained in a linked list, and a spin lock is used
 * to protect the free pages linked list
 *
 * a page element struct is used to insert a page in the free pages linked list
 * 
 * when i/o is initiated on a page, the page element struct is inserted in
 * a linked list of free page element structs
 *
 * when i/o completes on a page, a page element struct is removed from
 * the linked list of free page element structs, and the page element
 * struct is then used to insert the page in the free pages linked list
 *
 * the linked list of free page element structs is protected by a spinlock,
 * the same spinlock is used to protect the free pages linked list and
 * the linked list of free page element structs
 *
 * the pages are mapped into kernel virtual address space and the PTE
 * entries associated with the mappings are marked as non-cached (the
 * pages are also flushed from the processor caches)
 *
 * the reason for the non-cached mapping is a desire to avoid
 * cache invalidation operations for disk i/o
 *
 * the SATA controller on the A0 version of NS+ is not cache coherent,
 * thus 2 cache operations are typically performed for each DMA read 
 * from disk:
 *   - a cache flush prior to the DMA (to protect against writebacks
 *     during/after the DMA), and
 *   - a cache invalidate after the DMA (to protect against prefetching
 *     during the DMA)
 *
 * these cache operations are expensive from a performance perspective,
 * and there is a strong desire to avoid them 
 *
 * the csf implementation has hooks in the DMA cache management 
 * functions to skip the cache operations for pages that are
 * part of the private csf page pool 
 *
 * the page struct is expanded by 8 bytes to support the csf
 * implementation, the additional information includes:
 *   - a field that identifies the page as belonging to the
 *     private csf page pool, and
 *   - a field containing the kernel virtual address of the
 *     page associated with the non-cached mapping 
 *
 * use of the non-cached mappings is only practical in
 * conjunction with zero-copy sendfile implementations 
 *
 * copies from non-cached memory perform very poorly
 *
 * zero-copy sendfile has been implemented and integrated
 * with the csf implementation
 * 
 * a hook has been implemented in the file close system call,
 * the hook calls a csf function to delete csf data structures 
 * with references to the file being closed
 *
 * the current implementation should be viewed as a prototype
 * intended to demonstrate maximum performance
 *
 * productization of the prototype should include:
 *
 *   - integration of CONFIG_BROADCOM_CUSTOM_SENDFILE define with
 *     kernel build, so that the csf implementation is a
 *     kernel build option, the definition is currently in
 *     include/linux/mm_types.h
 * 
 * the SATA controller on the B0 version of NS+ will be cache coherent,
 * thus the cache coherency issues will no longer be present, however
 * the csf implementation may still be needed to maximize performance
 */

#ifndef FALSE 
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/* constants and tuning parameters */
#define CSF_NUM_PAGES   		          8448 /* number of pages to allocate */
#define CSF_MAX_RA_PAGES                          8192 /* max number of pages allowed in ra list */
#define CSF_MAX_RA_SIZE                             16 /* number of pages to ra of most recent request */  
#define CSF_RA_INDEX_WINDOW_BACK                  4096 /* max backward relative page index allowed in ra list */
#define CSF_RA_INDEX_WINDOW_FORWARD               4096 /* max forward relative page index allowed in ra list */
#define CSF_PRUNE_RA_INDEX_WINDOW_BACK              32 /* for more aggressive pruning of ra list based on page index */ 
#define CSF_PRUNE_RA_INDEX_WINDOW_FORWARD           32 
#define CSF_RA_AGE_OUT_SECS                          3 /* for pruning read ahead list entries based on age */
#define CSF_PAGE_MRU_ARRAY_SIZE                     16 /* number of page ranges tracked in mru array */ 
#define CSF_MIN_PAGES_TO_DO_RA                       1 /* min read size to kick off read ahead */ 
#define CSF_MAX_PAGE_INDEX      ((pgoff_t) 0xffffffff)

/*
 * read ahead states
 *
 * the states are currently pretty rudimentary
 *   INIT: determines whether access pattern is
 *         sequential or non-sequential
 *   SEQ:  sequential access pattern and all
 *         accesses in beginning portion of file 
 *   SEQ_STEADY: steady state of sequential access pattern 
 *   NON_SEQ: non-sequential access pattern and all
 *            accesses in beginning portion of file 
 *   NON_SEQ_STEADY: steady state of non-sequential access pattern 
 *
 *   State            Event                                                               Next State
 *   ===============================================================================================
 *   INIT             # of consecutive sequetial reads > CSF_RA_SEQ_INDEX_THRESHOLD       SEQ 
 *   INIT             non sequential read AND file size < CSF_RA_NON_SEQ_MIN_FILE_PAGES   SEQ 
 *   INIT             non sequential read AND file size >= CSF_RA_NON_SEQ_MIN_FILE_PAGES  NON_SEQ 
 *   SEQ              read index > CSF_RA_EARLY_INDEX_THRESHOLD                           SEQ_STEADY 
 *   SEQ_STEADY       (# of consecutive reads with index < CSF_RA_EARLY_INDEX_THRESHOLD)  INIT
 *                    is greater than CSF_RA_EARLY_INDEX_HYSTERESIS                       
 *   NON_SEQ          read index > CSF_RA_EARLY_INDEX_THRESHOLD                           NON_SEQ_STEADY 
 *   NON_SEQ_STEADY   (# of consecutive reads with index < CSF_RA_EARLY_INDEX_THRESHOLD)  INIT
 *                    is greater than CSF_RA_EARLY_INDEX_HYSTERESIS                       
 *
 * different types of read ahead are performed in the different states
 *
 *   - in sequential read ahead state, the reads are adjacent to, immediately succeeding,
 *     the most recent read request, the max size is CSF_MAX_RA_SIZE
 *
 *   - in non-sequential read ahead state, sequential reads are performed 
 *     but at a delta further into the file, the start index for these 
 *     read aheads is based on saved state info and not the indices most 
 *     recently read by the application, this is necessary to perform 
 *     sequential read aheads when the application access pattern is not 
 *     sequential, doing sequential reads is important to maximize performance, 
 *     the read ahead size is the same as the most recent read request, 
 *     which is important for achieving a similar work load on both processor cores
 */
typedef enum {
	CSF_RA_INIT_STATE,
        CSF_RA_SEQ_STATE,
        CSF_RA_SEQ_STEADY_STATE,
        CSF_RA_NON_SEQ_STATE, 
        CSF_RA_NON_SEQ_STEADY_STATE
} csf_ra_state_e;

/* parameters associated with read ahead state */
#define CSF_RA_INIT_STATE_IGNORE_CNT      3      /* ignore first few packets for state transition purposes,     */
                                                 /* some OS's read from both beginning and end of file          */
                                                 /* before doing sequential reads                               */
#define CSF_RA_NON_SEQ_MIN_FILE_PAGES  10000     /* min file size in pages for non-sequential mode,             */
                                                 /* just do sequential read ahead for smaller files             */  
#define CSF_RA_NON_SEQ_DELTA            1832     /* read ahead delta for for non-sequential mode                */
#define CSF_RA_EARLY_INDEX_THRESHOLD     512     /* threshold for determining that file acess has proceeded     */
                                                 /* past initial stage into steady state                        */
#define CSF_RA_SEQ_INDEX_THRESHOLD       10      /* threshold used to determine that access pattern is          */
                                                 /* sequential, required number of consecutive sequential       */
                                                 /* accesses for determination                                  */          
#define CSF_RA_EARLY_INDEX_HYSTERESIS     5      /* hysteresis threshold used to determine that access pattern  */
                                                 /* has tranistioned out of steady state, required number of    */
                                                 /* consecutive accesses with indices less than early threshold */ 
#define CSF_RA_FILE_MRU_ARRAY_SIZE       16      /* size of read ahead array tracking most recently used files  */
#define CSF_RA_FILE_MRU_AGE_OUT_SECS      3      /* entries ra file mru data structure are only valid if they   */
                                                 /* have been accessed within this interval                     */

/* controls whether debug information is collected/displayed */
#define CSF_DEBUG_ENABLE              FALSE 

/* controls frequency at which debug information is displayed */
#define CSF_DEBUG_CONTROL_CNT_INTERVAL 2500 

/* debug macros */
#if (CSF_DEBUG_ENABLE)
#define CSF_DEBUG_CNT_INC(CNT)            ((CNT)++)
#define CSF_DEBUG_CNT_ADD(CNT, AMOUNT)    ((CNT) += (AMOUNT))
#define CSF_DEBUG_MSG(STR)                printk(STR)
#define CSF_DEBUG(CONTROL_CNT, RA, DUMP_PAGE_INFO, DUMP_STATS, DUMP_TRACE)   \
	csf_debug(CONTROL_CNT, RA, DUMP_PAGE_INFO, DUMP_STATS, DUMP_TRACE)
#else
#define CSF_DEBUG_CNT_INC(CNT)
#define CSF_DEBUG_CNT_ADD(CNT, AMOUNT)
#define CSF_DEBUG_MSG(STR)   
#define CSF_DEBUG(CONTROL_CNT, RA, DUMP_PAGE_INFO, DUMP_STATS, DUMP_TRACE)
#endif

/* element for inserting page in list */
struct csf_page_list_element {
        struct csf_page_list_element *next;
        struct csf_page_list_element *prev;
        struct page                  *page;
};

/* range of page indices */
struct csf_page_range {
	pgoff_t start_index;
	pgoff_t end_index;
};

/* page info for use in read ahead list element */ 
struct csf_ra_sublist_element {
        struct page *page;
	pgoff_t      page_index;
};

/* element in read ahead list, may contain multiple pages */
struct csf_ra_list_element {
        struct csf_ra_list_element   *next;
        struct csf_ra_list_element   *prev;
        struct file                  *file;        /* file that element is associated with */
	struct timespec               insert_time; /* time element was inserted into ra list */ 
        uint32_t                      num_entries; /* valid entries in sublist */
        struct csf_page_range         page_range;  /* max page index range in sublist */
        struct csf_ra_sublist_element sublist[CSF_MAX_RA_SIZE]; 
};

/* read ahead list */
struct csf_ra_list {
	struct csf_ra_list_element *head;
	struct csf_ra_list_element *tail;
        uint32_t                    num_entries;
};

/* read ahead parameters */
struct csf_ra_parms {
        struct work_struct    work;        /* must be 1st field of struct */
        struct file          *in;          /* identifies file being read */
	struct address_space *mapping;     /* identifies address space of process reading file */
        unsigned int          nr_pages;    /* number of pages requested by read */
	pgoff_t               start_index; /* start index in file of preceding read request */
};

/* read ahead state info associated with an active file */
struct csf_ra_file_state {
        struct file     *file;                   /* identifies file, NULL => entry is invalid */
        struct timespec  last_access;            /* time last accessed */
        csf_ra_state_e   ra_state;               /* read ahead state */
        pgoff_t          expected_start_index;   /* used to determine if file is being accessed sequentially */
        uint32_t         seq_index_cnt;          /* count of consecutive sequential accesses, */
                                                 /* used in transitioning to SEQ state        */
        uint32_t         early_index_cnt;        /* count of consecutive accesses to beginning portion of file, */
                                                 /* used in transitioning back into INIT state                  */
	union {
        	uint32_t init_ignore_cnt;        /* number of initial reads ignored in INIT state */
        	pgoff_t  next_index;             /* index of next page to read in NON SEQ state */
	};
};

/* for tracking files most recently used by read ahead */
struct csf_ra_file_mru {
        uint32_t                 next_index;  /* index of array entry to allocate for new file */
        struct csf_ra_file_state state[CSF_RA_FILE_MRU_ARRAY_SIZE];
};

/* for tracking pages most recently consumed by application */
struct csf_page_mru {
        uint32_t              range_index;
	struct file          *file[CSF_PAGE_MRU_ARRAY_SIZE]; 
        struct csf_page_range range[CSF_PAGE_MRU_ARRAY_SIZE];
};

/* initialization flag */ 
bool csf_init_done = FALSE;

/* should probably dynamically allocate memory for the following list elements */
struct csf_page_list_element   csf_page_list_elements[CSF_NUM_PAGES];
struct csf_ra_list_element     csf_ra_list_elements[CSF_MAX_RA_PAGES];

/* head of linked list containing available csf_page_list_element structs */ 
struct csf_page_list_element *csf_page_list_elements_head;

/* head of linked list containing available pages */
struct csf_page_list_element *csf_page_list_head;

/* head of linked list containing available csf_ra_list_element structs */ 
struct csf_ra_list_element *csf_ra_list_elements_head;

/* linked list containing read ahead pages */
struct csf_ra_list csf_ra_list;

/* work queue for executing read ahead */
struct workqueue_struct *csf_wq;

/* parameters for read ahead */
struct csf_ra_parms csf_ra_parms;

/* for managing most recently used pages */
struct csf_page_mru csf_page_mru; 

/* for tracking files most recently used by read ahead */
struct csf_ra_file_mru csf_ra_file_mru;

/* controls whether debug is enabled */ 
/* read ahead semaphore */
struct semaphore csf_ra_sem;

/* lock for for protecting page list accesses */
spinlock_t csf_page_list_lock;

/* debug counters */
uint32_t csf_sync_io_cnt = 0;
uint32_t csf_ra_io_cnt = 0;
uint32_t csf_ra_calls = 0;
uint32_t csf_backward_discard_cnt = 0;
uint32_t csf_forward_discard_cnt = 0;
uint32_t csf_file_discard_cnt = 0;
uint32_t csf_find_and_remove_calls = 0;
uint32_t csf_find_and_remove_elements = 0;
uint32_t csf_find_calls = 0;
uint32_t csf_find_elements = 0;
uint32_t csf_sendfile_calls = 0;
uint32_t csf_sendfile_pages = 0;
uint32_t csf_perfect_ra = 0; 
uint32_t csf_stalls = 0; 

/* 
 * remove element from head of page list 
 *
 * returns ptr to allocated element or NULL if no element available 
 */
static inline
struct csf_page_list_element *csf_page_list_element_get(struct csf_page_list_element **head)
{
        struct csf_page_list_element *element;

        element = *head;
        if (unlikely(element == NULL)) {
                return NULL;
        }
        *head = element->next;
        return element;
}

/* 
 * insert element at head of page list 
 */
static inline
void csf_page_list_element_put(struct csf_page_list_element **head, 
                               struct csf_page_list_element *element)
{
        element->next = *head;
        *head = element;
        return;
}

/* 
 * allocate page list element 
 *
 * returns ptr to allocated element or NULL if no element available 
 */
static inline
struct csf_page_list_element *csf_page_list_element_alloc(void)
{
        return (csf_page_list_element_get(&csf_page_list_elements_head));
}

/* 
 * free page list element 
 */
static inline
void csf_page_list_element_free(struct csf_page_list_element *element)
{
        return (csf_page_list_element_put(&csf_page_list_elements_head, element));
}

/* 
 * allocate page 
 *
 * returns ptr to allocated page or NULL if no page available 
 */
static inline
struct page *csf_page_alloc(void)
{
        struct csf_page_list_element *element;
        struct page                  *page; 

        spin_lock_bh(&csf_page_list_lock);
        element = csf_page_list_element_get(&csf_page_list_head);
        if (unlikely(element == NULL)) {
        	spin_unlock(&csf_page_list_lock);
                return NULL;
        }
        page = element->page;
        csf_page_list_element_free(element);
        spin_unlock_bh(&csf_page_list_lock);
        return page;
}

/*
 * free page
 */
void csf_page_free(struct page *page)
{
        struct csf_page_list_element *element;

        if (likely(atomic_dec_and_test(&page->_count))) {
                spin_lock_bh(&csf_page_list_lock);
                atomic_set(&page->_count, 1);
                element = csf_page_list_element_alloc();
                element->page = page;
                csf_page_list_element_put(&csf_page_list_head, element);
                spin_unlock_bh(&csf_page_list_lock);
        } 
        return;
}

/* 
 * get kernel virtual address of page 
 *
 * returns kernel virtual address of page,
 * must be page managed by custom sendfile implementation
 */
void *csf_page_to_addr(struct page *page)
{
        return ((void *) page->csf_virtual);
}

/* 
 * remove read ahead element from head of list 
 *
 * returns ptr to element or NULL if list is empty
 */
static inline
struct csf_ra_list_element 
	*csf_ra_list_element_get(struct csf_ra_list_element **head)
{
        struct csf_ra_list_element *element;

        element = *head;
        if (unlikely(element == NULL)) {
                return NULL;
        }
        *head = element->next;
        return element;
}

/* 
 * insert read ahead element at tail of list 
 */
static inline
void csf_ra_list_element_put(struct csf_ra_list_element **head,
                             struct csf_ra_list_element *element)
{
        element->next = *head;
        *head = element;
        return;
}

/* 
 * populate element with pages and insert at tail of read ahead list 
 *
 * input:
 *   page_array - ptr to array containing addresses of pages to be inserted,
 *                array may contain NULL or address of page,
 *                entries containing NULL or skipped/ignored
 *   max_page_array_entries - max number of valid (i.e., non-NULL) entries 
 *                            in page_array
 *   start_index - file index associated with 0 index into page_array,
 *                 file index increases sequentially with page_array index 
 *   file - identifies file that pages are associated with 
 */
static inline
void csf_ra_list_insert(struct page **page_array, 
                        uint32_t      max_page_array_entries,
                        pgoff_t       start_index,
                        struct file  *file)
{
        uint32_t                    nr_pages; 
        struct csf_ra_list_element *element;
        struct page                *page;
	pgoff_t                     page_index;

        /* allocate and init the element */
        element = csf_ra_list_element_get(&csf_ra_list_elements_head);
        element->num_entries = 0;
        element->file = file;
        getrawmonotonic(&element->insert_time);

        /* insert pages into element */
        for (nr_pages = 0; nr_pages < max_page_array_entries; nr_pages++) {
                page = page_array[nr_pages];
                if (likely(page)) {
                        element->sublist[element->num_entries].page = page;
			page_index = start_index + nr_pages;
			element->sublist[element->num_entries].page_index = page_index; 
                        if (unlikely(!element->num_entries)) {
                        	element->page_range.start_index = 
                        		element->page_range.end_index = page_index;
                        } else {
                        	element->page_range.end_index = page_index;
                        }
			element->num_entries++;
                }
        }

	/* insert element into list */
        element->next = NULL;
        if (unlikely(csf_ra_list.head == NULL)) {
                csf_ra_list.head = csf_ra_list.tail = element;
                element->prev = NULL;
        } else {
                element->prev = csf_ra_list.tail;
                csf_ra_list.tail->next = element;
                csf_ra_list.tail = element;
        }
        csf_ra_list.num_entries += element->num_entries;
        return;
}

/* 
 * delete specified element from read ahead list 
 */
static inline
void csf_ra_list_delete(struct csf_ra_list_element *element)
{
       	if (likely(csf_ra_list.head == element)) {
       		if (unlikely(csf_ra_list.tail == element)) {
       			csf_ra_list.head = csf_ra_list.tail = NULL;
		} else {
       			csf_ra_list.head = element->next; 
       			element->next->prev = NULL; 
		}
	} else if (csf_ra_list.tail == element) {
       		csf_ra_list.tail = element->prev; 
               	element->prev->next = NULL;
	} else {
               	element->prev->next = element->next;
               	element->next->prev = element->prev;
	}
        csf_ra_list_element_put(&csf_ra_list_elements_head, element);
        return;
}

/* 
 * remove page at specified index of read ahead list element,
 *
 * page is freed if free_page is TRUE 
 */
static inline
void csf_ra_list_remove_page(struct csf_ra_list_element *element,
                             uint32_t                    sublist_index,
			     bool                        free_page)
{
        struct page *page;

        /* remove page from element sublist */
        page = element->sublist[sublist_index].page;
        element->sublist[sublist_index].page = NULL;
        element->num_entries--;
        csf_ra_list.num_entries--;

        /* if element is now empty, remove it from read ahead list, and insert in free list */
        if (unlikely(!element->num_entries)) {
		csf_ra_list_delete(element);
	}

        /* free removed page */
	if (unlikely(free_page)) {
		csf_page_free(page);
	}
 	return;
}

/* 
 * try to remove specified element from read ahead list,
 * if the element is removed all associated pages are freed, 
 * element can only be removed if all pages are up to date
 */
uint32_t csf_ra_list_try_remove(struct csf_ra_list_element *element)
{
        struct page *page;
	uint32_t     sublist_index, element_num_entries, removed_entries;

	/* free all the pages in the sublist that are up to date */
	for (sublist_index = 0, 
	     removed_entries = 0, 
	     element_num_entries = element->num_entries; 
             element_num_entries;	
	     sublist_index++) {
		page = element->sublist[sublist_index].page;
		if (likely(page)) {
                        if (likely(PageUptodate(page))) {
				csf_ra_list_remove_page(element, sublist_index, TRUE);
                        	removed_entries++; 
			}
			element_num_entries--;
		}
	}
        return removed_entries;
}

/*
 * remove specified element from read ahead list,
 * wait for I/O to complete if necessary
 */
void csf_ra_list_remove(struct csf_ra_list_element *element)
{
        struct page *page;
        uint32_t     sublist_index, element_num_entries;

        /* free all the pages in the sublist that are up to date */
        for (sublist_index = 0,
             element_num_entries = element->num_entries;
             element_num_entries;
             sublist_index++) {
                page = element->sublist[sublist_index].page;
                if (likely(page)) {
                        if (likely(PageUptodate(page))) {
                                csf_ra_list_remove_page(element, sublist_index, TRUE);
                        } else {
				lock_page(page);
                                csf_ra_list_remove_page(element, sublist_index, TRUE);
				unlock_page(page);
			}
                        element_num_entries--;
                }
        }
        return;
}

/* 
 * find pages in read ahead list whose indices are within specified range and remove 
 *
 * the addresses of pages that are found are returned in page_array,
 * page_array should be init with NULL entries prior to calling this function, 
 * the 0 index into page_array is associated with start_file_index,
 * the associated file index increases sequentially with the page_array index 
 *
 * this function is designed to be called when satifying a read request
 * (i.e., to get pages satisfying the request from the read ahead list)
 *
 * returns the number of pages found
 */ 
static inline
uint32_t csf_ra_list_find_and_remove(
	struct file  *file, 
        pgoff_t       start_file_index,
        pgoff_t       end_file_index,
        struct page **page_array)
{
	struct csf_ra_list_element *element, *next_element;
	struct page                *page;
	pgoff_t                     page_index; 
        uint32_t                    found_cnt, 
		                    target_cnt, 
				    sublist_index, 
				    valid_sublist_entries, 
				    element_num_entries;

        CSF_DEBUG_CNT_INC(csf_find_and_remove_calls);
	target_cnt = (end_file_index - start_file_index) + 1;
        found_cnt = 0;

	for (element = csf_ra_list.head;
             element != NULL; 
             element = next_element) {
		next_element = element->next;
        	CSF_DEBUG_CNT_INC(csf_find_and_remove_elements);
                if (likely((element->file == file) &&
                           !((start_file_index > element->page_range.end_index) ||
                             (end_file_index < element->page_range.start_index)))) {
			for (sublist_index = 0, 
			     valid_sublist_entries = 0, 
		  	     element_num_entries = element->num_entries;
			     valid_sublist_entries < element_num_entries;
			     sublist_index++) {
				page = element->sublist[sublist_index].page;
				if (likely(page)) {
					valid_sublist_entries++;
					page_index = element->sublist[sublist_index].page_index;
					if (likely((page_index >= start_file_index) &&
					           (page_index <= end_file_index))) {
						page_array[page_index - start_file_index] = page;
                                               	csf_ra_list_remove_page(element, 
					  	                        sublist_index, 
								        FALSE);
						found_cnt++;
						if (unlikely(found_cnt == target_cnt)) {
							return found_cnt;
                       				}
					}
				}
			}
                }
	}
	return found_cnt;
}

/* 
 * find pages with specified indices in read ahead list 
 *
 * the addresses of pages that are found are returned in page_array,
 * page_array should be init with NULL entries prior to calling this function, 
 * the 0 index into page_array is associated with start_file_index,
 * the associated file index increases sequentially with the page_array index 
 *
 * this function is designed to be called by the read ahead function 
 * to determine if pages in the planned read ahead i/o are already in
 * the read ahead list (i.e., no need to read pages that have already
 * been read)
 *
 * this function will also prune the read ahead list in a non-agressive
 * manner as it traverses the list, pruning means removing elements from
 * the list if their file indices are far from the current index window,
 * non-aggressive means that the allowed index distance is fairly large
 */ 
static inline
void csf_ra_list_find(struct file  *file, 
                      pgoff_t       start_file_index,
                      pgoff_t       end_file_index,
                      struct page **page_array)
{
	struct csf_ra_list_element *element, *next_element;
	struct page                *page;
	struct timespec             now;
	pgoff_t                     page_index; 
        uint32_t                    target_cnt, 
			            sublist_index, 
			            valid_sublist_entries, 
				    element_num_entries,
				    remove_cnt;

        CSF_DEBUG_CNT_INC(csf_find_calls);
        target_cnt = (end_file_index - start_file_index) + 1;

	for (element = csf_ra_list.head; element != NULL; element = next_element) {
		next_element = element->next;
        	CSF_DEBUG_CNT_INC(csf_find_elements);
		if (unlikely(element->file != file)) {
                        getrawmonotonic(&now);
                        if ((now.tv_sec - element->insert_time.tv_sec) > CSF_RA_AGE_OUT_SECS) {       
				remove_cnt = csf_ra_list_try_remove(element);
				CSF_DEBUG_CNT_ADD(csf_file_discard_cnt, remove_cnt); 
			}
		} else if (likely(!((start_file_index > element->page_range.end_index) || 
                                    (end_file_index < element->page_range.start_index)))) {
			for (sublist_index = 0, 
			     valid_sublist_entries = 0, 
		  	     element_num_entries = element->num_entries;
			     valid_sublist_entries < element_num_entries;
			     sublist_index++) {
				page = element->sublist[sublist_index].page;
				if (likely(page)) {
					valid_sublist_entries++;
					page_index = element->sublist[sublist_index].page_index;
					if (likely((page_index >= start_file_index) &&
					           (page_index <= end_file_index))) {
						page_array[page_index - start_file_index] = page;
						if (unlikely(!(--target_cnt))) {
							return;
                       				}
					}
				}
			}
		} else if (element->page_range.end_index < start_file_index) {
			if (unlikely(((start_file_index - element->page_range.end_index) >
			               CSF_RA_INDEX_WINDOW_BACK))) {
				remove_cnt = csf_ra_list_try_remove(element);
				CSF_DEBUG_CNT_ADD(csf_backward_discard_cnt, remove_cnt); 
                	}
		} else if (element->page_range.start_index > end_file_index) {
			if (unlikely(((element->page_range.start_index - end_file_index) >
			               CSF_RA_INDEX_WINDOW_FORWARD))) {
				remove_cnt = csf_ra_list_try_remove(element);
				CSF_DEBUG_CNT_ADD(csf_forward_discard_cnt, remove_cnt); 
                	}
		}
	}		
	return;
}

/* 
 * prune entries from read ahead list 
 *
 * this function prunes the read ahead list in an agressive
 * manner, pruning means removing elements from the list if 
 * their file indices are outside the current index window,
 * aggressive means that the allowed index distance is
 * realtively small 
 */
void csf_ra_list_prune(struct file *file, 
                       pgoff_t      start_file_index,
                       pgoff_t      end_file_index)
{
	struct csf_ra_list_element *element, *next_element;
	uint32_t                    remove_cnt;

	for (element = csf_ra_list.head; element != NULL; element = next_element) {
		next_element = element->next;
		if (unlikely(element->file != file)) {
			remove_cnt = csf_ra_list_try_remove(element);
			CSF_DEBUG_CNT_ADD(csf_file_discard_cnt, remove_cnt); 
		} else if (element->page_range.end_index < start_file_index) {
			if (unlikely(((start_file_index - element->page_range.end_index) >
			               CSF_PRUNE_RA_INDEX_WINDOW_BACK))) {
				remove_cnt = csf_ra_list_try_remove(element);
				CSF_DEBUG_CNT_ADD(csf_backward_discard_cnt, remove_cnt); 
                	}
		} else if (element->page_range.start_index > end_file_index) {
			if (unlikely(((element->page_range.start_index - end_file_index) >
			               CSF_PRUNE_RA_INDEX_WINDOW_FORWARD))) {
				remove_cnt = csf_ra_list_try_remove(element);
				CSF_DEBUG_CNT_ADD(csf_forward_discard_cnt, remove_cnt); 
                	}
		}
	}		
	return;
}

/* 
 * determine if page with specified index is in mru array 
 *
 * a most recently used (mru) array is utilized to track
 * the pages that were most recently read by the application
 * 
 * the intent is to not perform read ahead i/o for pages
 * that were recently consumed by the application
 */
static inline
bool csf_is_page_mru(struct file *file, pgoff_t index)
{
	uint32_t i;

 	for (i = 0; i < CSF_PAGE_MRU_ARRAY_SIZE; i++) {
		if (unlikely((file == csf_page_mru.file[i])               &&
			     (index >= csf_page_mru.range[i].start_index) && 
		             (index <= csf_page_mru.range[i].end_index))) {
			return TRUE;
		}
        }	
        return FALSE;
}

/* 
 * insert specified page index range in mru list 
 */
static inline
void csf_insert_page_mru(struct file *file, 
                         pgoff_t      start_index, 
                         pgoff_t      end_index)
{
        csf_page_mru.file[csf_page_mru.range_index] = file;
        csf_page_mru.range[csf_page_mru.range_index].start_index = start_index;
        csf_page_mru.range[csf_page_mru.range_index].end_index = end_index;
        if (unlikely(++csf_page_mru.range_index >= CSF_PAGE_MRU_ARRAY_SIZE)) {
		csf_page_mru.range_index = 0;
	} 
	return;
}

/* debug functions */
#if (CSF_DEBUG_ENABLE)

/* get number of free pages */
uint32_t csf_num_free_pages(void)
{
        struct csf_page_list_element *element;
	uint32_t                      free_page_cnt;

	for (element = csf_page_list_head, free_page_cnt = 0;
             element != NULL; 
             element = element->next) {
		free_page_cnt++;
	}
	return free_page_cnt;
}

/* get number of active pages being used for i/o or read ahead */
uint32_t csf_num_active_pages(void)
{
        struct csf_page_list_element *element;
	uint32_t                      active_page_cnt;

	for (element = csf_page_list_elements_head, active_page_cnt = 0;
             element != NULL; 
             element = element->next) {
		active_page_cnt++;
	}
	return active_page_cnt;
}

/* get number of elements on read ahead list */
uint32_t csf_num_ra_list_elements(void)
{
	struct csf_ra_list_element *element;
	uint32_t                    ra_entry_cnt;

	for (element = csf_ra_list.head, ra_entry_cnt = 0;
             element != NULL; 
             element = element->next) {
		ra_entry_cnt++;
	}
	return ra_entry_cnt;
}

/* get number of free read ahead list elements */
uint32_t csf_num_free_ra_list_elements(void)
{
        struct csf_ra_list_element *element;
        uint32_t                    free_cnt;

        for (element = csf_ra_list_elements_head, free_cnt = 0;
             element != NULL;
             element = element->next) {
                free_cnt++;
        }
        return free_cnt;
}

/* dump page allocation info */
void csf_dump_page_info(void)
{
	uint32_t free_page_cnt, active_page_cnt, ra_list_elem_cnt, ra_free_cnt;

        spin_lock_bh(&csf_page_list_lock);
        active_page_cnt = csf_num_active_pages();
        free_page_cnt = csf_num_free_pages();
        spin_unlock_bh(&csf_page_list_lock);
	ra_list_elem_cnt = csf_num_ra_list_elements();
	ra_free_cnt = csf_num_free_ra_list_elements();
        printk("tot pg = %u, active pg = %u, free pg = %u, i/o pg = %u, ra pg = %u, ra elem = %u, "
               "free ra = %u, tot ra = %u\n",
               free_page_cnt + active_page_cnt,
               active_page_cnt,
               free_page_cnt,
               active_page_cnt - csf_ra_list.num_entries,
               csf_ra_list.num_entries,
               ra_list_elem_cnt,
	       ra_free_cnt,
               ra_list_elem_cnt + ra_free_cnt);
	return;
}

/* dump debug stats */
void csf_dump_stats(void)
{
        if ((csf_sync_io_cnt + csf_ra_io_cnt) == 0) {
               	csf_ra_io_cnt++;
	}
        if (csf_find_and_remove_calls == 0) {
               	csf_find_and_remove_calls++;
	}
        if (csf_find_calls == 0) {
               	csf_find_calls++;
	}
        if (csf_sendfile_calls == 0) {
               	csf_sendfile_calls++;
	}
        if (csf_ra_calls == 0) {
               	csf_ra_calls++;
	}
        printk("back = %u, fwd = %u, file = %u, tot = %u, sync = %u, ra = %u, "
               "io = %u, dis pct = %u, sync pct = %u\n",
               csf_backward_discard_cnt,
               csf_forward_discard_cnt,
               csf_file_discard_cnt,
               csf_backward_discard_cnt + csf_forward_discard_cnt + csf_file_discard_cnt,
               csf_sync_io_cnt,
               csf_ra_io_cnt,
               csf_sync_io_cnt + csf_ra_io_cnt,
               ((csf_backward_discard_cnt + csf_forward_discard_cnt + csf_file_discard_cnt) * 100) /
               (csf_sync_io_cnt + csf_ra_io_cnt),
               (csf_sync_io_cnt * 100) / (csf_sync_io_cnt + csf_ra_io_cnt));
        printk("find and remove = %u, find = %u, sendfile calls = %u, "
               "sendfile pages = %u, avg pages = %u, pages/ra = %u\n",
               csf_find_and_remove_elements / csf_find_and_remove_calls,
               csf_find_elements / csf_find_calls,
               csf_sendfile_calls,
               csf_sendfile_pages,
               csf_sendfile_pages / csf_sendfile_calls,
	       csf_ra_io_cnt / csf_ra_calls);
	printk("perfect ra percent = %u, stalls = %u\n\n",
	       (csf_perfect_ra * 100) / csf_sendfile_calls,
	       csf_stalls);
        csf_backward_discard_cnt = csf_forward_discard_cnt = csf_file_discard_cnt =
        	csf_sync_io_cnt = csf_ra_io_cnt = csf_ra_calls = 
 		csf_find_and_remove_elements = csf_find_and_remove_calls = 
		csf_find_elements = csf_find_calls = csf_sendfile_calls = 
		csf_sendfile_pages = csf_perfect_ra = csf_stalls = 0;
	return;
}

/* print debug info */
void csf_debug(uint32_t             control_cnt, 
               struct csf_ra_parms *ra, 
               bool                 dump_page_info, 
               bool                 dump_stats,
               bool                 dump_trace)
{
        if (unlikely(!(control_cnt % CSF_DEBUG_CONTROL_CNT_INTERVAL))) {
		if (dump_page_info) {
			csf_dump_page_info();
		}
		if (dump_stats) {
			csf_dump_stats();
		}
		if (dump_trace) {
			printk("start index = %lu, end index = %lu, read size = %u\n",
                               ra->start_index,
                               ra->start_index + ra->nr_pages - 1,
                               ra->nr_pages);
		}
	}
	return;
}
#endif /* CSF_DEBUG_ENABLE */

/* 
 * core read ahead function 
 *
 * input:
 *   ra - ptr to struct containing read ahead parms,
 *        the following fields are used:
 *        	in      - ptr to file struct for input file that is being read 
 *        	mapping - ptr to address_space struct of process reading file
 *   start_index - file index at which read should start
 *   ra_nr_pages - number of pages to read
 */
static inline
void csf_ra_core(struct csf_ra_parms *ra,
	         pgoff_t              start_index,
		 uint32_t             ra_nr_pages)
{
	uint32_t          nr_pages, ra_io_nr_pages, free_cnt;
        struct list_head  readpages_list;
        struct blk_plug   plug;
        struct page      *page;
        struct page      *ra_pages[CSF_MAX_RA_SIZE];

        /* check whether any of desired pages are already in read ahead list */
        for (nr_pages = 0; nr_pages < ra_nr_pages; nr_pages++) {
                ra_pages[nr_pages] = NULL;
        }
        csf_ra_list_find(ra->in,
                         start_index,
                         start_index + ra_nr_pages - 1,
                         ra_pages);

        /*
         * allocate pages for read ahead i/o, and
         * insert pages in list required by i/o api
         */
        INIT_LIST_HEAD(&readpages_list);
        for (nr_pages = 0, ra_io_nr_pages = 0;
             nr_pages < ra_nr_pages;
             nr_pages++) {
                if (unlikely(ra_pages[nr_pages])) {
                        ra_pages[nr_pages] = NULL;
                } else if (likely(!csf_is_page_mru(ra->in,
                                                   start_index + nr_pages))) {
                        page = csf_page_alloc();
                        if (unlikely(!page)) {
                                CSF_DEBUG_MSG("csf: read ahead: page allocation failed\n");
                                goto clean_up;
                        }
                        ra_io_nr_pages++;
                        page->mapping = ra->mapping;
                        page->index = start_index + nr_pages;
                        ra_pages[nr_pages] = page;
                        list_add_tail(&page->lru, &readpages_list);
                        lock_page(page);
                }
        }

        if (likely(ra_io_nr_pages)) {
                /* start the read ahead i/o */
                blk_start_plug(&plug);
                if (unlikely(ra->mapping->a_ops->readpages(ra->in,
                                                           ra->mapping,
                                                           &readpages_list,
                                                           ra_io_nr_pages))) {
                        CSF_DEBUG_MSG("csf: read ahead I/O error\n");
                        blk_finish_plug(&plug);
                        goto clean_up;
                }
                blk_finish_plug(&plug);
                CSF_DEBUG_CNT_ADD(csf_ra_io_cnt, ra_io_nr_pages);

                /* insert entry into read ahead list for pages that i/o was started on */
                csf_ra_list_insert(ra_pages, ra_nr_pages, start_index, ra->in);
	}

	/* successful completion */
	return;

clean_up:
        /* read ahead failed, free allocated pages and return */
        for (free_cnt = 0; free_cnt < nr_pages; free_cnt++) {
                page = ra_pages[free_cnt];
                if (page) {
                        if (PageLocked(page)) {
                                unlock_page(page);
                        }
                        csf_page_free(page);
                }
        }
	return;
}

/*
 * get ptr to read ahead state struct for specified file
 *
 * if no struct is found for file, then allocate/init one 
 */
static inline
struct csf_ra_file_state *csf_get_ra_state(struct file *file) 
{
	uint32_t        i;
	struct timespec now;

	/* search for entry associated with file */
	for (i = 0; i < CSF_RA_FILE_MRU_ARRAY_SIZE; i++) {
		if (likely(csf_ra_file_mru.state[i].file == file)) {
                        getrawmonotonic(&now);
                        if (likely((now.tv_sec - csf_ra_file_mru.state[i].last_access.tv_sec) <= 
                                   CSF_RA_FILE_MRU_AGE_OUT_SECS)) {       
				/* found the entry */
				goto return_entry;
			} else {
				/* age entry out */
				goto init_entry;
			}
		}
        }
	/* no entry found, so allocate one */ 
        i = csf_ra_file_mru.next_index;
        if (unlikely(++csf_ra_file_mru.next_index >= CSF_RA_FILE_MRU_ARRAY_SIZE)) {
        	csf_ra_file_mru.next_index = 0; 
	}
        getrawmonotonic(&now);
	
	/* initialize new entry for file */
init_entry:
	csf_ra_file_mru.state[i].file = file;
	csf_ra_file_mru.state[i].expected_start_index = 0;
	csf_ra_file_mru.state[i].seq_index_cnt = 0;
	csf_ra_file_mru.state[i].init_ignore_cnt = 0;
	csf_ra_file_mru.state[i].ra_state = CSF_RA_INIT_STATE;

	/* return ptr to entry */
return_entry:
        csf_ra_file_mru.state[i].last_access = now;
	return &csf_ra_file_mru.state[i];
}

/*
 * implement read ahead state machine
 *
 * input:
 *   ra - ptr to struct containing read ahead parms,
 *        the following fields are used:
 *        	start_index
 *        	nr_pages
 *  state - ptr to read ahead state struct for file being read
 *  end_file_index - index of last page in file being read
 */
static inline
void csf_ra_state_machine(struct csf_ra_parms      *ra, 
                          struct csf_ra_file_state *state,
                          pgoff_t                   end_file_index)
{
	switch (state->ra_state) {
		case CSF_RA_INIT_STATE:
			if (state->init_ignore_cnt < CSF_RA_INIT_STATE_IGNORE_CNT) {
				state->init_ignore_cnt++;
	                        state->expected_start_index = ra->start_index + ra->nr_pages;
			} else if (ra->start_index == state->expected_start_index) {
		 		if (++state->seq_index_cnt > CSF_RA_SEQ_INDEX_THRESHOLD) { 
					state->ra_state = CSF_RA_SEQ_STATE;
				} else {
					state->expected_start_index += ra->nr_pages;
				}
			} else if (end_file_index >= CSF_RA_NON_SEQ_MIN_FILE_PAGES) { 
                        	state->next_index = CSF_RA_NON_SEQ_DELTA;
				state->ra_state = CSF_RA_NON_SEQ_STATE;
			} else {
				state->ra_state = CSF_RA_SEQ_STATE;
			}
			break;

		case CSF_RA_SEQ_STATE:
			if (unlikely(ra->start_index > CSF_RA_EARLY_INDEX_THRESHOLD)) {
				state->early_index_cnt = 0;
                		state->ra_state = CSF_RA_SEQ_STEADY_STATE;
			}
			break;

		case CSF_RA_NON_SEQ_STATE:
			if (unlikely(ra->start_index > CSF_RA_EARLY_INDEX_THRESHOLD)) {
				state->early_index_cnt = 0;
                		state->ra_state = CSF_RA_NON_SEQ_STEADY_STATE;
			}
			break;

		case CSF_RA_SEQ_STEADY_STATE:
		case CSF_RA_NON_SEQ_STEADY_STATE:
			if (likely(ra->start_index >= CSF_RA_EARLY_INDEX_THRESHOLD)) {
				state->early_index_cnt = 0;
			} else if (++state->early_index_cnt > CSF_RA_EARLY_INDEX_HYSTERESIS) { 
				state->init_ignore_cnt = 0;
	                	state->seq_index_cnt = 0;
	                	state->expected_start_index = ra->start_index + ra->nr_pages;
				state->ra_state = CSF_RA_INIT_STATE;
			}
			break;
	}
	return; 
}

/* 
 * perform read ahead 
 *
 * populate read ahead list predictively based on assumption
 * that file accesses are localized 
 */
void csf_read_ahead(struct work_struct *work)
{
        struct csf_ra_parms      *ra = (struct csf_ra_parms *) work;
        struct csf_ra_file_state *state; 
	uint32_t                  ra_nr_pages, ra_max_nr_pages;
	pgoff_t                   start_index, end_index;
	loff_t                    isize;

	CSF_DEBUG_CNT_INC(csf_ra_calls);

        CSF_DEBUG(csf_ra_calls, ra, FALSE, TRUE, FALSE);

        /* get ptr to state info for file being read */ 
        state = csf_get_ra_state(ra->in); 

        /* get end of file index */
        isize = i_size_read(ra->mapping->host);
        if (unlikely(!isize)) {
		/* can't read an empty file */
                goto done;
        }
        end_index = (isize - 1) >> PAGE_CACHE_SHIFT;

	/* implement read ahead state machine */
	csf_ra_state_machine(ra, state, end_index);

	/* set read ahead parameters based on current state */
	start_index = state->next_index; /* to remove "may be uninitialized" compiler warning */ 
	ra_nr_pages = ra->nr_pages;      /* to remove "may be uninitialized" compiler warning */ 
	switch (state->ra_state) {
		case CSF_RA_INIT_STATE: 
		case CSF_RA_SEQ_STATE: 
		case CSF_RA_SEQ_STEADY_STATE:
	        	start_index = ra->start_index + ra->nr_pages;
			ra_nr_pages = CSF_MAX_RA_SIZE;
			break;

		case CSF_RA_NON_SEQ_STATE:
		case CSF_RA_NON_SEQ_STEADY_STATE:
	                state->next_index += ra_nr_pages; 
			break;
	}

        /* don't read past end of file */
        if (unlikely(end_index < (start_index + ra_nr_pages - 1))) {
                if (unlikely(end_index < start_index)) {
                        goto done;
                }
                ra_nr_pages = (end_index - start_index) + 1;
        }
	ra_max_nr_pages = ra_nr_pages;

	/* limit the size of read ahead list to max capacity */
        ra_nr_pages = min(ra_max_nr_pages,
        	CSF_MAX_RA_PAGES - csf_ra_list.num_entries);

	/* aggressively prune if read ahead list is at max capacity */ 
        if (unlikely(!ra_nr_pages)) {
		CSF_DEBUG_MSG("csf: read ahead: out of pages: pruning\n");
        	csf_ra_list_prune(ra->in,
               	                  start_index,
                                  start_index + CSF_MAX_RA_SIZE - 1);
        	ra_nr_pages = min(ra_max_nr_pages,
                	CSF_MAX_RA_PAGES - csf_ra_list.num_entries);
        	if (!ra_nr_pages) {
			CSF_DEBUG_MSG("csf: read ahead: out of pages: pruning failed\n");
			goto done;
        	}
        }

        /* do the read ahead */
        csf_ra_core(ra, start_index, ra_nr_pages);

done:
 	/* 
         * update the most recently used data structure,
         * which tracks pages most recently consumed (i.e., read) by application
         */ 
        csf_insert_page_mru(ra->in,
                            ra->start_index, 
                            ra->start_index + ra->nr_pages - 1);
	/* 
         * indicate completion of read ahead function and return,
         * execution of next sendfile call is blocked until read 
         * ahead function has completed
	 */ 
        if (likely(num_online_cpus() != 1)) {
        	up(&csf_ra_sem);
	}
        return;
}

/* 
 * initialization function 
 *   - allocates private pool of page buffers and
 *     creates non-cached mappings in kernel virtual address
 *     space for each page
 *   - initializes data structures (e.g., lists, semaphores, locks, 
 *     work queue, etc.)
 *
 * returns 0 on success, non-zero if error 
 */
int __init csf_init(void)
{
        int          i;
        struct page *page;
        void        *vaddr;
        spinlock_t  *ptl;
        pte_t       *ptep, pte;

        csf_page_list_elements_head = NULL;
        csf_page_list_head = NULL;
        csf_ra_list_elements_head = NULL;
        csf_ra_list.head = NULL;
        csf_ra_list.tail = NULL;
        csf_ra_list.num_entries = 0;

	csf_page_mru.range_index = 0;
        for (i = 0; i < CSF_PAGE_MRU_ARRAY_SIZE; i++) {
        	csf_page_mru.file[i] = NULL;
        	csf_page_mru.range[i].start_index = CSF_MAX_PAGE_INDEX;
        	csf_page_mru.range[i].end_index = CSF_MAX_PAGE_INDEX;
        }

	csf_ra_file_mru.next_index = 0;
	for (i = 0; i < CSF_RA_FILE_MRU_ARRAY_SIZE; i++) {
		csf_ra_file_mru.state[i].file = NULL;
	}

        csf_wq = create_workqueue("cfs_queue");
        if (unlikely(!csf_wq)) {
        	printk("cfs: init failed: create_workqueue failed\n");
                goto csf_init_err;
        }
        INIT_WORK((struct work_struct *) &csf_ra_parms, csf_read_ahead);

        sema_init(&csf_ra_sem, 1);
        spin_lock_init(&csf_page_list_lock);
 
        for (i = 0; i < CSF_MAX_RA_PAGES; i++) {
                csf_ra_list_element_put(&csf_ra_list_elements_head,
                                        &csf_ra_list_elements[i]);
        }

        for (i = 0; i < CSF_NUM_PAGES; i++) {
                csf_page_list_element_free(&csf_page_list_elements[i]);
        }

        for (i = 0; i < CSF_NUM_PAGES; i++) {
                page = alloc_page(GFP_KERNEL);
                if (unlikely(!page)) {
                        printk("csf: init failed: alloc_page failed\n");
                        goto csf_init_err;
                }

                vaddr = vmap(&page, 
                             1, 
                             VM_IOREMAP,
                             __pgprot_modify(PAGE_KERNEL, L_PTE_MT_MASK, L_PTE_MT_UNCACHED));

                if (unlikely(!vaddr)) {
                        printk("csf: init failed: vmap failed\n");
                        goto csf_init_err;
                }

                if (unlikely(public_follow_pte(&init_mm, (uint32_t) vaddr, &ptep, &ptl))) {
                        printk("csf: init failed: public_follow_pte failed\n");
                        goto csf_init_err;
                } else {
                        pte = *ptep;
                        if ((pte_val(pte) & L_PTE_MT_UNCACHED) != L_PTE_MT_UNCACHED) {
                                printk("csf: init failed: uncached mapping failed\n");
                                goto csf_init_err;
                        }
                        __sync_icache_dcache(pte);
                        pte_unmap_unlock(pte, ptl);
                }

                page = vmalloc_to_page(vaddr);
                page->csf_magic = CSF_MAGIC;
                page->csf_virtual = (uint32_t) vaddr;
                csf_page_free(page);
        }

        csf_init_done = TRUE;
        return 0;

csf_init_err:
        if (!csf_wq) {
		flush_workqueue(csf_wq);
		destroy_workqueue(csf_wq);
        }
        while ((page = csf_page_alloc())) { 
	        __free_page(page);
        }
        return 1;
}
late_initcall(csf_init);

/*
 * delete csf data structures with references to specified file
 *
 * called when file is closed
 */
void csf_flush_file(struct file *filp)
{
        struct csf_ra_list_element *element, *next_element;
	uint32_t                    i;

        if (likely(num_online_cpus() != 1)) {
		/* acquire semaphore for exclusive access to read ahead data structures */
        	down(&csf_ra_sem);
	}

	/* delete entries in read ahead list that are associated with file */
        for (element = csf_ra_list.head; element != NULL; element = next_element) {
                next_element = element->next;
                if (unlikely(element->file == filp)) {
                        csf_ra_list_remove(element);
                } 
        }

	/* delete entry associated with file in read ahead file mru data structure */
        for (i = 0; i < CSF_RA_FILE_MRU_ARRAY_SIZE; i++) {
                if (unlikely(csf_ra_file_mru.state[i].file == filp)) {
                	csf_ra_file_mru.state[i].file = NULL;
			break;
                }
        }

        if (likely(num_online_cpus() != 1)) {
		/* release read ahead semaphore */
        	up(&csf_ra_sem);
        }
	return;
}

/* 
 * custom pipe buffer operation functions 
 */
void *csf_pipe_buf_map(struct pipe_inode_info *pipe,
                       struct pipe_buffer     *buf, 
                       int                     atomic)
{
        return csf_page_to_addr(buf->page);
}

void csf_pipe_buf_unmap(struct pipe_inode_info *pipe,
                        struct pipe_buffer     *buf, 
                        void                   *map_data)
{
        /* no unmapping required */
        return;
}

int csf_pipe_buf_steal(struct pipe_inode_info *pipe,
                       struct pipe_buffer     *buf)
{
        /* don't allow steal */
        return 1;
}

void csf_pipe_buf_release(struct pipe_inode_info *pipe,
                          struct pipe_buffer     *buf)
{
        csf_page_free(buf->page);
        return;
}

/* 
 * define pipe buf operations 
 */
static const struct pipe_buf_operations csf_pipe_buf_ops = {
        .can_merge = 0,
        .map = csf_pipe_buf_map,
        .unmap = csf_pipe_buf_unmap,
        .confirm = page_cache_pipe_buf_confirm,
        .release = csf_pipe_buf_release,
        .steal = csf_pipe_buf_steal,
        .get = generic_pipe_buf_get,
};

/* 
 * splice pipe descriptor operation functions 
 */
void csf_spd_release_page(struct splice_pipe_desc *spd, unsigned int i)
{
        csf_page_free(spd->pages[i]);
        return;
}

/* 
 * custom implementation of generic file splice read
 *   - utilizes read ahead list 
 *   - works in conjunction with read ahead function
 */
static int
__generic_file_splice_read(struct file            *in, 
                           loff_t                 *ppos,
			   struct pipe_inode_info *pipe, 
                           size_t                  len,
			   unsigned int            flags)
{
	struct address_space *mapping = in->f_mapping;
	unsigned int          loff, nr_pages, io_nr_pages, 
                              found_pages, plen, this_len;
        int                   ra_cpu_num;
	struct page          *page, *pages[PIPE_DEF_BUFFERS];
	struct partial_page   partial[PIPE_DEF_BUFFERS];
	pgoff_t               start_read_index, end_read_index, end_file_index;
	loff_t                isize;
        struct list_head      readpages_list;
        struct blk_plug       plug;
	struct splice_pipe_desc spd = {
		.pages = pages,
		.partial = partial,
		.nr_pages_max = PIPE_DEF_BUFFERS,
		.flags = flags,
		.ops = &csf_pipe_buf_ops,
		.spd_release = csf_spd_release_page,
	};

        /* make sure init was successful */ 
        if (unlikely(!csf_init_done)) {
        	return -ENOMEM;
        }

        barrier();

	/* do what we can before waiting for read ahead function completion */
        start_read_index = *ppos >> PAGE_CACHE_SHIFT;
        loff = *ppos & ~PAGE_CACHE_MASK;
        nr_pages = (len + loff + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;
        nr_pages = min(nr_pages, spd.nr_pages_max);
        end_read_index = start_read_index + nr_pages - 1;

        /* don't read past end of file */
        isize = i_size_read(mapping->host);
        if (unlikely(!isize)) {
                return 0;
        }
        end_file_index = (isize - 1) >> PAGE_CACHE_SHIFT;
        if (unlikely(end_file_index < end_read_index)) {
                if (unlikely(end_file_index < start_read_index)) {
                        return 0;
                }
                nr_pages = (end_file_index - start_read_index) + 1;
        	end_read_index = start_read_index + nr_pages - 1;
        }
        plen = ((isize - 1) & ~PAGE_CACHE_MASK) + 1;
	if (unlikely(nr_pages <= 1)) {
                if (unlikely((!nr_pages) || (plen <= loff))) {
			return 0;
                }
        }

       	CSF_DEBUG_CNT_INC(csf_sendfile_calls);
       	CSF_DEBUG_CNT_ADD(csf_sendfile_pages, nr_pages);

        for (spd.nr_pages = 0; spd.nr_pages < nr_pages; spd.nr_pages++) {
                spd.pages[spd.nr_pages] = NULL;
        }

        barrier();

	/* 
         * wait for previous read ahead function execution to complete,
	 * don't access read ahead list before this point
         */
#if (CSF_DEBUG_ENABLE)
        if (unlikely(down_trylock(&csf_ra_sem))) {
       		CSF_DEBUG_CNT_INC(csf_stalls);
                if (likely(num_online_cpus() != 1)) {
                	down(&csf_ra_sem);
        	}
        }
#else
        if (likely(num_online_cpus() != 1)) {
        	down(&csf_ra_sem);
        }
#endif

        barrier();

	/* check whether requested pages are in read ahead list */
        found_pages = csf_ra_list_find_and_remove(in, 
                                                  start_read_index,
                                                  end_read_index,
                                                  spd.pages);
 
        barrier();

	/* kickoff read ahead, don't access read ahead list after this */
        if (likely(nr_pages >= CSF_MIN_PAGES_TO_DO_RA)) {
                csf_ra_parms.in = in;
                csf_ra_parms.mapping = mapping;
                csf_ra_parms.nr_pages = nr_pages;
                csf_ra_parms.start_index = start_read_index;
                if (unlikely(num_online_cpus() == 1)) {
                        ra_cpu_num = raw_smp_processor_id();
                        csf_read_ahead((struct work_struct *) &csf_ra_parms);
                } else {
                        ra_cpu_num = raw_smp_processor_id() ? 0: 1;
                	queue_work_on(ra_cpu_num,
                       		      csf_wq,
                                      (struct work_struct *) &csf_ra_parms);
                }
        } else if (likely(num_online_cpus() != 1)) {
               	up(&csf_ra_sem);
        }

        barrier();

        /* expedite processing if all requested pages were in read ahead list */
        if (likely(found_pages == nr_pages)) {
       		CSF_DEBUG_CNT_INC(csf_perfect_ra);
	} else {
        	/*
         	 * allocate pages for i/o, and
         	 * insert pages in list required by i/o api
                 */
        	INIT_LIST_HEAD(&readpages_list);
		for (spd.nr_pages = 0, io_nr_pages = 0; 
                     spd.nr_pages < nr_pages; 
                     spd.nr_pages++) {
			if (unlikely(!spd.pages[spd.nr_pages])) {
                		page = csf_page_alloc();
				if (unlikely(!page)) {
                       			CSF_DEBUG_MSG("csf: sendfile failed: out of pages\n");
               	       			goto clean_up;
				}
                        	io_nr_pages++;
                		page->mapping = mapping;
                		page->index = start_read_index + spd.nr_pages;
				spd.pages[spd.nr_pages] = page;
                        	list_add_tail(&page->lru, &readpages_list);
                        	lock_page(page);
			}
		}

        	/* initiate i/o for needed pages */
        	if (likely(io_nr_pages)) {
                	/* start the read ahead i/o */
                        blk_start_plug(&plug);
                	if (unlikely(mapping->a_ops->readpages(in,
                                                               mapping,
                                                               &readpages_list,
                                                               io_nr_pages))) {
                       		CSF_DEBUG_MSG("csf: read I/O error\n");
                        	blk_finish_plug(&plug);
                       		goto clean_up;
        		}
                        blk_finish_plug(&plug);
                	CSF_DEBUG_CNT_ADD(csf_sync_io_cnt, io_nr_pages);
        	}
        }

        /* prepare for splice */
        if (unlikely(nr_pages == 1)) {
        	spd.nr_pages = 1;
                spd.partial[0].offset = loff;
                this_len = min_t(unsigned long, len, PAGE_CACHE_SIZE - loff);
                if (unlikely(end_read_index == end_file_index)) {
                        spd.partial[0].len =  min(this_len, plen);
                } else {
                        spd.partial[0].len = this_len;
                }
        } else {
                spd.partial[0].offset = loff;
                spd.partial[0].len = PAGE_CACHE_SIZE - loff;
                len -= (PAGE_CACHE_SIZE - loff);
                for (spd.nr_pages = 1;
                     spd.nr_pages < nr_pages - 1;
                     spd.nr_pages++) {
                	spd.partial[spd.nr_pages].offset = 0;
                     	spd.partial[spd.nr_pages].len = PAGE_CACHE_SIZE;
                        len -= PAGE_CACHE_SIZE;
                }
                spd.partial[spd.nr_pages].offset = 0;
                this_len = min_t(unsigned long, len, PAGE_CACHE_SIZE);
                if (unlikely(end_read_index == end_file_index)) {
                        spd.partial[spd.nr_pages].len = min(this_len, plen);
                } else {
                        spd.partial[spd.nr_pages].len = this_len;
                }
                spd.nr_pages++;
	}

        /* do splice and return */
 	return splice_to_pipe(pipe, &spd);

clean_up:
	/* take care of pages that need to be freed */
        for (spd.nr_pages = 0; spd.nr_pages < nr_pages; spd.nr_pages++) {
		page = spd.pages[spd.nr_pages];
       		if (page) {
			if (PageLocked(page)) {
		        	unlock_page(page);
			}	
                        csf_page_free(page);
                }
        }
        return -ENOMEM;
}

#else /* CONFIG_BROADCOM_CUSTOM_SENDFILE */

/* original kernel implementation */
 
static int
__generic_file_splice_read(struct file *in, loff_t *ppos,
			   struct pipe_inode_info *pipe, size_t len,
			   unsigned int flags)
{
	struct address_space *mapping = in->f_mapping;
	unsigned int loff, nr_pages, req_pages;
	struct page *pages[PIPE_DEF_BUFFERS];
	struct partial_page partial[PIPE_DEF_BUFFERS];
	struct page *page;
	pgoff_t index, end_index;
	loff_t isize;
	int error, page_nr;
	struct splice_pipe_desc spd = {
		.pages = pages,
		.partial = partial,
		.nr_pages_max = PIPE_DEF_BUFFERS,
		.flags = flags,
		.ops = &page_cache_pipe_buf_ops,
		.spd_release = spd_release_page,
	};

	if (splice_grow_spd(pipe, &spd))
		return -ENOMEM;

	index = *ppos >> PAGE_CACHE_SHIFT;
	loff = *ppos & ~PAGE_CACHE_MASK;
	req_pages = (len + loff + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;
	nr_pages = min(req_pages, spd.nr_pages_max);

	/*
	 * Lookup the (hopefully) full range of pages we need.
	 */
	spd.nr_pages = find_get_pages_contig(mapping, index, nr_pages, spd.pages);
	index += spd.nr_pages;

	/*
	 * If find_get_pages_contig() returned fewer pages than we needed,
	 * readahead/allocate the rest and fill in the holes.
	 */
	if (spd.nr_pages < nr_pages)
		page_cache_sync_readahead(mapping, &in->f_ra, in,
				index, req_pages - spd.nr_pages);

	error = 0;
	while (spd.nr_pages < nr_pages) {
		/*
		 * Page could be there, find_get_pages_contig() breaks on
		 * the first hole.
		 */
		page = find_get_page(mapping, index);
		if (!page) {
			/*
			 * page didn't exist, allocate one.
			 */
			page = page_cache_alloc_cold(mapping);
			if (!page)
				break;

			error = add_to_page_cache_lru(page, mapping, index,
						GFP_KERNEL);
			if (unlikely(error)) {
				page_cache_release(page);
				if (error == -EEXIST)
					continue;
				break;
			}
			/*
			 * add_to_page_cache() locks the page, unlock it
			 * to avoid convoluting the logic below even more.
			 */
			unlock_page(page);
		}

		spd.pages[spd.nr_pages++] = page;
		index++;
	}

	/*
	 * Now loop over the map and see if we need to start IO on any
	 * pages, fill in the partial map, etc.
	 */
	index = *ppos >> PAGE_CACHE_SHIFT;
	nr_pages = spd.nr_pages;
	spd.nr_pages = 0;
	for (page_nr = 0; page_nr < nr_pages; page_nr++) {
		unsigned int this_len;

		if (!len)
			break;

		/*
		 * this_len is the max we'll use from this page
		 */
		this_len = min_t(unsigned long, len, PAGE_CACHE_SIZE - loff);
		page = spd.pages[page_nr];

		if (PageReadahead(page))
			page_cache_async_readahead(mapping, &in->f_ra, in,
					page, index, req_pages - page_nr);

		/*
		 * If the page isn't uptodate, we may need to start io on it
		 */
		if (!PageUptodate(page)) {
			lock_page(page);

			/*
			 * Page was truncated, or invalidated by the
			 * filesystem.  Redo the find/create, but this time the
			 * page is kept locked, so there's no chance of another
			 * race with truncate/invalidate.
			 */
			if (!page->mapping) {
				unlock_page(page);
				page = find_or_create_page(mapping, index,
						mapping_gfp_mask(mapping));

				if (!page) {
					error = -ENOMEM;
					break;
				}
				page_cache_release(spd.pages[page_nr]);
				spd.pages[page_nr] = page;
			}
			/*
			 * page was already under io and is now done, great
			 */
			if (PageUptodate(page)) {
				unlock_page(page);
				goto fill_it;
			}

			/*
			 * need to read in the page
			 */
			error = mapping->a_ops->readpage(in, page);
			if (unlikely(error)) {
				/*
				 * We really should re-lookup the page here,
				 * but it complicates things a lot. Instead
				 * lets just do what we already stored, and
				 * we'll get it the next time we are called.
				 */
				if (error == AOP_TRUNCATED_PAGE)
					error = 0;

				break;
			}
		}
fill_it:
		/*
		 * i_size must be checked after PageUptodate.
		 */
		isize = i_size_read(mapping->host);
		end_index = (isize - 1) >> PAGE_CACHE_SHIFT;
		if (unlikely(!isize || index > end_index))
			break;

		/*
		 * if this is the last page, see if we need to shrink
		 * the length and stop
		 */
		if (end_index == index) {
			unsigned int plen;

			/*
			 * max good bytes in this page
			 */
			plen = ((isize - 1) & ~PAGE_CACHE_MASK) + 1;
			if (plen <= loff)
				break;

			/*
			 * force quit after adding this page
			 */
			this_len = min(this_len, plen - loff);
			len = this_len;
		}

		spd.partial[page_nr].offset = loff;
		spd.partial[page_nr].len = this_len;
		len -= this_len;
		loff = 0;
		spd.nr_pages++;
		index++;
	}

	/*
	 * Release any pages at the end, if we quit early. 'page_nr' is how far
	 * we got, 'nr_pages' is how many pages are in the map.
	 */
	while (page_nr < nr_pages)
		page_cache_release(spd.pages[page_nr++]);
	in->f_ra.prev_pos = (loff_t)index << PAGE_CACHE_SHIFT;

	if (spd.nr_pages)
		error = splice_to_pipe(pipe, &spd);

	splice_shrink_spd(&spd);
	return error;
}
#endif /* CONFIG_BROADCOM_CUSTOM_SENDFILE */

/**
 * generic_file_splice_read - splice data from file to a pipe
 * @in:		file to splice from
 * @ppos:	position in @in
 * @pipe:	pipe to splice to
 * @len:	number of bytes to splice
 * @flags:	splice modifier flags
 *
 * Description:
 *    Will read pages from given file and fill them into a pipe. Can be
 *    used as long as the address_space operations for the source implements
 *    a readpage() hook.
 *
 */
ssize_t generic_file_splice_read(struct file *in, loff_t *ppos,
				 struct pipe_inode_info *pipe, size_t len,
				 unsigned int flags)
{
	loff_t isize, left;
	int ret;

	isize = i_size_read(in->f_mapping->host);
	if (unlikely(*ppos >= isize))
		return 0;

	left = isize - *ppos;
	if (unlikely(left < len))
		len = left;

	ret = __generic_file_splice_read(in, ppos, pipe, len, flags);
	if (ret > 0) {
		*ppos += ret;
		file_accessed(in);
	}

	return ret;
}
EXPORT_SYMBOL(generic_file_splice_read);

static const struct pipe_buf_operations default_pipe_buf_ops = {
	.can_merge = 0,
	.map = generic_pipe_buf_map,
	.unmap = generic_pipe_buf_unmap,
	.confirm = generic_pipe_buf_confirm,
	.release = generic_pipe_buf_release,
	.steal = generic_pipe_buf_steal,
	.get = generic_pipe_buf_get,
};

ssize_t default_file_splice_read(struct file *in, loff_t *ppos,
				 struct pipe_inode_info *pipe, size_t len,
				 unsigned int flags)
{
	unsigned int nr_pages;
	unsigned int nr_freed;
	size_t offset;
	struct page *pages[PIPE_DEF_BUFFERS];
	struct partial_page partial[PIPE_DEF_BUFFERS];
	struct iovec *vec, __vec[PIPE_DEF_BUFFERS];
	ssize_t res;
	size_t this_len;
	int error;
	int i;
	struct splice_pipe_desc spd = {
		.pages = pages,
		.partial = partial,
		.nr_pages_max = PIPE_DEF_BUFFERS,
		.flags = flags,
		.ops = &default_pipe_buf_ops,
		.spd_release = spd_release_page,
	};

	if (splice_grow_spd(pipe, &spd))
		return -ENOMEM;

	res = -ENOMEM;
	vec = __vec;
	if (spd.nr_pages_max > PIPE_DEF_BUFFERS) {
		vec = kmalloc(spd.nr_pages_max * sizeof(struct iovec), GFP_KERNEL);
		if (!vec)
			goto shrink_ret;
	}

	offset = *ppos & ~PAGE_CACHE_MASK;
	nr_pages = (len + offset + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;

	for (i = 0; i < nr_pages && i < spd.nr_pages_max && len; i++) {
		struct page *page;

		page = alloc_page(GFP_USER);
		error = -ENOMEM;
		if (!page)
			goto err;

		this_len = min_t(size_t, len, PAGE_CACHE_SIZE - offset);
		vec[i].iov_base = (void __user *) page_address(page);
		vec[i].iov_len = this_len;
		spd.pages[i] = page;
		spd.nr_pages++;
		len -= this_len;
		offset = 0;
	}

	res = kernel_readv(in, vec, spd.nr_pages, *ppos);
	if (res < 0) {
		error = res;
		goto err;
	}

	error = 0;
	if (!res)
		goto err;

	nr_freed = 0;
	for (i = 0; i < spd.nr_pages; i++) {
		this_len = min_t(size_t, vec[i].iov_len, res);
		spd.partial[i].offset = 0;
		spd.partial[i].len = this_len;
		if (!this_len) {
			__free_page(spd.pages[i]);
			spd.pages[i] = NULL;
			nr_freed++;
		}
		res -= this_len;
	}
	spd.nr_pages -= nr_freed;

	res = splice_to_pipe(pipe, &spd);
	if (res > 0)
		*ppos += res;

shrink_ret:
	if (vec != __vec)
		kfree(vec);
	splice_shrink_spd(&spd);
	return res;

err:
	for (i = 0; i < spd.nr_pages; i++)
		__free_page(spd.pages[i]);

	res = error;
	goto shrink_ret;
}
EXPORT_SYMBOL(default_file_splice_read);

/*
 * Send 'sd->len' bytes to socket from 'sd->file' at position 'sd->pos'
 * using sendpage(). Return the number of bytes sent.
 */
static int pipe_to_sendpage(struct pipe_inode_info *pipe,
			    struct pipe_buffer *buf, struct splice_desc *sd)
{
	struct file *file = sd->u.file;
	loff_t pos = sd->pos;
	int more;

	if (!likely(file->f_op && file->f_op->sendpage))
		return -EINVAL;

	more = (sd->flags & SPLICE_F_MORE) ? MSG_MORE : 0;
	if (sd->len < sd->total_len)
		more |= MSG_SENDPAGE_NOTLAST;
	return file->f_op->sendpage(file, buf->page, buf->offset,
				    sd->len, &pos, more);
}

/*
 * This is a little more tricky than the file -> pipe splicing. There are
 * basically three cases:
 *
 *	- Destination page already exists in the address space and there
 *	  are users of it. For that case we have no other option that
 *	  copying the data. Tough luck.
 *	- Destination page already exists in the address space, but there
 *	  are no users of it. Make sure it's uptodate, then drop it. Fall
 *	  through to last case.
 *	- Destination page does not exist, we can add the pipe page to
 *	  the page cache and avoid the copy.
 *
 * If asked to move pages to the output file (SPLICE_F_MOVE is set in
 * sd->flags), we attempt to migrate pages from the pipe to the output
 * file address space page cache. This is possible if no one else has
 * the pipe page referenced outside of the pipe and page cache. If
 * SPLICE_F_MOVE isn't set, or we cannot move the page, we simply create
 * a new page in the output file page cache and fill/dirty that.
 */
int pipe_to_file(struct pipe_inode_info *pipe, struct pipe_buffer *buf,
		 struct splice_desc *sd)
{
	struct file *file = sd->u.file;
	struct address_space *mapping = file->f_mapping;
	unsigned int offset, this_len;
	struct page *page;
	void *fsdata;
	int ret;

	offset = sd->pos & ~PAGE_CACHE_MASK;

	this_len = sd->len;
	if (this_len + offset > PAGE_CACHE_SIZE)
		this_len = PAGE_CACHE_SIZE - offset;

	ret = pagecache_write_begin(file, mapping, sd->pos, this_len,
				AOP_FLAG_UNINTERRUPTIBLE, &page, &fsdata);
	if (unlikely(ret))
		goto out;

	if (buf->page != page) {
		char *src = buf->ops->map(pipe, buf, 1);
		char *dst = kmap_atomic(page);

		memcpy(dst + offset, src + buf->offset, this_len);
		flush_dcache_page(page);
		kunmap_atomic(dst);
		buf->ops->unmap(pipe, buf, src);
	}
	ret = pagecache_write_end(file, mapping, sd->pos, this_len, this_len,
				page, fsdata);
out:
	return ret;
}
EXPORT_SYMBOL(pipe_to_file);

static void wakeup_pipe_writers(struct pipe_inode_info *pipe)
{
	smp_mb();
	if (waitqueue_active(&pipe->wait))
		wake_up_interruptible(&pipe->wait);
	kill_fasync(&pipe->fasync_writers, SIGIO, POLL_OUT);
}

/**
 * splice_from_pipe_feed - feed available data from a pipe to a file
 * @pipe:	pipe to splice from
 * @sd:		information to @actor
 * @actor:	handler that splices the data
 *
 * Description:
 *    This function loops over the pipe and calls @actor to do the
 *    actual moving of a single struct pipe_buffer to the desired
 *    destination.  It returns when there's no more buffers left in
 *    the pipe or if the requested number of bytes (@sd->total_len)
 *    have been copied.  It returns a positive number (one) if the
 *    pipe needs to be filled with more data, zero if the required
 *    number of bytes have been copied and -errno on error.
 *
 *    This, together with splice_from_pipe_{begin,end,next}, may be
 *    used to implement the functionality of __splice_from_pipe() when
 *    locking is required around copying the pipe buffers to the
 *    destination.
 */
int splice_from_pipe_feed(struct pipe_inode_info *pipe, struct splice_desc *sd,
			  splice_actor *actor)
{
	int ret;

	while (pipe->nrbufs) {
		struct pipe_buffer *buf = pipe->bufs + pipe->curbuf;
		const struct pipe_buf_operations *ops = buf->ops;

		sd->len = buf->len;
		if (sd->len > sd->total_len)
			sd->len = sd->total_len;

		ret = buf->ops->confirm(pipe, buf);
		if (unlikely(ret)) {
			if (ret == -ENODATA)
				ret = 0;
			return ret;
		}

		ret = actor(pipe, buf, sd);
		if (ret <= 0)
			return ret;

		buf->offset += ret;
		buf->len -= ret;

		sd->num_spliced += ret;
		sd->len -= ret;
		sd->pos += ret;
		sd->total_len -= ret;

		if (!buf->len) {
			buf->ops = NULL;
			ops->release(pipe, buf);
			pipe->curbuf = (pipe->curbuf + 1) & (pipe->buffers - 1);
			pipe->nrbufs--;
			if (pipe->inode)
				sd->need_wakeup = true;
		}

		if (!sd->total_len)
			return 0;
	}

	return 1;
}
EXPORT_SYMBOL(splice_from_pipe_feed);

/**
 * splice_from_pipe_next - wait for some data to splice from
 * @pipe:	pipe to splice from
 * @sd:		information about the splice operation
 *
 * Description:
 *    This function will wait for some data and return a positive
 *    value (one) if pipe buffers are available.  It will return zero
 *    or -errno if no more data needs to be spliced.
 */
int splice_from_pipe_next(struct pipe_inode_info *pipe, struct splice_desc *sd)
{
	while (!pipe->nrbufs) {
		if (!pipe->writers)
			return 0;

		if (!pipe->waiting_writers && sd->num_spliced)
			return 0;

		if (sd->flags & SPLICE_F_NONBLOCK)
			return -EAGAIN;

		if (signal_pending(current))
			return -ERESTARTSYS;

		if (sd->need_wakeup) {
			wakeup_pipe_writers(pipe);
			sd->need_wakeup = false;
		}

		pipe_wait(pipe);
	}

	return 1;
}
EXPORT_SYMBOL(splice_from_pipe_next);

/**
 * splice_from_pipe_begin - start splicing from pipe
 * @sd:		information about the splice operation
 *
 * Description:
 *    This function should be called before a loop containing
 *    splice_from_pipe_next() and splice_from_pipe_feed() to
 *    initialize the necessary fields of @sd.
 */
void splice_from_pipe_begin(struct splice_desc *sd)
{
	sd->num_spliced = 0;
	sd->need_wakeup = false;
}
EXPORT_SYMBOL(splice_from_pipe_begin);

/**
 * splice_from_pipe_end - finish splicing from pipe
 * @pipe:	pipe to splice from
 * @sd:		information about the splice operation
 *
 * Description:
 *    This function will wake up pipe writers if necessary.  It should
 *    be called after a loop containing splice_from_pipe_next() and
 *    splice_from_pipe_feed().
 */
void splice_from_pipe_end(struct pipe_inode_info *pipe, struct splice_desc *sd)
{
	if (sd->need_wakeup)
		wakeup_pipe_writers(pipe);
}
EXPORT_SYMBOL(splice_from_pipe_end);

/**
 * __splice_from_pipe - splice data from a pipe to given actor
 * @pipe:	pipe to splice from
 * @sd:		information to @actor
 * @actor:	handler that splices the data
 *
 * Description:
 *    This function does little more than loop over the pipe and call
 *    @actor to do the actual moving of a single struct pipe_buffer to
 *    the desired destination. See pipe_to_file, pipe_to_sendpage, or
 *    pipe_to_user.
 *
 */
ssize_t __splice_from_pipe(struct pipe_inode_info *pipe, struct splice_desc *sd,
			   splice_actor *actor)
{
	int ret;

	splice_from_pipe_begin(sd);
	do {
		ret = splice_from_pipe_next(pipe, sd);
		if (ret > 0)
			ret = splice_from_pipe_feed(pipe, sd, actor);
	} while (ret > 0);
	splice_from_pipe_end(pipe, sd);

	return sd->num_spliced ? sd->num_spliced : ret;
}
EXPORT_SYMBOL(__splice_from_pipe);

/**
 * splice_from_pipe - splice data from a pipe to a file
 * @pipe:	pipe to splice from
 * @out:	file to splice to
 * @ppos:	position in @out
 * @len:	how many bytes to splice
 * @flags:	splice modifier flags
 * @actor:	handler that splices the data
 *
 * Description:
 *    See __splice_from_pipe. This function locks the pipe inode,
 *    otherwise it's identical to __splice_from_pipe().
 *
 */
ssize_t splice_from_pipe(struct pipe_inode_info *pipe, struct file *out,
			 loff_t *ppos, size_t len, unsigned int flags,
			 splice_actor *actor)
{
	ssize_t ret;
	struct splice_desc sd = {
		.total_len = len,
		.flags = flags,
		.pos = *ppos,
		.u.file = out,
	};

	pipe_lock(pipe);
	ret = __splice_from_pipe(pipe, &sd, actor);
	pipe_unlock(pipe);

	return ret;
}

/**
 * generic_file_splice_write - splice data from a pipe to a file
 * @pipe:	pipe info
 * @out:	file to write to
 * @ppos:	position in @out
 * @len:	number of bytes to splice
 * @flags:	splice modifier flags
 *
 * Description:
 *    Will either move or copy pages (determined by @flags options) from
 *    the given pipe inode to the given file.
 *
 */
ssize_t
generic_file_splice_write(struct pipe_inode_info *pipe, struct file *out,
			  loff_t *ppos, size_t len, unsigned int flags)
{
	struct address_space *mapping = out->f_mapping;
	struct inode *inode = mapping->host;
	struct splice_desc sd = {
		.total_len = len,
		.flags = flags,
		.pos = *ppos,
		.u.file = out,
	};
	ssize_t ret;

	sb_start_write(inode->i_sb);

	pipe_lock(pipe);

	splice_from_pipe_begin(&sd);
	do {
		ret = splice_from_pipe_next(pipe, &sd);
		if (ret <= 0)
			break;

		mutex_lock_nested(&inode->i_mutex, I_MUTEX_CHILD);
		ret = file_remove_suid(out);
		if (!ret) {
			ret = file_update_time(out);
			if (!ret)
				ret = splice_from_pipe_feed(pipe, &sd,
							    pipe_to_file);
		}
		mutex_unlock(&inode->i_mutex);
	} while (ret > 0);
	splice_from_pipe_end(pipe, &sd);

	pipe_unlock(pipe);

	if (sd.num_spliced)
		ret = sd.num_spliced;

	if (ret > 0) {
		unsigned long nr_pages;
		int err;

		nr_pages = (ret + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;

		err = generic_write_sync(out, *ppos, ret);
		if (err)
			ret = err;
		else
			*ppos += ret;
		balance_dirty_pages_ratelimited_nr(mapping, nr_pages);
	}
	sb_end_write(inode->i_sb);

	return ret;
}

EXPORT_SYMBOL(generic_file_splice_write);

static int write_pipe_buf(struct pipe_inode_info *pipe, struct pipe_buffer *buf,
			  struct splice_desc *sd)
{
	int ret;
	void *data;

	data = buf->ops->map(pipe, buf, 0);
	ret = kernel_write(sd->u.file, data + buf->offset, sd->len, sd->pos);
	buf->ops->unmap(pipe, buf, data);

	return ret;
}

static ssize_t default_file_splice_write(struct pipe_inode_info *pipe,
					 struct file *out, loff_t *ppos,
					 size_t len, unsigned int flags)
{
	ssize_t ret;

	ret = splice_from_pipe(pipe, out, ppos, len, flags, write_pipe_buf);
	if (ret > 0)
		*ppos += ret;

	return ret;
}

/**
 * generic_splice_sendpage - splice data from a pipe to a socket
 * @pipe:	pipe to splice from
 * @out:	socket to write to
 * @ppos:	position in @out
 * @len:	number of bytes to splice
 * @flags:	splice modifier flags
 *
 * Description:
 *    Will send @len bytes from the pipe to a network socket. No data copying
 *    is involved.
 *
 */
ssize_t generic_splice_sendpage(struct pipe_inode_info *pipe, struct file *out,
				loff_t *ppos, size_t len, unsigned int flags)
{
	return splice_from_pipe(pipe, out, ppos, len, flags, pipe_to_sendpage);
}

EXPORT_SYMBOL(generic_splice_sendpage);

/*
 * Attempt to initiate a splice from pipe to file.
 */
static long do_splice_from(struct pipe_inode_info *pipe, struct file *out,
			   loff_t *ppos, size_t len, unsigned int flags)
{
	ssize_t (*splice_write)(struct pipe_inode_info *, struct file *,
				loff_t *, size_t, unsigned int);
	int ret;

	if (unlikely(!(out->f_mode & FMODE_WRITE)))
		return -EBADF;

	if (unlikely(out->f_flags & O_APPEND))
		return -EINVAL;

	ret = rw_verify_area(WRITE, out, ppos, len);
	if (unlikely(ret < 0))
		return ret;

	if (out->f_op && out->f_op->splice_write)
		splice_write = out->f_op->splice_write;
	else
		splice_write = default_file_splice_write;

	return splice_write(pipe, out, ppos, len, flags);
}

/*
 * Attempt to initiate a splice from a file to a pipe.
 */
static long do_splice_to(struct file *in, loff_t *ppos,
			 struct pipe_inode_info *pipe, size_t len,
			 unsigned int flags)
{
	ssize_t (*splice_read)(struct file *, loff_t *,
			       struct pipe_inode_info *, size_t, unsigned int);
	int ret;

	if (unlikely(!(in->f_mode & FMODE_READ)))
		return -EBADF;

	ret = rw_verify_area(READ, in, ppos, len);
	if (unlikely(ret < 0))
		return ret;

	if (in->f_op && in->f_op->splice_read)
		splice_read = in->f_op->splice_read;
	else
		splice_read = default_file_splice_read;

	return splice_read(in, ppos, pipe, len, flags);
}

/**
 * splice_direct_to_actor - splices data directly between two non-pipes
 * @in:		file to splice from
 * @sd:		actor information on where to splice to
 * @actor:	handles the data splicing
 *
 * Description:
 *    This is a special case helper to splice directly between two
 *    points, without requiring an explicit pipe. Internally an allocated
 *    pipe is cached in the process, and reused during the lifetime of
 *    that process.
 *
 */
ssize_t splice_direct_to_actor(struct file *in, struct splice_desc *sd,
			       splice_direct_actor *actor)
{
	struct pipe_inode_info *pipe;
	long ret, bytes;
	umode_t i_mode;
	size_t len;
	int i, flags;

	/*
	 * We require the input being a regular file, as we don't want to
	 * randomly drop data for eg socket -> socket splicing. Use the
	 * piped splicing for that!
	 */
	i_mode = in->f_path.dentry->d_inode->i_mode;
	if (unlikely(!S_ISREG(i_mode) && !S_ISBLK(i_mode)))
		return -EINVAL;

	/*
	 * neither in nor out is a pipe, setup an internal pipe attached to
	 * 'out' and transfer the wanted data from 'in' to 'out' through that
	 */
	pipe = current->splice_pipe;
	if (unlikely(!pipe)) {
		pipe = alloc_pipe_info(NULL);
		if (!pipe)
			return -ENOMEM;

		/*
		 * We don't have an immediate reader, but we'll read the stuff
		 * out of the pipe right after the splice_to_pipe(). So set
		 * PIPE_READERS appropriately.
		 */
		pipe->readers = 1;

		current->splice_pipe = pipe;
	}

	/*
	 * Do the splice.
	 */
	ret = 0;
	bytes = 0;
	len = sd->total_len;
	flags = sd->flags;

	/*
	 * Don't block on output, we have to drain the direct pipe.
	 */
	sd->flags &= ~SPLICE_F_NONBLOCK;

	while (len) {
		size_t read_len;
		loff_t pos = sd->pos, prev_pos = pos;

		ret = do_splice_to(in, &pos, pipe, len, flags);
		if (unlikely(ret <= 0))
			goto out_release;

		read_len = ret;
		sd->total_len = read_len;

		/*
		 * NOTE: nonblocking mode only applies to the input. We
		 * must not do the output in nonblocking mode as then we
		 * could get stuck data in the internal pipe:
		 */
		ret = actor(pipe, sd);
		if (unlikely(ret <= 0)) {
			sd->pos = prev_pos;
			goto out_release;
		}

		bytes += ret;
		len -= ret;
		sd->pos = pos;

		if (ret < read_len) {
			sd->pos = prev_pos + ret;
			goto out_release;
		}
	}

done:
	pipe->nrbufs = pipe->curbuf = 0;
	file_accessed(in);
	return bytes;

out_release:
	/*
	 * If we did an incomplete transfer we must release
	 * the pipe buffers in question:
	 */
	for (i = 0; i < pipe->buffers; i++) {
		struct pipe_buffer *buf = pipe->bufs + i;

		if (buf->ops) {
			buf->ops->release(pipe, buf);
			buf->ops = NULL;
		}
	}

	if (!bytes)
		bytes = ret;

	goto done;
}
EXPORT_SYMBOL(splice_direct_to_actor);

static int direct_splice_actor(struct pipe_inode_info *pipe,
			       struct splice_desc *sd)
{
	struct file *file = sd->u.file;

	return do_splice_from(pipe, file, &file->f_pos, sd->total_len,
			      sd->flags);
}

/**
 * do_splice_direct - splices data directly between two files
 * @in:		file to splice from
 * @ppos:	input file offset
 * @out:	file to splice to
 * @len:	number of bytes to splice
 * @flags:	splice modifier flags
 *
 * Description:
 *    For use by do_sendfile(). splice can easily emulate sendfile, but
 *    doing it in the application would incur an extra system call
 *    (splice in + splice out, as compared to just sendfile()). So this helper
 *    can splice directly through a process-private pipe.
 *
 */
long do_splice_direct(struct file *in, loff_t *ppos, struct file *out,
		      size_t len, unsigned int flags)
{
	struct splice_desc sd = {
		.len		= len,
		.total_len	= len,
		.flags		= flags,
		.pos		= *ppos,
		.u.file		= out,
	};
	long ret;

	ret = splice_direct_to_actor(in, &sd, direct_splice_actor);
	if (ret > 0)
		*ppos = sd.pos;

	return ret;
}

static int splice_pipe_to_pipe(struct pipe_inode_info *ipipe,
			       struct pipe_inode_info *opipe,
			       size_t len, unsigned int flags);

/*
 * Determine where to splice to/from.
 */
static long do_splice(struct file *in, loff_t __user *off_in,
		      struct file *out, loff_t __user *off_out,
		      size_t len, unsigned int flags)
{
	struct pipe_inode_info *ipipe;
	struct pipe_inode_info *opipe;
	loff_t offset, *off;
	long ret;

	ipipe = get_pipe_info(in);
	opipe = get_pipe_info(out);

	if (ipipe && opipe) {
		if (off_in || off_out)
			return -ESPIPE;

		if (!(in->f_mode & FMODE_READ))
			return -EBADF;

		if (!(out->f_mode & FMODE_WRITE))
			return -EBADF;

		/* Splicing to self would be fun, but... */
		if (ipipe == opipe)
			return -EINVAL;

		return splice_pipe_to_pipe(ipipe, opipe, len, flags);
	}

	if (ipipe) {
		if (off_in)
			return -ESPIPE;
		if (off_out) {
			if (!(out->f_mode & FMODE_PWRITE))
				return -EINVAL;
			if (copy_from_user(&offset, off_out, sizeof(loff_t)))
				return -EFAULT;
			off = &offset;
		} else
			off = &out->f_pos;

		ret = do_splice_from(ipipe, out, off, len, flags);

		if (off_out && copy_to_user(off_out, off, sizeof(loff_t)))
			ret = -EFAULT;

		return ret;
	}

	if (opipe) {
		if (off_out)
			return -ESPIPE;
		if (off_in) {
			if (!(in->f_mode & FMODE_PREAD))
				return -EINVAL;
			if (copy_from_user(&offset, off_in, sizeof(loff_t)))
				return -EFAULT;
			off = &offset;
		} else
			off = &in->f_pos;

		ret = do_splice_to(in, off, opipe, len, flags);

		if (off_in && copy_to_user(off_in, off, sizeof(loff_t)))
			ret = -EFAULT;

		return ret;
	}

	return -EINVAL;
}

#ifdef CONFIG_BCM_CUSTOM_RECVFILE
static ssize_t do_splice_from_socket(struct file *file, struct socket *sock,
				     loff_t __user *ppos, size_t count)
{		
	struct address_space *mapping = file->f_mapping;
	struct inode	*inode = mapping->host;
	loff_t pos;
	int count_tmp;
	int err = 0;
	int cpage_ptr = 0;
	int cpage_alloc_cnt = 0;
	struct recvfile_ctl_blk *rv_cb;
	struct kvec *iov;
	struct msghdr msg;
	long rcvtimeo;
	int ret;

        rv_cb = kmalloc(sizeof(struct recvfile_ctl_blk) *
                        MAX_PAGES_PER_RECVFILE, GFP_KERNEL);
        if (!rv_cb) {
		return -ENOMEM;
        }
        iov = kmalloc(sizeof(struct kvec) * MAX_PAGES_PER_RECVFILE,
                      GFP_KERNEL);
        if (!iov) {
                kfree(rv_cb);
		return -ENOMEM;
        }                
	if (copy_from_user(&pos, ppos, sizeof(loff_t))) {
                kfree(rv_cb);
                kfree(iov);
		return -EFAULT;
        }

	if (count > MAX_PAGES_PER_RECVFILE * PAGE_SIZE) {
                kfree(rv_cb);
                kfree(iov);
		return -EINVAL;
	}    
	mutex_lock(&inode->i_mutex);

	/* We can write back this queue in page reclaim */
	current->backing_dev_info = mapping->backing_dev_info;

	err = generic_write_checks(file, &pos, &count, S_ISBLK(inode->i_mode));
	if (err != 0 || count == 0) {
		goto done;
        }

	if ((err = file_remove_suid(file))) {
                goto done;
        }
	if ((err = file_update_time(file))) {
                goto done;
        }

	count_tmp = count;
	do {
		unsigned long bytes;	/* Bytes to write to page */
		unsigned long offset;	/* Offset into pagecache page */
		struct page *page;
		void *fsdata;

		offset = (pos & (PAGE_CACHE_SIZE - 1));
		bytes = PAGE_CACHE_SIZE - offset;
		if (bytes > count_tmp)
			bytes = count_tmp;
                if (likely(mapping && mapping->a_ops && mapping->a_ops->write_begin)) {
                        ret = mapping->a_ops->write_begin(file, mapping, pos, bytes,
                                                          AOP_FLAG_UNINTERRUPTIBLE,
                                                          &page, &fsdata);
                } else {
                        kfree(rv_cb);
                        kfree(iov);
                        return -EINVAL;
                }

		if (unlikely(ret)) {
			err = ret;
			for (cpage_ptr = 0; cpage_ptr < cpage_alloc_cnt; cpage_ptr++) {
				kunmap(rv_cb[cpage_ptr].rv_page);
				ret = mapping->a_ops->write_end(file, mapping,
								rv_cb[cpage_ptr].rv_pos,
								rv_cb[cpage_ptr].rv_count,
								rv_cb[cpage_ptr].rv_count,
								rv_cb[cpage_ptr].rv_page,
								rv_cb[cpage_ptr].rv_fsdata);
			}
			goto done;
		}
#ifdef CONFIG_BCM_CUSTOM_RECVFILE_MAX_PERF
		page->csf_magic = CSF_WR_MAGIC;
#endif
		rv_cb[cpage_alloc_cnt].rv_page = page;
		rv_cb[cpage_alloc_cnt].rv_pos = pos;
		rv_cb[cpage_alloc_cnt].rv_count = bytes;
		rv_cb[cpage_alloc_cnt].rv_fsdata = fsdata;
		iov[cpage_alloc_cnt].iov_base = kmap(page) + offset;
		iov[cpage_alloc_cnt].iov_len = bytes;
		cpage_alloc_cnt++;
		count_tmp -= bytes;
		pos += bytes;
	} while (count_tmp);

	/* IOV is ready, receive the data from socket */
	msg.msg_name = NULL;
	msg.msg_namelen = 0;
	msg.msg_iov = (struct iovec *)&iov[0];
	msg.msg_iovlen = cpage_alloc_cnt ;
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	msg.msg_flags = MSG_KERNSPACE;
	rcvtimeo = sock->sk->sk_rcvtimeo;    
	sock->sk->sk_rcvtimeo = 8 * HZ;

	ret = kernel_recvmsg(sock, &msg, &iov[0], cpage_alloc_cnt, count,
			     MSG_WAITALL | MSG_NOCATCHSIG);

	sock->sk->sk_rcvtimeo = rcvtimeo;
	if (unlikely(ret != count)) {
                unsigned long bytes_left = ret;
                unsigned long bytes_this_page;

                /* save the number of bytes received to return to the caller */
                count = ret;
		for (cpage_ptr = 0; cpage_ptr < cpage_alloc_cnt; cpage_ptr++) {
                        if (bytes_left >= PAGE_CACHE_SIZE) {
                                bytes_this_page = PAGE_CACHE_SIZE;
                        } else {
                                bytes_this_page = bytes_left;
                        }
                        bytes_left -= bytes_this_page;
                        rv_cb[cpage_ptr].rv_page->csf_magic = 0;
			kunmap(rv_cb[cpage_ptr].rv_page);
			(void)mapping->a_ops->write_end(file, mapping,
							rv_cb[cpage_ptr].rv_pos,
							rv_cb[cpage_ptr].rv_count,
							bytes_this_page,
							rv_cb[cpage_ptr].rv_page,
							rv_cb[cpage_ptr].rv_fsdata);

		}
		goto done;
        }

	for (cpage_ptr=0, count=0; cpage_ptr < cpage_alloc_cnt; cpage_ptr++) {
		kunmap(rv_cb[cpage_ptr].rv_page);
		ret = mapping->a_ops->write_end(file, mapping,
						rv_cb[cpage_ptr].rv_pos,
						rv_cb[cpage_ptr].rv_count,
						rv_cb[cpage_ptr].rv_count,
						rv_cb[cpage_ptr].rv_page,
						rv_cb[cpage_ptr].rv_fsdata);
		if (unlikely(ret < 0))
			printk("%s: write_end fail,ret = %d\n", __func__, ret);
		count += rv_cb[cpage_ptr].rv_count;
	}
	balance_dirty_pages_ratelimited_nr(mapping, cpage_alloc_cnt);
	copy_to_user(ppos,&pos,sizeof(loff_t));
    
done:
        kfree(rv_cb);
        kfree(iov);
	current->backing_dev_info = NULL;    
	mutex_unlock(&inode->i_mutex);
        return err ? err : count;
}
#endif

/*
 * Map an iov into an array of pages and offset/length tupples. With the
 * partial_page structure, we can map several non-contiguous ranges into
 * our ones pages[] map instead of splitting that operation into pieces.
 * Could easily be exported as a generic helper for other users, in which
 * case one would probably want to add a 'max_nr_pages' parameter as well.
 */
static int get_iovec_page_array(const struct iovec __user *iov,
				unsigned int nr_vecs, struct page **pages,
				struct partial_page *partial, bool aligned,
				unsigned int pipe_buffers)
{
	int buffers = 0, error = 0;

	while (nr_vecs) {
		unsigned long off, npages;
		struct iovec entry;
		void __user *base;
		size_t len;
		int i;

		error = -EFAULT;
		if (copy_from_user(&entry, iov, sizeof(entry)))
			break;

		base = entry.iov_base;
		len = entry.iov_len;

		/*
		 * Sanity check this iovec. 0 read succeeds.
		 */
		error = 0;
		if (unlikely(!len))
			break;
		error = -EFAULT;
		if (!access_ok(VERIFY_READ, base, len))
			break;

		/*
		 * Get this base offset and number of pages, then map
		 * in the user pages.
		 */
		off = (unsigned long) base & ~PAGE_MASK;

		/*
		 * If asked for alignment, the offset must be zero and the
		 * length a multiple of the PAGE_SIZE.
		 */
		error = -EINVAL;
		if (aligned && (off || len & ~PAGE_MASK))
			break;

		npages = (off + len + PAGE_SIZE - 1) >> PAGE_SHIFT;
		if (npages > pipe_buffers - buffers)
			npages = pipe_buffers - buffers;

		error = get_user_pages_fast((unsigned long)base, npages,
					0, &pages[buffers]);

		if (unlikely(error <= 0))
			break;

		/*
		 * Fill this contiguous range into the partial page map.
		 */
		for (i = 0; i < error; i++) {
			const int plen = min_t(size_t, len, PAGE_SIZE - off);

			partial[buffers].offset = off;
			partial[buffers].len = plen;

			off = 0;
			len -= plen;
			buffers++;
		}

		/*
		 * We didn't complete this iov, stop here since it probably
		 * means we have to move some of this into a pipe to
		 * be able to continue.
		 */
		if (len)
			break;

		/*
		 * Don't continue if we mapped fewer pages than we asked for,
		 * or if we mapped the max number of pages that we have
		 * room for.
		 */
		if (error < npages || buffers == pipe_buffers)
			break;

		nr_vecs--;
		iov++;
	}

	if (buffers)
		return buffers;

	return error;
}

static int pipe_to_user(struct pipe_inode_info *pipe, struct pipe_buffer *buf,
			struct splice_desc *sd)
{
	char *src;
	int ret;

	/*
	 * See if we can use the atomic maps, by prefaulting in the
	 * pages and doing an atomic copy
	 */
	if (!fault_in_pages_writeable(sd->u.userptr, sd->len)) {
		src = buf->ops->map(pipe, buf, 1);
		ret = __copy_to_user_inatomic(sd->u.userptr, src + buf->offset,
							sd->len);
		buf->ops->unmap(pipe, buf, src);
		if (!ret) {
			ret = sd->len;
			goto out;
		}
	}

	/*
	 * No dice, use slow non-atomic map and copy
 	 */
	src = buf->ops->map(pipe, buf, 0);

	ret = sd->len;
	if (copy_to_user(sd->u.userptr, src + buf->offset, sd->len))
		ret = -EFAULT;

	buf->ops->unmap(pipe, buf, src);
out:
	if (ret > 0)
		sd->u.userptr += ret;
	return ret;
}

/*
 * For lack of a better implementation, implement vmsplice() to userspace
 * as a simple copy of the pipes pages to the user iov.
 */
static long vmsplice_to_user(struct file *file, const struct iovec __user *iov,
			     unsigned long nr_segs, unsigned int flags)
{
	struct pipe_inode_info *pipe;
	struct splice_desc sd;
	ssize_t size;
	int error;
	long ret;

	pipe = get_pipe_info(file);
	if (!pipe)
		return -EBADF;

	pipe_lock(pipe);

	error = ret = 0;
	while (nr_segs) {
		void __user *base;
		size_t len;

		/*
		 * Get user address base and length for this iovec.
		 */
		error = get_user(base, &iov->iov_base);
		if (unlikely(error))
			break;
		error = get_user(len, &iov->iov_len);
		if (unlikely(error))
			break;

		/*
		 * Sanity check this iovec. 0 read succeeds.
		 */
		if (unlikely(!len))
			break;
		if (unlikely(!base)) {
			error = -EFAULT;
			break;
		}

		if (unlikely(!access_ok(VERIFY_WRITE, base, len))) {
			error = -EFAULT;
			break;
		}

		sd.len = 0;
		sd.total_len = len;
		sd.flags = flags;
		sd.u.userptr = base;
		sd.pos = 0;

		size = __splice_from_pipe(pipe, &sd, pipe_to_user);
		if (size < 0) {
			if (!ret)
				ret = size;

			break;
		}

		ret += size;

		if (size < len)
			break;

		nr_segs--;
		iov++;
	}

	pipe_unlock(pipe);

	if (!ret)
		ret = error;

	return ret;
}

/*
 * vmsplice splices a user address range into a pipe. It can be thought of
 * as splice-from-memory, where the regular splice is splice-from-file (or
 * to file). In both cases the output is a pipe, naturally.
 */
static long vmsplice_to_pipe(struct file *file, const struct iovec __user *iov,
			     unsigned long nr_segs, unsigned int flags)
{
	struct pipe_inode_info *pipe;
	struct page *pages[PIPE_DEF_BUFFERS];
	struct partial_page partial[PIPE_DEF_BUFFERS];
	struct splice_pipe_desc spd = {
		.pages = pages,
		.partial = partial,
		.nr_pages_max = PIPE_DEF_BUFFERS,
		.flags = flags,
		.ops = &user_page_pipe_buf_ops,
		.spd_release = spd_release_page,
	};
	long ret;

	pipe = get_pipe_info(file);
	if (!pipe)
		return -EBADF;

	if (splice_grow_spd(pipe, &spd))
		return -ENOMEM;

	spd.nr_pages = get_iovec_page_array(iov, nr_segs, spd.pages,
					    spd.partial, false,
					    spd.nr_pages_max);
	if (spd.nr_pages <= 0)
		ret = spd.nr_pages;
	else
		ret = splice_to_pipe(pipe, &spd);

	splice_shrink_spd(&spd);
	return ret;
}

/*
 * Note that vmsplice only really supports true splicing _from_ user memory
 * to a pipe, not the other way around. Splicing from user memory is a simple
 * operation that can be supported without any funky alignment restrictions
 * or nasty vm tricks. We simply map in the user memory and fill them into
 * a pipe. The reverse isn't quite as easy, though. There are two possible
 * solutions for that:
 *
 *	- memcpy() the data internally, at which point we might as well just
 *	  do a regular read() on the buffer anyway.
 *	- Lots of nasty vm tricks, that are neither fast nor flexible (it
 *	  has restriction limitations on both ends of the pipe).
 *
 * Currently we punt and implement it as a normal copy, see pipe_to_user().
 *
 */
SYSCALL_DEFINE4(vmsplice, int, fd, const struct iovec __user *, iov,
		unsigned long, nr_segs, unsigned int, flags)
{
	struct file *file;
	long error;
	int fput;

	if (unlikely(nr_segs > UIO_MAXIOV))
		return -EINVAL;
	else if (unlikely(!nr_segs))
		return 0;

	error = -EBADF;
	file = fget_light(fd, &fput);
	if (file) {
		if (file->f_mode & FMODE_WRITE)
			error = vmsplice_to_pipe(file, iov, nr_segs, flags);
		else if (file->f_mode & FMODE_READ)
			error = vmsplice_to_user(file, iov, nr_segs, flags);

		fput_light(file, fput);
	}

	return error;
}

SYSCALL_DEFINE6(splice, int, fd_in, loff_t __user *, off_in,
		int, fd_out, loff_t __user *, off_out,
		size_t, len, unsigned int, flags)
{
	int error;
	struct file *in, *out;
	int fput_in, fput_out;
#ifdef CONFIG_BCM_CUSTOM_RECVFILE
	struct socket *sock = NULL;
#endif

	if (unlikely(!len))
		return 0;

	error = -EBADF;

#ifdef CONFIG_BCM_CUSTOM_RECVFILE
	/* check if fd_in is a socket */
	sock = sockfd_lookup(fd_in, &error);
	if (sock) {
		out = NULL;
		if (!sock->sk)
			goto done;
		out = fget_light(fd_out, &fput_out);
        
		if (out) {
			if (!(out->f_mode & FMODE_WRITE))
				goto done;
			error = do_splice_from_socket(out, sock, off_out, len);
		}       
done:
		if(out)
			fput_light(out, fput_out);      
		fput(sock->file);
		return error;
	}
#endif
	in = fget_light(fd_in, &fput_in);
	if (in) {
		if (in->f_mode & FMODE_READ) {
			out = fget_light(fd_out, &fput_out);
			if (out) {
				if (out->f_mode & FMODE_WRITE)
					error = do_splice(in, off_in,
							  out, off_out,
							  len, flags);
				fput_light(out, fput_out);
			}
		}

		fput_light(in, fput_in);
	}

	return error;
}

/*
 * Make sure there's data to read. Wait for input if we can, otherwise
 * return an appropriate error.
 */
static int ipipe_prep(struct pipe_inode_info *pipe, unsigned int flags)
{
	int ret;

	/*
	 * Check ->nrbufs without the inode lock first. This function
	 * is speculative anyways, so missing one is ok.
	 */
	if (pipe->nrbufs)
		return 0;

	ret = 0;
	pipe_lock(pipe);

	while (!pipe->nrbufs) {
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
		if (!pipe->writers)
			break;
		if (!pipe->waiting_writers) {
			if (flags & SPLICE_F_NONBLOCK) {
				ret = -EAGAIN;
				break;
			}
		}
		pipe_wait(pipe);
	}

	pipe_unlock(pipe);
	return ret;
}

/*
 * Make sure there's writeable room. Wait for room if we can, otherwise
 * return an appropriate error.
 */
static int opipe_prep(struct pipe_inode_info *pipe, unsigned int flags)
{
	int ret;

	/*
	 * Check ->nrbufs without the inode lock first. This function
	 * is speculative anyways, so missing one is ok.
	 */
	if (pipe->nrbufs < pipe->buffers)
		return 0;

	ret = 0;
	pipe_lock(pipe);

	while (pipe->nrbufs >= pipe->buffers) {
		if (!pipe->readers) {
			send_sig(SIGPIPE, current, 0);
			ret = -EPIPE;
			break;
		}
		if (flags & SPLICE_F_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
		pipe->waiting_writers++;
		pipe_wait(pipe);
		pipe->waiting_writers--;
	}

	pipe_unlock(pipe);
	return ret;
}

/*
 * Splice contents of ipipe to opipe.
 */
static int splice_pipe_to_pipe(struct pipe_inode_info *ipipe,
			       struct pipe_inode_info *opipe,
			       size_t len, unsigned int flags)
{
	struct pipe_buffer *ibuf, *obuf;
	int ret = 0, nbuf;
	bool input_wakeup = false;


retry:
	ret = ipipe_prep(ipipe, flags);
	if (ret)
		return ret;

	ret = opipe_prep(opipe, flags);
	if (ret)
		return ret;

	/*
	 * Potential ABBA deadlock, work around it by ordering lock
	 * grabbing by pipe info address. Otherwise two different processes
	 * could deadlock (one doing tee from A -> B, the other from B -> A).
	 */
	pipe_double_lock(ipipe, opipe);

	do {
		if (!opipe->readers) {
			send_sig(SIGPIPE, current, 0);
			if (!ret)
				ret = -EPIPE;
			break;
		}

		if (!ipipe->nrbufs && !ipipe->writers)
			break;

		/*
		 * Cannot make any progress, because either the input
		 * pipe is empty or the output pipe is full.
		 */
		if (!ipipe->nrbufs || opipe->nrbufs >= opipe->buffers) {
			/* Already processed some buffers, break */
			if (ret)
				break;

			if (flags & SPLICE_F_NONBLOCK) {
				ret = -EAGAIN;
				break;
			}

			/*
			 * We raced with another reader/writer and haven't
			 * managed to process any buffers.  A zero return
			 * value means EOF, so retry instead.
			 */
			pipe_unlock(ipipe);
			pipe_unlock(opipe);
			goto retry;
		}

		ibuf = ipipe->bufs + ipipe->curbuf;
		nbuf = (opipe->curbuf + opipe->nrbufs) & (opipe->buffers - 1);
		obuf = opipe->bufs + nbuf;

		if (len >= ibuf->len) {
			/*
			 * Simply move the whole buffer from ipipe to opipe
			 */
			*obuf = *ibuf;
			ibuf->ops = NULL;
			opipe->nrbufs++;
			ipipe->curbuf = (ipipe->curbuf + 1) & (ipipe->buffers - 1);
			ipipe->nrbufs--;
			input_wakeup = true;
		} else {
			/*
			 * Get a reference to this pipe buffer,
			 * so we can copy the contents over.
			 */
			ibuf->ops->get(ipipe, ibuf);
			*obuf = *ibuf;

			/*
			 * Don't inherit the gift flag, we need to
			 * prevent multiple steals of this page.
			 */
			obuf->flags &= ~PIPE_BUF_FLAG_GIFT;

			obuf->len = len;
			opipe->nrbufs++;
			ibuf->offset += obuf->len;
			ibuf->len -= obuf->len;
		}
		ret += obuf->len;
		len -= obuf->len;
	} while (len);

	pipe_unlock(ipipe);
	pipe_unlock(opipe);

	/*
	 * If we put data in the output pipe, wakeup any potential readers.
	 */
	if (ret > 0)
		wakeup_pipe_readers(opipe);

	if (input_wakeup)
		wakeup_pipe_writers(ipipe);

	return ret;
}

/*
 * Link contents of ipipe to opipe.
 */
static int link_pipe(struct pipe_inode_info *ipipe,
		     struct pipe_inode_info *opipe,
		     size_t len, unsigned int flags)
{
	struct pipe_buffer *ibuf, *obuf;
	int ret = 0, i = 0, nbuf;

	/*
	 * Potential ABBA deadlock, work around it by ordering lock
	 * grabbing by pipe info address. Otherwise two different processes
	 * could deadlock (one doing tee from A -> B, the other from B -> A).
	 */
	pipe_double_lock(ipipe, opipe);

	do {
		if (!opipe->readers) {
			send_sig(SIGPIPE, current, 0);
			if (!ret)
				ret = -EPIPE;
			break;
		}

		/*
		 * If we have iterated all input buffers or ran out of
		 * output room, break.
		 */
		if (i >= ipipe->nrbufs || opipe->nrbufs >= opipe->buffers)
			break;

		ibuf = ipipe->bufs + ((ipipe->curbuf + i) & (ipipe->buffers-1));
		nbuf = (opipe->curbuf + opipe->nrbufs) & (opipe->buffers - 1);

		/*
		 * Get a reference to this pipe buffer,
		 * so we can copy the contents over.
		 */
		ibuf->ops->get(ipipe, ibuf);

		obuf = opipe->bufs + nbuf;
		*obuf = *ibuf;

		/*
		 * Don't inherit the gift flag, we need to
		 * prevent multiple steals of this page.
		 */
		obuf->flags &= ~PIPE_BUF_FLAG_GIFT;

		if (obuf->len > len)
			obuf->len = len;

		opipe->nrbufs++;
		ret += obuf->len;
		len -= obuf->len;
		i++;
	} while (len);

	/*
	 * return EAGAIN if we have the potential of some data in the
	 * future, otherwise just return 0
	 */
	if (!ret && ipipe->waiting_writers && (flags & SPLICE_F_NONBLOCK))
		ret = -EAGAIN;

	pipe_unlock(ipipe);
	pipe_unlock(opipe);

	/*
	 * If we put data in the output pipe, wakeup any potential readers.
	 */
	if (ret > 0)
		wakeup_pipe_readers(opipe);

	return ret;
}

/*
 * This is a tee(1) implementation that works on pipes. It doesn't copy
 * any data, it simply references the 'in' pages on the 'out' pipe.
 * The 'flags' used are the SPLICE_F_* variants, currently the only
 * applicable one is SPLICE_F_NONBLOCK.
 */
static long do_tee(struct file *in, struct file *out, size_t len,
		   unsigned int flags)
{
	struct pipe_inode_info *ipipe = get_pipe_info(in);
	struct pipe_inode_info *opipe = get_pipe_info(out);
	int ret = -EINVAL;

	/*
	 * Duplicate the contents of ipipe to opipe without actually
	 * copying the data.
	 */
	if (ipipe && opipe && ipipe != opipe) {
		/*
		 * Keep going, unless we encounter an error. The ipipe/opipe
		 * ordering doesn't really matter.
		 */
		ret = ipipe_prep(ipipe, flags);
		if (!ret) {
			ret = opipe_prep(opipe, flags);
			if (!ret)
				ret = link_pipe(ipipe, opipe, len, flags);
		}
	}

	return ret;
}

SYSCALL_DEFINE4(tee, int, fdin, int, fdout, size_t, len, unsigned int, flags)
{
	struct file *in;
	int error, fput_in;

	if (unlikely(!len))
		return 0;

	error = -EBADF;
	in = fget_light(fdin, &fput_in);
	if (in) {
		if (in->f_mode & FMODE_READ) {
			int fput_out;
			struct file *out = fget_light(fdout, &fput_out);

			if (out) {
				if (out->f_mode & FMODE_WRITE)
					error = do_tee(in, out, len, flags);
				fput_light(out, fput_out);
			}
		}
 		fput_light(in, fput_in);
 	}

	return error;
}
