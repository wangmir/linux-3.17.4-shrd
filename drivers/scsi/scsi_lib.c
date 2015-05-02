/*
 * Copyright (C) 1999 Eric Youngdale
 * Copyright (C) 2014 Christoph Hellwig
 *
 *  SCSI queueing library.
 *      Initial versions: Eric Youngdale (eric@andante.org).
 *                        Based upon conversations with large numbers
 *                        of people at Linux Expo.
 */

#include <linux/bio.h>
#include <linux/bitops.h>
#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/mempool.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/hardirq.h>
#include <linux/scatterlist.h>
#include <linux/blk-mq.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_dbg.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_driver.h>
#include <scsi/scsi_eh.h>
#include <scsi/scsi_host.h>

#ifdef CONFIG_SCSI_SHRD_TEST0
#include <linux/rbtree.h>
#include <linux/task_io_accounting_ops.h>
#include <scsi/scsi_shrd.h>
#include "../../block/blk.h"
#include "sd.h"

#endif

#include <trace/events/scsi.h>

#include "scsi_priv.h"
#include "scsi_logging.h"


#define SG_MEMPOOL_NR		ARRAY_SIZE(scsi_sg_pools)
#define SG_MEMPOOL_SIZE		2

struct scsi_host_sg_pool {
	size_t		size;
	char		*name;
	struct kmem_cache	*slab;
	mempool_t	*pool;
};

#define SP(x) { x, "sgpool-" __stringify(x) }
#if (SCSI_MAX_SG_SEGMENTS < 32)
#error SCSI_MAX_SG_SEGMENTS is too small (must be 32 or greater)
#endif
static struct scsi_host_sg_pool scsi_sg_pools[] = {
	SP(8),
	SP(16),
#if (SCSI_MAX_SG_SEGMENTS > 32)
	SP(32),
#if (SCSI_MAX_SG_SEGMENTS > 64)
	SP(64),
#if (SCSI_MAX_SG_SEGMENTS > 128)
	SP(128),
#if (SCSI_MAX_SG_SEGMENTS > 256)
#error SCSI_MAX_SG_SEGMENTS is too large (256 MAX)
#endif
#endif
#endif
#endif
	SP(SCSI_MAX_SG_SEGMENTS)
};
#undef SP

struct kmem_cache *scsi_sdb_cache;

/*
 * When to reinvoke queueing after a resource shortage. It's 3 msecs to
 * not change behaviour from the previous unplug mechanism, experimentation
 * may prove this needs changing.
 */
#define SCSI_QUEUE_DELAY	3

static void
scsi_set_blocked(struct scsi_cmnd *cmd, int reason)
{
	struct Scsi_Host *host = cmd->device->host;
	struct scsi_device *device = cmd->device;
	struct scsi_target *starget = scsi_target(device);

	/*
	 * Set the appropriate busy bit for the device/host.
	 *
	 * If the host/device isn't busy, assume that something actually
	 * completed, and that we should be able to queue a command now.
	 *
	 * Note that the prior mid-layer assumption that any host could
	 * always queue at least one command is now broken.  The mid-layer
	 * will implement a user specifiable stall (see
	 * scsi_host.max_host_blocked and scsi_device.max_device_blocked)
	 * if a command is requeued with no other commands outstanding
	 * either for the device or for the host.
	 */
	switch (reason) {
	case SCSI_MLQUEUE_HOST_BUSY:
		atomic_set(&host->host_blocked, host->max_host_blocked);
		break;
	case SCSI_MLQUEUE_DEVICE_BUSY:
	case SCSI_MLQUEUE_EH_RETRY:
		atomic_set(&device->device_blocked,
			   device->max_device_blocked);
		break;
	case SCSI_MLQUEUE_TARGET_BUSY:
		atomic_set(&starget->target_blocked,
			   starget->max_target_blocked);
		break;
	}
}

static void scsi_mq_requeue_cmd(struct scsi_cmnd *cmd)
{
	struct scsi_device *sdev = cmd->device;
	struct request_queue *q = cmd->request->q;

	blk_mq_requeue_request(cmd->request);
	blk_mq_kick_requeue_list(q);
	put_device(&sdev->sdev_gendev);
}

/**
 * __scsi_queue_insert - private queue insertion
 * @cmd: The SCSI command being requeued
 * @reason:  The reason for the requeue
 * @unbusy: Whether the queue should be unbusied
 *
 * This is a private queue insertion.  The public interface
 * scsi_queue_insert() always assumes the queue should be unbusied
 * because it's always called before the completion.  This function is
 * for a requeue after completion, which should only occur in this
 * file.
 */
static void __scsi_queue_insert(struct scsi_cmnd *cmd, int reason, int unbusy)
{
	struct scsi_device *device = cmd->device;
	struct request_queue *q = device->request_queue;
	unsigned long flags;

	SCSI_LOG_MLQUEUE(1, scmd_printk(KERN_INFO, cmd,
		"Inserting command %p into mlqueue\n", cmd));

	scsi_set_blocked(cmd, reason);

	/*
	 * Decrement the counters, since these commands are no longer
	 * active on the host/device.
	 */
	if (unbusy)
		scsi_device_unbusy(device);

	/*
	 * Requeue this command.  It will go before all other commands
	 * that are already in the queue. Schedule requeue work under
	 * lock such that the kblockd_schedule_work() call happens
	 * before blk_cleanup_queue() finishes.
	 */
	cmd->result = 0;
	if (q->mq_ops) {
		scsi_mq_requeue_cmd(cmd);
		return;
	}
	spin_lock_irqsave(q->queue_lock, flags);
	blk_requeue_request(q, cmd->request);
	kblockd_schedule_work(&device->requeue_work);
	spin_unlock_irqrestore(q->queue_lock, flags);
}

/*
 * Function:    scsi_queue_insert()
 *
 * Purpose:     Insert a command in the midlevel queue.
 *
 * Arguments:   cmd    - command that we are adding to queue.
 *              reason - why we are inserting command to queue.
 *
 * Lock status: Assumed that lock is not held upon entry.
 *
 * Returns:     Nothing.
 *
 * Notes:       We do this for one of two cases.  Either the host is busy
 *              and it cannot accept any more commands for the time being,
 *              or the device returned QUEUE_FULL and can accept no more
 *              commands.
 * Notes:       This could be called either from an interrupt context or a
 *              normal process context.
 */
void scsi_queue_insert(struct scsi_cmnd *cmd, int reason)
{
	__scsi_queue_insert(cmd, reason, 1);
}
/**
 * scsi_execute - insert request and wait for the result
 * @sdev:	scsi device
 * @cmd:	scsi command
 * @data_direction: data direction
 * @buffer:	data buffer
 * @bufflen:	len of buffer
 * @sense:	optional sense buffer
 * @timeout:	request timeout in seconds
 * @retries:	number of times to retry request
 * @flags:	or into request flags;
 * @resid:	optional residual length
 *
 * returns the req->errors value which is the scsi_cmnd result
 * field.
 */
int scsi_execute(struct scsi_device *sdev, const unsigned char *cmd,
		 int data_direction, void *buffer, unsigned bufflen,
		 unsigned char *sense, int timeout, int retries, u64 flags,
		 int *resid)
{
	struct request *req;
	int write = (data_direction == DMA_TO_DEVICE);
	int ret = DRIVER_ERROR << 24;

	req = blk_get_request(sdev->request_queue, write, __GFP_WAIT);
	if (!req)
		return ret;
	blk_rq_set_block_pc(req);

	if (bufflen &&	blk_rq_map_kern(sdev->request_queue, req,
					buffer, bufflen, __GFP_WAIT))
		goto out;

	req->cmd_len = COMMAND_SIZE(cmd[0]);
	memcpy(req->cmd, cmd, req->cmd_len);
	req->sense = sense;
	req->sense_len = 0;
	req->retries = retries;
	req->timeout = timeout;
	req->cmd_flags |= flags | REQ_QUIET | REQ_PREEMPT;

	/*
	 * head injection *required* here otherwise quiesce won't work
	 */
	blk_execute_rq(req->q, NULL, req, 1);

	/*
	 * Some devices (USB mass-storage in particular) may transfer
	 * garbage data together with a residue indicating that the data
	 * is invalid.  Prevent the garbage from being misinterpreted
	 * and prevent security leaks by zeroing out the excess data.
	 */
	if (unlikely(req->resid_len > 0 && req->resid_len <= bufflen))
		memset(buffer + (bufflen - req->resid_len), 0, req->resid_len);

	if (resid)
		*resid = req->resid_len;
	ret = req->errors;
 out:
	blk_put_request(req);

	return ret;
}
EXPORT_SYMBOL(scsi_execute);

int scsi_execute_req_flags(struct scsi_device *sdev, const unsigned char *cmd,
		     int data_direction, void *buffer, unsigned bufflen,
		     struct scsi_sense_hdr *sshdr, int timeout, int retries,
		     int *resid, u64 flags)
{
	char *sense = NULL;
	int result;
	
	if (sshdr) {
		sense = kzalloc(SCSI_SENSE_BUFFERSIZE, GFP_NOIO);
		if (!sense)
			return DRIVER_ERROR << 24;
	}
	result = scsi_execute(sdev, cmd, data_direction, buffer, bufflen,
			      sense, timeout, retries, flags, resid);
	if (sshdr)
		scsi_normalize_sense(sense, SCSI_SENSE_BUFFERSIZE, sshdr);

	kfree(sense);
	return result;
}
EXPORT_SYMBOL(scsi_execute_req_flags);

/*
 * Function:    scsi_init_cmd_errh()
 *
 * Purpose:     Initialize cmd fields related to error handling.
 *
 * Arguments:   cmd	- command that is ready to be queued.
 *
 * Notes:       This function has the job of initializing a number of
 *              fields related to error handling.   Typically this will
 *              be called once for each command, as required.
 */
static void scsi_init_cmd_errh(struct scsi_cmnd *cmd)
{
	cmd->serial_number = 0;
	scsi_set_resid(cmd, 0);
	memset(cmd->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
	if (cmd->cmd_len == 0)
		cmd->cmd_len = scsi_command_size(cmd->cmnd);
}

void scsi_device_unbusy(struct scsi_device *sdev)
{
	struct Scsi_Host *shost = sdev->host;
	struct scsi_target *starget = scsi_target(sdev);
	unsigned long flags;

	atomic_dec(&shost->host_busy);
	if (starget->can_queue > 0)
		atomic_dec(&starget->target_busy);

	if (unlikely(scsi_host_in_recovery(shost) &&
		     (shost->host_failed || shost->host_eh_scheduled))) {
		spin_lock_irqsave(shost->host_lock, flags);
		scsi_eh_wakeup(shost);
		spin_unlock_irqrestore(shost->host_lock, flags);
	}

	atomic_dec(&sdev->device_busy);
}

static void scsi_kick_queue(struct request_queue *q)
{
	if (q->mq_ops)
		blk_mq_start_hw_queues(q);
	else
		blk_run_queue(q);
}

/*
 * Called for single_lun devices on IO completion. Clear starget_sdev_user,
 * and call blk_run_queue for all the scsi_devices on the target -
 * including current_sdev first.
 *
 * Called with *no* scsi locks held.
 */
static void scsi_single_lun_run(struct scsi_device *current_sdev)
{
	struct Scsi_Host *shost = current_sdev->host;
	struct scsi_device *sdev, *tmp;
	struct scsi_target *starget = scsi_target(current_sdev);
	unsigned long flags;

	spin_lock_irqsave(shost->host_lock, flags);
	starget->starget_sdev_user = NULL;
	spin_unlock_irqrestore(shost->host_lock, flags);

	/*
	 * Call blk_run_queue for all LUNs on the target, starting with
	 * current_sdev. We race with others (to set starget_sdev_user),
	 * but in most cases, we will be first. Ideally, each LU on the
	 * target would get some limited time or requests on the target.
	 */
	scsi_kick_queue(current_sdev->request_queue);

	spin_lock_irqsave(shost->host_lock, flags);
	if (starget->starget_sdev_user)
		goto out;
	list_for_each_entry_safe(sdev, tmp, &starget->devices,
			same_target_siblings) {
		if (sdev == current_sdev)
			continue;
		if (scsi_device_get(sdev))
			continue;

		spin_unlock_irqrestore(shost->host_lock, flags);
		scsi_kick_queue(sdev->request_queue);
		spin_lock_irqsave(shost->host_lock, flags);
	
		scsi_device_put(sdev);
	}
 out:
	spin_unlock_irqrestore(shost->host_lock, flags);
}

static inline bool scsi_device_is_busy(struct scsi_device *sdev)
{
	if (atomic_read(&sdev->device_busy) >= sdev->queue_depth)
		return true;
	if (atomic_read(&sdev->device_blocked) > 0)
		return true;
	return false;
}

static inline bool scsi_target_is_busy(struct scsi_target *starget)
{
	if (starget->can_queue > 0) {
		if (atomic_read(&starget->target_busy) >= starget->can_queue)
			return true;
		if (atomic_read(&starget->target_blocked) > 0)
			return true;
	}
	return false;
}

static inline bool scsi_host_is_busy(struct Scsi_Host *shost)
{
	if (shost->can_queue > 0 &&
	    atomic_read(&shost->host_busy) >= shost->can_queue)
		return true;
	if (atomic_read(&shost->host_blocked) > 0)
		return true;
	if (shost->host_self_blocked)
		return true;
	return false;
}

static void scsi_starved_list_run(struct Scsi_Host *shost)
{
	LIST_HEAD(starved_list);
	struct scsi_device *sdev;
	unsigned long flags;

	spin_lock_irqsave(shost->host_lock, flags);
	list_splice_init(&shost->starved_list, &starved_list);

	while (!list_empty(&starved_list)) {
		struct request_queue *slq;

		/*
		 * As long as shost is accepting commands and we have
		 * starved queues, call blk_run_queue. scsi_request_fn
		 * drops the queue_lock and can add us back to the
		 * starved_list.
		 *
		 * host_lock protects the starved_list and starved_entry.
		 * scsi_request_fn must get the host_lock before checking
		 * or modifying starved_list or starved_entry.
		 */
		if (scsi_host_is_busy(shost))
			break;

		sdev = list_entry(starved_list.next,
				  struct scsi_device, starved_entry);
		list_del_init(&sdev->starved_entry);
		if (scsi_target_is_busy(scsi_target(sdev))) {
			list_move_tail(&sdev->starved_entry,
				       &shost->starved_list);
			continue;
		}

		/*
		 * Once we drop the host lock, a racing scsi_remove_device()
		 * call may remove the sdev from the starved list and destroy
		 * it and the queue.  Mitigate by taking a reference to the
		 * queue and never touching the sdev again after we drop the
		 * host lock.  Note: if __scsi_remove_device() invokes
		 * blk_cleanup_queue() before the queue is run from this
		 * function then blk_run_queue() will return immediately since
		 * blk_cleanup_queue() marks the queue with QUEUE_FLAG_DYING.
		 */
		slq = sdev->request_queue;
		if (!blk_get_queue(slq))
			continue;
		spin_unlock_irqrestore(shost->host_lock, flags);

		scsi_kick_queue(slq);
		blk_put_queue(slq);

		spin_lock_irqsave(shost->host_lock, flags);
	}
	/* put any unprocessed entries back */
	list_splice(&starved_list, &shost->starved_list);
	spin_unlock_irqrestore(shost->host_lock, flags);
}

/*
 * Function:   scsi_run_queue()
 *
 * Purpose:    Select a proper request queue to serve next
 *
 * Arguments:  q       - last request's queue
 *
 * Returns:     Nothing
 *
 * Notes:      The previous command was completely finished, start
 *             a new one if possible.
 */
static void scsi_run_queue(struct request_queue *q)
{
	struct scsi_device *sdev = q->queuedata;

	if (scsi_target(sdev)->single_lun)
		scsi_single_lun_run(sdev);
	if (!list_empty(&sdev->host->starved_list))
		scsi_starved_list_run(sdev->host);

	if (q->mq_ops)
		blk_mq_start_stopped_hw_queues(q, false);
	else
		blk_run_queue(q);
}

void scsi_requeue_run_queue(struct work_struct *work)
{
	struct scsi_device *sdev;
	struct request_queue *q;

	sdev = container_of(work, struct scsi_device, requeue_work);
	q = sdev->request_queue;
	scsi_run_queue(q);
}

/*
 * Function:	scsi_requeue_command()
 *
 * Purpose:	Handle post-processing of completed commands.
 *
 * Arguments:	q	- queue to operate on
 *		cmd	- command that may need to be requeued.
 *
 * Returns:	Nothing
 *
 * Notes:	After command completion, there may be blocks left
 *		over which weren't finished by the previous command
 *		this can be for a number of reasons - the main one is
 *		I/O errors in the middle of the request, in which case
 *		we need to request the blocks that come after the bad
 *		sector.
 * Notes:	Upon return, cmd is a stale pointer.
 */
static void scsi_requeue_command(struct request_queue *q, struct scsi_cmnd *cmd)
{
	struct scsi_device *sdev = cmd->device;
	struct request *req = cmd->request;
	unsigned long flags;

	spin_lock_irqsave(q->queue_lock, flags);
	blk_unprep_request(req);
	req->special = NULL;
	scsi_put_command(cmd);
	blk_requeue_request(q, req);
	spin_unlock_irqrestore(q->queue_lock, flags);

	scsi_run_queue(q);

	put_device(&sdev->sdev_gendev);
}

void scsi_next_command(struct scsi_cmnd *cmd)
{
	struct scsi_device *sdev = cmd->device;
	struct request_queue *q = sdev->request_queue;

	scsi_put_command(cmd);
	scsi_run_queue(q);

	put_device(&sdev->sdev_gendev);
}

void scsi_run_host_queues(struct Scsi_Host *shost)
{
	struct scsi_device *sdev;

	shost_for_each_device(sdev, shost)
		scsi_run_queue(sdev->request_queue);
}

static inline unsigned int scsi_sgtable_index(unsigned short nents)
{
	unsigned int index;

	BUG_ON(nents > SCSI_MAX_SG_SEGMENTS);

	if (nents <= 8)
		index = 0;
	else
		index = get_count_order(nents) - 3;

	return index;
}

static void scsi_sg_free(struct scatterlist *sgl, unsigned int nents)
{
	struct scsi_host_sg_pool *sgp;

	sgp = scsi_sg_pools + scsi_sgtable_index(nents);
	mempool_free(sgl, sgp->pool);
}

static struct scatterlist *scsi_sg_alloc(unsigned int nents, gfp_t gfp_mask)
{
	struct scsi_host_sg_pool *sgp;

	sgp = scsi_sg_pools + scsi_sgtable_index(nents);
	return mempool_alloc(sgp->pool, gfp_mask);
}

static void scsi_free_sgtable(struct scsi_data_buffer *sdb, bool mq)
{
	if (mq && sdb->table.nents <= SCSI_MAX_SG_SEGMENTS)
		return;
	__sg_free_table(&sdb->table, SCSI_MAX_SG_SEGMENTS, mq, scsi_sg_free);
}

static int scsi_alloc_sgtable(struct scsi_data_buffer *sdb, int nents,
			      gfp_t gfp_mask, bool mq)
{
	struct scatterlist *first_chunk = NULL;
	int ret;

	BUG_ON(!nents);

	if (mq) {
		if (nents <= SCSI_MAX_SG_SEGMENTS) {
			sdb->table.nents = nents;
			sg_init_table(sdb->table.sgl, sdb->table.nents);
			return 0;
		}
		first_chunk = sdb->table.sgl;
	}

	ret = __sg_alloc_table(&sdb->table, nents, SCSI_MAX_SG_SEGMENTS,
			       first_chunk, gfp_mask, scsi_sg_alloc);
	if (unlikely(ret))
		scsi_free_sgtable(sdb, mq);
	return ret;
}

static void scsi_uninit_cmd(struct scsi_cmnd *cmd)
{
	if (cmd->request->cmd_type == REQ_TYPE_FS) {
		struct scsi_driver *drv = scsi_cmd_to_driver(cmd);

		if (drv->uninit_command)
			drv->uninit_command(cmd);
	}
}

static void scsi_mq_free_sgtables(struct scsi_cmnd *cmd)
{
	if (cmd->sdb.table.nents)
		scsi_free_sgtable(&cmd->sdb, true);
	if (cmd->request->next_rq && cmd->request->next_rq->special)
		scsi_free_sgtable(cmd->request->next_rq->special, true);
	if (scsi_prot_sg_count(cmd))
		scsi_free_sgtable(cmd->prot_sdb, true);
}

static void scsi_mq_uninit_cmd(struct scsi_cmnd *cmd)
{
	struct scsi_device *sdev = cmd->device;
	unsigned long flags;

	BUG_ON(list_empty(&cmd->list));

	scsi_mq_free_sgtables(cmd);
	scsi_uninit_cmd(cmd);

	spin_lock_irqsave(&sdev->list_lock, flags);
	list_del_init(&cmd->list);
	spin_unlock_irqrestore(&sdev->list_lock, flags);
}

/*
 * Function:    scsi_release_buffers()
 *
 * Purpose:     Free resources allocate for a scsi_command.
 *
 * Arguments:   cmd	- command that we are bailing.
 *
 * Lock status: Assumed that no lock is held upon entry.
 *
 * Returns:     Nothing
 *
 * Notes:       In the event that an upper level driver rejects a
 *		command, we must release resources allocated during
 *		the __init_io() function.  Primarily this would involve
 *		the scatter-gather table.
 */
static void scsi_release_buffers(struct scsi_cmnd *cmd)
{
	if (cmd->sdb.table.nents)
		scsi_free_sgtable(&cmd->sdb, false);

	memset(&cmd->sdb, 0, sizeof(cmd->sdb));

	if (scsi_prot_sg_count(cmd))
		scsi_free_sgtable(cmd->prot_sdb, false);
}

static void scsi_release_bidi_buffers(struct scsi_cmnd *cmd)
{
	struct scsi_data_buffer *bidi_sdb = cmd->request->next_rq->special;

	scsi_free_sgtable(bidi_sdb, false);
	kmem_cache_free(scsi_sdb_cache, bidi_sdb);
	cmd->request->next_rq->special = NULL;
}

static bool scsi_end_request(struct request *req, int error,
		unsigned int bytes, unsigned int bidi_bytes)
{
	struct scsi_cmnd *cmd = req->special;
	struct scsi_device *sdev = cmd->device;
	struct request_queue *q = sdev->request_queue;

#ifdef CONFIG_SCSI_SHRD_TEST0
	if(sdev->shrd_on)
		sdev_printk(KERN_INFO, sdev, "%s: start of ending\n", __func__);
#endif

	if (blk_update_request(req, error, bytes))
		return true;

	/* Bidi request must be completed as a whole */
	if (unlikely(bidi_bytes) &&
	    blk_update_request(req->next_rq, error, bidi_bytes))
		return true;

	if (blk_queue_add_random(q))
		add_disk_randomness(req->rq_disk);

	if (req->mq_ctx) {
		/*
		 * In the MQ case the command gets freed by __blk_mq_end_io,
		 * so we have to do all cleanup that depends on it earlier.
		 *
		 * We also can't kick the queues from irq context, so we
		 * will have to defer it to a workqueue.
		 */
		scsi_mq_uninit_cmd(cmd);

		__blk_mq_end_io(req, error);

		if (scsi_target(sdev)->single_lun ||
		    !list_empty(&sdev->host->starved_list))
			kblockd_schedule_work(&sdev->requeue_work);
		else
			blk_mq_start_stopped_hw_queues(q, true);

		put_device(&sdev->sdev_gendev);
	} else {
		unsigned long flags;

		if (bidi_bytes)
			scsi_release_bidi_buffers(cmd);

		spin_lock_irqsave(q->queue_lock, flags);

		blk_finish_request(req, error);
		spin_unlock_irqrestore(q->queue_lock, flags);

		scsi_release_buffers(cmd);
		scsi_next_command(cmd);
	}

#ifdef CONFIG_SCSI_SHRD_TEST0

	if(sdev->shrd_on){
		sdev_printk(KERN_ERR, sdev, "%s: finish end request\n", __func__);
	}
#endif

	return false;
}

/**
 * __scsi_error_from_host_byte - translate SCSI error code into errno
 * @cmd:	SCSI command (unused)
 * @result:	scsi error code
 *
 * Translate SCSI error code into standard UNIX errno.
 * Return values:
 * -ENOLINK	temporary transport failure
 * -EREMOTEIO	permanent target failure, do not retry
 * -EBADE	permanent nexus failure, retry on other path
 * -ENOSPC	No write space available
 * -ENODATA	Medium error
 * -EIO		unspecified I/O error
 */
static int __scsi_error_from_host_byte(struct scsi_cmnd *cmd, int result)
{
	int error = 0;

	switch(host_byte(result)) {
	case DID_TRANSPORT_FAILFAST:
		error = -ENOLINK;
		break;
	case DID_TARGET_FAILURE:
		set_host_byte(cmd, DID_OK);
		error = -EREMOTEIO;
		break;
	case DID_NEXUS_FAILURE:
		set_host_byte(cmd, DID_OK);
		error = -EBADE;
		break;
	case DID_ALLOC_FAILURE:
		set_host_byte(cmd, DID_OK);
		error = -ENOSPC;
		break;
	case DID_MEDIUM_ERROR:
		set_host_byte(cmd, DID_OK);
		error = -ENODATA;
		break;
	default:
		error = -EIO;
		break;
	}

	return error;
}

/*
 * Function:    scsi_io_completion()
 *
 * Purpose:     Completion processing for block device I/O requests.
 *
 * Arguments:   cmd   - command that is finished.
 *
 * Lock status: Assumed that no lock is held upon entry.
 *
 * Returns:     Nothing
 *
 * Notes:       We will finish off the specified number of sectors.  If we
 *		are done, the command block will be released and the queue
 *		function will be goosed.  If we are not done then we have to
 *		figure out what to do next:
 *
 *		a) We can call scsi_requeue_command().  The request
 *		   will be unprepared and put back on the queue.  Then
 *		   a new command will be created for it.  This should
 *		   be used if we made forward progress, or if we want
 *		   to switch from READ(10) to READ(6) for example.
 *
 *		b) We can call __scsi_queue_insert().  The request will
 *		   be put back on the queue and retried using the same
 *		   command as before, possibly after a delay.
 *
 *		c) We can call scsi_end_request() with -EIO to fail
 *		   the remainder of the request.
 */
void scsi_io_completion(struct scsi_cmnd *cmd, unsigned int good_bytes)
{
	int result = cmd->result;
	struct request_queue *q = cmd->device->request_queue;
	struct request *req = cmd->request;
	int error = 0;
	struct scsi_sense_hdr sshdr;
	int sense_valid = 0;
	int sense_deferred = 0;
	enum {ACTION_FAIL, ACTION_REPREP, ACTION_RETRY,
	      ACTION_DELAYED_RETRY} action;
	unsigned long wait_for = (cmd->allowed + 1) * req->timeout;

	if (result) {
		sense_valid = scsi_command_normalize_sense(cmd, &sshdr);
		if (sense_valid)
			sense_deferred = scsi_sense_is_deferred(&sshdr);
	}

	if (req->cmd_type == REQ_TYPE_BLOCK_PC) { /* SG_IO ioctl from block level */
		if (result) {
			if (sense_valid && req->sense) {
				/*
				 * SG_IO wants current and deferred errors
				 */
				int len = 8 + cmd->sense_buffer[7];

				if (len > SCSI_SENSE_BUFFERSIZE)
					len = SCSI_SENSE_BUFFERSIZE;
				memcpy(req->sense, cmd->sense_buffer,  len);
				req->sense_len = len;
			}
			if (!sense_deferred)
				error = __scsi_error_from_host_byte(cmd, result);
		}
		/*
		 * __scsi_error_from_host_byte may have reset the host_byte
		 */
		req->errors = cmd->result;

		req->resid_len = scsi_get_resid(cmd);

		if (scsi_bidi_cmnd(cmd)) {
			/*
			 * Bidi commands Must be complete as a whole,
			 * both sides at once.
			 */
			req->next_rq->resid_len = scsi_in(cmd)->resid;
			if (scsi_end_request(req, 0, blk_rq_bytes(req),
					blk_rq_bytes(req->next_rq)))
				BUG();
			return;
		}
	} else if (blk_rq_bytes(req) == 0 && result && !sense_deferred) {
		/*
		 * Certain non BLOCK_PC requests are commands that don't
		 * actually transfer anything (FLUSH), so cannot use
		 * good_bytes != blk_rq_bytes(req) as the signal for an error.
		 * This sets the error explicitly for the problem case.
		 */
		error = __scsi_error_from_host_byte(cmd, result);
	}

	/* no bidi support for !REQ_TYPE_BLOCK_PC yet */
	BUG_ON(blk_bidi_rq(req));

	/*
	 * Next deal with any sectors which we were able to correctly
	 * handle.
	 */
	SCSI_LOG_HLCOMPLETE(1, scmd_printk(KERN_INFO, cmd,
		"%u sectors total, %d bytes done.\n",
		blk_rq_sectors(req), good_bytes));

	/*
	 * Recovered errors need reporting, but they're always treated
	 * as success, so fiddle the result code here.  For BLOCK_PC
	 * we already took a copy of the original into rq->errors which
	 * is what gets returned to the user
	 */
	if (sense_valid && (sshdr.sense_key == RECOVERED_ERROR)) {
		/* if ATA PASS-THROUGH INFORMATION AVAILABLE skip
		 * print since caller wants ATA registers. Only occurs on
		 * SCSI ATA PASS_THROUGH commands when CK_COND=1
		 */
		if ((sshdr.asc == 0x0) && (sshdr.ascq == 0x1d))
			;
		else if (!(req->cmd_flags & REQ_QUIET))
			scsi_print_sense("", cmd);
		result = 0;
		/* BLOCK_PC may have set error */
		error = 0;
	}

	/*
	 * If we finished all bytes in the request we are done now.
	 */
	if (!scsi_end_request(req, error, good_bytes, 0))
		return;

	/*
	 * Kill remainder if no retrys.
	 */
	if (error && scsi_noretry_cmd(cmd)) {
		if (scsi_end_request(req, error, blk_rq_bytes(req), 0)){
			
			BUG();
		}
		return;
	}

	/*
	 * If there had been no error, but we have leftover bytes in the
	 * requeues just queue the command up again.
	 */
	if (result == 0)
		goto requeue;

	error = __scsi_error_from_host_byte(cmd, result);

	if (host_byte(result) == DID_RESET) {
		/* Third party bus reset or reset for error recovery
		 * reasons.  Just retry the command and see what
		 * happens.
		 */
		action = ACTION_RETRY;
	} else if (sense_valid && !sense_deferred) {
		switch (sshdr.sense_key) {
		case UNIT_ATTENTION:
			if (cmd->device->removable) {
				/* Detected disc change.  Set a bit
				 * and quietly refuse further access.
				 */
				cmd->device->changed = 1;
				action = ACTION_FAIL;
			} else {
				/* Must have been a power glitch, or a
				 * bus reset.  Could not have been a
				 * media change, so we just retry the
				 * command and see what happens.
				 */
				action = ACTION_RETRY;
			}
			break;
		case ILLEGAL_REQUEST:
			/* If we had an ILLEGAL REQUEST returned, then
			 * we may have performed an unsupported
			 * command.  The only thing this should be
			 * would be a ten byte read where only a six
			 * byte read was supported.  Also, on a system
			 * where READ CAPACITY failed, we may have
			 * read past the end of the disk.
			 */
			if ((cmd->device->use_10_for_rw &&
			    sshdr.asc == 0x20 && sshdr.ascq == 0x00) &&
			    (cmd->cmnd[0] == READ_10 ||
			     cmd->cmnd[0] == WRITE_10)) {
				/* This will issue a new 6-byte command. */
				cmd->device->use_10_for_rw = 0;
				action = ACTION_REPREP;
			} else if (sshdr.asc == 0x10) /* DIX */ {
				action = ACTION_FAIL;
				error = -EILSEQ;
			/* INVALID COMMAND OPCODE or INVALID FIELD IN CDB */
			} else if (sshdr.asc == 0x20 || sshdr.asc == 0x24) {
				action = ACTION_FAIL;
				error = -EREMOTEIO;
			} else
				action = ACTION_FAIL;
			break;
		case ABORTED_COMMAND:
			action = ACTION_FAIL;
			if (sshdr.asc == 0x10) /* DIF */
				error = -EILSEQ;
			break;
		case NOT_READY:
			/* If the device is in the process of becoming
			 * ready, or has a temporary blockage, retry.
			 */
			if (sshdr.asc == 0x04) {
				switch (sshdr.ascq) {
				case 0x01: /* becoming ready */
				case 0x04: /* format in progress */
				case 0x05: /* rebuild in progress */
				case 0x06: /* recalculation in progress */
				case 0x07: /* operation in progress */
				case 0x08: /* Long write in progress */
				case 0x09: /* self test in progress */
				case 0x14: /* space allocation in progress */
					action = ACTION_DELAYED_RETRY;
					break;
				default:
					action = ACTION_FAIL;
					break;
				}
			} else
				action = ACTION_FAIL;
			break;
		case VOLUME_OVERFLOW:
			/* See SSC3rXX or current. */
			action = ACTION_FAIL;
			break;
		default:
			action = ACTION_FAIL;
			break;
		}
	} else
		action = ACTION_FAIL;

	if (action != ACTION_FAIL &&
	    time_before(cmd->jiffies_at_alloc + wait_for, jiffies))
		action = ACTION_FAIL;

	switch (action) {
	case ACTION_FAIL:
		/* Give up and fail the remainder of the request */
		if (!(req->cmd_flags & REQ_QUIET)) {
			scsi_print_result(cmd);
			if (driver_byte(result) & DRIVER_SENSE)
				scsi_print_sense("", cmd);
			scsi_print_command(cmd);
		}
		if (!scsi_end_request(req, error, blk_rq_err_bytes(req), 0))
			return;
		/*FALLTHRU*/
	case ACTION_REPREP:
	requeue:
		/* Unprep the request and put it back at the head of the queue.
		 * A new command will be prepared and issued.
		 */
		if (q->mq_ops) {
			cmd->request->cmd_flags &= ~REQ_DONTPREP;
			scsi_mq_uninit_cmd(cmd);
			scsi_mq_requeue_cmd(cmd);
		} else {
			scsi_release_buffers(cmd);
			scsi_requeue_command(q, cmd);
		}
		break;
	case ACTION_RETRY:
		/* Retry the same command immediately */
		__scsi_queue_insert(cmd, SCSI_MLQUEUE_EH_RETRY, 0);
		break;
	case ACTION_DELAYED_RETRY:
		/* Retry the same command after a delay */
		__scsi_queue_insert(cmd, SCSI_MLQUEUE_DEVICE_BUSY, 0);
		break;
	}
}

static int scsi_init_sgtable(struct request *req, struct scsi_data_buffer *sdb,
			     gfp_t gfp_mask)
{
	int count;

	/*
	 * If sg table allocation fails, requeue request later.
	 */
	if (unlikely(scsi_alloc_sgtable(sdb, req->nr_phys_segments,
					gfp_mask, req->mq_ctx != NULL)))
		return BLKPREP_DEFER;

	/* 
	 * Next, walk the list, and fill in the addresses and sizes of
	 * each segment.
	 */
	count = blk_rq_map_sg(req->q, req, sdb->table.sgl);
	BUG_ON(count > sdb->table.nents);
	sdb->table.nents = count;
	sdb->length = blk_rq_bytes(req);

#ifdef CONFIG_SCSI_SHRD_TEST0
	if(((struct scsi_device *)req->q->queuedata)->shrd_on)
		sdev_printk(KERN_INFO, (struct scsi_device *)req->q->queuedata, "%s: nents %d, sectors %d\n", __func__, count, blk_rq_sectors(req));
#endif

	return BLKPREP_OK;
}

/*
 * Function:    scsi_init_io()
 *
 * Purpose:     SCSI I/O initialize function.
 *
 * Arguments:   cmd   - Command descriptor we wish to initialize
 *
 * Returns:     0 on success
 *		BLKPREP_DEFER if the failure is retryable
 *		BLKPREP_KILL if the failure is fatal
 */
int scsi_init_io(struct scsi_cmnd *cmd, gfp_t gfp_mask)
{
	struct scsi_device *sdev = cmd->device;
	struct request *rq = cmd->request;
	bool is_mq = (rq->mq_ctx != NULL);
	int error;

	BUG_ON(!rq->nr_phys_segments);

	error = scsi_init_sgtable(rq, &cmd->sdb, gfp_mask);
	if (error)
		goto err_exit;

	if (blk_bidi_rq(rq)) {
		if (!rq->q->mq_ops) {
			struct scsi_data_buffer *bidi_sdb =
				kmem_cache_zalloc(scsi_sdb_cache, GFP_ATOMIC);
			if (!bidi_sdb) {
				error = BLKPREP_DEFER;
				goto err_exit;
			}

			rq->next_rq->special = bidi_sdb;
		}

		error = scsi_init_sgtable(rq->next_rq, rq->next_rq->special,
					  GFP_ATOMIC);
		if (error)
			goto err_exit;
	}

	if (blk_integrity_rq(rq)) {
		struct scsi_data_buffer *prot_sdb = cmd->prot_sdb;
		int ivecs, count;

		BUG_ON(prot_sdb == NULL);
		ivecs = blk_rq_count_integrity_sg(rq->q, rq->bio);

		if (scsi_alloc_sgtable(prot_sdb, ivecs, gfp_mask, is_mq)) {
			error = BLKPREP_DEFER;
			goto err_exit;
		}

		count = blk_rq_map_integrity_sg(rq->q, rq->bio,
						prot_sdb->table.sgl);
		BUG_ON(unlikely(count > ivecs));
		BUG_ON(unlikely(count > queue_max_integrity_segments(rq->q)));

		cmd->prot_sdb = prot_sdb;
		cmd->prot_sdb->table.nents = count;
	}

	return BLKPREP_OK;
err_exit:
	if (is_mq) {
		scsi_mq_free_sgtables(cmd);
	} else {
		scsi_release_buffers(cmd);
		cmd->request->special = NULL;
		scsi_put_command(cmd);
		put_device(&sdev->sdev_gendev);
	}
	return error;
}
EXPORT_SYMBOL(scsi_init_io);

static struct scsi_cmnd *scsi_get_cmd_from_req(struct scsi_device *sdev,
		struct request *req)
{
	struct scsi_cmnd *cmd;

	if (!req->special) {
		/* Bail if we can't get a reference to the device */
		if (!get_device(&sdev->sdev_gendev))
			return NULL;
		
		//at the first access, cmd structure is allocated for the req
		cmd = scsi_get_command(sdev, GFP_ATOMIC);
		if (unlikely(!cmd)) {
			put_device(&sdev->sdev_gendev);
			return NULL;
		}
		req->special = cmd;
	} else {
		cmd = req->special;
	}

	/* pull a tag out of the request if we have one */
	cmd->tag = req->tag;
	cmd->request = req;

	cmd->cmnd = req->cmd;
	cmd->prot_op = SCSI_PROT_NORMAL;

	return cmd;
}

static int scsi_setup_blk_pc_cmnd(struct scsi_device *sdev, struct request *req)
{
	struct scsi_cmnd *cmd = req->special;

	/*
	 * BLOCK_PC requests may transfer data, in which case they must
	 * a bio attached to them.  Or they might contain a SCSI command
	 * that does not transfer data, in which case they may optionally
	 * submit a request without an attached bio.
	 */
	if (req->bio) {
		int ret = scsi_init_io(cmd, GFP_ATOMIC);
		if (unlikely(ret))
			return ret;
	} else {
		BUG_ON(blk_rq_bytes(req));

		memset(&cmd->sdb, 0, sizeof(cmd->sdb));
	}

	cmd->cmd_len = req->cmd_len;
	cmd->transfersize = blk_rq_bytes(req);
	cmd->allowed = req->retries;
	return BLKPREP_OK;
}

/*
 * Setup a REQ_TYPE_FS command.  These are simple request from filesystems
 * that still need to be translated to SCSI CDBs from the ULD.
 */
static int scsi_setup_fs_cmnd(struct scsi_device *sdev, struct request *req)
{
	struct scsi_cmnd *cmd = req->special;

	if (unlikely(sdev->scsi_dh_data && sdev->scsi_dh_data->scsi_dh
			 && sdev->scsi_dh_data->scsi_dh->prep_fn)) {
		int ret = sdev->scsi_dh_data->scsi_dh->prep_fn(sdev, req);
		if (ret != BLKPREP_OK)
			return ret;
	}

	memset(cmd->cmnd, 0, BLK_MAX_CDB);
	return scsi_cmd_to_driver(cmd)->init_command(cmd);
}

static int scsi_setup_cmnd(struct scsi_device *sdev, struct request *req)
{
	struct scsi_cmnd *cmd = req->special;

	if (!blk_rq_bytes(req))
		cmd->sc_data_direction = DMA_NONE;
	else if (rq_data_dir(req) == WRITE)
		cmd->sc_data_direction = DMA_TO_DEVICE;
	else
		cmd->sc_data_direction = DMA_FROM_DEVICE;

	switch (req->cmd_type) {
	case REQ_TYPE_FS:
		return scsi_setup_fs_cmnd(sdev, req);
	case REQ_TYPE_BLOCK_PC:
		return scsi_setup_blk_pc_cmnd(sdev, req);
	default:
		return BLKPREP_KILL;
	}
}

static int
scsi_prep_state_check(struct scsi_device *sdev, struct request *req)
{
	int ret = BLKPREP_OK;

	/*
	 * If the device is not in running state we will reject some
	 * or all commands.
	 */
	if (unlikely(sdev->sdev_state != SDEV_RUNNING)) {
		switch (sdev->sdev_state) {
		case SDEV_OFFLINE:
		case SDEV_TRANSPORT_OFFLINE:
			/*
			 * If the device is offline we refuse to process any
			 * commands.  The device must be brought online
			 * before trying any recovery commands.
			 */
			sdev_printk(KERN_ERR, sdev,
				    "rejecting I/O to offline device\n");
			ret = BLKPREP_KILL;
			break;
		case SDEV_DEL:
			/*
			 * If the device is fully deleted, we refuse to
			 * process any commands as well.
			 */
			sdev_printk(KERN_ERR, sdev,
				    "rejecting I/O to dead device\n");
			ret = BLKPREP_KILL;
			break;
		case SDEV_QUIESCE:
		case SDEV_BLOCK:
		case SDEV_CREATED_BLOCK:
			/*
			 * If the devices is blocked we defer normal commands.
			 */
			if (!(req->cmd_flags & REQ_PREEMPT))
				ret = BLKPREP_DEFER;
			break;
		default:
			/*
			 * For any other not fully online state we only allow
			 * special commands.  In particular any user initiated
			 * command is not allowed.
			 */
			if (!(req->cmd_flags & REQ_PREEMPT))
				ret = BLKPREP_KILL;
			break;
		}
	}
	return ret;
}

static int
scsi_prep_return(struct request_queue *q, struct request *req, int ret)
{
	struct scsi_device *sdev = q->queuedata;

	switch (ret) {
	case BLKPREP_KILL:
		req->errors = DID_NO_CONNECT << 16;
		/* release the command and kill it */
		if (req->special) {
			struct scsi_cmnd *cmd = req->special;
			scsi_release_buffers(cmd);
			scsi_put_command(cmd);
			put_device(&sdev->sdev_gendev);
			req->special = NULL;
		}
		break;
	case BLKPREP_DEFER:
		/*
		 * If we defer, the blk_peek_request() returns NULL, but the
		 * queue must be restarted, so we schedule a callback to happen
		 * shortly.
		 */
		if (atomic_read(&sdev->device_busy) == 0)
			blk_delay_queue(q, SCSI_QUEUE_DELAY);
		break;
	default:
		req->cmd_flags |= REQ_DONTPREP;
	}

	return ret;
}

static int scsi_prep_fn(struct request_queue *q, struct request *req)
{
	struct scsi_device *sdev = q->queuedata;
	struct scsi_cmnd *cmd;
	int ret;

	ret = scsi_prep_state_check(sdev, req);
	if (ret != BLKPREP_OK)
		goto out;

	cmd = scsi_get_cmd_from_req(sdev, req);
	if (unlikely(!cmd)) {
		ret = BLKPREP_DEFER;
		goto out;
	}

	ret = scsi_setup_cmnd(sdev, req);
out:
	return scsi_prep_return(q, req, ret);
}

static void scsi_unprep_fn(struct request_queue *q, struct request *req)
{
	scsi_uninit_cmd(req->special);
}

/*
 * scsi_dev_queue_ready: if we can send requests to sdev, return 1 else
 * return 0.
 *
 * Called with the queue_lock held.
 */
static inline int scsi_dev_queue_ready(struct request_queue *q,
				  struct scsi_device *sdev)
{
	unsigned int busy;

	busy = atomic_inc_return(&sdev->device_busy) - 1;
	if (atomic_read(&sdev->device_blocked)) {
		if (busy)
			goto out_dec;

		/*
		 * unblock after device_blocked iterates to zero
		 */
		if (atomic_dec_return(&sdev->device_blocked) > 0) {
			/*
			 * For the MQ case we take care of this in the caller.
			 */
			if (!q->mq_ops)
				blk_delay_queue(q, SCSI_QUEUE_DELAY);
			goto out_dec;
		}
		SCSI_LOG_MLQUEUE(3, sdev_printk(KERN_INFO, sdev,
				   "unblocking device at zero depth\n"));
	}

	if (busy >= sdev->queue_depth)
		goto out_dec;

	return 1;
out_dec:
	atomic_dec(&sdev->device_busy);
	return 0;
}

/*
 * scsi_target_queue_ready: checks if there we can send commands to target
 * @sdev: scsi device on starget to check.
 */
static inline int scsi_target_queue_ready(struct Scsi_Host *shost,
					   struct scsi_device *sdev)
{
	struct scsi_target *starget = scsi_target(sdev);
	unsigned int busy;

	if (starget->single_lun) {
		spin_lock_irq(shost->host_lock);
		if (starget->starget_sdev_user &&
		    starget->starget_sdev_user != sdev) {
			spin_unlock_irq(shost->host_lock);
			return 0;
		}
		starget->starget_sdev_user = sdev;
		spin_unlock_irq(shost->host_lock);
	}

	if (starget->can_queue <= 0)
		return 1;

	busy = atomic_inc_return(&starget->target_busy) - 1;
	if (atomic_read(&starget->target_blocked) > 0) {
		if (busy)
			goto starved;

		/*
		 * unblock after target_blocked iterates to zero
		 */
		if (atomic_dec_return(&starget->target_blocked) > 0)
			goto out_dec;

		SCSI_LOG_MLQUEUE(3, starget_printk(KERN_INFO, starget,
				 "unblocking target at zero depth\n"));
	}

	if (busy >= starget->can_queue)
		goto starved;

	return 1;

starved:
	spin_lock_irq(shost->host_lock);
	list_move_tail(&sdev->starved_entry, &shost->starved_list);
	spin_unlock_irq(shost->host_lock);
out_dec:
	if (starget->can_queue > 0)
		atomic_dec(&starget->target_busy);
	return 0;
}

/*
 * scsi_host_queue_ready: if we can send requests to shost, return 1 else
 * return 0. We must end up running the queue again whenever 0 is
 * returned, else IO can hang.
 */
static inline int scsi_host_queue_ready(struct request_queue *q,
				   struct Scsi_Host *shost,
				   struct scsi_device *sdev)
{
	unsigned int busy;

	if (scsi_host_in_recovery(shost))
		return 0;

	busy = atomic_inc_return(&shost->host_busy) - 1;
	if (atomic_read(&shost->host_blocked) > 0) {
		if (busy)
			goto starved;

		/*
		 * unblock after host_blocked iterates to zero
		 */
		if (atomic_dec_return(&shost->host_blocked) > 0)
			goto out_dec;

		SCSI_LOG_MLQUEUE(3,
			shost_printk(KERN_INFO, shost,
				     "unblocking host at zero depth\n"));
	}

	if (shost->can_queue > 0 && busy >= shost->can_queue)
		goto starved;
	if (shost->host_self_blocked)
		goto starved;

	/* We're OK to process the command, so we can't be starved */
	if (!list_empty(&sdev->starved_entry)) {
		spin_lock_irq(shost->host_lock);
		if (!list_empty(&sdev->starved_entry))
			list_del_init(&sdev->starved_entry);
		spin_unlock_irq(shost->host_lock);
	}

	return 1;

starved:
	spin_lock_irq(shost->host_lock);
	if (list_empty(&sdev->starved_entry))
		list_add_tail(&sdev->starved_entry, &shost->starved_list);
	spin_unlock_irq(shost->host_lock);
out_dec:
	atomic_dec(&shost->host_busy);
	return 0;
}

/*
 * Busy state exporting function for request stacking drivers.
 *
 * For efficiency, no lock is taken to check the busy state of
 * shost/starget/sdev, since the returned value is not guaranteed and
 * may be changed after request stacking drivers call the function,
 * regardless of taking lock or not.
 *
 * When scsi can't dispatch I/Os anymore and needs to kill I/Os scsi
 * needs to return 'not busy'. Otherwise, request stacking drivers
 * may hold requests forever.
 */
static int scsi_lld_busy(struct request_queue *q)
{
	struct scsi_device *sdev = q->queuedata;
	struct Scsi_Host *shost;

	if (blk_queue_dying(q))
		return 0;

	shost = sdev->host;

	/*
	 * Ignore host/starget busy state.
	 * Since block layer does not have a concept of fairness across
	 * multiple queues, congestion of host/starget needs to be handled
	 * in SCSI layer.
	 */
	if (scsi_host_in_recovery(shost) || scsi_device_is_busy(sdev))
		return 1;

	return 0;
}

/*
 * Kill a request for a dead device
 */
static void scsi_kill_request(struct request *req, struct request_queue *q)
{
	struct scsi_cmnd *cmd = req->special;
	struct scsi_device *sdev;
	struct scsi_target *starget;
	struct Scsi_Host *shost;

	blk_start_request(req);

	scmd_printk(KERN_INFO, cmd, "killing request\n");

	sdev = cmd->device;
	starget = scsi_target(sdev);
	shost = sdev->host;
	scsi_init_cmd_errh(cmd);
	cmd->result = DID_NO_CONNECT << 16;
	atomic_inc(&cmd->device->iorequest_cnt);

	/*
	 * SCSI request completion path will do scsi_device_unbusy(),
	 * bump busy counts.  To bump the counters, we need to dance
	 * with the locks as normal issue path does.
	 */
	atomic_inc(&sdev->device_busy);
	atomic_inc(&shost->host_busy);
	if (starget->can_queue > 0)
		atomic_inc(&starget->target_busy);

	blk_complete_request(req);
}

static void scsi_softirq_done(struct request *rq)
{
	struct scsi_cmnd *cmd = rq->special;
	unsigned long wait_for = (cmd->allowed + 1) * rq->timeout;
	int disposition;

	INIT_LIST_HEAD(&cmd->eh_entry);

	atomic_inc(&cmd->device->iodone_cnt);
	if (cmd->result)
		atomic_inc(&cmd->device->ioerr_cnt);

	disposition = scsi_decide_disposition(cmd);
	if (disposition != SUCCESS &&
	    time_before(cmd->jiffies_at_alloc + wait_for, jiffies)) {
		sdev_printk(KERN_ERR, cmd->device,
			    "timing out command, waited %lus\n",
			    wait_for/HZ);
		disposition = SUCCESS;
	}

	scsi_log_completion(cmd, disposition);

	switch (disposition) {
		case SUCCESS:
			scsi_finish_command(cmd);
			break;
		case NEEDS_RETRY:
			scsi_queue_insert(cmd, SCSI_MLQUEUE_EH_RETRY);
			break;
		case ADD_TO_MLQUEUE:
			scsi_queue_insert(cmd, SCSI_MLQUEUE_DEVICE_BUSY);
			break;
		default:
			if (!scsi_eh_scmd_add(cmd, 0))
				scsi_finish_command(cmd);
	}
}

/**
 * scsi_done - Invoke completion on finished SCSI command.
 * @cmd: The SCSI Command for which a low-level device driver (LLDD) gives
 * ownership back to SCSI Core -- i.e. the LLDD has finished with it.
 *
 * Description: This function is the mid-level's (SCSI Core) interrupt routine,
 * which regains ownership of the SCSI command (de facto) from a LLDD, and
 * calls blk_complete_request() for further processing.
 *
 * This function is interrupt context safe.
 */
static void scsi_done(struct scsi_cmnd *cmd)
{
	trace_scsi_dispatch_cmd_done(cmd);
	blk_complete_request(cmd->request);
}

//SHRD component
#ifdef CONFIG_SCSI_SHRD_TEST0

struct SHRD_MAP * scsi_shrd_map_search(struct rb_root *root, u32 addr){

	struct rb_node * node = root->rb_node;

	while (node){
		struct SHRD_MAP *map_entry  = container_of(node, struct SHRD_MAP, node);
		int result = addr - map_entry->o_addr;

		if(result < 0)
			node = node->rb_left;
		else if(result > 0)
			node = node->rb_right;
		else
			return map_entry;
	}
	return NULL;
}

int scsi_shrd_map_insert(struct rb_root *root, struct SHRD_MAP *map_entry){

	struct rb_node **new = &(root->rb_node), *parent = NULL;


	while (*new){
		struct SHRD_MAP *this = container_of(*new, struct SHRD_MAP, node);

		int result = map_entry->o_addr - this->o_addr;

		parent = *new;
		if(result < 0)
			new = &((*new)->rb_left);
		else if (result > 0)
			new = &((*new)->rb_right);
		else{
			//it means overwrite for the corresponding o_addr, so delete old map, invalid it, and insert new map.
			rb_erase(&this->node, root);
			memset(this, 0x00, sizeof(struct SHRD_MAP));
			scsi_shrd_map_insert(root, map_entry);
			
			return 1;
		}
	}

	rb_link_node(&map_entry->node, parent, new);
	rb_insert_color(&map_entry->node, root);

	return 1;
	
}

void scsi_shrd_map_remove(u32 oaddr, struct rb_root *tree){

	struct SHRD_MAP *map_entry = scsi_shrd_map_search(tree, oaddr);
	if(map_entry){
		rb_erase(&map_entry->node, tree);
		memset(map_entry, 0x00, sizeof(struct SHRD_MAP));
	}
}

static void scsi_shrd_bd_init(struct request_queue *q){

	struct scsi_device *sdev = q->queuedata;
	struct SHRD *shrd = sdev->shrd;

	shrd->bdev = lookup_bdev(DEV_PATH);
	if(IS_ERR(shrd->bdev)){
		sdev_printk(KERN_ERR, sdev, "%s: bdev lookup error\n", __func__);
		BUG();
		return;
	}

	if(!bdget(shrd->bdev->bd_dev)){
		sdev_printk(KERN_ERR, sdev, "%s: bdget error\n", __func__);
		BUG();
		return;
	}

	if(blkdev_get(shrd->bdev, FMODE_READ | FMODE_WRITE | FMODE_EXCL, shrd)){
		sdev_printk(KERN_ERR, sdev, "%s: blkdev_get error\n", __func__);
		bdput(shrd->bdev);
		BUG();
		return;
	}
	
}

u32 scsi_shrd_init(struct request_queue *q){

	struct scsi_device *sdev = q->queuedata;
	int idx, rtn = 0;
	sdev->shrd = NULL;

	sdev_printk(KERN_INFO, sdev, "%s: SHRD start shrd_init\n", __func__);


	q->prep_rq_fn = NULL; //in SHRD, prep is saperate from peek.
	
	sdev->shrd = (struct SHRD *)kmalloc(sizeof(struct SHRD), GFP_KERNEL);
	if(sdev->shrd == NULL){
		rtn = 1;
		goto fin;
		printk("SHRD::kmalloc failed on sdev->shrd\n");
	}
	
	memset(sdev->shrd, 0x00, sizeof(struct SHRD));
	
	sdev->shrd->shrd_rw_map = (struct SHRD_MAP *)kmalloc(sizeof(struct SHRD_MAP) * SHRD_RW_LOG_SIZE_IN_PAGE, GFP_KERNEL);
	if(sdev->shrd->shrd_rw_map == NULL){
		rtn = 1;
		kfree(sdev->shrd);
		printk("SHRD::kmalloc failed on sdev->shrd->shrd_rw_map\n");
		goto fin;
	}

	memset(sdev->shrd->shrd_rw_map, 0x00, sizeof(struct SHRD_MAP) * SHRD_RW_LOG_SIZE_IN_PAGE);
	
	sdev->shrd->shrd_jn_map = (struct SHRD_MAP *)kmalloc(sizeof(struct SHRD_MAP) * SHRD_JN_LOG_SIZE_IN_PAGE, GFP_KERNEL);
	if(sdev->shrd->shrd_jn_map == NULL){
		rtn = 1;
		kfree(sdev->shrd->shrd_rw_map);
		kfree(sdev->shrd);
		printk("SHRD::kmalloc failed on sdev->shrd->shrd_jn_map\n");
		goto fin;
	}

	memset(sdev->shrd->shrd_jn_map, 0x00, sizeof(struct SHRD_MAP) * SHRD_JN_LOG_SIZE_IN_PAGE);

	//WARNING::we need to change this. twrite_header should be allocated separatedly from twrite_cmd (as buddy, alloc_page)
	sdev->shrd->twrite_cmd = (struct SHRD_TWRITE *)kmalloc(sizeof(struct SHRD_TWRITE) * SHRD_TWRITE_ENTRIES, GFP_KERNEL);
	if(sdev->shrd->twrite_cmd == NULL){
		rtn = 1;
		kfree(sdev->shrd->shrd_rw_map);
		kfree(sdev->shrd->shrd_jn_map);
		kfree(sdev->shrd);
		printk("SHRD::kmalloc failed on sdev->shrd->twrite_cmd\n");
		goto fin;
	}

	for(idx = 0; idx < SHRD_TWRITE_ENTRIES; idx++){
		sdev->shrd->twrite_cmd[idx].twrite_hdr = (struct SHRD_TWRITE_HEADER *) alloc_pages(GFP_KERNEL, 0);
		//test
		sdev->shrd->twrite_cmd[idx].header =  bio_kmalloc(GFP_KERNEL, SHRD_NUM_CORES);
		sdev->shrd->twrite_cmd[idx].data =  bio_kmalloc(GFP_KERNEL, SHRD_NUM_MAX_TWRITE_ENTRY);
	
		//
		sdev->shrd->twrite_cmd[idx].header_rq = (struct request *)kmalloc(sizeof(struct request), GFP_KERNEL);
		sdev->shrd->twrite_cmd[idx].data_rq = (struct request *)kmalloc(sizeof(struct request), GFP_KERNEL);
		if(sdev->shrd->twrite_cmd[idx].twrite_hdr == NULL){
			printk("SHRD:: alloc failed during 4KB twrite header, what should i do?\n");
		}
	}

	sdev->shrd->remap_cmd = (struct SHRD_REMAP *)kmalloc(sizeof(struct SHRD_REMAP) * SHRD_REMAP_ENTRIES, GFP_KERNEL);
	if(sdev->shrd->remap_cmd == NULL){
		rtn = 1;
		kfree(sdev->shrd->shrd_rw_map);
		kfree(sdev->shrd->shrd_jn_map);
		kfree(sdev->shrd->twrite_cmd);
		kfree(sdev->shrd);
		
		printk("SHRD::kmalloc failed on sdev->shrd->remap_cmd\n");
		goto fin;
	}

	for(idx = 0; idx < SHRD_REMAP_ENTRIES; idx++){
		int iter;

		sdev->shrd->remap_cmd[idx].req = (struct request *)kmalloc(sizeof(struct request), GFP_KERNEL);
		for(iter = 0; iter < SHRD_REMAP_DATA_PAGE; iter++){
			sdev->shrd->remap_cmd[idx].remap_data[iter] = (struct SHRD_REMAP_DATA *) alloc_pages(GFP_KERNEL, 0);
			if(sdev->shrd->remap_cmd[idx].remap_data[iter] == NULL){
				printk("SHRD:: alloc failed during 4KB remap data, what should i do?\n");
			}
		}
	}
	
	// init spin_lock structure for log idx lock.
	spin_lock_init(&sdev->shrd->__rw_log_lock);
	spin_lock_init(&sdev->shrd->__jn_log_lock);

	sdev->shrd->rw_log_lock = &sdev->shrd->__rw_log_lock;
	sdev->shrd->jn_log_lock = &sdev->shrd->__jn_log_lock;

	//need to link all twrite and remap cmd into free_xx_cmd_list
	//in here.
	spin_lock_irq(sdev->shrd->rw_log_lock);
	INIT_LIST_HEAD(&sdev->shrd->free_twrite_cmd_list);

	for(idx= 0; idx < SHRD_TWRITE_ENTRIES; idx++){
		INIT_LIST_HEAD(&sdev->shrd->twrite_cmd[idx].req_list);
		sdev->shrd->twrite_cmd[idx].entry_num = idx;
		list_add_tail(&sdev->shrd->twrite_cmd[idx].twrite_cmd_list, &sdev->shrd->free_twrite_cmd_list);
	}
	spin_unlock_irq(sdev->shrd->rw_log_lock);

	spin_lock_irq(sdev->shrd->jn_log_lock);
	INIT_LIST_HEAD(&sdev->shrd->free_remap_cmd_list);

	for(idx=0; idx<SHRD_REMAP_ENTRIES; idx++){
		sdev->shrd->remap_cmd[idx].entry_num = idx;
		list_add_tail(&sdev->shrd->remap_cmd[idx].remap_cmd_list, &sdev->shrd->free_remap_cmd_list);
	}
	spin_unlock_irq(sdev->shrd->jn_log_lock);

	scsi_shrd_bd_init(q);

fin:
	return rtn;
}

static void scsi_shrd_request_init(struct request_queue *q, struct request *rq){

	memset(rq, 0, sizeof(*rq));

	INIT_LIST_HEAD(&rq->queuelist);
	INIT_LIST_HEAD(&rq->timeout_list);
	rq->cpu = -1;
	rq->q = q;
	rq->__sector = (sector_t) -1;
	INIT_HLIST_NODE(&rq->hash);
	RB_CLEAR_NODE(&rq->rb_node);
	rq->cmd = rq->__cmd;
	rq->cmd_len = BLK_MAX_CDB;
	rq->tag = -1;
	rq->start_time = jiffies;
	set_start_time_ns(rq);
	rq->part = NULL;

}

static void scsi_shrd_blk_queue_bio(struct request_queue *q, struct bio *bio, struct request *req){
	
	const bool sync = !!(bio->bi_rw & REQ_SYNC);
	int rw_flags, where = ELEVATOR_INSERT_FRONT;
	//struct request *req;

	printk("%s: start blk_queue_bio\n", __func__);

	blk_queue_bounce(q, &bio);

	rw_flags = bio_data_dir(bio);
	if (sync)
		rw_flags |= REQ_SYNC;
	 
	//req = get_request(q, rw_flags, bio, GFP_ATOMIC);
	//req = kmalloc(sizeof(struct request), GFP_ATOMIC);

	//blk_rq_init
	memset(req, 0, sizeof(*req));

	INIT_LIST_HEAD(&req->queuelist);
	INIT_LIST_HEAD(&req->timeout_list);
	req->cpu = -1;
	req->q = q;
	req->__sector = (sector_t) -1;
	INIT_HLIST_NODE(&req->hash);
	RB_CLEAR_NODE(&req->rb_node);
	req->cmd = req->__cmd;
	req->cmd_len = BLK_MAX_CDB;
	req->tag = -1;
	req->start_time = jiffies;
	set_start_time_ns(req);
	req->part = NULL;
	//
	
	if (unlikely(!req)) {
		printk("%s: get_request failed, BUG\n", __func__);
		BUG();
		panic("%s: get_request failed", __func__);
		bio_endio(bio, -ENODEV);	// @q is dead 
		return;
	}
	

	/*
	 * After dropping the lock and possibly sleeping here, our request
	 * may now be mergeable after it had proven unmergeable (above).
	 * We don't worry about that case for efficiency. It won't happen
	 * often, and the elevators are able to handle it.
	 */
	init_request_from_bio(req, bio);

	if (test_bit(QUEUE_FLAG_SAME_COMP, &q->queue_flags))
		req->cpu = raw_smp_processor_id();

	printk("%s: above blk_account_io_start\n", __func__);

	//spin_lock_irq(q->queue_lock); //it is correct?

	blk_account_io_start(req, true); 
	__elv_add_request(q, req, where);

	printk("%s: end blk_queue_bio\n", __func__);

}


/*
	Modified submit_bio() function (include generic_make_request, and blk_queue_bio.
	because shrd need to hold queue_lock and no need to make a new io_context and new run_queue
*/
static void scsi_shrd_submit_bio(int rw, struct bio *bio, struct request *req){

	struct request_queue *q = bdev_get_queue(bio->bi_bdev);

	printk("%s: start submit_bio\n", __func__);

	bio->bi_rw |= rw;

	/*
	 * If it's a regular read/write or a barrier with data attached,
	 * go through the normal accounting stuff before submission.
	 */
	if (bio_has_data(bio)) {
		unsigned int count;

		if (unlikely(rw & REQ_WRITE_SAME))
			count = bdev_logical_block_size(bio->bi_bdev) >> 9;
		else
			count = bio_sectors(bio);

		if (rw & WRITE) {
			count_vm_events(PGPGOUT, count);
		} else {
			task_io_account_read(bio->bi_iter.bi_size);
			count_vm_events(PGPGIN, count);
		}
	}

	scsi_shrd_blk_queue_bio(q, bio, req);
}

/*
	prep function for non_twrite request.

	This is same as usual prepare procedure in blk_peek_request, but we need to separate this from blk_peek_request
	because of packing twrite.
*/
static void scsi_shrd_prep_req(struct request_queue *q, struct request *rq){

	int ret;
	
	ret = scsi_prep_fn(q, rq);
	if (ret == BLKPREP_OK) {
		return;
	} else if (ret == BLKPREP_DEFER) {
		/*
		 * the request may have been (partially) prepped.
		 * we need to keep this request in the front to
		 * avoid resource deadlock.  REQ_STARTED will
		 * prevent other fs requests from passing this one.
		 */
		if (q->dma_drain_size && blk_rq_bytes(rq) &&
		    !(rq->cmd_flags & REQ_DONTPREP)) {
			/*
			 * remove the space for the drain we added
			 * so that we don't add it again
			 */
			--rq->nr_phys_segments;
		}

		rq = NULL;
		return;
	} else if (ret == BLKPREP_KILL) {
		rq->cmd_flags |= REQ_QUIET;
		/*
		 * Mark this request as started so we don't trigger
		 * any debug logic in the end I/O path.
		 */
		blk_start_request(rq);
		__blk_end_request_all(rq, -EIO);
	} else {
		printk(KERN_ERR "%s: bad return=%d\n", __func__, ret);
		return;
	}
}

/*
	Prep function for random write packing into twrite
*/
static struct SHRD_TWRITE * scsi_shrd_prep_rw_twrite(struct request_queue *q, struct request *rq){

	struct scsi_device *sdev = q->queuedata;
	struct SHRD *shrd = sdev->shrd;
	struct SHRD_TWRITE *twrite_entry = NULL;
	struct request *cur = rq, *next = NULL;
	bool put_back = true;
	u8 reqs = 0;
	u32 max_packed_rw = 0;
	u32 req_sectors = 0;
	u32 phys_segments = 0;
	u32 padding = 0;
	u32 log_addr;
	int ret;

	ret = scsi_prep_state_check(sdev, rq);
	if(ret != BLKPREP_OK){
		sdev_printk(KERN_INFO, sdev, "%s: nopack because of BLKPREP_OK\n", __func__);
		goto no_pack;
	}
	if(!(rq->cmd_flags & REQ_WRITE)){
		sdev_printk(KERN_INFO, sdev, "%s: nopack because of read request\n", __func__);
		goto no_pack;
	}
	if(rq->cmd_flags & REQ_DISCARD || rq->cmd_flags & REQ_FLUSH  || rq->cmd_flags & REQ_FUA){
		sdev_printk(KERN_INFO, sdev, "%s: nopack because of discard, flush,fua\n", __func__);
		goto no_pack;
	}
	if(rq->cmd_flags & REQ_WRITE_SAME){
		sdev_printk(KERN_INFO, sdev, "%s: nopack because of REQ_WRITE_SAME\n", __func__);
		goto no_pack;
	}
	if(blk_rq_sectors(rq) > SHRD_RW_THRESHOLD_IN_SECTOR){
		sdev_printk(KERN_INFO, sdev, "%s: nopack because of sequential write\n", __func__);
		goto no_pack;
	}
	if(blk_rq_sectors(rq) < SHRD_SECTORS_PER_PAGE){ //under 4KB write is not the common write situation.
		sdev_printk(KERN_INFO, sdev, "%s: nopack because of tiny requests\n", __func__);
		goto no_pack;
	}

	if(rq->rq_disk){
		shrd->rq_disk = rq->rq_disk;
	}
	if(rq->bio->bi_bdev){
		shrd->bdev = rq->bio->bi_bdev;
	}
	
	max_packed_rw = SHRD_MAX_TWRITE_IO_SIZE_IN_SECTOR;
	req_sectors += blk_rq_sectors(cur);
	phys_segments += cur->nr_phys_segments;

	//we need to get a address space for twrite (only for RW)
	//if there are not enough space (512KB), we can choose from two options. 1) packing small size, 2) move to log start addr
	//

	if(shrd->rw_log_new_idx >= shrd->rw_log_start_idx){
		//in here, rw log can goto log start when there are "enough space to allocate twrite".
		if((SHRD_RW_LOG_SIZE_IN_PAGE - shrd->rw_log_new_idx) >= SHRD_MIN_RW_LOGGING_IO_SIZE_IN_PAGE){
			//there are enough space in the tail of log. 
			max_packed_rw = (SHRD_RW_LOG_SIZE_IN_PAGE - shrd->rw_log_new_idx)  << 3; //page to sector
			if(max_packed_rw > SHRD_MAX_TWRITE_IO_SIZE_IN_SECTOR)
				max_packed_rw = SHRD_MAX_TWRITE_IO_SIZE_IN_SECTOR;
		}
		else{
			//we need to move the log start ptr into log front
			if(shrd->rw_log_start_idx >= SHRD_MAX_TWRITE_IO_SIZE_IN_SECTOR){
				shrd->rw_log_new_idx = 0; //goto log end because enoulgh slot is presented.
			}
			else{
				sdev_printk(KERN_INFO, sdev, "%s: nopack because log is full1\n", __func__);
				goto no_pack; //log is full, don't packed.
			}
		}
	}
	else{
		if((shrd->rw_log_start_idx - shrd->rw_log_new_idx) >= SHRD_MIN_RW_LOGGING_IO_SIZE_IN_PAGE){
			max_packed_rw = (shrd->rw_log_start_idx - shrd->rw_log_new_idx) <<3; //page to sector
			if(max_packed_rw > SHRD_MAX_TWRITE_IO_SIZE_IN_SECTOR)
				max_packed_rw = SHRD_MAX_TWRITE_IO_SIZE_IN_SECTOR;
		}
		else{
			sdev_printk(KERN_INFO, sdev, "%s: nopack because log is full2\n", __func__);
			goto no_pack;
		}
	}

	log_addr = shrd->rw_log_new_idx;
	
	//need to consider padding
	if((log_addr & 0x1) != ((blk_rq_pos(cur) / SHRD_SECTORS_PER_PAGE) & 0x1)){
		//not aligned, need to pad
		if(req_sectors + SHRD_SECTORS_PER_PAGE > max_packed_rw){
			req_sectors -= blk_rq_sectors(cur);
			sdev_printk(KERN_INFO, sdev, "%s: nopack because of padding\n", __func__);
			goto no_pack;
		}
		else{
			req_sectors += SHRD_SECTORS_PER_PAGE;
			phys_segments++;
			padding++;
			log_addr++;
		}
	}
	log_addr += blk_rq_sectors(cur) / SHRD_SECTORS_PER_PAGE;
	
	//we need to get a empty twrite cmd entry
	twrite_entry = shrd_get_twrite_entry(shrd);
	if(twrite_entry == NULL){
		sdev_printk(KERN_INFO, sdev, "%s: nopack because of NULL twrite\n", __func__);
		goto no_pack;
	}
	list_del(&twrite_entry->twrite_cmd_list);
	shrd_clear_twrite_entry(twrite_entry);

	blk_start_request(rq);
	
	//write request fetch start
	do{
		if(reqs >= SHRD_NUM_MAX_TWRITE_ENTRY - 1){
			put_back = false;
			break;
		}

		BUG_ON(!shrd->rq_disk);

		next = blk_fetch_request(q); //we need to start the request in order to get next request

		
		if(!next){
			sdev_printk(KERN_INFO, sdev, "%s: nopack because of noreq\n", __func__);
			put_back = false;
			break;
		}
		//test for rq_disk
		if(next->rq_disk != shrd->rq_disk){
			sdev_printk(KERN_ERR, sdev, "%s: next->rq_disk is not same as shrd->rq_disk\n", __func__); 
			BUG();
		}
		if(next->bio->bi_bdev != shrd->bdev){
			sdev_printk(KERN_ERR, sdev, "%s: next->bio->bi_bdev is not same as shrd->bdev\n", __func__);
			BUG();
		}

		if(next->cmd_flags & REQ_DISCARD || next->cmd_flags & REQ_FLUSH || next->cmd_flags & REQ_FUA){
			sdev_printk(KERN_INFO, sdev, "%s: nopack because of discard, flush, fua\n", __func__);
			break;
		}

		if(rq_data_dir(cur) != rq_data_dir(next)){
			sdev_printk(KERN_INFO, sdev, "%s: nopack because of read request\n", __func__);
			break;
		}

		if(blk_rq_sectors(next) > SHRD_RW_THRESHOLD_IN_SECTOR){
			sdev_printk(KERN_INFO, sdev, "%s: nopack because of large next\n", __func__);
			break;
		}
		
		if(blk_rq_sectors(next) < SHRD_SECTORS_PER_PAGE){ //under 4KB write is not the common write situation.
			sdev_printk(KERN_INFO, sdev, "%s: nopack because of tiny next requests\n", __func__);
			break;
		}

		req_sectors += blk_rq_sectors(next);
		phys_segments += next->nr_phys_segments;
		if(req_sectors > max_packed_rw){
			req_sectors -= blk_rq_sectors(next);
			phys_segments -= next->nr_phys_segments;
			sdev_printk(KERN_INFO, sdev, "%s: nopack because of max_packed_rw %d\n", __func__, max_packed_rw);
			break;
		}

		//need to consider padding
		if((log_addr & 0x1) != ((blk_rq_pos(next) / SHRD_SECTORS_PER_PAGE) & 0x1)){
			//not aligned, need to pad
			if(req_sectors + SHRD_SECTORS_PER_PAGE > max_packed_rw){
				req_sectors -= blk_rq_sectors(next);
				sdev_printk(KERN_INFO, sdev, "%s: nopack because of max_packed_rw %d\n", __func__, max_packed_rw);
				break;
			}
			else{
				req_sectors += SHRD_SECTORS_PER_PAGE;
				phys_segments++;
				padding++;
				log_addr++;
				
			}
		}
		log_addr += blk_rq_sectors(next) / SHRD_SECTORS_PER_PAGE;
		list_add_tail(&next->queuelist, &twrite_entry->req_list);
		cur = next;
		reqs++;
		
	}while(1);

	if(put_back)
		blk_requeue_request(q, next);

	if(reqs > 0){
		list_add(&rq->queuelist, &twrite_entry->req_list);
		twrite_entry->nr_requests = ++reqs;
		twrite_entry->blocks = req_sectors;
		twrite_entry->phys_segments = phys_segments;
	}
	else{
		sdev_printk(KERN_INFO, sdev, "%s: nopack because there are no requests to pack\n", __func__);
		shrd_put_twrite_entry(sdev->shrd, twrite_entry);
		blk_requeue_request(q, rq);
		twrite_entry = NULL;
		goto no_pack;
	}

	if(twrite_entry != NULL){
		sdev_printk(KERN_INFO, sdev, "%s: prep succeed, num entries %d, num blocks %d, num segments %d, num padding %d \n",
			__func__, twrite_entry->nr_requests, twrite_entry->blocks, twrite_entry->phys_segments, padding);
		return twrite_entry;
	}

no_pack:
	return twrite_entry;
}

static void scsi_shrd_setup_cmd(struct request *req, sector_t block, sector_t this_count, u8 direction){

	req->__sector = block;
	req->__data_len = this_count << 9;
	
	if (direction == WRITE) {
		req->cmd[0] = WRITE_6;

	} else if (direction == READ) {
		req->cmd[0] = READ_6;
	}
	req->cmd[0] += READ_10 - READ_6;
	req->cmd[1] = 0;
	req->cmd[2] = (unsigned char) (block >> 24) & 0xff;
	req->cmd[3] = (unsigned char) (block >> 16) & 0xff;
	req->cmd[4] = (unsigned char) (block >> 8) & 0xff;
	req->cmd[5] = (unsigned char) block & 0xff;
	req->cmd[6] = req->cmd[9] = 0;
	req->cmd[7] = (unsigned char) (this_count >> 8) & 0xff;
	req->cmd[8] = (unsigned char) this_count & 0xff;

	req->cmd_len = 10;
	
}

static void scsi_shrd_bio_endio(struct bio* bio, int err){

	//need to complete linked requests

	struct scsi_device *sdev = bio->bi_bdev->bd_queue->queuedata;
	struct SHRD *shrd = sdev->shrd;
	int ret;

	if(err)
		BUG();

	sdev_printk(KERN_INFO, sdev, "%d: %s: bio request is %s, bi_sector %u, bi_size %d, bio->bi_rw %X", smp_processor_id(), __func__, 
		(bio->bi_rw & REQ_SHRD_TWRITE_HDR) ? "twrite hdr": (bio->bi_rw & REQ_SHRD_TWRITE_DAT) ? "twrite data" : (bio->bi_rw & REQ_SHRD_REMAP) ? "remap" : "err",
		(u32)bio->bi_iter.bi_sector, bio->bi_iter.bi_size, (u32)bio->bi_rw);
	
	if(bio->bi_rw & REQ_SHRD_TWRITE_HDR){
		struct SHRD_TWRITE *twrite_entry = (struct SHRD_TWRITE *)bio->bi_private;
		sdev_printk(KERN_INFO, sdev, "%d: %s: twrite hdr completion: twrite: %llx, block: %d, nr_request: %d, phys_segment: %d\n", smp_processor_id(), __func__, (u64)twrite_entry, twrite_entry->blocks, twrite_entry->nr_requests, twrite_entry->phys_segments);
		BUG_ON(!twrite_entry);
	}
	else if(bio->bi_rw & REQ_SHRD_TWRITE_DAT){
		struct SHRD_TWRITE *twrite_entry;
		struct request *prq, *tmp;
		unsigned long flags;

		twrite_entry = (struct SHRD_TWRITE *)bio->bi_private;
		BUG_ON(!twrite_entry);

		sdev_printk(KERN_INFO, sdev, "%d: %s: twrite data completion: twrite block: %d, nr_request: %d, phys_segment: %d\n", smp_processor_id(), __func__, twrite_entry->blocks, twrite_entry->nr_requests, twrite_entry->phys_segments);

		list_for_each_entry_safe(prq, tmp, &twrite_entry->req_list, queuelist){
			if(!prq)
				BUG();
			if(!prq->bio)
				BUG();
			
			//sdev_printk(KERN_INFO, sdev, "%s: completion start, prq pos: %d, sectors: %d\n", __func__, blk_rq_pos(prq), blk_rq_sectors(prq));
			
			ret = blk_update_request(prq, 0, blk_rq_bytes(prq));
			BUG_ON(ret);

			list_del_init(&prq->queuelist);
			
			spin_lock_irqsave(prq->q->queue_lock, flags);
			blk_finish_request(prq, 0);
			spin_unlock_irqrestore(prq->q->queue_lock, flags);
			//sdev_printk(KERN_INFO, sdev, "%s: completion end, prq pos: %d, sectors: %d\n", __func__, blk_rq_pos(prq), blk_rq_sectors(prq));
		}

		shrd_put_twrite_entry(shrd, twrite_entry);
	}
	else if(bio->bi_rw & REQ_SHRD_REMAP){
		struct SHRD_REMAP *remap_entry = (struct SHRD_REMAP *)bio->bi_private;
		struct SHRD_REMAP_DATA *remap_data;
		u32 i;

		BUG_ON(!remap_entry);
		remap_data = remap_entry->remap_data[0];

		for(i = 0; i < remap_data->remap_count; i++){
			scsi_shrd_map_remove(remap_data->o_addr[i], &shrd->rw_mapping);
		}
		shrd->rw_log_start_idx = remap_data->t_addr_end + 1 - SHRD_RW_LOG_START_IN_PAGE;

		if(shrd->rw_log_start_idx >= SHRD_RW_LOG_SIZE_IN_PAGE)
			shrd->rw_log_start_idx -= SHRD_RW_LOG_SIZE_IN_PAGE;

		shrd_put_remap_entry(shrd, remap_entry);
	}

	bio_put(bio);
}

/*
	Make twrite command with request data.
*/
static struct bio* scsi_shrd_make_twrite_data_bio(struct request_queue *q, struct SHRD_TWRITE *twrite_entry){

	struct scsi_device *sdev = q->queuedata;
	struct SHRD *shrd = sdev->shrd;
	struct request *prq;
	struct bio *bio, *pbio;
	struct bio_vec bvec;
	struct bvec_iter iter;
	u32 len = 0;
	u32 start_addr = twrite_entry->twrite_hdr->t_addr_start;
	u32 idx = start_addr;

	bio = twrite_entry->data;
	//bio = bio_kmalloc(GFP_ATOMIC, SHRD_NUM_MAX_TWRITE_ENTRY);
	if(!bio){
		BUG();
		return NULL;
	}

	bio->bi_end_io = scsi_shrd_bio_endio;
	bio->bi_rw |= REQ_WRITE | REQ_SYNC |REQ_SHRD_TWRITE_DAT;
	bio->bi_iter.bi_sector = twrite_entry->twrite_hdr->t_addr_start << 3;
	bio->bi_bdev = shrd->bdev;
	bio->bi_private = twrite_entry;

	list_for_each_entry(prq, &twrite_entry->req_list, queuelist){
		if((idx & 0x1) != ((blk_rq_pos(prq) / SHRD_SECTORS_PER_PAGE) & 0x1)){
			//padding
			len = bio_add_page(bio, (struct page *)twrite_entry->twrite_hdr, PAGE_SIZE, 0);
			if(len < PAGE_SIZE){
				sdev_printk(KERN_INFO, sdev, "%s: bio_add_pc_page failed on dummy padding\n", __func__);
			}
			idx++;
		}
		pbio  = prq->bio;

		for_each_bio(pbio){
			bio_for_each_segment(bvec, pbio, iter){
				len = bio_add_page(bio, bvec.bv_page, bvec.bv_len, bvec.bv_offset);
				idx++;
				if(len < bvec.bv_len)
					sdev_printk(KERN_INFO, sdev, "%s: bio_add_pc_page failed on request packing\n", __func__);
			}
		}
	}
	
	return bio;
	
}

/*
	Make twrite command with header data.
*/
static struct bio *scsi_shrd_make_twrite_header_bio(struct request_queue *q, struct SHRD_TWRITE *twrite_entry){

	struct scsi_device *sdev = q->queuedata;
	struct SHRD *shrd = sdev->shrd;
	struct bio *bio;
	int i;

	bio = twrite_entry->header;
	//bio = bio_kmalloc(GFP_ATOMIC, SHRD_NUM_CORES);
	if(!bio)
		return NULL;

	bio->bi_iter.bi_sector = (twrite_entry->entry_num * SHRD_NUM_CORES + SHRD_CMD_START_IN_PAGE) * SHRD_SECTORS_PER_PAGE;
	bio->bi_end_io = scsi_shrd_bio_endio;
	bio->bi_rw |= REQ_WRITE | REQ_SYNC |REQ_SHRD_TWRITE_HDR;
	bio->bi_bdev = shrd->bdev;
	bio->bi_private = twrite_entry;

	for(i=0; i < SHRD_NUM_CORES; i++){
		if(bio_add_page(bio, (struct page *)twrite_entry->twrite_hdr, PAGE_SIZE, 0) < PAGE_SIZE){
			sdev_printk(KERN_INFO, sdev, "%s: SHRD bio_add_pc_page failed on CORE %d\n", __func__, i);
			return NULL;
		}
	}

	return bio;
}

/*
	make twrite header request and twrite data request
	and add requests into head of request_queue
	NOTE:: twrite header should be fetched first than twrite data reqeust, but two requests should be inserted into front, 
	thus, input twrite data first and then put twrite header using front insert.

	return: return twrite_header request (first handle)
*/
static void scsi_shrd_make_twrite_bios(struct request_queue *q, struct SHRD_TWRITE *twrite_entry){

	struct scsi_device *sdev = q->queuedata;
	struct bio *header, *data;

	sdev_printk(KERN_INFO, sdev, "%s start\n", __func__);

	header = scsi_shrd_make_twrite_header_bio(q, twrite_entry);
	if(!header){
		sdev_printk(KERN_ERR, sdev, "%s: header is NULL\n", __func__);
		return;
	}

	twrite_entry->header = header;
	
	data = scsi_shrd_make_twrite_data_bio(q, twrite_entry);
	if(!data){
		sdev_printk(KERN_ERR, sdev, "%s: data is NULL\n", __func__);
		BUG();
		return;
	}

	twrite_entry->data = data;

	sdev_printk(KERN_INFO, sdev, "%s: bio make complete, twrite_entry %llx, header bio %llx, bi_sector %u, bi_size %d, data bio %llx, bi_sector %d, bi_size %d\n", __func__
		, (u64)twrite_entry, (u64)header, header->bi_iter.bi_sector, header->bi_iter.bi_size, (u64)data, data->bi_iter.bi_sector, data->bi_iter.bi_size);

	//scsi_shrd_submit_bio(REQ_SYNC, data);
	//scsi_shrd_submit_bio(REQ_SYNC, header);

/*	
	spin_unlock_irq(q->queue_lock);
	submit_bio(REQ_SYNC, header);
	submit_bio(REQ_SYNC, data);   //need to check order of header and data.
	spin_lock_irq(q->queue_lock);

	*/
}

/*
	Packing twrite data with request list in twrite_entry
*/
static void scsi_shrd_packing_rw_twrite(struct request_queue *q, struct SHRD_TWRITE *twrite_entry){

	struct request *prq;
	struct scsi_device *sdev = q->queuedata;
	struct SHRD *shrd = sdev->shrd;
	struct SHRD_TWRITE_HEADER *header = twrite_entry->twrite_hdr;
	u32 idx = 0, sectors = twrite_entry->blocks, end;
	u32 cnt = 0;

	
	memset(header, 0x00, sizeof(struct SHRD_TWRITE_HEADER));

	//fill header with data

	//t_addr_start is determined as 
	header->t_addr_start = shrd->rw_log_new_idx + SHRD_RW_LOG_START_IN_PAGE;
	header->io_count = twrite_entry->blocks / SHRD_SECTORS_PER_PAGE;

	idx = shrd->rw_log_new_idx;

	end = shrd->rw_log_new_idx + (sectors >> 3);

	sdev_printk(KERN_INFO, sdev, "%s: packing start, twrite, block: %d, reqs:%d, seg:%d, idx: %d, end: %d \n", __func__, twrite_entry->blocks, twrite_entry->nr_requests, twrite_entry->phys_segments, idx, end);

	if(end >= SHRD_RW_LOG_SIZE_IN_PAGE)
		end -= SHRD_RW_LOG_SIZE_IN_PAGE;
	
	list_for_each_entry(prq, &twrite_entry->req_list, queuelist){

		int i;
		if(idx > end){
			sdev_printk(KERN_INFO, sdev, "%s: SHRD error on packing rw twrite, idx is larger than end, idx: %d, end: %d\n", __func__, idx, end);
			BUG();
		}
		
		if((idx & 0x1) != ((blk_rq_pos(prq) / SHRD_SECTORS_PER_PAGE) & 0x1)){
			//need padding
			
			header->o_addr[idx] = SHRD_INVALID_LPN;
			idx++;

			if(idx >= SHRD_RW_LOG_SIZE_IN_PAGE)
				idx -= SHRD_RW_LOG_SIZE_IN_PAGE;
		}

		for(i = 0; i < blk_rq_sectors(prq) / SHRD_SECTORS_PER_PAGE; i++){
			
			header->o_addr[idx] = blk_rq_pos(prq) / SHRD_SECTORS_PER_PAGE + i;
			
			shrd->shrd_rw_map[idx].t_addr = SHRD_RW_LOG_START_IN_PAGE+ idx;
			shrd->shrd_rw_map[idx].o_addr = header->o_addr[idx];
			
			BUG_ON(shrd->shrd_rw_map[idx].flags == SHRD_VALID_MAP); //should be remapped
			shrd->shrd_rw_map[idx].flags = SHRD_VALID_MAP;
			scsi_shrd_map_insert(&shrd->rw_mapping, &shrd->shrd_rw_map[idx]);

			idx++;
			if(idx >= SHRD_RW_LOG_START_IN_PAGE)
				idx -= SHRD_RW_LOG_SIZE_IN_PAGE;
		}
	}
	sdev_printk(KERN_INFO, sdev, "%s: packing end, log idx (%d) is changed to %d\n", __func__, shrd->rw_log_new_idx, idx);
	shrd->rw_log_new_idx = idx;

}

static void  scsi_shrd_make_remap_bio(struct request_queue *q, struct SHRD_REMAP *remap_entry){

	struct scsi_device *sdev = q->queuedata;
	struct SHRD *shrd = sdev->shrd;
	struct bio *bio;
	int i;

	bio = bio_kmalloc(GFP_ATOMIC, SHRD_NUM_CORES);
	if(!bio)
		return NULL;

	bio->bi_end_io = scsi_shrd_bio_endio;
	bio->bi_rw |= REQ_WRITE | REQ_SYNC |REQ_SHRD_REMAP;
	bio->bi_iter.bi_sector = remap_entry->entry_num * SHRD_NUM_CORES + SHRD_REMAP_CMD_START_IN_PAGE * SHRD_SECTORS_PER_PAGE;
	bio->bi_bdev = shrd->bdev;
	bio->bi_private = remap_entry;

	for(i=0; i < SHRD_NUM_CORES; i++){
		if(bio_add_page(bio, virt_to_page((void *)remap_entry->remap_data), PAGE_SIZE, 0) < PAGE_SIZE){
			sdev_printk(KERN_INFO, sdev, "%s: SHRD bio_add_pc_page failed on CORE %d\n", __func__, i);
			//return NULL;
		}
	}

	scsi_shrd_submit_bio(REQ_SYNC, bio, remap_entry->req);
/*	
	spin_unlock_irq(q->queue_lock);
	submit_bio(REQ_SYNC, bio);
	spin_lock_irq(q->queue_lock);

	*/
	
}

static struct SHRD_REMAP* __scsi_shrd_do_remap_rw_log(struct request_queue *q, u32 size){

	struct scsi_device *sdev = q->queuedata;
	struct SHRD *shrd = sdev->shrd;
	struct rb_node *node;
	struct SHRD_MAP *map;
	struct SHRD_REMAP *entry;
	u32 start_idx = shrd->rw_log_start_idx, end_idx;
	u32 idx = 0, cnt = 0;

	sdev_printk(KERN_INFO, sdev, "%s: do_remap\n", __func__);
	
	entry = shrd_get_remap_entry(shrd);
	if(entry == NULL){
		sdev_printk(KERN_INFO, sdev, "%s: has no remap entry\n", __func__);
		return NULL;
	}
	list_del(&entry->remap_cmd_list);
	shrd_clear_remap_entry(entry);

	for(idx = 0; idx < size; idx++){
		u32 log_idx = idx + start_idx;
		if(log_idx >= SHRD_RW_LOG_SIZE_IN_PAGE)
			log_idx -= SHRD_RW_LOG_SIZE_IN_PAGE;
		
		if(shrd->shrd_rw_map[log_idx].flags == SHRD_VALID_MAP){
			cnt++;
			if(cnt == SHRD_NUM_MAX_REMAP_ENTRY)
				break;
		}
	}

	end_idx = start_idx + idx;
	if(end_idx >= SHRD_RW_LOG_SIZE_IN_PAGE)
		end_idx -= SHRD_RW_LOG_SIZE_IN_PAGE;

	entry->remap_data[0]->t_addr_start = start_idx + SHRD_RW_LOG_START_IN_PAGE;
	entry->remap_data[0]->t_addr_end = end_idx + SHRD_RW_LOG_START_IN_PAGE;
	entry->remap_data[0]->remap_count = cnt;

	idx = 0;

	for(node = rb_first(&shrd->rw_mapping); node; rb_next(node)){

		map = rb_entry(node, struct SHRD_MAP, node);
		if(map->flags != SHRD_VALID_MAP)
			continue;

		if(end_idx > start_idx){
			if((map->t_addr >= start_idx) && (map->t_addr <= end_idx)){
				entry->remap_data[idx / SHRD_NUM_MAX_REMAP_ENTRY]->o_addr[idx % SHRD_NUM_MAX_REMAP_ENTRY] = map->o_addr;
				entry->remap_data[idx / SHRD_NUM_MAX_REMAP_ENTRY]->t_addr[idx % SHRD_NUM_MAX_REMAP_ENTRY] = map->t_addr;
				map->flags = SHRD_REMAPPING_MAP;
			}
		}
		else{
			if((map->t_addr >= start_idx) || (map->t_addr <= end_idx)){
				entry->remap_data[idx / SHRD_NUM_MAX_REMAP_ENTRY]->o_addr[idx % SHRD_NUM_MAX_REMAP_ENTRY] = map->o_addr;
				entry->remap_data[idx / SHRD_NUM_MAX_REMAP_ENTRY]->t_addr[idx % SHRD_NUM_MAX_REMAP_ENTRY] = map->t_addr;
				map->flags = SHRD_REMAPPING_MAP;
			}
		}
		
		idx++;

		if(idx == cnt)
			break;
	}

}

/*
	return 0 : no remap
	return 1: remap
	return -1: err
*/
static struct SHRD_REMAP* scsi_shrd_prep_remap_if_need(struct request_queue *q){

	struct scsi_device *sdev = q->queuedata;
	struct SHRD *shrd = sdev->shrd;
	struct SHRD_REMAP *remap_entry = NULL;
	u32 start_idx = shrd->rw_log_start_idx;
	u32 new_idx = shrd->rw_log_new_idx;
	u32 size; //used log size

	(new_idx >= start_idx) ? (size = new_idx - start_idx) : (size = SHRD_RW_LOG_SIZE_IN_PAGE - start_idx + new_idx);

	if(size > SHRD_RW_REMAP_THRESHOLD_IN_PAGE){
		//need to remap
		remap_entry = __scsi_shrd_do_remap_rw_log(q, size);
		sdev_printk(KERN_INFO, sdev, "%s: return valid remap entry\n", __func__);
	}

	return remap_entry;
	
}

static int scsi_shrd_check_read_requests(struct request_queue *q, struct request *rq){

	u32 rq_sectors = blk_rq_sectors(rq);
	u32 rq_pages = rq_sectors / SHRD_SECTORS_PER_PAGE + ((rq_sectors % SHRD_SECTORS_PER_PAGE == 0) ? 0 : 1);
	u32 iter = 0;
/*
	for(iter = 0; iter < rq_pages; iter++){

		//check every pages for the requests
		//it can be overhead so we might need to use bloom filter
	}
	*/
	
}

#endif

/*
 * Function:    scsi_request_fn()
 *
 * Purpose:     Main strategy routine for SCSI.
 *
 * Arguments:   q       - Pointer to actual queue.
 *
 * Returns:     Nothing
 *
 * Lock status: IO request lock assumed to be held when called.
 */
static void scsi_request_fn(struct request_queue *q)
	__releases(q->queue_lock)
	__acquires(q->queue_lock)
{
	struct scsi_device *sdev = q->queuedata;
	struct Scsi_Host *shost;
	struct scsi_cmnd *cmd;
	struct request *req;
	
	/*
	 * To start with, we keep looping until the queue is empty, or until
	 * the host is no longer able to accept any more requests.
	 */
	shost = sdev->host;
	if(sdev->shrd_on)
		sdev_printk(KERN_INFO, sdev, "%d: %s: active request fn is %d\n", smp_processor_id(), __func__, q->request_fn_active);
	
	for (;;) {
		int rtn;
		/*
		 * get next queueable request.  We do this early to make sure
		 * that the request is fully prepared even if we cannot
		 * accept it.
		 */
		req = blk_peek_request(q);
		if (!req){
			if(sdev->shrd_on)
				sdev_printk(KERN_INFO, sdev, "%d: %s: break because req NULL on peek\n", smp_processor_id(), __func__);
			break;

		}

#ifdef CONFIG_SCSI_SHRD_TEST0
		if(sdev->shrd_on){
			
			BUG_ON(!sdev->shrd);

			struct SHRD_TWRITE *twrite_entry = NULL;
			struct SHRD_REMAP *remap_entry = NULL;
			struct bio *bio = req->bio;

			sdev_printk(KERN_INFO, sdev, "%d: %s: SHRD request handling start req pos: %d, sectors: %d\n", smp_processor_id(), __func__, blk_rq_pos(req), blk_rq_sectors(req));

			if(req->bio){
				if(sdev->shrd->bdev != req->bio->bi_bdev)
					sdev_printk(KERN_ERR, sdev, "%d: %s: shrd->bdev is different from req->bio->bi_bdev\n", smp_processor_id(), __func__);
			}
			
			if(bio && bio->bi_rw & REQ_SHRD_TWRITE_HDR){ //handle former speical function for SHRD
				sdev_printk(KERN_INFO, sdev, "%d: %s: SHRD handle twrite hdr request\n",smp_processor_id(), __func__);
				goto spcmd;
			}
			else if(bio && bio->bi_rw & REQ_SHRD_TWRITE_DAT){
				sdev_printk(KERN_INFO, sdev, "%d: %s: SHRD handle twrite data request\n", smp_processor_id(), __func__);
				goto spcmd;
			}
			else if(bio && bio->bi_rw & REQ_SHRD_REMAP){
				sdev_printk(KERN_INFO, sdev, "%d: %s: SHRD handle remap request\n", smp_processor_id(), __func__);
				goto spcmd;
			}
			else if(remap_entry = scsi_shrd_prep_remap_if_need(q)){
				//need remap?
				BUG_ON(!remap_entry);
				sdev_printk(KERN_INFO, sdev, "%d: %s: SHRD handle try remap\n",  smp_processor_id(), __func__);
				scsi_shrd_make_remap_bio(q, remap_entry);
				continue;
			}
			else if(twrite_entry = scsi_shrd_prep_rw_twrite(q,req)){
				BUG_ON(!twrite_entry);
				sdev_printk(KERN_INFO, sdev, "%d: %s: SHRD handle try twrite\n", smp_processor_id(), __func__);
				//need twrite?
				scsi_shrd_packing_rw_twrite(q, twrite_entry);
				scsi_shrd_make_twrite_bios(q, twrite_entry);
				if(!twrite_entry->data->bi_bdev)
					BUG();
				if(!twrite_entry->header->bi_bdev)
					BUG();
				scsi_shrd_submit_bio(REQ_SYNC, twrite_entry->data, twrite_entry->data_rq);
				scsi_shrd_submit_bio(REQ_SYNC, twrite_entry->header, twrite_entry->header_rq);

				continue;
			}
			else if(!rq_data_dir(req)){
				sdev_printk(KERN_INFO, sdev, "%d: %s: SHRD handle generic read function\n", smp_processor_id(), __func__);
				//read request, need to check whether need to change the address or not.
			}
			else {	//generic prep procedure (for non_twrite request)
				sdev_printk(KERN_INFO, sdev, "%d: %s: SHRD handle generic write function\n", smp_processor_id(), __func__);
				
			}
spcmd:
			if (!(req->cmd_flags & REQ_DONTPREP))
				scsi_shrd_prep_req(q, req);
		}
#endif
		if (unlikely(!scsi_device_online(sdev))) {
			sdev_printk(KERN_ERR, sdev,
				    "rejecting I/O to offline device\n");
			scsi_kill_request(req, q);
			continue;
		}

		if (!scsi_dev_queue_ready(q, sdev)){
			if(sdev->shrd_on)
				sdev_printk(KERN_INFO, sdev, "%s: break because dev queue is not ready\n", __func__);
			break;
		}

		/*
		 * Remove the request from the request list.
		 */
		if (!(blk_queue_tagged(q) && !blk_queue_start_tag(q, req)))
			blk_start_request(req);
#ifdef CONFIG_SCSI_SHRD_TEST0
		if(sdev->shrd_on)
			sdev_printk(KERN_INFO, sdev, "%d: %s: blk_start_request complete, will unlock irq\n", smp_processor_id(), __func__);
#endif

		spin_unlock_irq(q->queue_lock);
		cmd = req->special;

#ifdef CONFIG_SCSI_SHRD_TEST0
		if(sdev->shrd_on)
			sdev_printk(KERN_INFO, sdev, "%d: %s: spin_unlock_irq complete\n", smp_processor_id(), __func__);
#endif

		
		if (unlikely(cmd == NULL)) {
			printk(KERN_CRIT "impossible request in %s.\n"
					 "please mail a stack trace to "
					 "linux-scsi@vger.kernel.org\n",
					 __func__);
			blk_dump_rq_flags(req, "foo");
			BUG();
		}

		/*
		 * We hit this when the driver is using a host wide
		 * tag map. For device level tag maps the queue_depth check
		 * in the device ready fn would prevent us from trying
		 * to allocate a tag. Since the map is a shared host resource
		 * we add the dev to the starved list so it eventually gets
		 * a run when a tag is freed.
		 */
		if (blk_queue_tagged(q) && !blk_rq_tagged(req)) {
			spin_lock_irq(shost->host_lock);
			if (list_empty(&sdev->starved_entry))
				list_add_tail(&sdev->starved_entry,
					      &shost->starved_list);
			spin_unlock_irq(shost->host_lock);
			goto not_ready;
		}

		if (!scsi_target_queue_ready(shost, sdev))
			goto not_ready;

		if (!scsi_host_queue_ready(q, shost, sdev))
			goto host_not_ready;

		/*
		 * Finally, initialize any error handling parameters, and set up
		 * the timers for timeouts.
		 */
		scsi_init_cmd_errh(cmd);

		/*
		 * Dispatch the command to the low-level driver.
		 */
		cmd->scsi_done = scsi_done;
		rtn = scsi_dispatch_cmd(cmd);
		if (rtn) {
			scsi_queue_insert(cmd, rtn);
			spin_lock_irq(q->queue_lock);
			if(sdev->shrd_on){
				sdev_printk(KERN_INFO, sdev, "%d: %s: dispatch failed, insert to midqueue\n", smp_processor_id(), __func__);
			}
			goto out_delay;
		}
		if(sdev->shrd_on){
			sdev_printk(KERN_INFO, sdev, "%d: %s: dispatch complete,req pos: %d, sectors: %d\n", smp_processor_id(), __func__, blk_rq_pos(req), blk_rq_sectors(req));
		}
		spin_lock_irq(q->queue_lock);
	}

	if(sdev->shrd_on)
		sdev_printk(KERN_INFO, sdev, "%d: %s: return\n", smp_processor_id(), __func__);
	return;

 host_not_ready:
 	if(sdev->shrd_on)
		sdev_printk(KERN_INFO, sdev, "%s: host not ready\n", __func__);
	if (scsi_target(sdev)->can_queue > 0)
		atomic_dec(&scsi_target(sdev)->target_busy);
 not_ready:
	/*
	 * lock q, handle tag, requeue req, and decrement device_busy. We
	 * must return with queue_lock held.
	 *
	 * Decrementing device_busy without checking it is OK, as all such
	 * cases (host limits or settings) should run the queue at some
	 * later time.
	 */

	if(sdev->shrd_on)
		sdev_printk(KERN_INFO, sdev, "%s: not ready\n", __func__);
	
	spin_lock_irq(q->queue_lock);
	blk_requeue_request(q, req);
	atomic_dec(&sdev->device_busy);
out_delay:
	
	if(sdev->shrd_on)
		sdev_printk(KERN_INFO, sdev, "%s: out delay\n", __func__);
	
	if (!atomic_read(&sdev->device_busy) && !scsi_device_blocked(sdev))
		blk_delay_queue(q, SCSI_QUEUE_DELAY);
}

static inline int prep_to_mq(int ret)
{
	switch (ret) {
	case BLKPREP_OK:
		return 0;
	case BLKPREP_DEFER:
		return BLK_MQ_RQ_QUEUE_BUSY;
	default:
		return BLK_MQ_RQ_QUEUE_ERROR;
	}
}

static int scsi_mq_prep_fn(struct request *req)
{
	struct scsi_cmnd *cmd = blk_mq_rq_to_pdu(req);
	struct scsi_device *sdev = req->q->queuedata;
	struct Scsi_Host *shost = sdev->host;
	unsigned char *sense_buf = cmd->sense_buffer;
	struct scatterlist *sg;

	memset(cmd, 0, sizeof(struct scsi_cmnd));

	req->special = cmd;

	cmd->request = req;
	cmd->device = sdev;
	cmd->sense_buffer = sense_buf;

	cmd->tag = req->tag;

	cmd->cmnd = req->cmd;
	cmd->prot_op = SCSI_PROT_NORMAL;

	INIT_LIST_HEAD(&cmd->list);
	INIT_DELAYED_WORK(&cmd->abort_work, scmd_eh_abort_handler);
	cmd->jiffies_at_alloc = jiffies;

	/*
	 * XXX: cmd_list lookups are only used by two drivers, try to get
	 * rid of this list in common code.
	 */
	spin_lock_irq(&sdev->list_lock);
	list_add_tail(&cmd->list, &sdev->cmd_list);
	spin_unlock_irq(&sdev->list_lock);

	sg = (void *)cmd + sizeof(struct scsi_cmnd) + shost->hostt->cmd_size;
	cmd->sdb.table.sgl = sg;

	if (scsi_host_get_prot(shost)) {
		cmd->prot_sdb = (void *)sg +
			shost->sg_tablesize * sizeof(struct scatterlist);
		memset(cmd->prot_sdb, 0, sizeof(struct scsi_data_buffer));

		cmd->prot_sdb->table.sgl =
			(struct scatterlist *)(cmd->prot_sdb + 1);
	}

	if (blk_bidi_rq(req)) {
		struct request *next_rq = req->next_rq;
		struct scsi_data_buffer *bidi_sdb = blk_mq_rq_to_pdu(next_rq);

		memset(bidi_sdb, 0, sizeof(struct scsi_data_buffer));
		bidi_sdb->table.sgl =
			(struct scatterlist *)(bidi_sdb + 1);

		next_rq->special = bidi_sdb;
	}

	return scsi_setup_cmnd(sdev, req);
}

static void scsi_mq_done(struct scsi_cmnd *cmd)
{
	trace_scsi_dispatch_cmd_done(cmd);
	blk_mq_complete_request(cmd->request);
}

static int scsi_queue_rq(struct blk_mq_hw_ctx *hctx, struct request *req)
{
	struct request_queue *q = req->q;
	struct scsi_device *sdev = q->queuedata;
	struct Scsi_Host *shost = sdev->host;
	struct scsi_cmnd *cmd = blk_mq_rq_to_pdu(req);
	int ret;
	int reason;

	ret = prep_to_mq(scsi_prep_state_check(sdev, req));
	if (ret)
		goto out;

	ret = BLK_MQ_RQ_QUEUE_BUSY;
	if (!get_device(&sdev->sdev_gendev))
		goto out;

	if (!scsi_dev_queue_ready(q, sdev))
		goto out_put_device;
	if (!scsi_target_queue_ready(shost, sdev))
		goto out_dec_device_busy;
	if (!scsi_host_queue_ready(q, shost, sdev))
		goto out_dec_target_busy;

	if (!(req->cmd_flags & REQ_DONTPREP)) {
		ret = prep_to_mq(scsi_mq_prep_fn(req));
		if (ret)
			goto out_dec_host_busy;
		req->cmd_flags |= REQ_DONTPREP;
	}

	if (blk_queue_tagged(q))
		req->cmd_flags |= REQ_QUEUED;
	else
		req->cmd_flags &= ~REQ_QUEUED;

	scsi_init_cmd_errh(cmd);
	cmd->scsi_done = scsi_mq_done;

	reason = scsi_dispatch_cmd(cmd);
	if (reason) {
		scsi_set_blocked(cmd, reason);
		ret = BLK_MQ_RQ_QUEUE_BUSY;
		goto out_dec_host_busy;
	}

	return BLK_MQ_RQ_QUEUE_OK;

out_dec_host_busy:
	atomic_dec(&shost->host_busy);
out_dec_target_busy:
	if (scsi_target(sdev)->can_queue > 0)
		atomic_dec(&scsi_target(sdev)->target_busy);
out_dec_device_busy:
	atomic_dec(&sdev->device_busy);
out_put_device:
	put_device(&sdev->sdev_gendev);
out:
	switch (ret) {
	case BLK_MQ_RQ_QUEUE_BUSY:
		blk_mq_stop_hw_queue(hctx);
		if (atomic_read(&sdev->device_busy) == 0 &&
		    !scsi_device_blocked(sdev))
			blk_mq_delay_queue(hctx, SCSI_QUEUE_DELAY);
		break;
	case BLK_MQ_RQ_QUEUE_ERROR:
		/*
		 * Make sure to release all allocated ressources when
		 * we hit an error, as we will never see this command
		 * again.
		 */
		if (req->cmd_flags & REQ_DONTPREP)
			scsi_mq_uninit_cmd(cmd);
		break;
	default:
		break;
	}
	return ret;
}

static int scsi_init_request(void *data, struct request *rq,
		unsigned int hctx_idx, unsigned int request_idx,
		unsigned int numa_node)
{
	struct scsi_cmnd *cmd = blk_mq_rq_to_pdu(rq);

	cmd->sense_buffer = kzalloc_node(SCSI_SENSE_BUFFERSIZE, GFP_KERNEL,
			numa_node);
	if (!cmd->sense_buffer)
		return -ENOMEM;
	return 0;
}

static void scsi_exit_request(void *data, struct request *rq,
		unsigned int hctx_idx, unsigned int request_idx)
{
	struct scsi_cmnd *cmd = blk_mq_rq_to_pdu(rq);

	kfree(cmd->sense_buffer);
}

static u64 scsi_calculate_bounce_limit(struct Scsi_Host *shost)
{
	struct device *host_dev;
	u64 bounce_limit = 0xffffffff;

	if (shost->unchecked_isa_dma)
		return BLK_BOUNCE_ISA;
	/*
	 * Platforms with virtual-DMA translation
	 * hardware have no practical limit.
	 */
	if (!PCI_DMA_BUS_IS_PHYS)
		return BLK_BOUNCE_ANY;

	host_dev = scsi_get_device(shost);
	if (host_dev && host_dev->dma_mask)
		bounce_limit = (u64)dma_max_pfn(host_dev) << PAGE_SHIFT;

	return bounce_limit;
}

static void __scsi_init_queue(struct Scsi_Host *shost, struct request_queue *q)
{
	struct device *dev = shost->dma_dev;

	/*
	 * this limit is imposed by hardware restrictions
	 */
	blk_queue_max_segments(q, min_t(unsigned short, shost->sg_tablesize,
					SCSI_MAX_SG_CHAIN_SEGMENTS));

	if (scsi_host_prot_dma(shost)) {
		shost->sg_prot_tablesize =
			min_not_zero(shost->sg_prot_tablesize,
				     (unsigned short)SCSI_MAX_PROT_SG_SEGMENTS);
		BUG_ON(shost->sg_prot_tablesize < shost->sg_tablesize);
		blk_queue_max_integrity_segments(q, shost->sg_prot_tablesize);
	}

	blk_queue_max_hw_sectors(q, shost->max_sectors);
	blk_queue_bounce_limit(q, scsi_calculate_bounce_limit(shost));
	blk_queue_segment_boundary(q, shost->dma_boundary);
	dma_set_seg_boundary(dev, shost->dma_boundary);

	blk_queue_max_segment_size(q, dma_get_max_seg_size(dev));

	if (!shost->use_clustering)
		q->limits.cluster = 0;

	/*
	 * set a reasonable default alignment on word boundaries: the
	 * host and device may alter it using
	 * blk_queue_update_dma_alignment() later.
	 */
	blk_queue_dma_alignment(q, 0x03);
}

struct request_queue *__scsi_alloc_queue(struct Scsi_Host *shost,
					 request_fn_proc *request_fn)
{
	struct request_queue *q;

	q = blk_init_queue(request_fn, NULL);
	if (!q)
		return NULL;
	__scsi_init_queue(shost, q);
	return q;
}
EXPORT_SYMBOL(__scsi_alloc_queue);

struct request_queue *scsi_alloc_queue(struct scsi_device *sdev)
{
	int rtn = 0;
	struct request_queue *q;

	q = __scsi_alloc_queue(sdev->host, scsi_request_fn);
	if (!q)
		return NULL;

	blk_queue_prep_rq(q, scsi_prep_fn);
	blk_queue_unprep_rq(q, scsi_unprep_fn);
	blk_queue_softirq_done(q, scsi_softirq_done);
	blk_queue_rq_timed_out(q, scsi_times_out);
	blk_queue_lld_busy(q, scsi_lld_busy);

	return q;
}

static struct blk_mq_ops scsi_mq_ops = {
	.map_queue	= blk_mq_map_queue,
	.queue_rq	= scsi_queue_rq,
	.complete	= scsi_softirq_done,
	.timeout	= scsi_times_out,
	.init_request	= scsi_init_request,
	.exit_request	= scsi_exit_request,
};

struct request_queue *scsi_mq_alloc_queue(struct scsi_device *sdev)
{
	sdev->request_queue = blk_mq_init_queue(&sdev->host->tag_set);
	if (IS_ERR(sdev->request_queue))
		return NULL;

	sdev->request_queue->queuedata = sdev;
	__scsi_init_queue(sdev->host, sdev->request_queue);
	return sdev->request_queue;
}

int scsi_mq_setup_tags(struct Scsi_Host *shost)
{
	unsigned int cmd_size, sgl_size, tbl_size;

	tbl_size = shost->sg_tablesize;
	if (tbl_size > SCSI_MAX_SG_SEGMENTS)
		tbl_size = SCSI_MAX_SG_SEGMENTS;
	sgl_size = tbl_size * sizeof(struct scatterlist);
	cmd_size = sizeof(struct scsi_cmnd) + shost->hostt->cmd_size + sgl_size;
	if (scsi_host_get_prot(shost))
		cmd_size += sizeof(struct scsi_data_buffer) + sgl_size;

	memset(&shost->tag_set, 0, sizeof(shost->tag_set));
	shost->tag_set.ops = &scsi_mq_ops;
	shost->tag_set.nr_hw_queues = 1;
	shost->tag_set.queue_depth = shost->can_queue;
	shost->tag_set.cmd_size = cmd_size;
	shost->tag_set.numa_node = NUMA_NO_NODE;
	shost->tag_set.flags = BLK_MQ_F_SHOULD_MERGE | BLK_MQ_F_SG_MERGE;
	shost->tag_set.driver_data = shost;

	return blk_mq_alloc_tag_set(&shost->tag_set);
}

void scsi_mq_destroy_tags(struct Scsi_Host *shost)
{
	blk_mq_free_tag_set(&shost->tag_set);
}

/*
 * Function:    scsi_block_requests()
 *
 * Purpose:     Utility function used by low-level drivers to prevent further
 *		commands from being queued to the device.
 *
 * Arguments:   shost       - Host in question
 *
 * Returns:     Nothing
 *
 * Lock status: No locks are assumed held.
 *
 * Notes:       There is no timer nor any other means by which the requests
 *		get unblocked other than the low-level driver calling
 *		scsi_unblock_requests().
 */
void scsi_block_requests(struct Scsi_Host *shost)
{
	shost->host_self_blocked = 1;
}
EXPORT_SYMBOL(scsi_block_requests);

/*
 * Function:    scsi_unblock_requests()
 *
 * Purpose:     Utility function used by low-level drivers to allow further
 *		commands from being queued to the device.
 *
 * Arguments:   shost       - Host in question
 *
 * Returns:     Nothing
 *
 * Lock status: No locks are assumed held.
 *
 * Notes:       There is no timer nor any other means by which the requests
 *		get unblocked other than the low-level driver calling
 *		scsi_unblock_requests().
 *
 *		This is done as an API function so that changes to the
 *		internals of the scsi mid-layer won't require wholesale
 *		changes to drivers that use this feature.
 */
void scsi_unblock_requests(struct Scsi_Host *shost)
{
	shost->host_self_blocked = 0;
	scsi_run_host_queues(shost);
}
EXPORT_SYMBOL(scsi_unblock_requests);

int __init scsi_init_queue(void)
{
	int i;

	scsi_sdb_cache = kmem_cache_create("scsi_data_buffer",
					   sizeof(struct scsi_data_buffer),
					   0, 0, NULL);
	if (!scsi_sdb_cache) {
		printk(KERN_ERR "SCSI: can't init scsi sdb cache\n");
		return -ENOMEM;
	}

	for (i = 0; i < SG_MEMPOOL_NR; i++) {
		struct scsi_host_sg_pool *sgp = scsi_sg_pools + i;
		int size = sgp->size * sizeof(struct scatterlist);

		sgp->slab = kmem_cache_create(sgp->name, size, 0,
				SLAB_HWCACHE_ALIGN, NULL);
		if (!sgp->slab) {
			printk(KERN_ERR "SCSI: can't init sg slab %s\n",
					sgp->name);
			goto cleanup_sdb;
		}

		sgp->pool = mempool_create_slab_pool(SG_MEMPOOL_SIZE,
						     sgp->slab);
		if (!sgp->pool) {
			printk(KERN_ERR "SCSI: can't init sg mempool %s\n",
					sgp->name);
			goto cleanup_sdb;
		}
	}

	return 0;

cleanup_sdb:
	for (i = 0; i < SG_MEMPOOL_NR; i++) {
		struct scsi_host_sg_pool *sgp = scsi_sg_pools + i;
		if (sgp->pool)
			mempool_destroy(sgp->pool);
		if (sgp->slab)
			kmem_cache_destroy(sgp->slab);
	}
	kmem_cache_destroy(scsi_sdb_cache);

	return -ENOMEM;
}

void scsi_exit_queue(void)
{
	int i;

	kmem_cache_destroy(scsi_sdb_cache);

	for (i = 0; i < SG_MEMPOOL_NR; i++) {
		struct scsi_host_sg_pool *sgp = scsi_sg_pools + i;
		mempool_destroy(sgp->pool);
		kmem_cache_destroy(sgp->slab);
	}
}

/**
 *	scsi_mode_select - issue a mode select
 *	@sdev:	SCSI device to be queried
 *	@pf:	Page format bit (1 == standard, 0 == vendor specific)
 *	@sp:	Save page bit (0 == don't save, 1 == save)
 *	@modepage: mode page being requested
 *	@buffer: request buffer (may not be smaller than eight bytes)
 *	@len:	length of request buffer.
 *	@timeout: command timeout
 *	@retries: number of retries before failing
 *	@data: returns a structure abstracting the mode header data
 *	@sshdr: place to put sense data (or NULL if no sense to be collected).
 *		must be SCSI_SENSE_BUFFERSIZE big.
 *
 *	Returns zero if successful; negative error number or scsi
 *	status on error
 *
 */
int
scsi_mode_select(struct scsi_device *sdev, int pf, int sp, int modepage,
		 unsigned char *buffer, int len, int timeout, int retries,
		 struct scsi_mode_data *data, struct scsi_sense_hdr *sshdr)
{
	unsigned char cmd[10];
	unsigned char *real_buffer;
	int ret;

	memset(cmd, 0, sizeof(cmd));
	cmd[1] = (pf ? 0x10 : 0) | (sp ? 0x01 : 0);

	if (sdev->use_10_for_ms) {
		if (len > 65535)
			return -EINVAL;
		real_buffer = kmalloc(8 + len, GFP_KERNEL);
		if (!real_buffer)
			return -ENOMEM;
		memcpy(real_buffer + 8, buffer, len);
		len += 8;
		real_buffer[0] = 0;
		real_buffer[1] = 0;
		real_buffer[2] = data->medium_type;
		real_buffer[3] = data->device_specific;
		real_buffer[4] = data->longlba ? 0x01 : 0;
		real_buffer[5] = 0;
		real_buffer[6] = data->block_descriptor_length >> 8;
		real_buffer[7] = data->block_descriptor_length;

		cmd[0] = MODE_SELECT_10;
		cmd[7] = len >> 8;
		cmd[8] = len;
	} else {
		if (len > 255 || data->block_descriptor_length > 255 ||
		    data->longlba)
			return -EINVAL;

		real_buffer = kmalloc(4 + len, GFP_KERNEL);
		if (!real_buffer)
			return -ENOMEM;
		memcpy(real_buffer + 4, buffer, len);
		len += 4;
		real_buffer[0] = 0;
		real_buffer[1] = data->medium_type;
		real_buffer[2] = data->device_specific;
		real_buffer[3] = data->block_descriptor_length;
		

		cmd[0] = MODE_SELECT;
		cmd[4] = len;
	}

	ret = scsi_execute_req(sdev, cmd, DMA_TO_DEVICE, real_buffer, len,
			       sshdr, timeout, retries, NULL);
	kfree(real_buffer);
	return ret;
}
EXPORT_SYMBOL_GPL(scsi_mode_select);

/**
 *	scsi_mode_sense - issue a mode sense, falling back from 10 to six bytes if necessary.
 *	@sdev:	SCSI device to be queried
 *	@dbd:	set if mode sense will allow block descriptors to be returned
 *	@modepage: mode page being requested
 *	@buffer: request buffer (may not be smaller than eight bytes)
 *	@len:	length of request buffer.
 *	@timeout: command timeout
 *	@retries: number of retries before failing
 *	@data: returns a structure abstracting the mode header data
 *	@sshdr: place to put sense data (or NULL if no sense to be collected).
 *		must be SCSI_SENSE_BUFFERSIZE big.
 *
 *	Returns zero if unsuccessful, or the header offset (either 4
 *	or 8 depending on whether a six or ten byte command was
 *	issued) if successful.
 */
int
scsi_mode_sense(struct scsi_device *sdev, int dbd, int modepage,
		  unsigned char *buffer, int len, int timeout, int retries,
		  struct scsi_mode_data *data, struct scsi_sense_hdr *sshdr)
{
	unsigned char cmd[12];
	int use_10_for_ms;
	int header_length;
	int result;
	struct scsi_sense_hdr my_sshdr;

	memset(data, 0, sizeof(*data));
	memset(&cmd[0], 0, 12);
	cmd[1] = dbd & 0x18;	/* allows DBD and LLBA bits */
	cmd[2] = modepage;

	/* caller might not be interested in sense, but we need it */
	if (!sshdr)
		sshdr = &my_sshdr;

 retry:
	use_10_for_ms = sdev->use_10_for_ms;

	if (use_10_for_ms) {
		if (len < 8)
			len = 8;

		cmd[0] = MODE_SENSE_10;
		cmd[8] = len;
		header_length = 8;
	} else {
		if (len < 4)
			len = 4;

		cmd[0] = MODE_SENSE;
		cmd[4] = len;
		header_length = 4;
	}

	memset(buffer, 0, len);

	result = scsi_execute_req(sdev, cmd, DMA_FROM_DEVICE, buffer, len,
				  sshdr, timeout, retries, NULL);

	/* This code looks awful: what it's doing is making sure an
	 * ILLEGAL REQUEST sense return identifies the actual command
	 * byte as the problem.  MODE_SENSE commands can return
	 * ILLEGAL REQUEST if the code page isn't supported */

	if (use_10_for_ms && !scsi_status_is_good(result) &&
	    (driver_byte(result) & DRIVER_SENSE)) {
		if (scsi_sense_valid(sshdr)) {
			if ((sshdr->sense_key == ILLEGAL_REQUEST) &&
			    (sshdr->asc == 0x20) && (sshdr->ascq == 0)) {
				/* 
				 * Invalid command operation code
				 */
				sdev->use_10_for_ms = 0;
				goto retry;
			}
		}
	}

	if(scsi_status_is_good(result)) {
		if (unlikely(buffer[0] == 0x86 && buffer[1] == 0x0b &&
			     (modepage == 6 || modepage == 8))) {
			/* Initio breakage? */
			header_length = 0;
			data->length = 13;
			data->medium_type = 0;
			data->device_specific = 0;
			data->longlba = 0;
			data->block_descriptor_length = 0;
		} else if(use_10_for_ms) {
			data->length = buffer[0]*256 + buffer[1] + 2;
			data->medium_type = buffer[2];
			data->device_specific = buffer[3];
			data->longlba = buffer[4] & 0x01;
			data->block_descriptor_length = buffer[6]*256
				+ buffer[7];
		} else {
			data->length = buffer[0] + 1;
			data->medium_type = buffer[1];
			data->device_specific = buffer[2];
			data->block_descriptor_length = buffer[3];
		}
		data->header_length = header_length;
	}

	return result;
}
EXPORT_SYMBOL(scsi_mode_sense);

/**
 *	scsi_test_unit_ready - test if unit is ready
 *	@sdev:	scsi device to change the state of.
 *	@timeout: command timeout
 *	@retries: number of retries before failing
 *	@sshdr_external: Optional pointer to struct scsi_sense_hdr for
 *		returning sense. Make sure that this is cleared before passing
 *		in.
 *
 *	Returns zero if unsuccessful or an error if TUR failed.  For
 *	removable media, UNIT_ATTENTION sets ->changed flag.
 **/
int
scsi_test_unit_ready(struct scsi_device *sdev, int timeout, int retries,
		     struct scsi_sense_hdr *sshdr_external)
{
	char cmd[] = {
		TEST_UNIT_READY, 0, 0, 0, 0, 0,
	};
	struct scsi_sense_hdr *sshdr;
	int result;

	if (!sshdr_external)
		sshdr = kzalloc(sizeof(*sshdr), GFP_KERNEL);
	else
		sshdr = sshdr_external;

	/* try to eat the UNIT_ATTENTION if there are enough retries */
	do {
		result = scsi_execute_req(sdev, cmd, DMA_NONE, NULL, 0, sshdr,
					  timeout, retries, NULL);
		if (sdev->removable && scsi_sense_valid(sshdr) &&
		    sshdr->sense_key == UNIT_ATTENTION)
			sdev->changed = 1;
	} while (scsi_sense_valid(sshdr) &&
		 sshdr->sense_key == UNIT_ATTENTION && --retries);

	if (!sshdr_external)
		kfree(sshdr);
	return result;
}
EXPORT_SYMBOL(scsi_test_unit_ready);

/**
 *	scsi_device_set_state - Take the given device through the device state model.
 *	@sdev:	scsi device to change the state of.
 *	@state:	state to change to.
 *
 *	Returns zero if unsuccessful or an error if the requested 
 *	transition is illegal.
 */
int
scsi_device_set_state(struct scsi_device *sdev, enum scsi_device_state state)
{
	enum scsi_device_state oldstate = sdev->sdev_state;

	if (state == oldstate)
		return 0;

	switch (state) {
	case SDEV_CREATED:
		switch (oldstate) {
		case SDEV_CREATED_BLOCK:
			break;
		default:
			goto illegal;
		}
		break;
			
	case SDEV_RUNNING:
		switch (oldstate) {
		case SDEV_CREATED:
		case SDEV_OFFLINE:
		case SDEV_TRANSPORT_OFFLINE:
		case SDEV_QUIESCE:
		case SDEV_BLOCK:
			break;
		default:
			goto illegal;
		}
		break;

	case SDEV_QUIESCE:
		switch (oldstate) {
		case SDEV_RUNNING:
		case SDEV_OFFLINE:
		case SDEV_TRANSPORT_OFFLINE:
			break;
		default:
			goto illegal;
		}
		break;

	case SDEV_OFFLINE:
	case SDEV_TRANSPORT_OFFLINE:
		switch (oldstate) {
		case SDEV_CREATED:
		case SDEV_RUNNING:
		case SDEV_QUIESCE:
		case SDEV_BLOCK:
			break;
		default:
			goto illegal;
		}
		break;

	case SDEV_BLOCK:
		switch (oldstate) {
		case SDEV_RUNNING:
		case SDEV_CREATED_BLOCK:
			break;
		default:
			goto illegal;
		}
		break;

	case SDEV_CREATED_BLOCK:
		switch (oldstate) {
		case SDEV_CREATED:
			break;
		default:
			goto illegal;
		}
		break;

	case SDEV_CANCEL:
		switch (oldstate) {
		case SDEV_CREATED:
		case SDEV_RUNNING:
		case SDEV_QUIESCE:
		case SDEV_OFFLINE:
		case SDEV_TRANSPORT_OFFLINE:
		case SDEV_BLOCK:
			break;
		default:
			goto illegal;
		}
		break;

	case SDEV_DEL:
		switch (oldstate) {
		case SDEV_CREATED:
		case SDEV_RUNNING:
		case SDEV_OFFLINE:
		case SDEV_TRANSPORT_OFFLINE:
		case SDEV_CANCEL:
		case SDEV_CREATED_BLOCK:
			break;
		default:
			goto illegal;
		}
		break;

	}
	sdev->sdev_state = state;
	return 0;

 illegal:
	SCSI_LOG_ERROR_RECOVERY(1,
				sdev_printk(KERN_ERR, sdev,
					    "Illegal state transition %s->%s",
					    scsi_device_state_name(oldstate),
					    scsi_device_state_name(state))
				);
	return -EINVAL;
}
EXPORT_SYMBOL(scsi_device_set_state);

/**
 * 	sdev_evt_emit - emit a single SCSI device uevent
 *	@sdev: associated SCSI device
 *	@evt: event to emit
 *
 *	Send a single uevent (scsi_event) to the associated scsi_device.
 */
static void scsi_evt_emit(struct scsi_device *sdev, struct scsi_event *evt)
{
	int idx = 0;
	char *envp[3];

	switch (evt->evt_type) {
	case SDEV_EVT_MEDIA_CHANGE:
		envp[idx++] = "SDEV_MEDIA_CHANGE=1";
		break;
	case SDEV_EVT_INQUIRY_CHANGE_REPORTED:
		envp[idx++] = "SDEV_UA=INQUIRY_DATA_HAS_CHANGED";
		break;
	case SDEV_EVT_CAPACITY_CHANGE_REPORTED:
		envp[idx++] = "SDEV_UA=CAPACITY_DATA_HAS_CHANGED";
		break;
	case SDEV_EVT_SOFT_THRESHOLD_REACHED_REPORTED:
	       envp[idx++] = "SDEV_UA=THIN_PROVISIONING_SOFT_THRESHOLD_REACHED";
		break;
	case SDEV_EVT_MODE_PARAMETER_CHANGE_REPORTED:
		envp[idx++] = "SDEV_UA=MODE_PARAMETERS_CHANGED";
		break;
	case SDEV_EVT_LUN_CHANGE_REPORTED:
		envp[idx++] = "SDEV_UA=REPORTED_LUNS_DATA_HAS_CHANGED";
		break;
	default:
		/* do nothing */
		break;
	}

	envp[idx++] = NULL;

	kobject_uevent_env(&sdev->sdev_gendev.kobj, KOBJ_CHANGE, envp);
}

/**
 * 	sdev_evt_thread - send a uevent for each scsi event
 *	@work: work struct for scsi_device
 *
 *	Dispatch queued events to their associated scsi_device kobjects
 *	as uevents.
 */
void scsi_evt_thread(struct work_struct *work)
{
	struct scsi_device *sdev;
	enum scsi_device_event evt_type;
	LIST_HEAD(event_list);

	sdev = container_of(work, struct scsi_device, event_work);

	for (evt_type = SDEV_EVT_FIRST; evt_type <= SDEV_EVT_LAST; evt_type++)
		if (test_and_clear_bit(evt_type, sdev->pending_events))
			sdev_evt_send_simple(sdev, evt_type, GFP_KERNEL);

	while (1) {
		struct scsi_event *evt;
		struct list_head *this, *tmp;
		unsigned long flags;

		spin_lock_irqsave(&sdev->list_lock, flags);
		list_splice_init(&sdev->event_list, &event_list);
		spin_unlock_irqrestore(&sdev->list_lock, flags);

		if (list_empty(&event_list))
			break;

		list_for_each_safe(this, tmp, &event_list) {
			evt = list_entry(this, struct scsi_event, node);
			list_del(&evt->node);
			scsi_evt_emit(sdev, evt);
			kfree(evt);
		}
	}
}

/**
 * 	sdev_evt_send - send asserted event to uevent thread
 *	@sdev: scsi_device event occurred on
 *	@evt: event to send
 *
 *	Assert scsi device event asynchronously.
 */
void sdev_evt_send(struct scsi_device *sdev, struct scsi_event *evt)
{
	unsigned long flags;

#if 0
	/* FIXME: currently this check eliminates all media change events
	 * for polled devices.  Need to update to discriminate between AN
	 * and polled events */
	if (!test_bit(evt->evt_type, sdev->supported_events)) {
		kfree(evt);
		return;
	}
#endif

	spin_lock_irqsave(&sdev->list_lock, flags);
	list_add_tail(&evt->node, &sdev->event_list);
	schedule_work(&sdev->event_work);
	spin_unlock_irqrestore(&sdev->list_lock, flags);
}
EXPORT_SYMBOL_GPL(sdev_evt_send);

/**
 * 	sdev_evt_alloc - allocate a new scsi event
 *	@evt_type: type of event to allocate
 *	@gfpflags: GFP flags for allocation
 *
 *	Allocates and returns a new scsi_event.
 */
struct scsi_event *sdev_evt_alloc(enum scsi_device_event evt_type,
				  gfp_t gfpflags)
{
	struct scsi_event *evt = kzalloc(sizeof(struct scsi_event), gfpflags);
	if (!evt)
		return NULL;

	evt->evt_type = evt_type;
	INIT_LIST_HEAD(&evt->node);

	/* evt_type-specific initialization, if any */
	switch (evt_type) {
	case SDEV_EVT_MEDIA_CHANGE:
	case SDEV_EVT_INQUIRY_CHANGE_REPORTED:
	case SDEV_EVT_CAPACITY_CHANGE_REPORTED:
	case SDEV_EVT_SOFT_THRESHOLD_REACHED_REPORTED:
	case SDEV_EVT_MODE_PARAMETER_CHANGE_REPORTED:
	case SDEV_EVT_LUN_CHANGE_REPORTED:
	default:
		/* do nothing */
		break;
	}

	return evt;
}
EXPORT_SYMBOL_GPL(sdev_evt_alloc);

/**
 * 	sdev_evt_send_simple - send asserted event to uevent thread
 *	@sdev: scsi_device event occurred on
 *	@evt_type: type of event to send
 *	@gfpflags: GFP flags for allocation
 *
 *	Assert scsi device event asynchronously, given an event type.
 */
void sdev_evt_send_simple(struct scsi_device *sdev,
			  enum scsi_device_event evt_type, gfp_t gfpflags)
{
	struct scsi_event *evt = sdev_evt_alloc(evt_type, gfpflags);
	if (!evt) {
		sdev_printk(KERN_ERR, sdev, "event %d eaten due to OOM\n",
			    evt_type);
		return;
	}

	sdev_evt_send(sdev, evt);
}
EXPORT_SYMBOL_GPL(sdev_evt_send_simple);

/**
 *	scsi_device_quiesce - Block user issued commands.
 *	@sdev:	scsi device to quiesce.
 *
 *	This works by trying to transition to the SDEV_QUIESCE state
 *	(which must be a legal transition).  When the device is in this
 *	state, only special requests will be accepted, all others will
 *	be deferred.  Since special requests may also be requeued requests,
 *	a successful return doesn't guarantee the device will be 
 *	totally quiescent.
 *
 *	Must be called with user context, may sleep.
 *
 *	Returns zero if unsuccessful or an error if not.
 */
int
scsi_device_quiesce(struct scsi_device *sdev)
{
	int err = scsi_device_set_state(sdev, SDEV_QUIESCE);
	if (err)
		return err;

	scsi_run_queue(sdev->request_queue);
	while (atomic_read(&sdev->device_busy)) {
		msleep_interruptible(200);
		scsi_run_queue(sdev->request_queue);
	}
	return 0;
}
EXPORT_SYMBOL(scsi_device_quiesce);

/**
 *	scsi_device_resume - Restart user issued commands to a quiesced device.
 *	@sdev:	scsi device to resume.
 *
 *	Moves the device from quiesced back to running and restarts the
 *	queues.
 *
 *	Must be called with user context, may sleep.
 */
void scsi_device_resume(struct scsi_device *sdev)
{
	/* check if the device state was mutated prior to resume, and if
	 * so assume the state is being managed elsewhere (for example
	 * device deleted during suspend)
	 */
	if (sdev->sdev_state != SDEV_QUIESCE ||
	    scsi_device_set_state(sdev, SDEV_RUNNING))
		return;
	scsi_run_queue(sdev->request_queue);
}
EXPORT_SYMBOL(scsi_device_resume);

static void
device_quiesce_fn(struct scsi_device *sdev, void *data)
{
	scsi_device_quiesce(sdev);
}

void
scsi_target_quiesce(struct scsi_target *starget)
{
	starget_for_each_device(starget, NULL, device_quiesce_fn);
}
EXPORT_SYMBOL(scsi_target_quiesce);

static void
device_resume_fn(struct scsi_device *sdev, void *data)
{
	scsi_device_resume(sdev);
}

void
scsi_target_resume(struct scsi_target *starget)
{
	starget_for_each_device(starget, NULL, device_resume_fn);
}
EXPORT_SYMBOL(scsi_target_resume);

/**
 * scsi_internal_device_block - internal function to put a device temporarily into the SDEV_BLOCK state
 * @sdev:	device to block
 *
 * Block request made by scsi lld's to temporarily stop all
 * scsi commands on the specified device.  Called from interrupt
 * or normal process context.
 *
 * Returns zero if successful or error if not
 *
 * Notes:       
 *	This routine transitions the device to the SDEV_BLOCK state
 *	(which must be a legal transition).  When the device is in this
 *	state, all commands are deferred until the scsi lld reenables
 *	the device with scsi_device_unblock or device_block_tmo fires.
 */
int
scsi_internal_device_block(struct scsi_device *sdev)
{
	struct request_queue *q = sdev->request_queue;
	unsigned long flags;
	int err = 0;

	err = scsi_device_set_state(sdev, SDEV_BLOCK);
	if (err) {
		err = scsi_device_set_state(sdev, SDEV_CREATED_BLOCK);

		if (err)
			return err;
	}

	/* 
	 * The device has transitioned to SDEV_BLOCK.  Stop the
	 * block layer from calling the midlayer with this device's
	 * request queue. 
	 */
	if (q->mq_ops) {
		blk_mq_stop_hw_queues(q);
	} else {
		spin_lock_irqsave(q->queue_lock, flags);
		blk_stop_queue(q);
		spin_unlock_irqrestore(q->queue_lock, flags);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(scsi_internal_device_block);
 
/**
 * scsi_internal_device_unblock - resume a device after a block request
 * @sdev:	device to resume
 * @new_state:	state to set devices to after unblocking
 *
 * Called by scsi lld's or the midlayer to restart the device queue
 * for the previously suspended scsi device.  Called from interrupt or
 * normal process context.
 *
 * Returns zero if successful or error if not.
 *
 * Notes:       
 *	This routine transitions the device to the SDEV_RUNNING state
 *	or to one of the offline states (which must be a legal transition)
 *	allowing the midlayer to goose the queue for this device.
 */
int
scsi_internal_device_unblock(struct scsi_device *sdev,
			     enum scsi_device_state new_state)
{
	struct request_queue *q = sdev->request_queue; 
	unsigned long flags;

	/*
	 * Try to transition the scsi device to SDEV_RUNNING or one of the
	 * offlined states and goose the device queue if successful.
	 */
	if ((sdev->sdev_state == SDEV_BLOCK) ||
	    (sdev->sdev_state == SDEV_TRANSPORT_OFFLINE))
		sdev->sdev_state = new_state;
	else if (sdev->sdev_state == SDEV_CREATED_BLOCK) {
		if (new_state == SDEV_TRANSPORT_OFFLINE ||
		    new_state == SDEV_OFFLINE)
			sdev->sdev_state = new_state;
		else
			sdev->sdev_state = SDEV_CREATED;
	} else if (sdev->sdev_state != SDEV_CANCEL &&
		 sdev->sdev_state != SDEV_OFFLINE)
		return -EINVAL;

	if (q->mq_ops) {
		blk_mq_start_stopped_hw_queues(q, false);
	} else {
		spin_lock_irqsave(q->queue_lock, flags);
		blk_start_queue(q);
		spin_unlock_irqrestore(q->queue_lock, flags);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(scsi_internal_device_unblock);

static void
device_block(struct scsi_device *sdev, void *data)
{
	scsi_internal_device_block(sdev);
}

static int
target_block(struct device *dev, void *data)
{
	if (scsi_is_target_device(dev))
		starget_for_each_device(to_scsi_target(dev), NULL,
					device_block);
	return 0;
}

void
scsi_target_block(struct device *dev)
{
	if (scsi_is_target_device(dev))
		starget_for_each_device(to_scsi_target(dev), NULL,
					device_block);
	else
		device_for_each_child(dev, NULL, target_block);
}
EXPORT_SYMBOL_GPL(scsi_target_block);

static void
device_unblock(struct scsi_device *sdev, void *data)
{
	scsi_internal_device_unblock(sdev, *(enum scsi_device_state *)data);
}

static int
target_unblock(struct device *dev, void *data)
{
	if (scsi_is_target_device(dev))
		starget_for_each_device(to_scsi_target(dev), data,
					device_unblock);
	return 0;
}

void
scsi_target_unblock(struct device *dev, enum scsi_device_state new_state)
{
	if (scsi_is_target_device(dev))
		starget_for_each_device(to_scsi_target(dev), &new_state,
					device_unblock);
	else
		device_for_each_child(dev, &new_state, target_unblock);
}
EXPORT_SYMBOL_GPL(scsi_target_unblock);

/**
 * scsi_kmap_atomic_sg - find and atomically map an sg-elemnt
 * @sgl:	scatter-gather list
 * @sg_count:	number of segments in sg
 * @offset:	offset in bytes into sg, on return offset into the mapped area
 * @len:	bytes to map, on return number of bytes mapped
 *
 * Returns virtual address of the start of the mapped page
 */
void *scsi_kmap_atomic_sg(struct scatterlist *sgl, int sg_count,
			  size_t *offset, size_t *len)
{
	int i;
	size_t sg_len = 0, len_complete = 0;
	struct scatterlist *sg;
	struct page *page;

	WARN_ON(!irqs_disabled());

	for_each_sg(sgl, sg, sg_count, i) {
		len_complete = sg_len; /* Complete sg-entries */
		sg_len += sg->length;
		if (sg_len > *offset)
			break;
	}

	if (unlikely(i == sg_count)) {
		printk(KERN_ERR "%s: Bytes in sg: %zu, requested offset %zu, "
			"elements %d\n",
		       __func__, sg_len, *offset, sg_count);
		WARN_ON(1);
		return NULL;
	}

	/* Offset starting from the beginning of first page in this sg-entry */
	*offset = *offset - len_complete + sg->offset;

	/* Assumption: contiguous pages can be accessed as "page + i" */
	page = nth_page(sg_page(sg), (*offset >> PAGE_SHIFT));
	*offset &= ~PAGE_MASK;

	/* Bytes in this sg-entry from *offset to the end of the page */
	sg_len = PAGE_SIZE - *offset;
	if (*len > sg_len)
		*len = sg_len;

	return kmap_atomic(page);
}
EXPORT_SYMBOL(scsi_kmap_atomic_sg);

/**
 * scsi_kunmap_atomic_sg - atomically unmap a virtual address, previously mapped with scsi_kmap_atomic_sg
 * @virt:	virtual address to be unmapped
 */
void scsi_kunmap_atomic_sg(void *virt)
{
	kunmap_atomic(virt);
}
EXPORT_SYMBOL(scsi_kunmap_atomic_sg);

void sdev_disable_disk_events(struct scsi_device *sdev)
{
	atomic_inc(&sdev->disk_events_disable_depth);
}
EXPORT_SYMBOL(sdev_disable_disk_events);

void sdev_enable_disk_events(struct scsi_device *sdev)
{
	if (WARN_ON_ONCE(atomic_read(&sdev->disk_events_disable_depth) <= 0))
		return;
	atomic_dec(&sdev->disk_events_disable_depth);
}
EXPORT_SYMBOL(sdev_enable_disk_events);
