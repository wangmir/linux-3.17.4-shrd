#ifndef _SCSI_SHRD_H
#define _SCSI_SHRD_H

#include <linux/list.h>


#ifdef CONFIG_SCSI_SHRD_TEST0

#define SHRD_SECTORS_PER_PAGE 8

#define SHRD_RW_THRESHOLD_IN_SECTOR 32

#define SHRD_TWRITE_ENTRIES 32
#define SHRD_REMAP_ENTRIES 32

#define SHRD_RW_LOG_SIZE_IN_MB 32
#define SHRD_RW_LOG_SIZE_IN_PAGE (SHRD_RW_LOG_SIZE_IN_MB * 256)

#define SHRD_JN_LOG_SIZE_IN_MB 128
#define SHRD_JN_LOG_SIZE_IN_PAGE (SHRD_JN_LOG_SIZE_IN_MB * 256)

#define SHRD_MIN_RW_LOGGING_IO_SIZE_IN_PAGE 64

#define SHRD_MAX_TWRITE_IO_SIZE_IN_SECTOR 1024
#define SHRD_NUM_MAX_TWRITE_ENTRY 128
#define SHRD_NUM_MAX_REMAP_ENTRY 511

#define SHRD_INVALID_LPN 0x7fffffff

struct SHRD_MAP {
	struct rb_node node;
	u32 o_addr;
	u32 t_addr; //taddr is actually same as the array offset of each map e.g. shrd_rw_map[i] == shrd_rw_map[i].t_addr;
};

/*
	This structure is less than 1KB, 
	but will be requested as two 4KB page with same data (or 4KB can be duplicated from HIL of SSD)
*/
struct SHRD_TWRITE_HEADER{
	u32 t_addr_start; 		//indicate WAL log start page addr
	u32 io_count; 		//indicate how many writes (page) will be requested
	u32 o_addr[128]; 		//o_addr (or h_addr) for each page (1<<32: padding, 1<<31: invalid(like journal header or journal commit))
};

struct SHRD_TWRITE{
	struct list_head twrite_cmd_list;  //is used for ongoing list and free cmd list
	struct list_head req_list; //'struct request' entries from twrite data will be listed from here.
	struct SHRD_TWRITE_HEADER *twrite_hdr;
	u32 blocks;
	u32 phys_segments;
	u8 nr_entries;
	u8 in_use; 
};
/*
	WARNING:: we need to modify TWRITE_HEADER and REMAP_DATA as buddy page rather than kmalloc memory.
	because these data will be requested as a bio (or sglist)
*/

/*
	This structure will be array with two 4KB entry.
	Each remap header is for each FTL core.
*/
struct SHRD_REMAP_DATA{
	u32 t_addr_start;
	u32 remap_count;
	u32 t_addr[511];
	u32 o_addr[511];
};

struct SHRD_REMAP{
	struct list_head remap_cmd_list; //is used for ongoing list and free cmd list
	struct SHRD_REMAP_DATA *remap_data;
	u8 in_use;
};

struct SHRD{	
	
	//SHRD map table RB tree root
	struct rb_root rw_mapping;
	struct rb_root jn_mapping;
	
	//SHRD map table (no need to acquire lock because we already allocated entries with log address)
	struct SHRD_MAP *shrd_rw_map;
	struct SHRD_MAP *shrd_jn_map;

	//SHRD command entry (each cmd has 32 command entries (at the init()) to serve NCQ command)
	struct SHRD_TWRITE *twrite_cmd;
	struct SHRD_REMAP *remap_cmd;

	//these lists are used for finding free cmd entries and complete ongoing cmd entries.
	struct list_head free_twrite_cmd_list;
	struct list_head ongoing_twrite_cmd_list;
	struct list_head free_remap_cmd_list;
	struct list_head ongoing_remap_cmd_list;

	//for each index indicator for write and remap, should acquire lock to handle each entries.
	u32 rw_log_start_idx;
	u32 rw_log_new_idx;
	//u32 rw_log_allocated_idx; 
	u32 jn_log_start_idx;
	u32 jn_log_new_idx;
	//u32 jn_log_allocated_idx;  

	/*
	protect each log enries from double allocation. __xx_log_lock should never be used directly.
	Use *xx_log_lock rather than __xx_log_lock
	*/
	spinlock_t __rw_log_lock;
	spinlock_t *rw_log_lock;
	spinlock_t __jn_log_lock;
	spinlock_t *jn_log_lock;
	
};

static inline struct SHRD_TWRITE* shrd_get_twrite_entry(struct SHRD *shrd){

	return list_first_entry_or_null(&shrd->free_twrite_cmd_list, struct SHRD_TWRITE, twrite_cmd_list);
}

static inline void shrd_put_twrite_entry(struct SHRD *shrd, struct SHRD_TWRITE* entry){

	list_add_tail(&entry->twrite_cmd_list, &shrd->free_twrite_cmd_list);
}

static inline struct SHRD_REMAP* shrd_get_remap_entry(struct SHRD *shrd){

	return list_first_entry_or_null(&shrd->free_remap_cmd_list, struct SHRD_REMAP, remap_cmd_list);
}

static inline void shrd_put_remap_entry(struct SHRD *shrd, struct SHRD_REMAP* entry){
	list_add_tail(&entry->remap_cmd_list, &shrd->free_remap_cmd_list);
}

static inline void shrd_clear_twrite_entry(struct SHRD_TWRITE* entry){

	LIST_HEAD_INIT(&entry->req_list);
	entry->twrite_cmd_list = NULL;
	entry->blocks = 0;
	entry->nr_entries = 0;
	entry->in_use = 0;

}

static inline void shrd_clear_remap_entry(struct SHRD_REMAP* entry){

	entry->remap_cmd_list = NULL;
	entry->in_use = 0;
}

#endif
#endif
