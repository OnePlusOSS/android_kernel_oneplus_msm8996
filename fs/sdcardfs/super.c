/*
 * fs/sdcardfs/super.c
 *
 * Copyright (c) 2013 Samsung Electronics Co. Ltd
 *   Authors: Daeho Jeong, Woojoong Lee, Seunghwan Hyun,
 *               Sunghwan Yun, Sungjong Seo
 *
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by
 *
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009     Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 *
 * This file is dual licensed.  It may be redistributed and/or modified
 * under the terms of the Apache 2.0 License OR version 2 of the GNU
 * General Public License.
 */

#include "sdcardfs.h"

/*
 * The inode cache is used with alloc_inode for both our inode info and the
 * vfs inode.
 */
static struct kmem_cache *sdcardfs_inode_cachep;

/* final actions when unmounting a file system */
static void sdcardfs_put_super(struct super_block *sb)
{
	struct sdcardfs_sb_info *spd;
	struct super_block *s;

	spd = SDCARDFS_SB(sb);
	if (!spd)
		return;

	printk(KERN_ERR "sdcardfs: umounted dev_name %s\n",
				spd->devpath ? spd->devpath : "");
	if(spd->devpath)
		kfree(spd->devpath);

	if(spd->obbpath_s) {
		kfree(spd->obbpath_s);
		path_put(&spd->obbpath);
	}

	if(spd->options.label)
		kfree(spd->options.label);

	/* decrement lower super references */
	s = sdcardfs_lower_super(sb);
	sdcardfs_set_lower_super(sb, NULL);
	atomic_dec(&s->s_active);

	if(spd->pkgl_id)
		packagelist_destroy(spd->pkgl_id);

	kfree(spd);
	sb->s_fs_info = NULL;
}

static int sdcardfs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	int err;
	struct path lower_path;
	u32 min_blocks;
	struct sdcardfs_sb_info *sbi = SDCARDFS_SB(dentry->d_sb);

	sdcardfs_get_lower_path(dentry, &lower_path);
	err = vfs_statfs(&lower_path, buf);
	sdcardfs_put_lower_path(dentry, &lower_path);

	if (sbi->options.reserved_mb) {
		/* Invalid statfs informations. */
		if (buf->f_bsize == 0) {
			printk(KERN_ERR "Returned block size is zero.\n");
			return -EINVAL;
		}

		min_blocks = ((sbi->options.reserved_mb * 1024 * 1024)/buf->f_bsize);
		buf->f_blocks -= min_blocks;

		if (buf->f_bavail > min_blocks)
			buf->f_bavail -= min_blocks;
		else
			buf->f_bavail = 0;

		/* Make reserved blocks invisiable to media storage */
		buf->f_bfree = buf->f_bavail;
	}

	/* set return buf to our f/s to avoid confusing user-level utils */
	buf->f_type = SDCARDFS_SUPER_MAGIC;

	return err;
}

/*
 * @flags: numeric mount options
 * @options: mount options string
 */
static int sdcardfs_remount_fs(struct super_block *sb, int *flags, char *options)
{
	int err = 0;

	/*
	 * The VFS will take care of "ro" and "rw" flags among others.  We
	 * can safely accept a few flags (RDONLY, MANDLOCK), and honor
	 * SILENT, but anything else left over is an error.
	 */
	if ((*flags & ~(MS_RDONLY | MS_MANDLOCK | MS_SILENT)) != 0) {
		printk(KERN_ERR
		       "sdcardfs: remount flags 0x%x unsupported\n", *flags);
		err = -EINVAL;
	}

	return err;
}

/*
 * Called by iput() when the inode reference count reached zero
 * and the inode is not hashed anywhere.  Used to clear anything
 * that needs to be, before the inode is completely destroyed and put
 * on the inode free list.
 */
static void sdcardfs_evict_inode(struct inode *inode)
{
	struct inode *lower_inode;

	truncate_inode_pages(&inode->i_data, 0);
	clear_inode(inode);
	/*
	 * Decrement a reference to a lower_inode, which was incremented
	 * by our read_inode when it was created initially.
	 */
	lower_inode = sdcardfs_lower_inode(inode);
	sdcardfs_set_lower_inode(inode, NULL);
	iput(lower_inode);
}

static struct inode *sdcardfs_alloc_inode(struct super_block *sb)
{
	struct sdcardfs_inode_info *i;

	i = kmem_cache_alloc(sdcardfs_inode_cachep, GFP_KERNEL);
	if (!i)
		return NULL;

	/* memset everything up to the inode to 0 */
	memset(i, 0, offsetof(struct sdcardfs_inode_info, vfs_inode));

	i->vfs_inode.i_version = 1;
	return &i->vfs_inode;
}

static void sdcardfs_destroy_inode(struct inode *inode)
{
	kmem_cache_free(sdcardfs_inode_cachep, SDCARDFS_I(inode));
}

/* sdcardfs inode cache constructor */
static void init_once(void *obj)
{
	struct sdcardfs_inode_info *i = obj;

	inode_init_once(&i->vfs_inode);
}

int sdcardfs_init_inode_cache(void)
{
	int err = 0;

	sdcardfs_inode_cachep =
		kmem_cache_create("sdcardfs_inode_cache",
				  sizeof(struct sdcardfs_inode_info), 0,
				  SLAB_RECLAIM_ACCOUNT, init_once);
	if (!sdcardfs_inode_cachep)
		err = -ENOMEM;
	return err;
}

/* sdcardfs inode cache destructor */
void sdcardfs_destroy_inode_cache(void)
{
	if (sdcardfs_inode_cachep)
		kmem_cache_destroy(sdcardfs_inode_cachep);
}

/*
 * Used only in nfs, to kill any pending RPC tasks, so that subsequent
 * code can actually succeed and won't leave tasks that need handling.
 */

long sdcardfs_propagate_unlink(struct inode *parent, char* pathname) {
	long ret = 0;
	char *propagate_path = NULL;
	struct sdcardfs_sb_info *sbi;
	struct path sibling_path;
	const struct cred *saved_cred = NULL;

	sbi = SDCARDFS_SB(parent->i_sb);
	propagate_path = kmalloc(PATH_MAX, GFP_KERNEL);
	OVERRIDE_ROOT_CRED(saved_cred);
	if (sbi->options.type != TYPE_NONE && sbi->options.type != TYPE_DEFAULT) {
		snprintf(propagate_path, PATH_MAX, "/mnt/runtime/default/%s%s",
				sbi->options.label, pathname);
		ret = (long)kern_path(propagate_path, LOOKUP_FOLLOW, &sibling_path);
		if (!ret)
			path_put(&sibling_path);
	}

	if (sbi->options.type != TYPE_NONE && sbi->options.type != TYPE_READ) {
		snprintf(propagate_path, PATH_MAX, "/mnt/runtime/read/%s%s",
				sbi->options.label, pathname);
		ret = (long)kern_path(propagate_path, LOOKUP_FOLLOW, &sibling_path);
		if (!ret)
			path_put(&sibling_path);
	}

	if (sbi->options.type != TYPE_NONE && sbi->options.type != TYPE_WRITE) {
		snprintf(propagate_path, PATH_MAX, "/mnt/runtime/write/%s%s",
				sbi->options.label, pathname);
		ret = (long)kern_path(propagate_path, LOOKUP_FOLLOW, &sibling_path);
		if (!ret)
			path_put(&sibling_path);
	}

	if (sbi->options.type != TYPE_NONE) {
		snprintf(propagate_path, PATH_MAX, "/storage/%s%s",
				sbi->options.label, pathname);
		ret = (long)kern_path(propagate_path, LOOKUP_FOLLOW, &sibling_path);
		if (!ret)
			path_put(&sibling_path);
	}
	REVERT_CRED(saved_cred);
	kfree(propagate_path);
	return ret;
}

static void sdcardfs_umount_begin(struct super_block *sb)
{
	struct super_block *lower_sb;

	lower_sb = sdcardfs_lower_super(sb);
	if (lower_sb && lower_sb->s_op && lower_sb->s_op->umount_begin)
		lower_sb->s_op->umount_begin(lower_sb);
}

static int sdcardfs_show_options(struct seq_file *m, struct dentry *root)
{
	struct sdcardfs_sb_info *sbi = SDCARDFS_SB(root->d_sb);
	struct sdcardfs_mount_options *opts = &sbi->options;

	if (opts->fs_low_uid != 0)
		seq_printf(m, ",low_uid=%u", opts->fs_low_uid);
	if (opts->fs_low_gid != 0)
		seq_printf(m, ",low_gid=%u", opts->fs_low_gid);
	if (opts->gid != 0)
		seq_printf(m, ",gid=%u", opts->gid);
	if (opts->userid != 0)
		seq_printf(m, ",userid=%u", opts->userid);
	if (opts->multi_user)
		seq_printf(m, ",multi_user");
	if (opts->mask != 0)
		seq_printf(m, ",mask=%04o", opts->mask);
	if (opts->reserved_mb != 0)
		seq_printf(m, ",reserved=%uMB", opts->reserved_mb);

	return 0;
};

const struct super_operations sdcardfs_sops = {
	.put_super	= sdcardfs_put_super,
	.statfs		= sdcardfs_statfs,
	.remount_fs	= sdcardfs_remount_fs,
	.evict_inode	= sdcardfs_evict_inode,
	.umount_begin	= sdcardfs_umount_begin,
	.show_options	= sdcardfs_show_options,
	.alloc_inode	= sdcardfs_alloc_inode,
	.destroy_inode	= sdcardfs_destroy_inode,
	.drop_inode	= generic_delete_inode,
	.unlink_callback = sdcardfs_propagate_unlink,
};
