/*
 * The sdcardfs
 *
 * Copyright (c) 2013 Samsung Electronics Co. Ltd
 *   Authors: Daeho Jeong, Woojoong Lee, Kitae Lee, Yeongjin Gil
 *
 * Revision History
 * 2014.06.24 : Release Version 2.1.0
 *    - Add sdcardfs version
 *    - Add kernel log when put_super
 * 2014.07.21 : Release Version 2.1.1
 *    - Add sdcardfs_copy_inode_attr() to fix permission issue
 *    - Delete mmap_sem lock in sdcardfs_setattr() to avoid deadlock
 * 2014.11.12 : Release Version 2.1.2
 *    - Add get_lower_file function pointer in file_operations
 * 2014.11.25 : Release Version 2.1.3
 *    - Add error handling routine in sdcardfs_d_revalidate
 *          when dentry is equal to lower_dentry
 * 2015.03.25 : Release Version 2.1.4
 *    - Add FMODE_NONMAPPABLE, FMODE_NONCACHEABLE flag to file->f_mode
 *    - Modify do_mmap_pgoff because of new f_mode flags
 * 2015.07.   : Release Version 3.0.0
 * 2015.11.24 : Release Version 3.1.0
 *    - Add unlink_callback(), get_lower_inode()
 *    - Add mount option type, label
 */

#define SDCARDFS_VERSION "3.1.0"
