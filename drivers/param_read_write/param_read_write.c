/*
 * drivers/param_read_write/param_read_write.c
 *
 * hefaxi@filesystems,2015/04/30
 *
 * This program is used to read/write param partition in kernel
 */

#include <linux/fs.h>
#include <linux/file.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/miscdevice.h>

#include <linux/param_rw.h>


#define PARAM_PARTITION "/dev/block/bootdevice/by-name/param"
static uint default_param_data_dump_size= DEFAULT_PARAM_DUMP_SIZE;

typedef struct{
	phys_addr_t paddr;
	size_t size;
	void *vaddr;
	void *buffer;
	struct mutex mutex;
}param_ram_zone_t;

static DEFINE_MUTEX(param_lock);
static bool param_init_done = 0;
static param_ram_zone_t param_ram_zone;

static int write_param_partition(const char *buf, unsigned long count,
            loff_t offset)
{
	struct file *filp;
	mm_segment_t fs;
	int ret = 0;

	filp = filp_open(PARAM_PARTITION,O_RDWR|O_SYNC,0);
	if(IS_ERR(filp)) {
	    ret = PTR_ERR(filp);
		pr_err("open file %s failed.(%d)\n",PARAM_PARTITION,ret);
		return ret;
	}

	fs = get_fs();
	set_fs(get_ds());

	ret = filp->f_op->llseek(filp, offset, SEEK_SET);
	if(ret < 0){
		pr_err("%s: llseek failed.(%d)\n",__func__,ret);
		goto out;
	}
	ret = filp->f_op->write(filp,(char __user *)buf,count,&filp->f_pos);

out:
	set_fs(fs);
	filp_close(filp,NULL);
	return ret;
}

static int get_param_by_index_and_offset(uint32 sid_index,
            uint32 offset, void * buf, int length)
{
    int ret = length;
    uint32 file_offset;
	mutex_lock(&param_ram_zone.mutex);
	pr_info("%s[%d]  sid_index = %d offset = %d buf = %p length = %d\n",
			__func__, __LINE__,sid_index,offset,buf,length);

    file_offset = PARAM_SID_LENGTH*sid_index+ offset;

	if(buf && (sid_index < PARAM_SID_INVALID) && ((offset + length)
	                <= PARAM_SID_LENGTH))
		memcpy(buf,(param_ram_zone.buffer +file_offset), length);
	else{
		pr_info("%s:invaild argument,sid_index=%d offset=%d buf=%p length=%d\n",
		      __func__,sid_index,offset,buf,length);
		ret = -EINVAL;
	}

	mutex_unlock(&param_ram_zone.mutex);
	return ret;
}

static int set_param_by_index_and_offset(uint32 sid_index,
        uint32 offset, void * buf, int length)
{
    int ret;
    uint32 file_offset;
	mutex_lock(&param_ram_zone.mutex);
	pr_info("%s[%d]sid_index = %d offset = %d buf = %p length = %d\n",
			__func__, __LINE__,sid_index,offset,buf,length);

    file_offset = PARAM_SID_LENGTH*sid_index + offset;

	if(buf && (sid_index < PARAM_SID_INVALID) &&
            ((offset + length) <= PARAM_SID_LENGTH))
		memcpy((param_ram_zone.buffer+file_offset),buf,length);
	else{
		pr_info("%s:invaild argument,sid_index=%d offset=%d buf=%p length=%d\n",
		      __func__,sid_index,offset,buf,length);
		ret = -EINVAL;
		goto out;
	}

    ret = write_param_partition((param_ram_zone.buffer+file_offset),
                    length,file_offset);
	if ( ret < 0){
		pr_info("Error write param partition.(%d)\n",ret);
	}
out:
	mutex_unlock(&param_ram_zone.mutex);
	return ret;
}

static void *persistent_ram_vmap(phys_addr_t start, size_t size)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	prot = pgprot_noncached(PAGE_KERNEL);

	pages = kmalloc(sizeof(struct page *) * page_count, GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n", __func__,
				page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);
	return vaddr;
}

static int param_ram_buffer_map(phys_addr_t start, phys_addr_t size,
		param_ram_zone_t *prz)
{
	prz->paddr = start;
	prz->size = size;
	prz->vaddr = persistent_ram_vmap(start, size);

	if (!prz->vaddr) {
		pr_err("%s: Failed to map 0x%llx pages at 0x%llx\n", __func__,
				(unsigned long long)size, (unsigned long long)start);
		return -ENOMEM;
	}

	prz->buffer = prz->vaddr + offset_in_page(start);
	return 0;
}

//This function just for test
static int set_param_gamma_select(uint * lcd_gamma_select)
{
	int ret;
	uint32 sid_index= PARAM_SID_LCD;
	uint32 offset = offsetof(param_lcd_t, gamma_select);

	ret = set_param_by_index_and_offset(sid_index,offset,lcd_gamma_select, sizeof(*lcd_gamma_select));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
	return ret;
}

static int param_test = 0;
static int param_set_test(const char *val, struct kernel_param *kp)
{
	int ret;
	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	ret = set_param_gamma_select(&param_test);
	pr_info("%s[%d]  param_test = %d ret = %d\n",
	        __func__, __LINE__, param_test,ret);
	return 0;
}

module_param_call(param_test, param_set_test, param_get_int, &param_test, 0644);

static ssize_t param_read(struct file *file, char __user *buff,
            size_t count, loff_t *pos)
{
	void * temp_buffer;
	int ret;
	if (mutex_lock_interruptible(&param_lock))
		return -ERESTARTSYS;

	temp_buffer = kzalloc(count, GFP_KERNEL);
	ret = get_param_by_index_and_offset(*pos/PARAM_SID_LENGTH,
	        *pos%PARAM_SID_LENGTH, temp_buffer, count);
	if(ret < 0){
		pr_err("get_param_by_index_and_offset failure %d\n",ret);
		goto out;
	}

	ret =copy_to_user(buff, temp_buffer, count);
	if (ret < 0) {
		pr_info("copy_to_user failure %d\n", ret );
		goto out;
	}
	*pos += ret;

out:
	kfree(temp_buffer);
	mutex_unlock(&param_lock);
	return ret;
}

static ssize_t param_write(struct file *file, const char __user *buff,
        size_t count, loff_t *pos)
{
	void * temp_buffer;
	int ret;
	if (mutex_lock_interruptible(&param_lock))
		return -ERESTARTSYS;

	temp_buffer = kzalloc(count, GFP_KERNEL);

	ret =copy_from_user(temp_buffer, buff, count);
	if (ret < 0) {
		pr_info("copy_from_user failure %d\n", ret);
		goto out;
	}

	ret = set_param_by_index_and_offset(*pos/PARAM_SID_LENGTH,
	        *pos%PARAM_SID_LENGTH, temp_buffer, count);
	if(ret < 0){
		pr_err("set_param_by_index_and_offset failure %d\n",ret);
		goto out;
	}

	*pos += ret;
out:
	kfree(temp_buffer);
	mutex_unlock(&param_lock);
	return ret;

}

static const struct file_operations param_fops = {
	.owner          = THIS_MODULE,
	.read           = param_read,
	.write          = param_write,
	.llseek		= default_llseek,
};

struct miscdevice param_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "param",
	.fops = &param_fops,
};

static int __init param_core_init(void)
{
	int i;
	if(param_ram_buffer_map((phys_addr_t)param_ram_zone.paddr,
	        param_ram_zone.size, (param_ram_zone_t *)&param_ram_zone)){
		pr_err("param_ram_buffer_map failred\n");
		return -1;
	}
	mutex_init(&param_ram_zone.mutex);
	for(i = 0 ; i < PARAM_SID_INVALID; i ++){
		break;//do not dump param
		printk("===dump chunk %d===\n", i);
		print_hex_dump (KERN_ERR, "",DUMP_PREFIX_OFFSET,16, 4,
		    param_ram_zone.buffer +1024*i, default_param_data_dump_size,1);
	}
	param_init_done= 1;
	return 0;
}
pure_initcall(param_core_init);


static int dump_index = -1;
static int convert_to_hex(char *dst, const char *src, size_t len)
{
    int i;
    int rowsize = 16;
    int groupsize = 4;
    int ret = 0;
    int count;
    int linelen;
    int ascii = 1;
    int remaining = len;

    unsigned char linebuf[64];
    for (i = 0; i < len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;
		hex_dump_to_buffer(src + i, linelen, rowsize, groupsize,
				   linebuf, sizeof(linebuf), ascii);
		count = strlen(linebuf);
		sprintf(dst+ret,"%s\n",linebuf);
		ret += count+1;
	};

	return ret;
}

static ssize_t param_dump_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int count = 0;
    char *tmp_buf;
    int ret;

    tmp_buf = kzalloc(PAGE_SIZE,GFP_KERNEL);
    if(!tmp_buf){
        pr_err("%s: kzalloc failed.\n",__func__);
        return -ENOMEM;
    }

    if(dump_index >= 0 && dump_index < PARAM_SID_INVALID){
        count = get_param_by_index_and_offset(dump_index,
                0,tmp_buf,PARAM_SID_LENGTH);
    }else{
        int i,ret;
        for(i = 0; i < PARAM_SID_INVALID; ++i){
            ret = get_param_by_index_and_offset(i,
                0,tmp_buf+count,DEFAULT_PARAM_DUMP_SIZE);
            count += ret;
        }
    }

    if(count > 0)
        ret = convert_to_hex(buf,tmp_buf,count);

    kfree(tmp_buf);
    return ret;
}
static ssize_t param_dump_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t size)
{
    if(!strncmp("all",buf,3)){
        dump_index = -1;
    }else{
        dump_index = simple_strtoul(buf,NULL,10);
    }

    return strnlen(buf,size);
}
static DEVICE_ATTR(param_dump,0644,
    param_dump_show,param_dump_store);

static int __init param_device_init(void)
{
	int ret;
	ret = misc_register(&param_misc);
	if(ret){
		pr_err("misc_register failure %d\n",ret);
		return -1;
	}

	if(device_create_file(param_misc.this_device,
	    &dev_attr_param_dump) < 0)
	{
	    pr_err("Failed to create device file!(%s)!\n",
                dev_attr_param_dump.attr.name);
		ret = -1;
	}

	return ret;
}
device_initcall(param_device_init);

void init_param_mem_base_size(phys_addr_t base, unsigned long size)
{
	param_ram_zone.paddr = base;
	param_ram_zone.size = size;
}
EXPORT_SYMBOL(init_param_mem_base_size);

/*
*Add more function here
*
*/
int get_param_camera_laser_sensor_offset(uint * laser_sensor_offset)
{
	int ret;
	uint32 sid_index= PARAM_SID_CAMERA;
	uint32 offset = offsetof(param_camera_t, laser_sensor_offset);

	ret = get_param_by_index_and_offset(sid_index,offset,
	            laser_sensor_offset, sizeof(*laser_sensor_offset));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
        pr_info("laser_sensor_offset = %d\n", *laser_sensor_offset);
	return ret;
}
EXPORT_SYMBOL(get_param_camera_laser_sensor_offset);

int set_param_camera_laser_sensor_offset(uint * laser_sensor_offset)
{
	int ret;
	uint32 sid_index= PARAM_SID_CAMERA;
	uint32 offset = offsetof(param_camera_t, laser_sensor_offset);

	ret = set_param_by_index_and_offset(sid_index,offset,
	            laser_sensor_offset, sizeof(*laser_sensor_offset));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
        pr_info("set laser_sensor_offset = %d\n", *laser_sensor_offset);
	return ret;
}
EXPORT_SYMBOL(set_param_camera_laser_sensor_offset);

int get_param_camera_laser_sensor_cross_talk(uint * laser_sensor_cross_talk)
{
	int ret;
	uint32 sid_index= PARAM_SID_CAMERA;
	uint32 offset = offsetof(param_camera_t, laser_sensor_cross_talk);

	ret = get_param_by_index_and_offset(sid_index,offset,
	            laser_sensor_cross_talk, sizeof(*laser_sensor_cross_talk));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
        pr_info("laser_sensor_cross_talk = %d\n", *laser_sensor_cross_talk);
	return ret;
}
EXPORT_SYMBOL(get_param_camera_laser_sensor_cross_talk);

int set_param_camera_laser_sensor_cross_talk(uint * laser_sensor_cross_talk)
{
	int ret;
	uint32 sid_index= PARAM_SID_CAMERA;
	uint32 offset = offsetof(param_camera_t, laser_sensor_cross_talk);

	ret = set_param_by_index_and_offset(sid_index,offset,
	        laser_sensor_cross_talk, sizeof(*laser_sensor_cross_talk));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
	pr_info("set laser_sensor_cross_talk = %d\n", *laser_sensor_cross_talk);
	return ret;
}
EXPORT_SYMBOL(set_param_camera_laser_sensor_cross_talk);

int get_param_gamma_select(uint * gamma_select)
{
	int ret;
	uint32 sid_index= PARAM_SID_LCD;
	uint32 offset = offsetof(param_lcd_t, gamma_select);

	ret = get_param_by_index_and_offset(sid_index,offset,gamma_select,
	            sizeof(*gamma_select));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(get_param_gamma_select);

/* Only for wlan evm chip */
int get_param_nvm_boarddata(uint * nvm_boarddata_select)
{
	int ret;
	uint32 sid_index= PARAM_SID_MISC;
	uint32 offset = offsetof(param_misc_t, use_special_boarddata);

	ret = get_param_by_index_and_offset(sid_index,offset,nvm_boarddata_select,
	            sizeof(*nvm_boarddata_select));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(get_param_nvm_boarddata);

int get_param_pcba_number(char *pcba_number_select)
{
	int ret;
	uint32 sid_index= PARAM_SID_PRODUCT;
	uint32 offset = offsetof(param_product_t, pcba_number);

	param_product_t project_info;
	ret = get_param_by_index_and_offset(sid_index,offset,pcba_number_select, sizeof(project_info.pcba_number));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(get_param_pcba_number);
