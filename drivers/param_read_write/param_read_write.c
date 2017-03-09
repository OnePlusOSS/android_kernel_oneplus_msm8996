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
#include <linux/proc_fs.h>
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

static int is_angela = 0;
static int param_set_angela(const char *val, struct kernel_param *kp)
{
	int is_angela_in_ufs = 0;
	uint32 sid_index= PARAM_SID_SALEINFO;
	uint32 offset = offsetof(param_saleinfo_t, is_angela);

	get_param_by_index_and_offset(sid_index,offset, &is_angela_in_ufs, sizeof(is_angela_in_ufs));

	sscanf(val, "%d", &is_angela);
	if(is_angela_in_ufs != is_angela)
		set_param_by_index_and_offset(sid_index,offset, &is_angela, sizeof(is_angela));

	return 0;
}

static int param_get_angela(char *val, struct kernel_param *kp)
{

	int cnt = 0;
	uint32 sid_index= PARAM_SID_SALEINFO;
	uint32 offset = offsetof(param_saleinfo_t, is_angela);

	get_param_by_index_and_offset(sid_index,offset, &is_angela, sizeof(is_angela));
	cnt = sprintf(val, "%d", is_angela);

	return cnt;
}

module_param_call(is_angela, param_set_angela, param_get_angela, &is_angela, 0644);

static int data_stage = 0;
static int param_set_data_stage(const char *val, struct kernel_param *kp)
{
	int data_in_ufs = 0;
	uint32 sid_index= PARAM_SID_DOWNLOAD;
	uint32 offset = offsetof(param_download_t, data_stage);

	get_param_by_index_and_offset(sid_index,offset, &data_in_ufs, sizeof(data_stage));

	sscanf(val, "%d", &data_stage);
	if(data_in_ufs != data_stage && (data_stage == BOOT_INTO_ANDROID || data_stage == BOOT_UP))
		set_param_by_index_and_offset(sid_index,offset, &data_stage, sizeof(data_stage));

	offset = offsetof(param_download_t, boot_stage);
	get_param_by_index_and_offset(sid_index,offset, &data_in_ufs, sizeof(data_in_ufs));
	if(data_in_ufs != data_stage && (data_stage == BOOT_INTO_ANDROID))
		set_param_by_index_and_offset(sid_index,offset, &data_stage, sizeof(data_stage));

	return 0;
}

static int param_get_data_stage(char *val, struct kernel_param *kp)
{

	int cnt = 0;
	uint32 sid_index= PARAM_SID_DOWNLOAD;
	uint32 offset = offsetof(param_download_t, data_stage);

	get_param_by_index_and_offset(sid_index,offset, &data_stage, sizeof(data_stage));
	cnt = sprintf(val, "%d", data_stage);
	pr_err("param_get_data_stage data_stage:%d\n",data_stage);

	offset = offsetof(param_download_t, boot_stage);
	get_param_by_index_and_offset(sid_index,offset, &data_stage, sizeof(data_stage));
	pr_err("param_get_data_stage boot_stage:%d\n",data_stage);

	return cnt;
}

module_param_call(data_stage, param_set_data_stage, param_get_data_stage, &data_stage, 0644);

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

static ssize_t  proc_network_info_read(struct file *f, char __user *buf, size_t count, loff_t *ppos)
{
	uint32 sid_index = PARAM_SID_PRODUCT;
	uint32 offset = offsetof(param_product_t, network_info_array);
	char buffer[11];

	memset(buffer, 0, sizeof(buffer));

	/*Split data to 4 byte to avoid device crash*/
	get_param_by_index_and_offset(sid_index, offset, buffer, 4);
	get_param_by_index_and_offset(sid_index, offset+4, buffer+4, 4);
	get_param_by_index_and_offset(sid_index, offset+8, buffer+8, 2);

	return simple_read_from_buffer(buf, count, ppos, buffer, 10);

}

static ssize_t  proc_network_info_write(struct file *f, const char __user *buf, size_t count, loff_t *ppos)
{
	uint32 sid_index = PARAM_SID_PRODUCT;
	uint32 offset = offsetof(param_product_t, network_info_array);
	char buffer[11];

	memset(buffer, 0, sizeof(buffer));

	if (copy_from_user(buffer, buf, 10))
		return -EFAULT;

	/*Split data to 4 byte to avoid device crash*/
	set_param_by_index_and_offset(sid_index, offset, buffer, 4);
	set_param_by_index_and_offset(sid_index, offset+4, buffer+4, 4);
	set_param_by_index_and_offset(sid_index, offset+8, buffer+8, 2);

	return count;
}

static const struct file_operations network_info_knob_fops = {
	.open	= simple_open,
	.read	= proc_network_info_read,
	.write	= proc_network_info_write,
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

	proc_create_data("network_info", 0666, NULL, &network_info_knob_fops, NULL);

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

#define MAX_TYPE_RECORD_COUNT 48
#define PARAM_CHG_TYPE_RECORD_SIZE 20

static ssize_t param_charger_type_dump_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret=0;
    int i=0;
    char temp_crash_record_value[PARAM_CHG_TYPE_RECORD_SIZE];
    int count_size=0;
    char temp_buffer[1536];
    uint32 sid_index= PARAM_SID_BATTERY;

    /* Find the first crash record offset */
    uint32 offset = offsetof(param_chg_type_record_t, type_record_0);

    sprintf(temp_buffer,"==PARAM CHG TYPE RECORD DUMP:SDP:1OTHER:2DCP:3CDP:4DASH:5Absent:6HVDCP:7Other:8NONE:9==\n");
    count_size = count_size+50;

    for(i=0; i<MAX_TYPE_RECORD_COUNT; i++) {

        /* Clean temp buffer */
        memset(temp_crash_record_value, 0, PARAM_CHG_TYPE_RECORD_SIZE);

        /* PARAM_CRASH_RECORD_SIZE-1 -> drop last unnecessary char '.' */
        ret = get_param_by_index_and_offset(sid_index, offset, temp_crash_record_value, PARAM_CHG_TYPE_RECORD_SIZE-1);

       // pr_info("temp_crash_record_value = %s, %d\n", temp_crash_record_value,offset);

        /* If the first char is '\0', it means this record is empty */
        if(temp_crash_record_value[0] == '\0') {
            strcat(temp_buffer, "EMPTY\n");
            count_size = count_size+6;
        } else {

            /* Add crash record string into temp buffer */
            strcat(temp_buffer, temp_crash_record_value);
            count_size = count_size+PARAM_CHG_TYPE_RECORD_SIZE;

            strcat(temp_buffer, "\n");;
        }


        if(ret < 0){
            pr_info("%s[%d]  failed\n",__func__, __LINE__);
            return ret;
        }

        /* Go to next crash record */
        offset = offset+ PARAM_CHG_TYPE_RECORD_SIZE;
    }
    strcat(temp_buffer, " \0");
    strncpy(buf, temp_buffer, count_size);
    //pr_info("buf = %s, %d\n", buf, count_size);
    return count_size;
}


#define MAX_RECORD_COUNT 16
#define PARAM_CRASH_RECORD_SIZE 20
static ssize_t param_crash_record_dump_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret=0;
    int i=0;
    char temp_crash_record_value[PARAM_CRASH_RECORD_SIZE];
    int count_size=0;
    char temp_buffer[1024];
    uint32 sid_index= PARAM_SID_CRASH_RECORD;

    /* Find the first crash record offset */
    uint32 offset = offsetof(param_crash_record_t, crash_record_0);

    sprintf(temp_buffer,"============ PARAM CRASH RECORD DUMP ============\n");
    count_size = count_size+50;


    for(i=0; i<MAX_RECORD_COUNT; i++) {

        /* Clean temp buffer */
        memset(temp_crash_record_value, 0, PARAM_CRASH_RECORD_SIZE);

        /* PARAM_CRASH_RECORD_SIZE-1 -> drop last unnecessary char '.' */
        ret = get_param_by_index_and_offset(sid_index, offset, temp_crash_record_value, PARAM_CRASH_RECORD_SIZE-1);

        //pr_info("temp_crash_record_value = %s, %d\n", temp_crash_record_value,offset);

        /* If the first char is '\0', it means this record is empty */
        if(temp_crash_record_value[0] == '\0') {
            strcat(temp_buffer, "EMPTY\n");
            count_size = count_size+6;
        } else {

            /* Add crash record string into temp buffer */
            strcat(temp_buffer, temp_crash_record_value);
            count_size = count_size+PARAM_CRASH_RECORD_SIZE;

            strcat(temp_buffer, "\n");;
        }


        if(ret < 0){
            pr_info("%s[%d]  failed\n",__func__, __LINE__);
            return ret;
        }

        /* Go to next crash record */
        offset = offset+ PARAM_CRASH_RECORD_SIZE;
    }
    strcat(temp_buffer, " \0");
    strncpy(buf, temp_buffer, count_size);
    //pr_info("buf = %s, %d\n", buf, count_size);
    return count_size;
}


static DEVICE_ATTR(param_dump,0644,
    param_dump_show,param_dump_store);

static DEVICE_ATTR(param_crash_record_dump,0444,
    param_crash_record_dump_show,NULL);
static DEVICE_ATTR(param_charger_type_dump,0444,
    param_charger_type_dump_show,NULL);

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

	if(device_create_file(param_misc.this_device,
	    &dev_attr_param_crash_record_dump) < 0)
	{
	    pr_err("Failed to create device file!(%s)!\n",
                dev_attr_param_crash_record_dump.attr.name);
		ret = -1;
	}

	if(device_create_file(param_misc.this_device,
	    &dev_attr_param_charger_type_dump) < 0)
	{
	    pr_err("Failed to create device file!(%s)!\n",
                dev_attr_param_charger_type_dump.attr.name);
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


int set_param_lcm_srgb_mode(uint * lcm_srgb_mode)
{
	int ret;
	uint32 sid_index= PARAM_SID_LCD;
	uint32 offset = offsetof(param_lcd_t, lcm_srgb_mode);

	ret = set_param_by_index_and_offset(sid_index,offset,
	            lcm_srgb_mode, sizeof(*lcm_srgb_mode));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
        pr_err("set lcm_srgb_mode = %d\n", *lcm_srgb_mode);
	return ret;
}
EXPORT_SYMBOL(set_param_lcm_srgb_mode);

int get_param_lcm_srgb_mode(uint *lcm_srgb_mode)
{
	int ret;
	uint32 sid_index= PARAM_SID_LCD;
	uint32 offset = offsetof(param_lcd_t, lcm_srgb_mode);

	ret = get_param_by_index_and_offset(sid_index,offset,lcm_srgb_mode, sizeof(*lcm_srgb_mode));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(get_param_lcm_srgb_mode);
/*********************************************************************/
int set_param_lcm_adobe_rgb_mode(uint * lcm_adobe_rgb_mode)
{
	int ret;
	uint32 sid_index= PARAM_SID_LCD;
	uint32 offset = offsetof(param_lcd_t, lcm_adobe_rgb_mode);

	ret = set_param_by_index_and_offset(sid_index,offset,
	            lcm_adobe_rgb_mode, sizeof(*lcm_adobe_rgb_mode));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
        pr_err("set lcm_srgb_mode = %d\n", *lcm_adobe_rgb_mode);
	return ret;
}
EXPORT_SYMBOL(set_param_lcm_adobe_rgb_mode);

int get_param_lcm_adobe_rgb_mode(uint *lcm_adobe_rgb_mode)
{
	int ret;
	uint32 sid_index= PARAM_SID_LCD;
	uint32 offset = offsetof(param_lcd_t, lcm_adobe_rgb_mode);

	ret = get_param_by_index_and_offset(sid_index,offset,lcm_adobe_rgb_mode, sizeof(*lcm_adobe_rgb_mode));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(get_param_lcm_adobe_rgb_mode);
/***********************************************************************/
int set_param_lcm_dci_p3_mode(uint * lcm_dci_p3_mode)
{
	int ret;
	uint32 sid_index= PARAM_SID_LCD;
	uint32 offset = offsetof(param_lcd_t, lcm_dci_p3_mode);

	ret = set_param_by_index_and_offset(sid_index,offset,
	            lcm_dci_p3_mode, sizeof(*lcm_dci_p3_mode));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
        pr_err("set lcm_srgb_mode = %d\n", *lcm_dci_p3_mode);
	return ret;
}
EXPORT_SYMBOL(set_param_lcm_dci_p3_mode);

int get_param_lcm_dci_p3_mode(uint *lcm_dci_p3_mode)
{
	int ret;
	uint32 sid_index= PARAM_SID_LCD;
	uint32 offset = offsetof(param_lcd_t, lcm_dci_p3_mode);

	ret = get_param_by_index_and_offset(sid_index,offset,lcm_dci_p3_mode, sizeof(*lcm_dci_p3_mode));
	if(ret < 0){
		pr_info("%s[%d]  failed\n",__func__, __LINE__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(get_param_lcm_dci_p3_mode);
/***************************************************************************/
int get_param_download_info(param_download_t *download_info)
{
	int ret;
	uint32 sid_index = PARAM_SID_DOWNLOAD;

	ret = get_param_by_index_and_offset(sid_index, 0, download_info, sizeof(param_download_t));

	if(ret < 0){
		pr_info("%s[%d]  failed!\n",__func__, __LINE__);
	}
	return ret;
}
EXPORT_SYMBOL(get_param_download_info);

int get_param_charger_type_count(uint *type_record_count)
{
    int ret;
    uint32 sid_index = PARAM_SID_BATTERY;

    uint32 offset = offsetof(param_chg_type_record_t, type_count);

    ret = get_param_by_index_and_offset(sid_index, offset, type_record_count, sizeof(*type_record_count));

    if(ret < 0){
        pr_info("%s[%d]  failed!\n",__func__, __LINE__);
    }
    return ret;
}
EXPORT_SYMBOL(get_param_charger_type_count);

int set_param_charger_type_count(uint *type_record_count)
{
    int ret;
    uint32 sid_index = PARAM_SID_BATTERY;

    uint32 offset = offsetof(param_chg_type_record_t, type_count);

    ret = set_param_by_index_and_offset(sid_index, offset, type_record_count, sizeof(*type_record_count));

    if(ret < 0){
        pr_info("%s[%d]  failed!\n",__func__, __LINE__);
    }
    return ret;
}
EXPORT_SYMBOL(set_param_charger_type_count);


int set_param_charger_type_value(uint offset, char *crash_record_value, uint size)
{
    int ret;
    uint32 sid_index = PARAM_SID_BATTERY;

    ret = set_param_by_index_and_offset(sid_index, offset, crash_record_value, size);

    if(ret < 0){
        pr_info("%s[%d]  failed!\n",__func__, __LINE__);
    }
    return ret;
}
EXPORT_SYMBOL(set_param_charger_type_value);


int get_param_crash_record_count(uint *crash_record_count)
{
    int ret;
    uint32 sid_index = PARAM_SID_CRASH_RECORD;

    uint32 offset = offsetof(param_crash_record_t, crash_count);

    ret = get_param_by_index_and_offset(sid_index, offset, crash_record_count, sizeof(*crash_record_count));

    if(ret < 0){
        pr_info("%s[%d]  failed!\n",__func__, __LINE__);
    }
    return ret;
}
EXPORT_SYMBOL(get_param_crash_record_count);

int set_param_crash_record_count(uint *crash_record_count)
{
    int ret;
    uint32 sid_index = PARAM_SID_CRASH_RECORD;

    uint32 offset = offsetof(param_crash_record_t, crash_count);

    ret = set_param_by_index_and_offset(sid_index, offset, crash_record_count, sizeof(*crash_record_count));

    if(ret < 0){
        pr_info("%s[%d]  failed!\n",__func__, __LINE__);
    }
    return ret;
}
EXPORT_SYMBOL(set_param_crash_record_count);


int set_param_crash_record_value(uint offset, char *crash_record_value, uint size)
{
    int ret;
    uint32 sid_index = PARAM_SID_CRASH_RECORD;

    ret = set_param_by_index_and_offset(sid_index, offset, crash_record_value, size);

    if(ret < 0){
        pr_info("%s[%d]  failed!\n",__func__, __LINE__);
    }
    return ret;
}
EXPORT_SYMBOL(set_param_crash_record_value);
