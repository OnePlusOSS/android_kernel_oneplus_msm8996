/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
 /******************************************************************************
  *
  *  The original Work has been changed by NXP Semiconductors.
  *
  *  Copyright (C) 2015 NXP Semiconductors
  *
  *  Licensed under the Apache License, Version 2.0 (the "License");
  *  you may not use this file except in compliance with the License.
  *  You may obtain a copy of the License at
  *
  *  http://www.apache.org/licenses/LICENSE-2.0
  *
  *  Unless required by applicable law or agreed to in writing, software
  *  distributed under the License is distributed on an "AS IS" BASIS,
  *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  *  See the License for the specific language governing permissions and
  *  limitations under the License.
  *
  ******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <pn544.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/wakelock.h>


//ruanbanmao add
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/project_info.h>



#define DRAGON_NFC 1
#define SIG_NFC 44
#define MAX_BUFFER_SIZE 512

struct pn544_dev    {
    wait_queue_head_t   read_wq;
    struct mutex        read_mutex;
    struct i2c_client   *client;
    struct miscdevice   pn544_device;
	struct wake_lock    pn544_wake_lock; /*NFC CLK_REQ*/	
    unsigned int        ven_gpio;
    unsigned int        firm_gpio;
    unsigned int        irq_gpio;
    unsigned int        ese_pwr_gpio; /* gpio used by SPI to provide power to p61 via NFCC */
    struct mutex        p61_state_mutex; /* used to make p61_current_state flag secure */
    p61_access_state_t  p61_current_state; /* stores the current P61 state */
    bool                nfc_ven_enabled; /* stores the VEN pin state powered by Nfc */
    bool                spi_ven_enabled; /* stores the VEN pin state powered by Spi */
    bool                irq_enabled;
	bool                clk_enabled; /*NFC CLK_REQ*/
    spinlock_t          irq_enabled_lock;
	spinlock_t          clk_enabled_lock; /*NFC CLK_REQ*/
    long                nfc_service_pid; /*used to signal the nfc the nfc service */
	const char *clk_src_name; /*ruanbanmao add for 15801 nfc 2015/10/16*/
	unsigned int clk_gpio; /*ruanbanmao add for 15801 nfc 2015/10/16*/
	struct	clk	*s_clk;
	struct regulator *p544_regulator;
	struct regulator *p544_regulator_s4;
	int irq; /*NFC CLK_REQ*/

};

/*ruanbanmao add for debug 2015/1217*/
/* Different driver debug lever */
#if 0
enum Pn544_DEBUG_LEVEL {
    Pn544_DEBUG_OFF,
    Pn544_FULL_DEBUG
};

static unsigned char debug_level = Pn544_FULL_DEBUG;
//#define P61_DBG_MSG(msg...)  printk(KERN_INFO "[NXP-P61] :  " msg);

#define Pn544_DBG_MSG(msg...)  \
        switch(debug_level)      \
        {                        \
        case Pn544_DEBUG_OFF:      \
        break;                 \
        case Pn544_FULL_DEBUG:     \
        printk(KERN_INFO "[NXP-pn544] :  " msg); \
        break; \
        default:                 \
        printk(KERN_ERR "[NXP-pn544] :  Wrong debug level %d", debug_level); \
        break; \
        } \

#define Pn544_ERR_MSG(msg...) printk(KERN_ERR "[NFC-pn544] : " msg );
#endif
/*ruanbanmao add for debug 2015/1217*/

//#define NFC_DEBUG

#ifdef NFC_DEBUG

#ifdef pr_err
#undef pr_err
#endif

#ifdef pr_warning
#undef pr_warning
#endif

#ifdef pr_info
#undef pr_info
#endif

#ifdef pr_debug
#undef pr_debug
#endif


#define pr_debug(msg...) printk(KERN_ERR "[NXP-pn544] :  " msg);
#define pr_err(msg...) printk(KERN_ERR "[NXP-pn544] :  " msg);
#define pr_warning(msg...) printk(KERN_ERR "[NXP-pn544] :  " msg);
#define pr_info(msg...) printk(KERN_ERR "[NXP-pn544] :  " msg);

#endif

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
    unsigned long flags;

    spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
    if (pn544_dev->irq_enabled) {
        disable_irq_nosync(pn544_dev->client->irq);
        pn544_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
    struct pn544_dev *pn544_dev = dev_id;
    pr_debug("%s ",__func__);


    pn544_disable_irq(pn544_dev);

    /* Wake up waiting readers */
    wake_up(&pn544_dev->read_wq);

    return IRQ_HANDLED;
}

/*NFC CLK_EQ*/
static void pn544_disable_clk(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->clk_enabled_lock, flags);
    if (pn544_dev->clk_enabled) {
        disable_irq_nosync(pn544_dev->irq);
        pn544_dev->clk_enabled = false;
    }	
	printk("###clkreq_disable_irq###");
	spin_unlock_irqrestore(&pn544_dev->clk_enabled_lock, flags);
}

static void pn544_enable_clk(struct pn544_dev *pn544_dev)
{
	unsigned long flags;
    
	spin_lock_irqsave(&pn544_dev->clk_enabled_lock, flags);
    if (!pn544_dev->clk_enabled) {
        pn544_dev->clk_enabled = true;
		enable_irq(pn544_dev->irq);
    }	
	printk("###clkreq_enable_irq###");
	spin_unlock_irqrestore(&pn544_dev->clk_enabled_lock, flags);
}

static irqreturn_t clk_req_handler(int irq, void *dev_id)
{
    struct pn544_dev *pn544_dev = dev_id;
    printk("%s ",__func__);

    if(!gpio_get_value(pn544_dev->clk_gpio)){
	   pn544_disable_clk(pn544_dev);	
       wake_unlock(&pn544_dev->pn544_wake_lock);		
	}

    return IRQ_HANDLED;
}
/* End, NFC CLK_REQ*/

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn544_dev *pn544_dev = filp->private_data;
    char tmp[MAX_BUFFER_SIZE];
    int ret;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    pr_debug("%s : reading   %zu bytes. irqgpio=%d\n", __func__, count,gpio_get_value(pn544_dev->irq_gpio));

    mutex_lock(&pn544_dev->read_mutex);

    if (!gpio_get_value(pn544_dev->irq_gpio)) {
        if (filp->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            goto fail;
        }

        while (1) {
            ret = 0;
            pn544_dev->irq_enabled = true;
            enable_irq(pn544_dev->client->irq);
            /*If IRQ line is already high, which means IRQ was high
              just before enabling the interrupt, skip waiting for interrupt,
              as interrupt would have been disabled by then in the interrupt handler*/
            if (!gpio_get_value(pn544_dev->irq_gpio)){
            ret = wait_event_interruptible(
                    pn544_dev->read_wq,
                    !pn544_dev->irq_enabled);
            }


            pn544_disable_irq(pn544_dev);

            if (ret)
                goto fail;

            if (gpio_get_value(pn544_dev->irq_gpio))
                break;

            pr_warning("%s: spurious interrupt detected\n", __func__);
        }
    }

    /* Read data */
    ret = i2c_master_recv(pn544_dev->client, tmp, count);

    mutex_unlock(&pn544_dev->read_mutex);

    /* pn544 seems to be slow in handling I2C read requests
     * so add 1ms delay after recv operation */
    udelay(1000);

    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        return ret;
    }
    if (ret > count) {
        pr_err("%s: received too many bytes from i2c (%d)\n",
                __func__, ret);
        return -EIO;
    }
    if (copy_to_user(buf, tmp, ret)) {
        pr_warning("%s : failed to copy to user space\n", __func__);
        return -EFAULT;
    }
    return ret;

    fail:
    mutex_unlock(&pn544_dev->read_mutex);
    return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn544_dev  *pn544_dev;
    char tmp[MAX_BUFFER_SIZE];
    int ret;

    pn544_dev = filp->private_data;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    if (copy_from_user(tmp, buf, count)) {
        pr_err("%s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }

    pr_debug("%s : writing %zu bytes.\n", __func__, count);
    /* Write data */
    ret = i2c_master_send(pn544_dev->client, tmp, count);
    if (ret != count) {
        pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
        ret = -EIO;
    }

    /* pn544 seems to be slow in handling I2C write requests
     * so add 1ms delay after I2C send oparation */
    udelay(1000);

    return ret;
}

static void p61_update_access_state(struct pn544_dev *pn544_dev, p61_access_state_t current_state, bool set)
{
    pr_info("%s: Enter current_state = %x\n", __func__, pn544_dev->p61_current_state);
    if (current_state)
    {
        if(set){
            if(pn544_dev->p61_current_state == P61_STATE_IDLE)
                pn544_dev->p61_current_state = P61_STATE_INVALID;
            pn544_dev->p61_current_state |= current_state;
        }
        else{
            pn544_dev->p61_current_state ^= current_state;
            if(!pn544_dev->p61_current_state)
                pn544_dev->p61_current_state = P61_STATE_IDLE;
        }
    }
    pr_info("%s: Exit current_state = %x\n", __func__, pn544_dev->p61_current_state);
}

static void p61_get_access_state(struct pn544_dev *pn544_dev, p61_access_state_t *current_state)
{

    if (current_state == NULL) {
        //*current_state = P61_STATE_INVALID;
        pr_err("%s : invalid state of p61_access_state_t current state  \n", __func__);
    } else {
        *current_state = pn544_dev->p61_current_state;
    }
}
static void p61_access_lock(struct pn544_dev *pn544_dev)
{
    pr_info("%s: Enter\n", __func__);
    mutex_lock(&pn544_dev->p61_state_mutex);
    pr_info("%s: Exit\n", __func__);
}
static void p61_access_unlock(struct pn544_dev *pn544_dev)
{
    pr_info("%s: Enter\n", __func__);
    mutex_unlock(&pn544_dev->p61_state_mutex);
    pr_info("%s: Exit\n", __func__);
}

static void signal_handler(p61_access_state_t state, long nfc_pid)
{
    struct siginfo sinfo;
    pid_t pid;
    struct task_struct *task;
    int sigret = 0;
    pr_info("%s: Enter\n", __func__);

    memset(&sinfo, 0, sizeof(struct siginfo));
    sinfo.si_signo = SIG_NFC;
    sinfo.si_code = SI_QUEUE;
    sinfo.si_int = state;
    pid = nfc_pid;

    task = pid_task(find_vpid(pid), PIDTYPE_PID);
    if(task)
    {
        pr_info("%s.\n", task->comm);
        sigret = send_sig_info(SIG_NFC, &sinfo, task);
        if(sigret < 0){
            pr_info("send_sig_info failed..... sigret %d.\n", sigret);
            //msleep(60);
        }
    }
    else{
        pr_info("finding task from PID failed\r\n");
    }
    pr_info("%s: Exit\n", __func__);
}
static int pn544_dev_open(struct inode *inode, struct file *filp)
{
    struct pn544_dev *pn544_dev = container_of(filp->private_data,
            struct pn544_dev,
            pn544_device);

    filp->private_data = pn544_dev;

    pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

    return 0;
}

long  pn544_dev_ioctl(struct file *filp, unsigned int cmd,
        unsigned long arg)
{
    struct pn544_dev *pn544_dev = filp->private_data;
    pr_info("%s : cmd = %d, arg = %ld\n", __func__, cmd, arg);
    p61_access_lock(pn544_dev);
    switch (cmd) {
    case PN544_SET_PWR:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        if (arg == 2) {
            if (current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO))
            {
                /* NFCC fw/download should not be allowed if p61 is used
                 * by SPI
                 */
                pr_info("%s NFCC should not be allowed to reset/FW download \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
            pn544_dev->nfc_ven_enabled = true;
            if (pn544_dev->spi_ven_enabled == false)
            {
                /* power on with firmware download (requires hw reset)
                 */
                pr_info("%s power on with firmware\n", __func__);
                gpio_set_value(pn544_dev->ven_gpio, 1);
                msleep(10);
                if (pn544_dev->firm_gpio) {
                    p61_update_access_state(pn544_dev, P61_STATE_DWNLD, true);
                    gpio_set_value(pn544_dev->firm_gpio, 1);
                }
                msleep(10);
                gpio_set_value(pn544_dev->ven_gpio, 0);
                msleep(10);
                gpio_set_value(pn544_dev->ven_gpio, 1);
                msleep(10);
            }
        } else if (arg == 1) {
            /* power on */
            pr_info("%s power on\n", __func__);
            if (pn544_dev->firm_gpio) {
                if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0){
                    p61_update_access_state(pn544_dev, P61_STATE_IDLE, true);
                }
                gpio_set_value(pn544_dev->firm_gpio, 0);
            }

            pn544_dev->nfc_ven_enabled = true;
            if (pn544_dev->spi_ven_enabled == false) {
                gpio_set_value(pn544_dev->ven_gpio, 1);
            }
        } else if (arg == 0) {
            /* power off */
            pr_info("%s power off\n", __func__);
            if (pn544_dev->firm_gpio) {
                if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0){
                    p61_update_access_state(pn544_dev, P61_STATE_IDLE, true);
                }
                gpio_set_value(pn544_dev->firm_gpio, 0);
            }

            pn544_dev->nfc_ven_enabled = false;
            /* Don't change Ven state if spi made it high */
            if (pn544_dev->spi_ven_enabled == false) {
                gpio_set_value(pn544_dev->ven_gpio, 0);
            }
        } else {
            pr_err("%s bad arg %lu\n", __func__, arg);
            /* changed the p61 state to idle*/
            p61_access_unlock(pn544_dev);
            return -EINVAL;
        }
    }
    break;
    case P61_SET_SPI_PWR:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        if (arg == 1) {
            pr_info("%s : PN61_SET_SPI_PWR - power on ese\n", __func__);
            if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
            {
                p61_update_access_state(pn544_dev, P61_STATE_SPI, true);
                /*To handle triple mode protection signal
                NFC service when SPI session started*/
                if (current_state & P61_STATE_WIRED){
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                }
                pn544_dev->spi_ven_enabled = true;
                if (pn544_dev->nfc_ven_enabled == false)
                {
                    /* provide power to NFCC if, NFC service not provided */
                    gpio_set_value(pn544_dev->ven_gpio, 1);
                    msleep(10);
                }
                /* pull the gpio to high once NFCC is power on*/
                gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
            } else {
                pr_info("%s : PN61_SET_SPI_PWR -  power on ese failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        } else if (arg == 0) {
            pr_info("%s : PN61_SET_SPI_PWR - power off ese\n", __func__);
            if(current_state & P61_STATE_SPI_PRIO){
                p61_update_access_state(pn544_dev, P61_STATE_SPI_PRIO, false);
                if (current_state & P61_STATE_WIRED)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO_END, pn544_dev->nfc_service_pid);
                }
                else{
                    pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                }
                }
                if (!(current_state & P61_STATE_WIRED))
                    gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                pn544_dev->spi_ven_enabled = false;
                 if (pn544_dev->nfc_ven_enabled == false) {
                     gpio_set_value(pn544_dev->ven_gpio, 0);
                     msleep(10);
                 }
              }else if(current_state & P61_STATE_SPI){
                  p61_update_access_state(pn544_dev, P61_STATE_SPI, false);
                  if (!(current_state & P61_STATE_WIRED))
                  {
                      gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                  }
                  /*If JCOP3.2 or 3.3 for handling triple mode
                  protection signal NFC service */
                  else
                  {
                      if (current_state & P61_STATE_WIRED)
                      {
                          if(pn544_dev->nfc_service_pid){
                              pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                              signal_handler(P61_STATE_SPI_END, pn544_dev->nfc_service_pid);
                           }
                           else{
                               pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                           }
                      }
                  }
                  pn544_dev->spi_ven_enabled = false;
                  if (pn544_dev->nfc_ven_enabled == false) {
                      gpio_set_value(pn544_dev->ven_gpio, 0);
                      msleep(10);
                  }
            } else {
                pr_err("%s : PN61_SET_SPI_PWR - failed, current_state = %x \n",
                        __func__, pn544_dev->p61_current_state);
                p61_access_unlock(pn544_dev);
                return -EPERM; /* Operation not permitted */
            }
        }else if (arg == 2) {
            pr_info("%s : PN61_SET_SPI_PWR - reset\n", __func__);
            if (current_state & (P61_STATE_IDLE|P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
                if (pn544_dev->spi_ven_enabled == false)
                {
                    pn544_dev->spi_ven_enabled = true;
                    if (pn544_dev->nfc_ven_enabled == false) {
                        /* provide power to NFCC if, NFC service not provided */
                        gpio_set_value(pn544_dev->ven_gpio, 1);
                        msleep(10);
                    }
                }
                gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                msleep(10);
                gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
                msleep(10);
            } else {
                pr_info("%s : PN61_SET_SPI_PWR - reset  failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        }else if (arg == 3) {
            pr_info("%s : PN61_SET_SPI_PWR - Prio Session Start power on ese\n", __func__);
            if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0) {
                p61_update_access_state(pn544_dev, P61_STATE_SPI_PRIO, true);
                if (current_state & P61_STATE_WIRED){
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                }
                pn544_dev->spi_ven_enabled = true;
                if (pn544_dev->nfc_ven_enabled == false) {
                    /* provide power to NFCC if, NFC service not provided */
                    gpio_set_value(pn544_dev->ven_gpio, 1);
                    msleep(10);
                }
                /* pull the gpio to high once NFCC is power on*/
                gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
            }else {
                pr_info("%s : Prio Session Start power on ese failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        }else if (arg == 4) {
            if (current_state & P61_STATE_SPI_PRIO)
            {
                pr_info("%s : PN61_SET_SPI_PWR - Prio Session Ending...\n", __func__);
                p61_update_access_state(pn544_dev, P61_STATE_SPI_PRIO, false);
                /*after SPI prio timeout, the state is changing from SPI prio to SPI */
                p61_update_access_state(pn544_dev, P61_STATE_SPI, true);
                if (current_state & P61_STATE_WIRED)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO_END, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
               }
            }
            else
            {
                pr_info("%s : PN61_SET_SPI_PWR -  Prio Session End failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBADRQC; /* Device or resource busy */
            }
        }else {
            pr_info("%s bad ese pwr arg %lu\n", __func__, arg);
            p61_access_unlock(pn544_dev);
            return -EBADRQC; /* Invalid request code */
        }
    }
    break;

    case P61_GET_PWR_STATUS:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        pr_info("%s: P61_GET_PWR_STATUS  = %x",__func__, current_state);
        put_user(current_state, (int __user *)arg);
    }
    break;

    case P61_SET_WIRED_ACCESS:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        if (arg == 1)
        {
            if (current_state)
            {
                pr_info("%s : P61_SET_WIRED_ACCESS - enabling\n", __func__);
                p61_update_access_state(pn544_dev, P61_STATE_WIRED, true);
                if (current_state & P61_STATE_SPI_PRIO)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                }
                if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
                    gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
            } else {
                pr_info("%s : P61_SET_WIRED_ACCESS -  enabling failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        } else if (arg == 0) {
            pr_info("%s : P61_SET_WIRED_ACCESS - disabling \n", __func__);
            if (current_state & P61_STATE_WIRED){
                p61_update_access_state(pn544_dev, P61_STATE_WIRED, false);
                if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
                    gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
            } else {
                pr_err("%s : P61_SET_WIRED_ACCESS - failed, current_state = %x \n",
                        __func__, pn544_dev->p61_current_state);
                p61_access_unlock(pn544_dev);
                return -EPERM; /* Operation not permitted */
            }
        }
        else {
            pr_info("%s P61_SET_WIRED_ACCESS - bad arg %lu\n", __func__, arg);
            p61_access_unlock(pn544_dev);
            return -EBADRQC; /* Invalid request code */
        }
    }
    break;
    case P544_SET_NFC_SERVICE_PID:
    {
        pr_info("%s : The NFC Service PID is %ld\n", __func__, arg);
        pn544_dev->nfc_service_pid = arg;

    }
    break;
    default:
        pr_err("%s bad ioctl %u\n", __func__, cmd);
        p61_access_unlock(pn544_dev);
        return -EINVAL;
    }
    p61_access_unlock(pn544_dev);
    return 0;
}

static const struct file_operations pn544_dev_fops = {
        .owner  = THIS_MODULE,
        .llseek = no_llseek,
        .read   = pn544_dev_read,
        .write  = pn544_dev_write,
        .open   = pn544_dev_open,
        .unlocked_ioctl  = pn544_dev_ioctl,
};
#if DRAGON_NFC
static int pn544_parse_dt(struct device *dev,
    struct pn544_i2c_platform_data *data)
{
    struct device_node *np = dev->of_node;
    int errorno = 0;
    int r =0;
        data->irq_gpio = of_get_named_gpio(np, "nxp,pn544-irq", 0);
        if ((!gpio_is_valid(data->irq_gpio)))
                return -EINVAL;

        data->ven_gpio = of_get_named_gpio(np, "nxp,pn544-ven", 0);
        if ((!gpio_is_valid(data->ven_gpio)))
                return -EINVAL;

        data->firm_gpio = of_get_named_gpio(np, "nxp,pn544-fw-dwnld", 0);
        if ((!gpio_is_valid(data->firm_gpio)))
                return -EINVAL;

        data->ese_pwr_gpio = of_get_named_gpio(np, "nxp,pn544-ese-pwr", 0);
        if ((!gpio_is_valid(data->ese_pwr_gpio)))
                return -EINVAL;
		//ruanbanmao add for 15801 nfc begin.
        data->clk_gpio = of_get_named_gpio(np, "nxp,pn544-clk-gpio", 0);
        if ((!gpio_is_valid(data->clk_gpio)))
                return -EINVAL;
        r = of_property_read_string(np, "qcom,clk-src", &data->clk_src_name);
        //ruanbanmao add for 15801 nfc, end

    pr_info("%s: %d, %d, %d, %d ,%d, %d\n", __func__,
        data->irq_gpio, data->ven_gpio, data->firm_gpio, data->ese_pwr_gpio, data->clk_gpio, errorno);

    return errorno;
}
#endif

static int pn544_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret;
    struct pn544_i2c_platform_data *platform_data;
    struct pn544_dev *pn544_dev;
#if !DRAGON_NFC
    platform_data = client->dev.platform_data;
#else
    struct device_node *node = client->dev.of_node;
    pr_info("Enter %s !!!\n",__func__);//ruanbanmao add for debug

    if (node) {
        platform_data = devm_kzalloc(&client->dev,
            sizeof(struct pn544_i2c_platform_data), GFP_KERNEL);
        if (!platform_data) {
            dev_err(&client->dev,
                "nfc-nci probe: Failed to allocate memory\n");
            return -ENOMEM;
        }
        ret = pn544_parse_dt(&client->dev, platform_data);
        if (ret)
        {
            pr_info("%s pn544_parse_dt failed", __func__);
        }
        client->irq = gpio_to_irq(platform_data->irq_gpio);
        if (client->irq < 0)
        {
            pr_info("%s gpio to irq failed", __func__);
        }
    } else {
        platform_data = client->dev.platform_data;
    }
#endif
    if (platform_data == NULL) {
        pr_err("%s : nfc probe fail\n", __func__);
        return  -ENODEV;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s : need I2C_FUNC_I2C\n", __func__);
        return  -ENODEV;
    }
//ruanbanmao modify for 15801,2015/10/20
//#if !DRAGON_NFC
#if 1
    ret = gpio_request(platform_data->irq_gpio, "nfc_int");
    if (ret)
        return  -ENODEV;
    ret = gpio_request(platform_data->ven_gpio, "nfc_ven");
    if (ret)
        goto err_ven;
    ret = gpio_request(platform_data->ese_pwr_gpio, "nfc_ese_pwr");
    if (ret)
        goto err_ese_pwr;
    ret = gpio_request(platform_data->clk_gpio, "nfc_clk_gpio");
    if(ret)
        goto err_clk_gpio;
    if (platform_data->firm_gpio) {
        ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
        if (ret)
            goto err_firm;
    }
#endif
    pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
    if (pn544_dev == NULL) {
        dev_err(&client->dev,
                "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }
    /*NFC don't contrl ldo 9 .ldo 9 contrled by modem, this will cause can,t recognise sim card*/
    /*
	//ruanbanmao add for 15801 nfc, 2015/10/20, begin
    pn544_dev->p544_regulator = regulator_get( &client->dev, "nfc_voltage");
    if (IS_ERR(pn544_dev->p544_regulator))
    {
        ret = PTR_ERR(pn544_dev->p544_regulator);
        pr_err(" Error to get nfc_voltage. (error code) = %d\n", ret);
		 goto err_get_nfc_voltage;
    }
    else
    {
        pr_info("successfully got nfc_voltage\n");
    }

    ret = regulator_set_voltage(pn544_dev->p544_regulator, 1800000, 1800000);
    if (ret != 0)
    {
        pr_err("Error setting the nfc_voltage. (error code)= %d\n", ret);
        goto err_set_nfc_voltage;//ruanbanmao modify
    }
    else
    {
        ret = regulator_enable(pn544_dev->p544_regulator);
        pr_info("successfully set nfc_voltage\n");
    }
	*/
    pn544_dev->p544_regulator_s4= regulator_get( &client->dev, "nfc_voltage_s4");
    if (IS_ERR(pn544_dev->p544_regulator_s4))
    {
        ret = PTR_ERR(pn544_dev->p544_regulator_s4);
        pr_err(" Error to get p544_regulator_s4. (error code) = %d\n", ret);
       goto err_set_nfc_voltage;
    }
    else
    {
        pr_info("successfully got regulator_s4\n");
    }

    ret = regulator_set_voltage(pn544_dev->p544_regulator_s4, 1800000, 1800000);
    if (ret != 0)
    {
        pr_err("Error setting the regulator voltage_s4. (error code)= %d\n", ret);
        goto err_set_p544_regulator_s4;
		 }
    else
    {
        ret = regulator_enable(pn544_dev->p544_regulator_s4);
        pr_info("successfully set regulator voltage_s4\n");
    }
    //ruanbanmao add for 15801 nfc, 2015/10/20, end

    pn544_dev->irq_gpio = platform_data->irq_gpio;
    pn544_dev->ven_gpio  = platform_data->ven_gpio;
    pn544_dev->firm_gpio  = platform_data->firm_gpio;
    pn544_dev->ese_pwr_gpio  = platform_data->ese_pwr_gpio;
    pn544_dev->p61_current_state = P61_STATE_IDLE;
    pn544_dev->nfc_ven_enabled = false;
    pn544_dev->spi_ven_enabled = false;
    pn544_dev->client   = client;

	//ruanbanmao add for 15801 nfc, 2015/10/20, begin
    // get and set clock.
	pn544_dev->clk_src_name = platform_data->clk_src_name;
	pn544_dev->clk_gpio = platform_data->clk_gpio;

    if (!strcmp(pn544_dev->clk_src_name, "BBCLK2")) {
        pr_info("get BBCLK2!!!\n");
        pn544_dev->s_clk = clk_get(&pn544_dev->client->dev, "ref_clk");
        if (IS_ERR(pn544_dev->s_clk)) {
        pr_err("no pclk defined\n");
		goto err_set_p544_regulator_s4;
        }
    }
    clk_set_rate(pn544_dev->s_clk, 19200000);
    ret = clk_prepare_enable(pn544_dev->s_clk);
    pr_info("clk_prepare_enable. ret = %d\n", ret);
    if (ret) {
        pr_err("Can't enable s_clk\n");
		   // return err;
    }
	//ruanbanmao add for 15801 nfc, 2015/10/20, end

    ret = gpio_direction_input(pn544_dev->irq_gpio);
    if (ret < 0) {
        pr_err("%s :not able to set irq_gpio as input\n", __func__);
        //ruanbanmao modify for 15801 nfc.
        goto err_gpio_set;
    }
    ret = gpio_direction_output(pn544_dev->ven_gpio, 0);
    if (ret < 0) {
        pr_err("%s : not able to set ven_gpio as output\n", __func__);
        //ruanbanmao modify for 15801 nfc.
        goto err_gpio_set;
    }
    ret = gpio_direction_output(pn544_dev->ese_pwr_gpio, 0);
    if (ret < 0) {
        pr_err("%s : not able to set ese_pwr gpio as output\n", __func__);
        //ruanbanmao modify for 15801 nfc.
        goto err_gpio_set;
    }
    if (platform_data->firm_gpio) {
        ret = gpio_direction_output(pn544_dev->firm_gpio, 0);
        if (ret < 0) {
            pr_err("%s : not able to set firm_gpio as output\n",
                    __func__);
        //ruanbanmao modify for 15801 nfc.
        goto err_gpio_set;
        }
    }
    //ruanbanmao add for NFC clock. 2016-01-06
    ret = gpio_direction_input(pn544_dev->clk_gpio);
    if(ret <0){
        pr_err("%s : not able to set clk_gpio as input\n",
                    __func__);
        goto err_gpio_set;
    }
    /* init mutex and queues */
    init_waitqueue_head(&pn544_dev->read_wq);
    mutex_init(&pn544_dev->read_mutex);
    mutex_init(&pn544_dev->p61_state_mutex);
    spin_lock_init(&pn544_dev->irq_enabled_lock);
	/*NFC CLK_REQ*/
	spin_lock_init(&pn544_dev->clk_enabled_lock);
	wake_lock_init(&pn544_dev->pn544_wake_lock,
			WAKE_LOCK_SUSPEND, "pn544_wake_lock");
    /* End, NFC CLK_REQ*/
    pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
    pn544_dev->pn544_device.name = "nq-nci";
    pn544_dev->pn544_device.fops = &pn544_dev_fops;

    ret = misc_register(&pn544_dev->pn544_device);
    if (ret) {
        pr_err("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }
			
    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
    pn544_dev->irq_enabled = true;
    ret = request_irq(client->irq, pn544_dev_irq_handler,
            IRQF_TRIGGER_HIGH, client->name, pn544_dev);
    if (ret) {
        dev_err(&client->dev, "request_irq failed\n");
        goto err_request_irq_failed;
    }
    pn544_disable_irq(pn544_dev);    
    i2c_set_clientdata(client, pn544_dev);
    /*add for wake up ap side*/
    enable_irq_wake(pn544_dev->client->irq);
    push_component_info(NFC, "NQ220", "NXP");
	
	/* NFC CLK_REQ */	
	pr_info("%s pn544_dev->clk_gpio = %d\n", __func__, gpio_get_value(pn544_dev->clk_gpio));
	pn544_dev->irq = gpio_to_irq(pn544_dev->clk_gpio);
		if (pn544_dev->irq < 0)
        {
            pr_info("%s gpio to irq failed", __func__);
        } 
		
	pr_info("%s : requesting IRQ %d\n", __func__, pn544_dev->irq);
	pn544_dev->clk_enabled = true;
	ret = request_irq(pn544_dev->irq, clk_req_handler,
			  IRQF_TRIGGER_FALLING, "CLK_REQ_DETECT", pn544_dev);
			  
	if (ret) {
		dev_err(&client->dev, "%s: request_irq failed\n", __func__);
		goto err_request_irq_failed;
	}
    pn544_disable_clk(pn544_dev);
	/* End, NFC CLK_REQ*/
    return 0;

    err_request_irq_failed:
    misc_deregister(&pn544_dev->pn544_device);
    err_misc_register:
    mutex_destroy(&pn544_dev->read_mutex);
    mutex_destroy(&pn544_dev->p61_state_mutex);
    //ruanbanmao add for 15801 nfc, 2015/10/20, begin
    err_gpio_set:
    clk_disable(pn544_dev->s_clk);
    clk_put(pn544_dev->s_clk);
    err_set_p544_regulator_s4:
    regulator_put(pn544_dev->p544_regulator_s4);
    err_set_nfc_voltage:
    regulator_put(pn544_dev->p544_regulator);
	//err_get_nfc_voltage:
    ////ruanbanmao add for 15801 nfc, 2015/10/20, end
    kfree(pn544_dev);
    err_exit:
    if (pn544_dev->firm_gpio)
        gpio_free(platform_data->firm_gpio);
    err_clk_gpio:
    gpio_free(platform_data->clk_gpio);
    err_firm:
    gpio_free(platform_data->ese_pwr_gpio);
    err_ese_pwr:
    gpio_free(platform_data->ven_gpio);
    err_ven:
    gpio_free(platform_data->irq_gpio);
    return ret;
}

static int pn544_remove(struct i2c_client *client)
{
    struct pn544_dev *pn544_dev;

    pn544_dev = i2c_get_clientdata(client);
    //ruanbanmao add for p61, begin
    clk_disable(pn544_dev->s_clk);
    clk_put(pn544_dev->s_clk);
    regulator_put(pn544_dev->p544_regulator_s4);
    regulator_put(pn544_dev->p544_regulator);
    //end
    free_irq(client->irq, pn544_dev);
	free_irq(pn544_dev->irq, pn544_dev); /*NFC CLK_REQ*/
    misc_deregister(&pn544_dev->pn544_device);
    mutex_destroy(&pn544_dev->read_mutex);
    mutex_destroy(&pn544_dev->p61_state_mutex);
    gpio_free(pn544_dev->irq_gpio);
    gpio_free(pn544_dev->ven_gpio);
    gpio_free(pn544_dev->ese_pwr_gpio);
    pn544_dev->p61_current_state = P61_STATE_INVALID;
    pn544_dev->nfc_ven_enabled = false;
    pn544_dev->spi_ven_enabled = false;

    if (pn544_dev->firm_gpio)
        gpio_free(pn544_dev->firm_gpio);
    kfree(pn544_dev);

    return 0;
}

static const struct i2c_device_id pn544_id[] = {
        { "pn544", 0 },
        { }
};
#if DRAGON_NFC
static struct of_device_id pn544_i2c_dt_match[] = {
    {
        .compatible = "nxp,pn544",
    },
    {}
};
#endif
/*NFC CLK_REQ*/
static int pn544_suspend(struct i2c_client *client, pm_message_t message)
{   
	struct pn544_dev *pn544_dev = i2c_get_clientdata(client);
    
    printk("%s pn544_dev->clk_gpio = %d\n", __func__, gpio_get_value(pn544_dev->clk_gpio));	
	if(gpio_get_value(pn544_dev->clk_gpio)){
		wake_lock(&pn544_dev->pn544_wake_lock);		
		pn544_enable_clk(pn544_dev);
	}
	return 0;
}

static int pn544_resume(struct i2c_client *client)
{
	//struct pn544_dev *pn544_dev = i2c_get_clientdata(client);
    
	//pn544_disable_clk(pn544_dev);
	//wake_unlock(&pn544_dev->pn544_wake_lock);

	return 0;
}


static struct i2c_driver pn544_driver = {
        .id_table   = pn544_id,
        .probe      = pn544_probe,
        .remove     = pn544_remove, /*NFC CLK_REQ*/
		.suspend	= pn544_suspend, /*NFC CLK_REQ*/     
	    .resume		= pn544_resume,
        .driver     = {
                .owner = THIS_MODULE,
                .name  = "pn544",
#if DRAGON_NFC
                .of_match_table = pn544_i2c_dt_match,
#endif
        },
};
/* End, NFC CLK_REQ*/

/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
    pr_info("Loading pn544 driver\n");
    return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
    pr_info("Unloading pn544 driver\n");
    i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
