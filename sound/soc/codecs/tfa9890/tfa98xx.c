/*
 * Copyright (C) NXP Semiconductors (PLMA)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <sound/tfa98xx.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/version.h>
#include <sound/sounddebug.h>
#include "tfa98xx-core.h"
#include "tfa98xx-regs.h"
#include "tfa_container.h"
#include "tfa_dsp.h"
int testLogOn = 0;
EXPORT_SYMBOL_GPL(testLogOn);


void tfa98xx_play_stop(void);
EXPORT_SYMBOL_GPL(tfa98xx_play_stop);


extern int recoverMtp0;

#define I2C_RETRY_DELAY		5 /* ms */
#define I2C_RETRIES		5
#define PLL_SYNC_RETRIES	10
#define MTPB_RETRIES		5

/* SNDRV_PCM_RATE_KNOT -> 12000, 24000 Hz, limit with constraint list */
#define TFA98XX_RATES (SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_KNOT)
#define TFA98XX_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE)

#define TFA98XX_STATUS_UP_MASK	(TFA98XX_STATUSREG_PLLS | \
				 TFA98XX_STATUSREG_CLKS | \
				 TFA98XX_STATUSREG_VDDS | \
				 TFA98XX_STATUSREG_AREFS)
struct tfa98xx *g_tfa98xx = NULL;
EXPORT_SYMBOL_GPL(g_tfa98xx);

//su
struct device_node *tfa_codec_np = NULL;
EXPORT_SYMBOL_GPL(tfa_codec_np);


static ssize_t tfa98xx_state_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
    pr_err("%s",__func__);

	return 0;
}
static ssize_t tfa98xx_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    struct snd_soc_codec *codec;
    struct tfa98xx *tfa98xx;
    unsigned short mtp;
	unsigned short status;
    int done = 0;
    u32 re25 = 0;
    int try = 0;
    u16 mtp0;

    if(g_tfa98xx == NULL)
    {
        pr_err("%s g_tfa98xx = NULL\n",__func__);
        return 0;
    }

    tfa98xx = g_tfa98xx;
    codec = tfa98xx->codec;

    mutex_lock(&tfa98xx->dsp_init_lock);

        /*
	     * check the contents of  MTP register for non-zero,
	     * this indicates that the subsys is ready
	     */
	    mtp0 = (u16)snd_soc_read(codec, 0x84);

        /* NXP: if 0x84 is wrong, restore the correct mtp settings */
        if (!mtp0 || recoverMtp0)
        {
		    pr_err("%s mtp0 error,now recovery mtp.\n",__func__);
            if(recoverMtp0)
            {
                pr_err("%s mtp0 error detect in cold start up.\n",__func__);
                recoverMtp0 = false;
            }
            tfa98xx_restore_mtp(tfa98xx);
        }

	if (!tfa98xx_is_pwdn(tfa98xx)) {
		tfa98xx_dsp_stop(tfa98xx);
	}
    tfaRunColdStartup(tfa98xx);
    msleep(5);

    mtp = snd_soc_read(codec, TFA98XX_MTP);
	/* reset MTPEX bit if needed */
	if ( (mtp & TFA98XX_KEY2_PROTECTED_SPKR_CAL_MTP_MTPOTC) && (mtp & TFA98XX_KEY2_PROTECTED_SPKR_CAL_MTP_MTPEX))
	{
	    snd_soc_write(codec, 0x0B, 0x5A); /* unlock key2 */
        snd_soc_write(codec, TFA98XX_MTP, 1); /* MTPOTC=1, MTPEX=0 */
        snd_soc_write(codec, 0x62, 1<<11); /* CIMTP=1 */
    }

	do {
        try ++;
		msleep(10);
        status = snd_soc_read(codec, TFA98XX_STATUSREG);
	} while (((status & TFA98XX_STATUSREG_MTPB) == TFA98XX_STATUSREG_MTPB) && (try < 100));

    if(try == 100)
    {
        pr_err("%s try read TFA98XX_STATUSREG_MTPB time out\n",__func__);
        goto err;
    }

	if (!tfa98xx_is_pwdn(tfa98xx)) {
		tfa98xx_dsp_stop(tfa98xx);
	}
    msleep(10);
    tfa98xx->dsp_init = TFA98XX_DSP_INIT_RECOVER;
	/* start the DSP using the latest profile / vstep */
	if (!tfa98xx_dsp_start(tfa98xx, tfa98xx->profile_ctl, tfa98xx->vstep_ctl))
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;

        /*
	     * check the contents of  MTP register for non-zero,
	     * this indicates that the subsys is ready
	     */
	    mtp0 = (u16)snd_soc_read(codec, 0x84);

        /* NXP: if 0x84 is wrong, restore the correct mtp settings */
        if (!mtp0 )
        {
		    pr_err("%s mtp0 error,now recovery mtp.\n",__func__);
            tfa98xx_restore_mtp(tfa98xx);
        }

    mtp = snd_soc_read(codec, TFA98XX_MTP);
	done = (mtp & TFA98XX_MTP_MTPEX);
    tfa98xx_dsp_get_calibration_impedance(tfa98xx, &re25);

    pr_debug("%s done =%d re=%d\n",__func__,done,re25);
    mutex_unlock(&tfa98xx->dsp_init_lock);
    return sprintf(buf,"%d:%d",done,re25);

err:
    pr_err("%s calibrate fail\n",__func__);
    mutex_unlock(&tfa98xx->dsp_init_lock);
    return sprintf(buf,"%d:%d",0,0);

}



static struct device_attribute tfa98xx_state_attr =
     __ATTR(calibra, 0444, tfa98xx_state_show, tfa98xx_state_store);

static ssize_t tfa98xx_Log_state_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
    pr_err("%s",__func__);

    if (sysfs_streq(buf, "LogOn"))
    {
        testLogOn = 1;
    }
    else if(sysfs_streq(buf, "LogOff"))
    {
        testLogOn = 0;
    }
    else
    {
        testLogOn = 0;
        count = -EINVAL;
    }
    return count;
}

static ssize_t tfa98xx_Log_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    return 0;
}

static struct device_attribute tfa98xx_Log_state_attr =
     __ATTR(Log, S_IWUSR|S_IRUGO, tfa98xx_Log_state_show, tfa98xx_Log_state_store);



/*
 * I2C Read/Write Functions
 */

int tfa98xx_i2c_read(struct i2c_client *tfa98xx_client,	u8 reg, u8 *value,
		     int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = tfa98xx_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = tfa98xx_client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = value,
		},
	};

	do {
		err = i2c_transfer(tfa98xx_client->adapter, msgs,
							ARRAY_SIZE(msgs));
		if (err != ARRAY_SIZE(msgs))
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&tfa98xx_client->dev, "read transfer error %d\n" , err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

int tfa98xx_bulk_write_raw(struct snd_soc_codec *codec, const u8 *data,
				u8 count)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int ret;

	ret = i2c_master_send(tfa98xx->i2c, data, count);
	if (ret == count) {
		return 0;
	} else if (ret < 0) {
		pr_err("Error I2C send %d\n", ret);
		return ret;
	} else {
		pr_err("Error I2C send size mismatch %d\n", ret);
		return -EIO;
	}
}



static void tfa98xx_dsp_init(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, init_work);

	pr_debug("profile %d, vstep %d\n", tfa98xx->profile_ctl,
		 tfa98xx->vstep_ctl);

	mutex_lock(&tfa98xx->dsp_init_lock);

	/* start the DSP using the latest profile / vstep */
	if (!tfa98xx_dsp_start(tfa98xx, tfa98xx->profile_ctl,
			       tfa98xx->vstep_ctl))
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;

	mutex_unlock(&tfa98xx->dsp_init_lock);
}


static void tfa98xx_stop(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, stop_work);

    pr_err("%s\n",__func__);

	mutex_lock(&tfa98xx->dsp_init_lock);


		/*
		 * need to wait for amp to stop switching, to minimize
		 * pop, else I2S clk is going away too soon interrupting
		 * the dsp from smothering the amp pop while turning it
		 * off, It shouldn't take more than 50 ms for the amp
		 * switching to stop.
		 */
		tfa98xx_dsp_stop(tfa98xx);

	mutex_unlock(&tfa98xx->dsp_init_lock);
}


/*
 * ASOC OPS
*/

#if 0
static u32 tfa98xx_asrc_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};

static struct snd_pcm_hw_constraint_list constraints_12_24 = {
	.list   = tfa98xx_asrc_rates,
	.count  = ARRAY_SIZE(tfa98xx_asrc_rates),
};
#endif

static int tfa98xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct tfa98xx *tfa98xx =
				snd_soc_codec_get_drvdata(codec_dai->codec);

	pr_debug("%s() freq %d, dir %d\n", __func__, freq, dir);

	tfa98xx->sysclk = freq;
	return 0;
}

static int tfa98xx_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 val;

	pr_debug("\n");

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* default value */
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
	default:
		/* only supports Slave mode */
		pr_err("tfa98xx: invalid DAI master/slave interface\n");
		return -EINVAL;
	}
	val = snd_soc_read(codec, TFA98XX_AUDIOREG);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* default value */
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val &= ~(TFA98XX_FORMAT_MASK);
		val |= TFA98XX_FORMAT_LSB;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val &= ~(TFA98XX_FORMAT_MASK);
		val |= TFA98XX_FORMAT_MSB;
		break;
	default:
		pr_err("tfa98xx: invalid DAI interface format\n");
		return -EINVAL;
	}

	snd_soc_write(codec, TFA98XX_AUDIOREG, val);

	return 0;
}

static int tfa98xx_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

	pr_debug("Store rate: %d\n", params_rate(params));

	/* Store rate for further use during DSP init */
	tfa98xx->rate = params_rate(params);

	return 0;
}

static int tfa98xx_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

	pr_err("state: %d\n", mute);

	mutex_lock(&tfa98xx->dsp_init_lock);

	if (mute) {
        tfa98xx->stop_ref = 1;
	} else {
		/*
		 * start monitor thread to check IC status bit 5secs, and
		 * re-init IC to recover.
		 */

        tfa98xx->stop_ref = 0;

		tfa98xx_dsp_quickstart(tfa98xx, tfa98xx->profile_ctl, tfa98xx->vstep_ctl);
	}

	mutex_unlock(&tfa98xx->dsp_init_lock);

	return 0;
}

static int tfa98xx_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{

#if 0
	if (NULL == substream->runtime) {
		pr_warn("runtime is NULL. I think it is offload now.\n");
		return 0;
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   &constraints_12_24);
#endif

	return 0;
}

static void tfa98xx_shutdown(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	//struct snd_soc_codec *codec = dai->codec;
	//struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

}

/* Trigger callback is atomic function, It gets called when pcm is started */

static int tfa98xx_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	//struct snd_soc_codec *codec = dai->codec;
	//struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	pr_err("trigger:%d(0:stop 1:start)\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        break;
	case SNDRV_PCM_TRIGGER_STOP:
        //queue_work(tfa98xx->tfa98xx_wq, &tfa98xx->stop_work);
        break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*
 * ASOC controls
 */

static const struct snd_soc_dapm_widget tfa98xx_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("I2S1"),
	SND_SOC_DAPM_MIXER("NXP Output Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes[] = {
	{"NXP Output Mixer", NULL, "Playback"},
};


/*
 * Helpers for profile selection controls
 */
int tfa98xx_get_profile_ctl(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&tfa98xx->dsp_init_lock);
	ucontrol->value.integer.value[0] = tfa98xx->profile_ctl;
	mutex_unlock(&tfa98xx->dsp_init_lock);
	return 0;
}

int tfa98xx_set_profile_ctl(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	struct tfaprofile *profiles =
				(struct tfaprofile *)kcontrol->private_value;
	struct tfaprofile *prof_old = &profiles[tfa98xx->profile_current];
	struct tfaprofile *prof_new;

	if (profiles == NULL) {
		pr_err("No profiles\n");
		return -EINVAL;
	}

	mutex_lock(&tfa98xx->dsp_init_lock);

	if (tfa98xx->profile_ctl == ucontrol->value.integer.value[0])
{
	mutex_unlock(&tfa98xx->dsp_init_lock);
		return 0;
}
	tfa98xx->profile_ctl = ucontrol->value.integer.value[0];
	prof_new = &profiles[tfa98xx->profile_ctl];

	pr_debug("active profile %d, new profile %d\n",
		 tfa98xx->profile_current, tfa98xx->profile_ctl);

	/*
	   adjust the vstep value based on the number of volume steps of the
	   new profile.
	*/
	prof_new->vstep =  (int)(tfa98xx->vstep_ctl * prof_new->vsteps
				/ prof_old->vsteps);

	pr_debug("active vstep %d, new vstep %d\n", prof_old->vstep,
		 prof_new->vstep);

	if (tfa98xx_is_amp_running(tfa98xx)) {
		/*
		   When switching profile when the amplifier is running,
		   there might be pops because of the mute/unmute during
		   profile switch.
		 */
		pr_debug("Warning: switching profile while amplifier is running\n");
		tfa98xx_dsp_start(tfa98xx, tfa98xx->profile_ctl,
				  prof_new->vstep);
	}

	prof_old->vstep = tfa98xx->vstep_ctl;
	tfa98xx->vstep_ctl = prof_new->vstep;

	mutex_unlock(&tfa98xx->dsp_init_lock);
	return 0;
}

int tfa98xx_info_profile_ctl(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 5;// tfa98xx->profile_count;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max =5 ;// tfa98xx->profile_count -1;

	return 0;
}

/*
 * Helpers for volume through vstep controls
 */
int tfa98xx_get_vol_ctl(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct tfaprofile *profiles =
				(struct tfaprofile *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int index = tfa98xx->profile_ctl;
	struct tfaprofile *prof = &profiles[index];


	if (profiles == NULL) {
		pr_err("No profiles\n");
		return -EINVAL;
	}
	mutex_lock(&tfa98xx->dsp_init_lock);
	ucontrol->value.integer.value[0] = prof->vsteps - prof->vstep - 1;

	mutex_unlock(&tfa98xx->dsp_init_lock);
	return 0;
}

int tfa98xx_set_vol_ctl(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct tfaprofile *profiles =
				(struct tfaprofile *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int index = tfa98xx->profile_ctl;
	struct tfaprofile *prof = &profiles[index];

	if (profiles == NULL) {
		pr_err("No profiles\n");
		return -EINVAL;
	}

	mutex_lock(&tfa98xx->dsp_init_lock);

	if (prof->vstep == prof->vsteps - ucontrol->value.integer.value[0] - 1)
{
	mutex_unlock(&tfa98xx->dsp_init_lock);
		return 0;
		}


	prof->vstep = prof->vsteps - ucontrol->value.integer.value[0] - 1;

	if (prof->vstep < 0)
		prof->vstep = 0;

	if (tfa98xx_is_amp_running(tfa98xx))
		tfa98xx_dsp_start(tfa98xx, tfa98xx->profile_ctl, prof->vstep);

	tfa98xx->vstep_ctl = prof->vstep;

	mutex_unlock(&tfa98xx->dsp_init_lock);
	return 1;
}

int tfa98xx_info_vol_ctl(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_info *uinfo)
{
	struct tfaprofile *profiles =
				(struct tfaprofile *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	struct tfaprofile *prof = &profiles[tfa98xx->profile_ctl];

	if (profiles == NULL) {
		pr_err("No profiles\n");
		return -EINVAL;
	}
	mutex_lock(&tfa98xx->dsp_init_lock);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = prof->vsteps - 1;
	mutex_unlock(&tfa98xx->dsp_init_lock);
	return 0;
}

int tfa98xx_get_stop_ctl(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

int tfa98xx_set_stop_ctl(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	//struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	//struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

	//pr_err("%s %ld\n", __func__,ucontrol->value.integer.value[0]);
#if 0
	if ((ucontrol->value.integer.value[0] != 0) &&
	    !tfa98xx_is_pwdn(tfa98xx)) {

		tfa98xx_dsp_stop(tfa98xx);
	}

	ucontrol->value.integer.value[0] = 0;
#endif

	return 1;
}



void tfa98xx_play_stop(void)
{
	struct tfa98xx *tfa98xx = g_tfa98xx;

    pr_err("tfa stop\n");

    if(g_tfa98xx == NULL)
    {
       pr_err("g_tfa98xx == null\n");
      return;
    }

	mutex_lock(&tfa98xx->dsp_init_lock);

    if(tfa98xx->stop_ref == 0)
    {
        pr_err("stop_ref:0\n");
        mutex_unlock(&tfa98xx->dsp_init_lock);
        return;
    }

	/*
	 * need to wait for amp to stop switching, to minimize
	 * pop, else I2S clk is going away too soon interrupting
	 * the dsp from smothering the amp pop while turning it
	 * off, It shouldn't take more than 50 ms for the amp
	 * switching to stop.
	 */
	tfa98xx_dsp_stop(tfa98xx);
    tfa98xx->stop_ref = 0;
	mutex_unlock(&tfa98xx->dsp_init_lock);
}



#define MAX_CONTROL_NAME	32

static char prof_name[MAX_CONTROL_NAME];
static char vol_name[MAX_CONTROL_NAME];
static char stop_name[MAX_CONTROL_NAME];

static struct snd_kcontrol_new tfa98xx_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = prof_name,
		.info = tfa98xx_info_profile_ctl,
		.get = tfa98xx_get_profile_ctl,
		.put = tfa98xx_set_profile_ctl,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = vol_name,
		.info = tfa98xx_info_vol_ctl,
		.get = tfa98xx_get_vol_ctl,
		.put = tfa98xx_set_vol_ctl,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = stop_name,
		.info = snd_soc_info_bool_ext,
		.get = tfa98xx_get_stop_ctl,
		.put = tfa98xx_set_stop_ctl,
	}
};


static const struct snd_soc_dai_ops tfa98xx_ops = {
	.hw_params	= tfa98xx_hw_params,
	.digital_mute	= tfa98xx_digital_mute,
	.set_fmt	= tfa98xx_set_dai_fmt,
	.set_sysclk	= tfa98xx_set_dai_sysclk,
	.startup	= tfa98xx_startup,
	.shutdown	= tfa98xx_shutdown,
	.trigger	= tfa98xx_trigger,
};


static struct snd_soc_dai_driver tfa98xx_dai = {
	.name = "tfa98xx_codec",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = TFA98XX_RATES,
		     .formats = TFA98XX_FORMATS,},
	.ops = &tfa98xx_ops,
	.symmetric_rates = 1,
};

static int tfa98xx_probe(struct snd_soc_codec *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int ret;
	u16 rev;

	codec->control_data = tfa98xx->regmap;
	tfa98xx->codec = codec;
	codec->cache_bypass = true;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
pr_err("%s snd_soc_codec_set_cache_io\n",__func__);
	ret = snd_soc_codec_set_cache_io(codec, 8, 16, SND_SOC_REGMAP);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
#endif

	/*
	 * some device require a dummy read in order to generate
	 * i2c clocks when accessing the device for the first time
	 */
	snd_soc_read(codec, TFA98XX_REVISIONNUMBER);

	rev = snd_soc_read(codec, TFA98XX_REVISIONNUMBER);
	dev_info(codec->dev, "ID revision 0x%04x\n", rev);
	tfa98xx->rev = rev & 0xff;
	tfa98xx->subrev = (rev >> 8) & 0xff;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,12,0)

pr_err("%s snd_soc_dapm_new_controls\n",__func__);

	snd_soc_dapm_new_controls(&codec->dapm, tfa98xx_dapm_widgets,
				  ARRAY_SIZE(tfa98xx_dapm_widgets));

	snd_soc_dapm_add_routes(&codec->dapm, tfa98xx_dapm_routes,
				ARRAY_SIZE(tfa98xx_dapm_routes));

	snd_soc_dapm_new_widgets(&codec->dapm);
	snd_soc_dapm_sync(&codec->dapm);
#endif

	ret = tfa98xx_cnt_loadfile(tfa98xx, 0);
	if (ret)
		return ret;

	tfa98xx->profile_current = 0;
	tfa98xx->vstep_current = 0;
	tfa98xx->profile_ctl = 0;
	tfa98xx->vstep_ctl = 0;

	/* Overwrite kcontrol values that need container information */
	tfa98xx_controls[0].private_value = (unsigned long)tfa98xx->profiles,
	tfa98xx_controls[1].private_value = (unsigned long)tfa98xx->profiles,
	scnprintf(prof_name, MAX_CONTROL_NAME, "%s Profile", tfa98xx->fw.name);
	scnprintf(vol_name, MAX_CONTROL_NAME, "%s Master Volume",
		  tfa98xx->fw.name);
	scnprintf(stop_name, MAX_CONTROL_NAME, "%s Stop", tfa98xx->fw.name);

	snd_soc_add_codec_controls(codec, tfa98xx_controls,
				   ARRAY_SIZE(tfa98xx_controls));

	dev_info(codec->dev, "tfa98xx codec registered");
	g_tfa98xx = tfa98xx;
	return 0;
}

static int tfa98xx_remove(struct snd_soc_codec *codec)
{
	dev_info(codec->dev, "tfa98xx codec removed");
	return 0;
}

static struct snd_soc_codec_driver tfa98xx_soc_codec = {
	.probe = tfa98xx_probe,
	.remove = tfa98xx_remove,

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)
	.dapm_widgets = tfa98xx_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tfa98xx_dapm_widgets),
	.dapm_routes = tfa98xx_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(tfa98xx_dapm_routes),
#endif
};

static const struct regmap_config tfa98xx_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = TFA98XX_MAX_REGISTER,
	.cache_type = REGCACHE_NONE,
};

static int tfa98xx_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct tfa98xx *tfa98xx;
	int ret;
	struct device_node *np = i2c->dev.of_node;
    int error = 0;

    pr_err("%s\n",__func__);


    if(np!=NULL)
        tfa_codec_np =np;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	tfa98xx = devm_kzalloc(&i2c->dev, sizeof(struct tfa98xx),
			       GFP_KERNEL);
	if (tfa98xx == NULL)
		return -ENOMEM;

	tfa98xx->i2c = i2c;
	tfa98xx->dsp_init = TFA98XX_DSP_INIT_PENDING;

	tfa98xx->regmap = devm_regmap_init_i2c(i2c, &tfa98xx_regmap);
	if (IS_ERR(tfa98xx->regmap)) {
		ret = PTR_ERR(tfa98xx->regmap);
		dev_err(&i2c->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, tfa98xx);
	mutex_init(&tfa98xx->dsp_init_lock);

	/* work queue will be used to load DSP fw on first audio playback */
	tfa98xx->tfa98xx_wq = create_singlethread_workqueue("tfa98xx");
	if (tfa98xx->tfa98xx_wq == NULL) {
		ret = -ENOMEM;
		goto wq_fail;
	}

	tfa98xx->rst_gpio = of_get_named_gpio(np, "reset_gpio",0);
	ret = gpio_request(tfa98xx->rst_gpio, "tfa reset gpio");
	if (ret < 0)
	{
		pr_err("%s: tfa reset gpio_request failed: %d\n",__func__, ret);
		goto gpio_fail;
	}
	gpio_direction_output(tfa98xx->rst_gpio, 1);
	udelay(100);
	gpio_direction_output(tfa98xx->rst_gpio, 0);

	INIT_WORK(&tfa98xx->init_work, tfa98xx_dsp_init);
	INIT_WORK(&tfa98xx->stop_work, tfa98xx_stop);
    tfa98xx->stop_ref = 0;

	/* register codec */
	ret = snd_soc_register_codec(&i2c->dev, &tfa98xx_soc_codec,
				     &tfa98xx_dai, 1);
	if (ret < 0) {
		pr_err("%s: Error registering tfa98xx codec", __func__);
		goto codec_fail;
	}

	pr_debug("tfa98xx probed successfully!");


	error = sysfs_create_file(&i2c->dev.kobj, &tfa98xx_state_attr.attr);
    if(error < 0)
    {
        pr_err("%s sysfs_create_file tfa98xx_state_attr err.",__func__);
    }
	error = sysfs_create_file(&i2c->dev.kobj, &tfa98xx_Log_state_attr.attr);
    if(error < 0)
    {
        pr_err("%s sysfs_create_file tfa98xx_Log_state_attr err.",__func__);
    }

	return ret;

codec_fail:
	destroy_workqueue(tfa98xx->tfa98xx_wq);
gpio_fail:
wq_fail:
	snd_soc_unregister_codec(&i2c->dev);

	return ret;
}

static int tfa98xx_i2c_remove(struct i2c_client *client)
{
	struct tfa98xx *tfa98xx = i2c_get_clientdata(client);
	snd_soc_unregister_codec(&client->dev);
	destroy_workqueue(tfa98xx->tfa98xx_wq);
	return 0;
}

static const struct i2c_device_id tfa98xx_i2c_id[] = {
	{ "tfa98xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa98xx_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id tfa98xx_match_tbl[] = {
	{ .compatible = "nxp,tfa9890" },
	{ },
};
MODULE_DEVICE_TABLE(of, tfa98xx_match_tbl);
#endif

static struct i2c_driver tfa98xx_i2c_driver = {
	.driver = {
		.name = "tfa98xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tfa98xx_match_tbl),
	},
	.probe =    tfa98xx_i2c_probe,
	.remove =   tfa98xx_i2c_remove,
	.id_table = tfa98xx_i2c_id,
};

module_i2c_driver(tfa98xx_i2c_driver);

MODULE_DESCRIPTION("ASoC tfa98xx codec driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP");
