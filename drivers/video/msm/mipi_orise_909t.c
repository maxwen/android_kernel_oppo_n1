/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_orise.h"
#include "mdp4.h"

#include <mach/gpio.h>              //for 8064gpio
#include <linux/mfd/pm8xxx/pm8921.h>    //for 8921
#include <linux/i2c/lm3528.h>   //lm3528
#include <linux/i2c/lm3630_bl.h>    //lm3630
#include <linux/irq.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/switch.h>

#define PM8921_GPIO_BASE        NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
#define LCD_TE_GPIO  83

extern bool get_panel_info(void);

spinlock_t te_count_lock;
unsigned long flags;
#define MIPI_CMD_INIT
//#define MIPI_READ
#define PANEL_CABC

extern struct platform_device *g_mdp_dev;
static struct mipi_dsi_panel_platform_data *mipi_orise_pdata;
static struct dsi_buf orise_tx_buf;
static struct dsi_buf orise_rx_buf;

static int globle_bkl;
static int irq;
static int flag_lcd_node_onoff = 0;
static int te_state = 0;
spinlock_t te_state_lock;

bool bk_device_3630= false;
bool bk_device_3528= false;

static struct wake_lock te_wake_lock;
/* OPPO 2013-03-07 zhengzk Add begin for reason */
static struct switch_dev display_switch;
/* OPPO 2013-03-07 zhengzk Add end */

#ifdef PANEL_CABC
enum
{
    CABC_CLOSE = 0,
    CABC_LOW_MODE,
    CABC_MIDDLE_MODE,
    CABC_HIGH_MODE,

};
static int cabc_mode = CABC_CLOSE;
#endif


extern int mipi_dsi_off(struct platform_device *pdev);
extern int mipi_dsi_on(struct platform_device *pdev);

static inline void init_suspend(void)
{
    wake_lock_init(&te_wake_lock, WAKE_LOCK_SUSPEND, "techeck");
}

static inline void deinit_suspend(void)
{
    wake_lock_destroy(&te_wake_lock);
}

static inline void prevent_suspend(void)
{
    wake_lock_timeout(&te_wake_lock, 5 * HZ);
}

static inline void allow_suspend(void)
{
    wake_unlock(&te_wake_lock);
}

#ifdef MIPI_CMD_INIT
static char protect_off[2] =
{
    0xb0,   //Manufacuture Command Acceses Protect Off ( Hsync out option)
    0x04,
};


static char hsync_output[4] =
{
    0xc3,   //Hsync Output ( Hsync out option)
    0x01, 0x00, 0x10,
};

static char protect_on[2] =
{
    0xb0,   //Manufacuture Command Acceses Protect On  ( Hsync out option)
    0x03,
};

static char cabc_control[2] =
{
    0x55,   //CABC control
    0x03,
};

static char bkl_control[2] =
{
    0x53,   //Back light control
    0x2c,
};

static char te_out[2] =
{
    0x35,   //TE OUT
    0x00,
};

static char display_on[2] =
{
    0x29,   //display on
    0x00,
};

static char sleep_out[2] =
{
    0x11,   //sleep out
    0x00,
};

static char brightness_setting[3] =
{
    0x51,   //brightness_setting
    0xff,
};

static char display_off[2] =
{
    0x28,   //display off
    0x00,
};

static char sleep_in[2] =
{
    0x10,   //sleep in
    0x00,
};

static char disable_nvm[2] =
{
    0xd6,   //Deep standby off (option)
    0x01,
};

static char pwm_freqency[8] =
{
    0xce,   //Deep standby off (option)
    0x00,
    0x01,
    0x88,
    0xc1,
    0x00,
    0x1e,
    0x04
};

#ifdef PANEL_CABC
static char cabc_off[2] = { 0x55, 0x00};
static char cabc_user_interface_image[2] = { 0x55, 0x01};
static char cabc_still_image[2] = { 0x55, 0x02};
static char cabc_video_image[2] = { 0x55, 0x03};

static char cabc_movie_bl_ctr[] = { 0x07,0xba,0x00,0x78,0x64,0x10,0x64,0xff};

static char cabc_min_brightness[2] = { 0x5e, 0x00};

static struct dsi_cmd_desc cabc_off_sequence[] =
{
    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(cabc_off), cabc_off
    },
    {
        DTYPE_DCS_WRITE, 1, 0, 0, 10,
        sizeof(display_on), display_on
    },
};

static struct dsi_cmd_desc cabc_user_interface_image_sequence[] =
{
    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(cabc_user_interface_image), cabc_user_interface_image
    },
    {
        DTYPE_DCS_WRITE, 1, 0, 0, 10,
        sizeof(display_on), display_on
    },
};

static struct dsi_cmd_desc cabc_still_image_sequence[] =
{
    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(cabc_still_image), cabc_still_image
    },
    {
        DTYPE_DCS_WRITE, 1, 0, 0, 10,
        sizeof(display_on), display_on
    },
};

static struct dsi_cmd_desc cabc_video_image_sequence[] =
{

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 0,
        sizeof(cabc_min_brightness), cabc_min_brightness
    },
    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 0,
        sizeof(cabc_movie_bl_ctr), cabc_movie_bl_ctr
    },
    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(cabc_video_image), cabc_video_image
    },
    {
        DTYPE_DCS_WRITE, 1, 0, 0, 10,
        sizeof(display_on), display_on
    },
};
#endif


static struct dsi_cmd_desc jdi_mipi_initial_sequence[] =
{
    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(protect_off), protect_off
    },
    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(hsync_output), hsync_output
    },

    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(pwm_freqency), pwm_freqency
    },

    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(disable_nvm), disable_nvm
    },

    {
        DTYPE_GEN_WRITE2, 1, 0, 0, 10,
        sizeof(protect_on), protect_on
    },

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(brightness_setting), brightness_setting
    },

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(bkl_control), bkl_control
    },

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(cabc_control), cabc_control
    },

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(te_out), te_out
    },

    {
        DTYPE_DCS_WRITE, 1, 0, 0, 10,
        sizeof(display_on), display_on
    },

    {
        DTYPE_DCS_WRITE, 1, 0, 0, 120,
        sizeof(sleep_out), sleep_out
    },

};

/* OPPO 2013-10-05 gousj Add begin for sharp panel flicker */
#ifdef CONFIG_VENDOR_EDIT
static struct dsi_cmd_desc cmd_mipi_display_off[] =
{

    {
        DTYPE_DCS_LWRITE, 1, 0, 0, 150,
        sizeof(display_off), display_off
    },
};


static struct dsi_cmd_desc cmd_mipi_sleep_in[] =
{
    {
        DTYPE_DCS_LWRITE, 1, 0, 0, 150,
        sizeof(sleep_in), sleep_in
    },
};
#endif //CONFIG_VENDOR_EDIT

static struct dsi_cmd_desc sharp_mipi_initial_sequence[] =
{
    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(protect_off), protect_off
    },
    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(hsync_output), hsync_output
    },


    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(pwm_freqency), pwm_freqency
    },

    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(disable_nvm), disable_nvm
    },

    {
        DTYPE_GEN_WRITE2, 1, 0, 0, 10,
        sizeof(protect_on), protect_on
    },


    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(brightness_setting), brightness_setting
    },


    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(bkl_control), bkl_control
    },

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(cabc_control), cabc_control
    },


    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(te_out), te_out
    },

    {
        DTYPE_DCS_WRITE, 1, 0, 0, 10,
        sizeof(display_on), display_on
    },

    {
        DTYPE_DCS_WRITE, 1, 0, 0, 120,
        sizeof(sleep_out), sleep_out
    },


};

static struct dsi_cmd_desc cmd_mipi_resume_sequence[] =
{
    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(protect_off), protect_off
    },
    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(hsync_output), hsync_output
    },

    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(pwm_freqency), pwm_freqency
    },

    {
        DTYPE_GEN_LWRITE, 1, 0, 0, 10,
        sizeof(disable_nvm), disable_nvm
    },

    {
        DTYPE_GEN_WRITE2, 1, 0, 0, 10,
        sizeof(protect_on), protect_on
    },

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(brightness_setting), brightness_setting
    },

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(bkl_control), bkl_control
    },

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(cabc_control), cabc_control
    },

    {
        DTYPE_DCS_WRITE1, 1, 0, 0, 10,
        sizeof(te_out), te_out
    },

    {
        DTYPE_DCS_WRITE, 1, 0, 0, 10,
        sizeof(display_on), display_on
    },

    {
        DTYPE_DCS_WRITE, 1, 0, 0, 120,
        sizeof(sleep_out), sleep_out
    },
};

static struct dsi_cmd_desc cmd_brightness_setting[] =
{
    {
        DTYPE_DCS_LWRITE, 1, 0, 0, 10,
        sizeof(brightness_setting), brightness_setting
    },
};

#endif

/* OPPO 2013-03-07 zhengzk Add begin for reason */
static int operate_display_switch(void)
{
    int ret = 0;
    printk("%s:state=%d.\n", __func__, te_state);

    spin_lock_irqsave(&te_state_lock, flags);
    if(te_state)
        te_state = 0;
    else
        te_state = 1;
    spin_unlock_irqrestore(&te_state_lock, flags);

    switch_set_state(&display_switch, te_state);
    return ret;
}
/* OPPO 2013-03-07 zhengzk Add end */

#ifdef PANEL_CABC
static int set_cabc_resume_mode(int mode)
{
    int ret;

    switch(mode)
    {
        case 0:
            mipi_dsi_cmds_tx(&orise_tx_buf, cabc_off_sequence,
                             ARRAY_SIZE(cabc_off_sequence));
            break;
        case 1:
            mipi_dsi_cmds_tx(&orise_tx_buf, cabc_user_interface_image_sequence,
                             ARRAY_SIZE(cabc_user_interface_image_sequence));
            break;
        case 2:
            mipi_dsi_cmds_tx(&orise_tx_buf, cabc_still_image_sequence,
                             ARRAY_SIZE(cabc_still_image_sequence));
            break;
        case 3:
            mipi_dsi_cmds_tx(&orise_tx_buf, cabc_video_image_sequence,
                             ARRAY_SIZE(cabc_video_image_sequence));
            break;
        default:
            pr_err("%s  %d is not supported!\n",__func__,mode);
            ret = -1;
            break;
    }
    printk(KERN_INFO "set cabc mode %d finished\n",mode);
    set_backlight_pwm(1);
    return ret;
}
#endif


#ifdef MIPI_READ
static char addr_buf[2] = {0x53, 0x00};     //read register backlight ctrl
static struct dsi_cmd_desc cmd_mipi_addr_buf =
{
    DTYPE_GEN_READ/*DTYPE_DCS_READ*/, 1, 0, 1, 5, sizeof(addr_buf), addr_buf
};

static int mipi_orise_rd(struct msm_fb_data_type *mfd, char addr)
{
    struct dsi_buf *rp, *tp;
    struct dsi_cmd_desc *cmd;
    int *lp;
    addr_buf[0] = addr;
    tp = &orise_tx_buf;
    rp = &orise_rx_buf;
    mipi_dsi_buf_init(rp);
    mipi_dsi_buf_init(tp);

    cmd = &cmd_mipi_addr_buf;
    mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 4);
    lp = (uint32 *)rp->data;
    pr_info("mipi_orise_rd addr=%x, data=%x\n", addr, *lp);
    return *lp;
}
#endif

static bool flag_lcd_resume = false;
static bool flag_lcd_reset = false;
static bool flag_lcd_off = false;
static int te_count = 0;
static int irq_state = 1;

struct delayed_work techeck_work;

static int mipi_orise_lcd_on(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    mfd = platform_get_drvdata(pdev);

    if (!mfd)
        return -ENODEV;
    if (mfd->key != MFD_KEY)
        return -EINVAL;

#ifdef MIPI_CMD_INIT
    if(flag_lcd_resume)
    {
        //printk("huyu-------%s: lcd resume!\n",__func__);
        if(!flag_lcd_reset)
        {
            mipi_dsi_cmds_tx(&orise_tx_buf, cmd_mipi_resume_sequence,
                             ARRAY_SIZE(cmd_mipi_resume_sequence));
#ifdef PANEL_CABC
            set_cabc_resume_mode(cabc_mode);
#endif
        }
        else
        {
            mipi_dsi_cmds_tx(&orise_tx_buf, cmd_mipi_resume_sequence,
                             ARRAY_SIZE(cmd_mipi_resume_sequence));
            printk("huyu-------%s: lcd ESD reset initial!\n",__func__);
            mdelay(130);
            //printk("huyu-------%s: lcd cmd_brightness_setting!\n",__func__);
            mipi_dsi_cmds_tx(&orise_tx_buf, cmd_brightness_setting,
                             ARRAY_SIZE(cmd_brightness_setting));
            spin_lock_irqsave(&te_count_lock, flags);
            flag_lcd_reset = false;
            spin_unlock_irqrestore(&te_count_lock, flags);
        }

        spin_lock_irqsave(&te_count_lock, flags);
        te_count = 0;
        spin_unlock_irqrestore(&te_count_lock, flags);
        irq_state++;
        enable_irq(irq);
        schedule_delayed_work(&techeck_work, msecs_to_jiffies(5000));
    }
    else
    {
        //printk("huyu-------%s: lcd initial!\n",__func__);
        /*OPPO Neal modify for sharp panel*/
        if(get_panel_info() == 0)
        {
            mipi_dsi_cmds_tx(&orise_tx_buf, sharp_mipi_initial_sequence,
                             ARRAY_SIZE(sharp_mipi_initial_sequence));
            printk("Neal %s: sharp lcd initial!\n",__func__);
            mdelay(130);
            //printk("huyu-------%s: lcd cmd_brightness_setting!\n",__func__);
            mipi_dsi_cmds_tx(&orise_tx_buf, cmd_brightness_setting,
                             ARRAY_SIZE(cmd_brightness_setting));
        }
        else
        {
            mipi_dsi_cmds_tx(&orise_tx_buf, jdi_mipi_initial_sequence,
                             ARRAY_SIZE(jdi_mipi_initial_sequence));
            printk("Neal %s: jdi lcd initial!\n",__func__);
            mdelay(130);
            //printk("huyu-------%s: lcd cmd_brightness_setting!\n",__func__);
            mipi_dsi_cmds_tx(&orise_tx_buf, cmd_brightness_setting,
                             ARRAY_SIZE(cmd_brightness_setting));
        }
        /*OPPO Neal midify end*/

#ifdef PANEL_CABC
        cabc_mode = CABC_HIGH_MODE;
        //set_cabc_resume_mode(cabc_mode);
#endif
        flag_lcd_resume = true;
    }
#endif
    flag_lcd_off = false;
    flag_lcd_node_onoff = false;

    printk("1080p mipi_orise_lcd_on complete\n");
    return 0;
}

static int mipi_orise_lcd_off(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    mfd = platform_get_drvdata(pdev);
    if (!mfd)
        return -ENODEV;
    if (mfd->key != MFD_KEY)
        return -EINVAL;

    flag_lcd_off = true;

    cancel_delayed_work_sync(&techeck_work);
    mdelay(5);
    irq_state--;
    disable_irq(irq);
	set_backlight_pwm(1);
    printk("1080p mipi_orise_lcd_off complete\n");

    return 0;
}

extern bool bk_device_3630;
extern bool bk_device_3528;
/* OPPO 2013-10-05 gousj Add begin for sharp panel flicker */
#ifdef CONFIG_VENDOR_EDIT
int mipi_orise_display_off(struct platform_device *pdev)
{
    mipi_dsi_cmds_tx(&orise_tx_buf, cmd_mipi_display_off,
                     ARRAY_SIZE(cmd_mipi_display_off));
    return 0;
}

int mipi_orise_sleep_in(struct platform_device *pdev)
{
    mipi_dsi_cmds_tx(&orise_tx_buf, cmd_mipi_sleep_in,
                     ARRAY_SIZE(cmd_mipi_sleep_in));
    return 0;
}
#endif
/* OPPO 2013-10-05 gousj Add end */


#ifdef PANEL_CABC
static DEFINE_MUTEX(cabc_mutex);
static int set_cabc(int level)
{
    int ret = 0;
    pr_info("%s Neal level = %d\n",__func__,level);

    set_backlight_pwm(1);

    mutex_lock(&cabc_mutex);
    //mipi_dsi_clk_cfg(1);
    //mipi_set_tx_power_mode(0);
     /*OPPO 2013-10-11 zhzhyon Add for reason*/
    if(flag_lcd_off == true)
    {
        printk(KERN_INFO "lcd is off,don't allow to set cabc\n");
        cabc_mode = level;
        mutex_unlock(&cabc_mutex);
        return 0;
    }
    /*OPPO 2013-10-11 zhzhyon Add end*/


    switch(level)
    {
        case 0:
            mipi_dsi_cmds_tx(&orise_tx_buf, cabc_off_sequence,
                             ARRAY_SIZE(cabc_off_sequence));
            cabc_mode = CABC_CLOSE;
            break;
        case 1:
            mipi_dsi_cmds_tx(&orise_tx_buf, cabc_user_interface_image_sequence,
                             ARRAY_SIZE(cabc_user_interface_image_sequence));
            cabc_mode = CABC_LOW_MODE;
            break;
        case 2:
            mipi_dsi_cmds_tx(&orise_tx_buf, cabc_still_image_sequence,
                             ARRAY_SIZE(cabc_still_image_sequence));
            cabc_mode = CABC_MIDDLE_MODE;
            break;
        case 3:
            mipi_dsi_cmds_tx(&orise_tx_buf, cabc_video_image_sequence,
                             ARRAY_SIZE(cabc_video_image_sequence));
            cabc_mode = CABC_HIGH_MODE;
            break;
        default:
            pr_err("%s Leavel %d is not supported!\n",__func__,level);
            ret = -1;
            break;
    }
    //mipi_set_tx_power_mode(1);
    //mipi_dsi_clk_cfg(0);
    mutex_unlock(&cabc_mutex);
    return ret;

}
/*OPPO 2013-09-27 zhzhyon Add for reason*/
static ssize_t attr_orise_get_cabc(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{

    printk(KERN_INFO "get cabc mode = %d\n",cabc_mode);

    return sprintf(buf, "%d", cabc_mode);

}
/*OPPO 2013-09-27 zhzhyon Add end*/
static ssize_t attr_orise_cabc(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
    int level = 0;
    sscanf(buf, "%du", &level);
    set_cabc(level);
    return count;
}
#endif


static void mipi_orise_set_backlight(struct msm_fb_data_type *mfd)
{
    int bl_level;
    bl_level = mfd->bl_level;
    if(bl_level <0)
        bl_level = 0;

    if(bk_device_3630 == true && bk_device_3528 == false)
    {
        if(bl_level > 255)
            bl_level = 255;
        lm3630_bkl_control(bl_level);//3630
        pr_debug("Neal 3630 bkl = %d\n",bl_level);
    }
    else if(bk_device_3630 == false && bk_device_3528 == true)
    {
        if(bl_level > 127)
            bl_level = 127;
        lm3528_bkl_control(bl_level);
        pr_debug("Neal 3528 bkl = %d\n",bl_level);
    }
    globle_bkl = bl_level;
}

static ssize_t attr_orise_reinit(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    if(flag_lcd_off)
    {
        printk("%s: system is suspending, don't do this!\n",__func__);
        return 0;
    }

    prevent_suspend();
    flag_lcd_resume = true;
    mipi_dsi_off(g_mdp_dev);
    mdelay(100);
    mipi_dsi_on(g_mdp_dev);
    allow_suspend();
    return 0;
}

static ssize_t attr_orise_lcdon(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    if(!flag_lcd_node_onoff)
        return 0;

    flag_lcd_node_onoff = false;
    prevent_suspend();
    mipi_dsi_on(g_mdp_dev);
    allow_suspend();

    return 0;
}

static ssize_t attr_orise_lcdoff(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{

    if(flag_lcd_node_onoff)
        return 0;

    flag_lcd_node_onoff = true;
    prevent_suspend();
    mipi_dsi_off(g_mdp_dev);
    allow_suspend();

    return 0;
}

static ssize_t attr_orise_rda_bkl(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    ssize_t ret = strnlen(buf, PAGE_SIZE);
    int cmd;
    int bkl_from_ic = 0;
    bkl_from_ic = lm3630_bkl_readout();
    sscanf(buf, "%d", &cmd);
    printk("backlight from userspace=%d, read from IC=%d\n", globle_bkl, bkl_from_ic);
    return ret;
}

static ssize_t attr_orise_dispswitch(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    printk("ESD function test--------\n");
    operate_display_switch();
    return 0;
}

static DEVICE_ATTR(reinit, S_IRUGO , attr_orise_reinit, NULL);
static DEVICE_ATTR(lcdon, S_IRUGO , attr_orise_lcdon, NULL);
static DEVICE_ATTR(lcdoff, S_IRUGO , attr_orise_lcdoff, NULL);
static DEVICE_ATTR(backlight, S_IRUGO , attr_orise_rda_bkl, NULL);
static DEVICE_ATTR(dispswitch, S_IRUGO , attr_orise_dispswitch, NULL);
#ifdef PANEL_CABC
static DEVICE_ATTR(cabc, 0644 , attr_orise_get_cabc, attr_orise_cabc);
#endif


static struct attribute *fs_attrs[] =
{
#ifdef PANEL_CABC
		&dev_attr_cabc.attr,
#endif
    &dev_attr_reinit.attr,
    &dev_attr_lcdon.attr,
    &dev_attr_lcdoff.attr,
    &dev_attr_backlight.attr,
    &dev_attr_dispswitch.attr,
    NULL,
};

static struct attribute_group fs_attr_bkl_ctrl =
{
    .attrs = fs_attrs,
};

static irqreturn_t TE_irq_thread_fn(int irq, void *dev_id)
{
    spin_lock_irqsave(&te_count_lock, flags);
    te_count ++;
    spin_unlock_irqrestore(&te_count_lock, flags);
    //printk("huyu------%s: te_count = %d\n",__func__, te_count);
    return IRQ_HANDLED;
}

static void techeck_work_func( struct work_struct *work )
{
    //printk("huyu------%s: te_count = %d \n",__func__, te_count);
    if(flag_lcd_off)
    {

        printk("huyu------%s: lcd is off ing ! don't do this ! te_count = %d \n",__func__,te_count);
        return ;
    }
    if(te_count < 50)
    {
        printk("huyu------%s: lcd resetting ! te_count = %d \n",__func__,te_count);
        printk("irq_state=%d\n", irq_state);
        flag_lcd_resume = true;

        spin_lock_irqsave(&te_count_lock, flags);
        flag_lcd_reset = true;
        spin_unlock_irqrestore(&te_count_lock, flags);
        operate_display_switch();

        spin_lock_irqsave(&te_count_lock, flags);
        te_count = 0;
        spin_unlock_irqrestore(&te_count_lock, flags);
        schedule_delayed_work(&techeck_work, msecs_to_jiffies(2000));
        return ;

    }
    spin_lock_irqsave(&te_count_lock, flags);
    te_count = 0;
    spin_unlock_irqrestore(&te_count_lock, flags);
    schedule_delayed_work(&techeck_work, msecs_to_jiffies(2000));
}

static int __devinit mipi_orise_lcd_probe(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct platform_device *current_pdev;
    static struct mipi_dsi_phy_ctrl *phy_settings;
    int rc = 0;

    if (pdev->id == 0)
    {
        mipi_orise_pdata = pdev->dev.platform_data;

        if (mipi_orise_pdata
            && mipi_orise_pdata->phy_ctrl_settings)
        {
            phy_settings = (mipi_orise_pdata->phy_ctrl_settings);
        }
        return 0;
    }

    current_pdev = msm_fb_add_device(pdev);

    if (current_pdev)
    {
        mfd = platform_get_drvdata(current_pdev);
        if (!mfd)
            return -ENODEV;
        if (mfd->key != MFD_KEY)
            return -EINVAL;

        mipi  = &mfd->panel_info.mipi;

        if (phy_settings != NULL)
            mipi->dsi_phy_db = phy_settings;
    }

    rc = gpio_request(LCD_TE_GPIO, "LCD_TE_GPIO#");
    if (rc < 0)
    {
        pr_err("MIPI GPIO LCD_TE_GPIO request failed: %d\n", rc);
        return -ENODEV;
    }
    rc = gpio_direction_input(LCD_TE_GPIO);
    if (rc < 0)
    {
        pr_err("MIPI GPIO LCD_TE_GPIO set failed: %d\n", rc);
        return -ENODEV;
    }

    irq = gpio_to_irq(LCD_TE_GPIO);

    rc = request_threaded_irq(irq, NULL, TE_irq_thread_fn,
                              IRQF_TRIGGER_RISING, "LCD_TE",NULL);
    if (rc < 0)
    {
        pr_err("Unable to register IRQ handler\n");
        return -ENODEV;
    }

    INIT_DELAYED_WORK(&techeck_work, techeck_work_func );
    schedule_delayed_work(&techeck_work, msecs_to_jiffies(20000));
    init_suspend();

    /* ATTR node: root@android:/sys/devices/virtual/graphics/fb0/orise_bkl */
    mfd = platform_get_drvdata(current_pdev);
    rc = sysfs_create_group(&mfd->fbi->dev->kobj, &fs_attr_bkl_ctrl);
    if (rc)
    {
        pr_err("%s: fs_attr_bkl_ctrl sysfs group creation failed, rc=%d\n", __func__, rc);
        return rc;
    }

    /* OPPO 2013-03-07 zhengzk Add begin for reason */
    display_switch.name = "dispswitch";

    rc = switch_dev_register(&display_switch);
    if (rc)
    {
        pr_err("Unable to register display switch device\n");
        return rc;
    }
    /* OPPO 2013-03-07 zhengzk Add end */

    return 0;
}

static struct platform_driver this_driver =
{
    .probe = mipi_orise_lcd_probe,
    .driver = {
        .name = "mipi_orise",
    },
};

static struct msm_fb_panel_data orise_panel_data =
{
    .on = mipi_orise_lcd_on,
    .off = mipi_orise_lcd_off,
    .set_backlight = mipi_orise_set_backlight,
};

static int ch_used[3];

int mipi_orise_device_register(struct msm_panel_info *pinfo,
                               u32 channel, u32 panel)
{
    struct platform_device *pdev = NULL;
    int ret;

    if ((channel >= 3) || ch_used[channel])
        return -ENODEV;

    ch_used[channel] = TRUE;

    pdev = platform_device_alloc("mipi_orise", (panel << 8)|channel);
    if (!pdev)
        return -ENOMEM;

    orise_panel_data.panel_info = *pinfo;

    ret = platform_device_add_data(pdev, &orise_panel_data,
                                   sizeof(orise_panel_data));
    if (ret)
    {
        printk(KERN_ERR
               "%s: platform_device_add_data failed!\n", __func__);
        goto err_device_put;
    }

    ret = platform_device_add(pdev);
    if (ret)
    {
        printk(KERN_ERR
               "%s: platform_device_register failed!\n", __func__);
        goto err_device_put;
    }

    return 0;

err_device_put:
    platform_device_put(pdev);
    return ret;
}

static int __init mipi_orise_lcd_init(void)
{
    mipi_dsi_buf_alloc(&orise_tx_buf, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&orise_rx_buf, DSI_BUF_SIZE);

    printk("lcd is 1080p!--\n");
    return platform_driver_register(&this_driver);
}

module_init(mipi_orise_lcd_init);
