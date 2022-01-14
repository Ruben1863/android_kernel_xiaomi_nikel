/* dev_info.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* dev_info.c
 * private function from Longcheer (LCT) software.
 * This is designed to record information of all special devices for phonecells,
 * such as LCM, MCP, Touchpannel, cameras, sensors and so on.
 * By   : shaohui@longcheer.net
 * Date : 2012/11/26
 *
 * */
/*=================================================================*/
/*
*	Date	: 2013.01.31
*	By		: shaohui
*	Discription:
*	change function devinfo_check_add_device(struct devinfo_struct *dev)
*	For if a same device (just diffrent used state) found, replace it!
*/
/*=================================================================*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/list.h>


/* TO define some */
#include <linux/dev_info.h>
#include <linux/seq_file.h>
#include <linux/miscdevice.h>

static DEFINE_SPINLOCK(dev_lock);
static struct list_head active_devinfo_list;

static unsigned int device_num = 0;
/*********************************************************************************
 * This functions is designed to clear struct as default value;
 * Input:	devinfo_struct
 * Output:  0
 * Note:
 * *******************************************************************************/
#if 0
static int devinfo_declare_NULL_device( struct devinfo_struct *dev)
{

//	struct devinfo_struct *dev = container_of(dev_type, struct devinfo_struct,
//					       device_type);
	dev->device_module	= DEVINFO_NULL;
	dev->device_vendor	= DEVINFO_NULL;
	dev->device_ic		= DEVINFO_NULL;
	dev->device_version	= DEVINFO_NULL;
	dev->device_info	= DEVINFO_NULL;
	dev->device_used	= DEVINFO_UNUSED ;
	return 0;
}
#endif
/*********************************************************************************
 * This functions is designed to add a new devinfo struct to list;
 * Input:	devinfo_struct
 * Output:  0
 * Note:
 * *******************************************************************************/
int devinfo_declare_new_device(struct devinfo_struct *dev)
{
    unsigned long irqflags;

//	struct devinfo_struct *dev = container_of(dev_type, struct devinfo_struct,
//					       device_type);
//	devinfo_declare_NULL_device(dev);
printk("[DEVINFO]add new device:%s,%s,%s,%s,%s,%s\n",dev->device_module,dev->device_vendor,dev->device_ic,dev->device_version,dev->device_info,dev->device_used);
    spin_lock_irqsave(&dev_lock, irqflags);
    list_add_tail(&dev->device_link, &active_devinfo_list);
    spin_unlock_irqrestore(&dev_lock, irqflags);

	device_num ++;

	return 0;
}

/*********************************************************************************
 * This functions is designed to delate a certain devinfo;
 * Input:
 * Output:  0
 * Note: Not complete!
 * *******************************************************************************/
int devinfo_delete_new_device(struct devinfo_struct *dev)
{

	// it is unnessary to call this function.??
	return 0;
}

/*********************************************************************************
 * This functions is designed to show all list;
 * Input:	NULL
 * Output:  0
 * Note: Do not complete!
 * *******************************************************************************/
#if 0
static int devinfo_show_device(void)
{
//NULL fuc
	struct devinfo_struct *dev ;

    list_for_each_entry(dev, &active_devinfo_list, device_link) {
	}
	return 0;
}
#endif
/*********************************************************************************
 * This functions is designed to modify a certain devinfo params;
 * Input:	devinfo_struct
 * Output:  0
 * Note:
 * *******************************************************************************/
int devinfo_mod_para(char* name,char* type,char* new_value)
{
    unsigned long irqflags;

	struct devinfo_struct *dev;// = container_of(name, struct devinfo_struct,
					     //  type);
//	devinfo_declare_NULL_device(dev);
//printk("[DEVINFO]add new device:%s,%s,%s,%s,%s,%s\n",dev->device_module,dev->device_vendor,dev->device_ic,dev->device_version,dev->device_info,dev->device_used);
    spin_lock_irqsave(&dev_lock, irqflags);
    list_for_each_entry(dev, &active_devinfo_list, device_link) {
//	if(strcmp(name,dev->type)==0)
//	{
//		printk("[DEVINFO] find the name\n");
//	}


	}
    spin_unlock_irqrestore(&dev_lock, irqflags);

	device_num ++;

	return 0;
}


/*********************************************************************************
 * This functions is designed to check if declared already, and add new device if not yet;
 * Input:	devinfo_struct
 * Output:  1 / 0
 * Note: return 1 for there have a same device registed,0 for new device
 * *******************************************************************************/
int devinfo_check_add_device(struct devinfo_struct *dev)
{
    unsigned long irqflags;
	struct devinfo_struct *dev_all;
	printk("[DEVINFO] devinfo_check!\n");
	spin_lock_irqsave(&dev_lock, irqflags);
	if(list_empty(&active_devinfo_list) != 1)
    list_for_each_entry(dev_all, &active_devinfo_list, device_link) {
	printk("[DEVINFO] dev type:%s\n",dev->device_type);
	printk("[DEVINFO] dev list type:%s\n",dev_all->device_type);
	if((strcmp(dev_all->device_type,dev->device_type)==0) && (strcmp(dev_all->device_module,dev->device_module)==0) &&
		(strcmp(dev_all->device_vendor,dev->device_vendor)==0) && (strcmp(dev_all->device_ic,dev->device_ic)==0) &&
			(strcmp(dev_all->device_version,dev->device_version)==0) &&(strcmp(dev_all->device_info,dev->device_info)==0))
		// It will be replaced if there is a used device found! Do not mention HOT plug device! 2013.01.31
		//	(strcmp(dev_all->device_used,dev->device_used)==0))
	{
		if(strcmp(dev_all->device_used,dev->device_used)==0)
		{
			printk("[DEVINFO] find the same device\n");
		}else if(strcmp(dev_all->device_used,DEVINFO_UNUSED)==0)
		{
		//we belive that we find a existed device! del the unexisted one!
			printk("[DEVINFO] find device,but unused state!\n");
			list_del(&dev_all->device_link);
    		list_add_tail(&dev->device_link, &active_devinfo_list);
		//	list_replace(&dev_all->device_link, &active_devinfo_list);
			spin_unlock_irqrestore(&dev_lock, irqflags);
			return 0;
		}else{
		//If a for-existed device is found lost,Do nothing~
			printk("[DEVINFO] find device,but used state!\n");
		}

		spin_unlock_irqrestore(&dev_lock, irqflags);
		return 1;
	}

	}
    list_add_tail(&dev->device_link, &active_devinfo_list);
    spin_unlock_irqrestore(&dev_lock, irqflags);
	return 0;
}









/*misc device start*/


static int misc_devinfo_open(struct inode *inode, struct file *file)
{
	return 0;
}

static long misc_devinfo_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	return 0;
}

/*----------------------------------------------------------------------------*/
static struct file_operations devinfo_fops = {
	.owner = THIS_MODULE,
	.open = misc_devinfo_open,
//	.release = misc_devinfo_release,
	.unlocked_ioctl = misc_devinfo_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice devinfo_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "devinfo",
	.fops = &devinfo_fops,
};

/*misc device end*/




static int devinfo_probe(struct platform_device *pdev)
{
		printk("DEVINFO,devinfoprobe\n");
		INIT_LIST_HEAD(&active_devinfo_list);
		device_num = 0;

		return 0;
}

static struct platform_driver devinfo_driver = {
    .driver.name = "devinfo",
    .probe = devinfo_probe,
    #ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = NULL,
    .resume     = NULL,
    #endif
};
static struct platform_device devinfo_device = {
    .name = "devinfo",
};








static int devinfo_stats_show(struct seq_file *m, void *unused)
{
    unsigned long irqflags;
	struct devinfo_struct *dev ;
    int ret;

    spin_lock_irqsave(&dev_lock, irqflags);
    ret = seq_puts(m, "name\t\tmodule\t\tvender\t\tIC\tVersion\tinfo\t\tused\n");
    list_for_each_entry(dev, &active_devinfo_list, device_link)
    	seq_printf(m, "%s\t\t%s\t\t%s\t\t%s\t%s\t%s\t\t%s\n",
			dev->device_type,dev->device_module,dev->device_vendor,dev->device_ic,dev->device_version,dev->device_info,dev->device_used);
    spin_unlock_irqrestore(&dev_lock, irqflags);
    return 0;
}
static int devinfo_stats_open(struct inode *inode, struct file *file)
{
    return single_open(file, devinfo_stats_show, NULL);
}


static const struct file_operations devinfo_stats_fops = {
    .owner = THIS_MODULE,
    .open = devinfo_stats_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};




static unsigned long ram_size;

void setRamInfo(unsigned long size)
{
	ram_size = size;
}

static int raminfo_stats_show(struct seq_file *m, void *unused)
{
	seq_printf(m,"%ld\n",ram_size);
	return 0;
}
static int raminfo_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, raminfo_stats_show, NULL);
}

static const struct file_operations raminfo_stats_fops = {
	.owner = THIS_MODULE,
	.open = raminfo_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};



static int __init slt_devinfo_init(void)
{
    int ret;
	printk("DEVINFO,devinfo init!\n");
    ret = platform_device_register(&devinfo_device);
    if (ret) {
        pr_err("[slt_devinfo_init]: platform_device_register failed\n");
        goto err_platform_device_register;
    }
    ret = platform_driver_register(&devinfo_driver);
    if (ret) {
        pr_err("[slt_devinfo_init]: platform_driver_register failed\n");
        goto err_platform_driver_register;
    }
// add for proc
    proc_create("devicesinfo", S_IRUGO, NULL, &devinfo_stats_fops);
//add for raminfo
    proc_create("raminfo", S_IRUGO, NULL, &raminfo_stats_fops);
    return 0;


err_platform_driver_register:
    platform_device_unregister(&devinfo_device);
err_platform_device_register:
    return ret;
}

static void  __exit slt_devinfo_exit(void)
{
    remove_proc_entry("devicesinfo", NULL);
    remove_proc_entry("raminfo", NULL);
    platform_driver_unregister(&devinfo_driver);
    platform_device_unregister(&devinfo_device);

}

core_initcall(slt_devinfo_init);
module_exit(slt_devinfo_exit);
MODULE_AUTHOR("shaohui@longcheer.net");
MODULE_LICENSE("GPL");
