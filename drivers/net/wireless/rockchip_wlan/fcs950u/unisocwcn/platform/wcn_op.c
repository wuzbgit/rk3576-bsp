#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/miscdevice.h>
#include "marlin_platform.h"
#include <linux/module.h>
#include <linux/major.h>
#include <linux/proc_fs.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include "wcn_bus.h"
#include "wcn_dbg.h"
#include "../sleep/slp_mgr.h"

#define WCN_OP_NAME	"wcn_op"

#define  IOCTL_WCN_OP_READ		0xFF01
#define  IOCTL_WCN_OP_WRITE		0xFF02

struct wcn_op_attr_t {
	unsigned int addr;
	void __user *val;
	unsigned int length;
};

static int wcn_op_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int wcn_op_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int wcn_op_read(struct wcn_op_attr_t *wcn_op_attr, void *pval)
{
	int ret;

	if (unlikely(marlin_get_download_status() != true))
		return -EIO;

	ret = sprdwcn_bus_direct_read(wcn_op_attr->addr, pval,
				      wcn_op_attr->length);
	if (ret < 0) {
		WCN_ERR("%s read reg error:%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int wcn_op_write(struct wcn_op_attr_t *wcn_op_attr, void *ptr)
{
	int ret = 0;

	if (unlikely(marlin_get_download_status() != true))
		return -EIO;

	ret = sprdwcn_bus_direct_write(wcn_op_attr->addr, ptr, wcn_op_attr->length);
	if (ret < 0) {
		WCN_ERR("%s write reg error:%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static long wcn_op_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = -1;
	struct wcn_op_attr_t wcn_op_attr;
	unsigned int __user *pbuf = (unsigned int __user *)arg;
	void *ptr = NULL;

	if (pbuf == NULL)
		return  ret;

	if (copy_from_user(&wcn_op_attr, pbuf, sizeof(wcn_op_attr))) {
		WCN_ERR("%s copy from user error!\n", __func__);
		return -EFAULT;
	}

	WCN_INFO("WCN OPERATION IOCTL: 0x%x, addr=0x%x, val=0x%p, lenght=%u.\n",
		cmd, wcn_op_attr.addr, wcn_op_attr.val, wcn_op_attr.length);

	ptr = kmalloc(wcn_op_attr.length, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	if (copy_from_user(ptr, wcn_op_attr.val, sizeof(wcn_op_attr.length))) {
		WCN_ERR("%s copy from user error! Address invalid\n", __func__);
		kfree(ptr);
		return -EFAULT;
	}

	switch (cmd) {

	case IOCTL_WCN_OP_READ:
		ret = wcn_op_read(&wcn_op_attr, (unsigned int *)ptr);
		if (ret == 0) {
			if (copy_to_user(wcn_op_attr.val, ptr,
					 wcn_op_attr.length)) {
				WCN_ERR("%s copy from user error!\n", __func__);
				kfree(ptr);
				return -EFAULT;
			}
		} else
			WCN_ERR("wcn_op_read return fail\n");
		break;

	case IOCTL_WCN_OP_WRITE:
		ret = wcn_op_write(&wcn_op_attr, ptr);
		break;

	default:
		WCN_ERR("%s invalid command\n", __func__);
		break;
	}

	kfree(ptr);

	return ret;
}

static const struct file_operations wcn_op_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl	= wcn_op_ioctl,
	.open  = wcn_op_open,
	.release = wcn_op_release,
};

static struct miscdevice wcn_op_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = WCN_OP_NAME,
	.fops = &wcn_op_fops,
};

int wcn_op_init(void)
{
	int ret;

	WCN_INFO("%s\n", __func__);
	ret = misc_register(&wcn_op_device);
	if (ret) {
		WCN_ERR("wcn operation dev add failed!!!\n");
		return ret;
	}

	return 0;
}

void wcn_op_exit(void)
{
	misc_deregister(&wcn_op_device);
}
