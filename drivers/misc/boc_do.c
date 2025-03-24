#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#include <linux/of_gpio.h>

#define DO_IOCTL_SET _IOW('D', 1, int)

struct boc_do_dev {
    struct miscdevice misc;
    int gpio_num;
    int value;
};

static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct boc_do_dev *do_dev = dev_get_drvdata(dev->parent);

    return sprintf(buf, "%d\n", do_dev->value);
}

static ssize_t value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    int val = -1;
    struct boc_do_dev *do_dev = dev_get_drvdata(dev->parent);

    if (kstrtoint(buf, 10, &val) < 0) {
        return -EINVAL;
    }

    if (val < 0 || val > 1)
        return -EINVAL;
    
    gpio_set_value(do_dev->gpio_num, val);
    do_dev->value = val;

    return count;
}
static DEVICE_ATTR_RW(value);

static struct attribute *do_attrs[] = {
    &dev_attr_value.attr,
    NULL
};

static const struct attribute_group do_group = {
    .attrs = do_attrs,
};

static const struct attribute_group *do_groups[] = {
    &do_group,
    NULL
};

static long do_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    int val;
	struct boc_do_dev *do_dev =
		container_of(filp->private_data, struct boc_do_dev, misc);

    if (copy_from_user(&val, (int __user *)arg, sizeof(int)))
        return -EFAULT;
    
    if (val < 0 || val > 1)
        return -EINVAL;

    switch (cmd) {
        case DO_IOCTL_SET:
            gpio_set_value(do_dev->gpio_num, val ? 1 : 0);
            do_dev->value = val;
            break;
        default:
            return -ENOTTY;
    }
    return 0;
}

static struct file_operations do_fops = {
    .unlocked_ioctl = do_ioctl,
    .owner = THIS_MODULE,
};

static int boc_do_probe(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    struct device_node *np = dev->of_node;
    struct boc_do_dev *do_dev;
    int gpio_num = -1;
    
    dev_info(dev,"doing probe\n");

    gpio_num = of_get_named_gpio(np, "do-gpios", 0);
    if (!gpio_is_valid(gpio_num)) {
        dev_err(dev, "Invalid GPIO\n");
        return -ENODEV;
    }

    if (gpio_request(gpio_num, "do-gpio")) {
        dev_err(dev, "GPIO %d busy\n", do_dev->gpio_num);
        return -EBUSY;
    }
    gpio_direction_output(gpio_num, 0);
    
    do_dev = kzalloc(sizeof(*do_dev), GFP_KERNEL);
    if (!do_dev) {
        dev_err(dev,"alloc do dev failed\n");
        return -ENOMEM;
    }

    do_dev->value = 0;
    do_dev->gpio_num = gpio_num;
    do_dev->misc.minor = MISC_DYNAMIC_MINOR;
    do_dev->misc.name = "do";
    do_dev->misc.fops = &do_fops;
    do_dev->misc.groups = do_groups;
    do_dev->misc.parent = dev;

    dev_set_drvdata(dev, do_dev);

    misc_register(&do_dev->misc);

    dev_info(dev, "boc do probe success\n");
    pr_err("%s,%d:boc do probe success\n",__func__,__LINE__);
    return 0;
}

static int boc_do_remove(struct platform_device *pdev)
{
    struct boc_do_dev *do_dev = dev_get_drvdata(&pdev->dev);

    gpio_free(do_dev->gpio_num);

    misc_deregister(&do_dev->misc);

    return 0;
}

static const struct of_device_id boc_do_of_match[] = {
    { .compatible = "boc,do" },
    {}
};
MODULE_DEVICE_TABLE(of, boc_do_of_match);

static struct platform_driver do_driver = {
    .driver = {
        .name = "boc_do",
        .of_match_table = boc_do_of_match,
    },
    .probe = boc_do_probe,
    .remove = boc_do_remove,
};
module_platform_driver(do_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kerwin boc.tech");
