/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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
 *
 */
#include <linux/bug.h>
#include <linux/delay.h>
#ifdef CONFIG_SC2342_INTEG
#include "gnss.h"
#endif
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include "marlin_platform.h"
#include "wcn_bus.h"

#include "../wcn_gnss.h"
#include "../../include/wcn_glb_reg.h"
#include "gnss_common.h"
#include "gnss_dump.h"
#include "wcn_glb.h"

#define GNSSCOMM_INFO(format, arg...) pr_info("gnss_ctl: " format, ## arg)
#define GNSSCOMM_ERR(format, arg...) pr_err("gnss_ctl: " format, ## arg)

#define GNSS_DATA_BASE_TYPE_H  16
#define GNSS_MAX_STRING_LEN	10
/* gnss mem dump success return value is 3 */
#define GNSS_DUMP_DATA_SUCCESS	3
#define FIRMWARE_FILEPATHNAME_LENGTH_MAX 256

struct gnss_common_ctl {
	struct device *dev;
	unsigned long chip_ver;
	unsigned int gnss_status;
	unsigned int gnss_subsys;
	char firmware_path[FIRMWARE_FILEPATHNAME_LENGTH_MAX];
};

static struct gnss_common_ctl gnss_common_ctl_dev;

enum gnss_status_e {
	GNSS_STATUS_POWEROFF = 0,
	GNSS_STATUS_POWERON,
	GNSS_STATUS_ASSERT,
	GNSS_STATUS_POWEROFF_GOING,
	GNSS_STATUS_POWERON_GOING,
	GNSS_STATUS_MAX,
};
#ifdef CONFIG_SC2342_INTEG
enum gnss_cp_status_subtype {
	GNSS_CP_STATUS_CALI = 1,
	GNSS_CP_STATUS_INIT = 2,
	GNSS_CP_STATUS_INIT_DONE = 3,
	GNSS_CP_STATUS_IDLEOFF = 4,
	GNSS_CP_STATUS_IDLEON = 5,
	GNSS_CP_STATUS_SLEEP = 6,
	GNSS_CP_STATUS__MAX,
};

struct completion gnss_dump_complete;
#endif
static unsigned int gnssver = 0x22;
static const struct of_device_id gnss_common_ctl_of_match[] = {
	{.compatible = "sprd,gnss_common_ctl", .data = (void *)&gnssver},
	{},
};

#ifndef CONFIG_SC2342_INTEG
struct gnss_cali {
	bool cali_done;
	u32 *cali_data;
};
static struct gnss_cali gnss_cali_data;
static u32 *gnss_efuse_data;


#ifdef GNSSDEBUG
static void gnss_cali_done_isr(void)
{
	complete(&marlin_dev->gnss_cali_done);
	GNSSCOMM_INFO("gnss cali done");
}
#endif
static int gnss_data_init(void)
{
	gnss_cali_data.cali_done = false;

	gnss_cali_data.cali_data = kzalloc(GNSS_CALI_DATA_SIZE, GFP_KERNEL);
	if (gnss_cali_data.cali_data == NULL) {
		GNSSCOMM_ERR("%s malloc fail.\n", __func__);
		return -ENOMEM;
	}

	#ifdef GNSSDEBUG
	init_completion(&marlin_dev.gnss_cali_done);
	sdio_pub_int_RegCb(GNSS_CALI_DONE, (PUB_INT_ISR)gnss_cali_done_isr);
	#endif
	gnss_efuse_data = kzalloc(GNSS_EFUSE_DATA_SIZE, GFP_KERNEL);
	if (gnss_efuse_data == NULL) {
		GNSSCOMM_ERR("%s malloc efuse data fail.\n", __func__);
		return -ENOMEM;
	}

	return 0;
}

static int gnss_write_cali_data(void)
{
	GNSSCOMM_INFO("gnss write calidata, flag %d\n",
			gnss_cali_data.cali_done);
	if (gnss_cali_data.cali_done) {
		sprdwcn_bus_direct_write(GNSS_CALI_ADDRESS,
			gnss_cali_data.cali_data, GNSS_CALI_DATA_SIZE);
	}
	return 0;
}

static int gnss_write_efuse_data(void)
{
	GNSSCOMM_INFO("%s flag %d\n", __func__,	gnss_cali_data.cali_done);
	if (gnss_cali_data.cali_done && (gnss_efuse_data != NULL))
		sprdwcn_bus_direct_write(GNSS_EFUSE_ADDRESS, gnss_efuse_data,
					 GNSS_EFUSE_DATA_SIZE);

	return 0;
}

int gnss_write_data(void)
{
	int ret = 0;

	gnss_write_cali_data();
	ret = gnss_write_efuse_data();

	return ret;
}

static int gnss_backup_cali(void)
{
	int i = 15;
	int tempvalue = 0;

	if (!gnss_cali_data.cali_done) {
		GNSSCOMM_INFO("%s begin\n", __func__);
		if (gnss_cali_data.cali_data != NULL) {
			while (i--) {
				sprdwcn_bus_direct_read(GNSS_CALI_ADDRESS,
					gnss_cali_data.cali_data,
					GNSS_CALI_DATA_SIZE);
				tempvalue = *(gnss_cali_data.cali_data);
				GNSSCOMM_ERR(" cali %d time, value is 0x%x\n",
							i, tempvalue);
				if (tempvalue != GNSS_CALI_DONE_FLAG) {
					msleep(100);
					continue;
				}
				GNSSCOMM_INFO(" cali success\n");
				gnss_cali_data.cali_done = true;
				break;
			}
		}
	} else
		GNSSCOMM_INFO(" no need back again\n");

	return 0;
}

static int gnss_backup_efuse(void)
{
	int ret = 1;

	/* efuse data is ok when cali done */
	if (gnss_cali_data.cali_done && (gnss_efuse_data != NULL)) {
		sprdwcn_bus_direct_read(GNSS_EFUSE_ADDRESS, gnss_efuse_data,
					GNSS_EFUSE_DATA_SIZE);
		ret = 0;
		GNSSCOMM_ERR("%s 0x%x\n", __func__, *gnss_efuse_data);
	} else
		GNSSCOMM_INFO("%s no need back again\n", __func__);

	return ret;
}

int gnss_backup_data(void)
{
	int ret;

	gnss_backup_cali();
	ret = gnss_backup_efuse();

	return ret;
}

static int gnss_boot_wait(void)
{
	int ret = -1;
	int i = 125;
	u32 *buffer = NULL;

	buffer = kzalloc(GNSS_BOOTSTATUS_SIZE, GFP_KERNEL);
	if (buffer == NULL) {
		GNSSCOMM_ERR("%s, malloc fail\n", __func__);
		return -1;
	}
	while (i--) {
		sprdwcn_bus_direct_read(GNSS_BOOTSTATUS_ADDRESS, buffer,
					GNSS_BOOTSTATUS_SIZE);
		GNSSCOMM_ERR("boot read %d time, value is 0x%x\n", i, *buffer);
		if (*buffer != GNSS_BOOTSTATUS_MAGIC) {
			msleep(20);
			continue;
		}
		ret = 0;
		GNSSCOMM_INFO("boot read success\n");
		break;
	}
	kfree(buffer);

	return ret;
}

#ifdef CONFIG_UMW2652
static int gnss_tsen_enable(int type)
{
	struct platform_device *pdev_regmap;
	struct device_node *regmap_np;
	static struct regmap *regmap;
	static unsigned int base;
	int ret;
	unsigned int value, temp;

	if (base == 0) {
		regmap_np = of_find_compatible_node(NULL, NULL,
						    "sprd,sc27xx-syscon");
		if (!regmap_np) {
			GNSSCOMM_ERR("%s, error np\n", __func__);
			return -EINVAL;
		}
		pdev_regmap = of_find_device_by_node(regmap_np);
		if (!pdev_regmap) {
			GNSSCOMM_ERR("%s, error regmap\n", __func__);
			return -EINVAL;
		}
		regmap = dev_get_regmap(pdev_regmap->dev.parent, NULL);
		ret = of_property_read_u32_index(regmap_np, "reg", 0, &base);
		if (ret) {
			GNSSCOMM_ERR("%s, error base\n", __func__);
			return -EINVAL;
		}
	}
	GNSSCOMM_ERR("%s, base 0x%x\n", __func__, base);
	regmap_read(regmap, (REGS_ANA_APB_BASE + XTL_WAIT_CTRL0), &value);
	GNSSCOMM_ERR("%s, XTL_WAIT_CTRL0 value read 0x%x\n", __func__, value);
	temp = value | BIT_XTL_EN;
	regmap_write(regmap, (REGS_ANA_APB_BASE + XTL_WAIT_CTRL0), temp);
	regmap_read(regmap, (REGS_ANA_APB_BASE + XTL_WAIT_CTRL0), &value);
	GNSSCOMM_ERR("%s, 2nd read 0x%x\n", __func__, value);

	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL0), &value);
	GNSSCOMM_ERR("%s, TSEN_CTRL0 value read 0x%x\n", __func__, value);
	temp = value | BIT_TSEN_CLK_SRC_SEL | BIT_TSEN_ADCLDO_EN;
	regmap_write(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL0), temp);
	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL0), &value);
	GNSSCOMM_ERR("%s, 2nd read 0x%x\n", __func__, value);

	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL1), &value);
	GNSSCOMM_ERR("%s, TSEN_CTRL1 value read 0x%x\n", __func__, value);
	temp = value | BIT_TSEN_SDADC_EN | BIT_TSEN_CLK_EN | BIT_TSEN_UGBUF_EN;
	regmap_write(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL1), temp);
	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL1), &value);
	GNSSCOMM_ERR("%s, 2nd read 0x%x\n", __func__, value);

	regmap_read(regmap, (SC2730_PIN_REG_BASE + PTEST0), &value);
	GNSSCOMM_ERR("%s, PTEST0 value read 0x%x\n", __func__, value);
	temp = (value & (~PTEST0_MASK)) | PTEST0_sel(2);
	regmap_write(regmap, (SC2730_PIN_REG_BASE + PTEST0), temp);
	regmap_read(regmap, (SC2730_PIN_REG_BASE + PTEST0), &value);
	GNSSCOMM_ERR("%s, 2nd read 0x%x\n", __func__, value);

	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL3), &value);
	GNSSCOMM_ERR("%s, TSEN_CTRL3 value read 0x%x\n", __func__, value);
	temp = value | BIT_TSEN_EN;
	temp = (temp & (~BIT_TSEN_TIME_SEL_MASK)) | BIT_TSEN_TIME_sel(2);
	regmap_write(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL3), temp);
	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL3), &value);
	GNSSCOMM_ERR("%s, 2nd read 0x%x\n", __func__, value);

	if (type == TSEN_EXT)
		regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL4), &value);
	else
		regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL5), &value);
	GNSSCOMM_ERR("%s, 0x%x read 0x%x\n", __func__, TSEN_CTRL4, value);

	return 0;
}

static int gnss_tsen_disable(int type)
{
	struct platform_device *pdev_regmap;
	struct device_node *regmap_np;
	static struct regmap *regmap;
	static unsigned int base;
	int ret;
	unsigned int value, temp;

	if (base == 0) {
		regmap_np = of_find_compatible_node(NULL, NULL,
						    "sprd,sc27xx-syscon");
		if (!regmap_np) {
			GNSSCOMM_ERR("%s, error np\n", __func__);
			return -EINVAL;
		}
		pdev_regmap = of_find_device_by_node(regmap_np);
		if (!pdev_regmap) {
			GNSSCOMM_ERR("%s, error regmap\n", __func__);
			return -EINVAL;
		}
		regmap = dev_get_regmap(pdev_regmap->dev.parent, NULL);
		ret = of_property_read_u32_index(regmap_np, "reg", 0, &base);
		if (ret) {
			GNSSCOMM_ERR("%s, error base\n", __func__);
			return -EINVAL;
		}
	}
	GNSSCOMM_ERR("%s, base 0x%x\n", __func__, base);

	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL0), &value);
	GNSSCOMM_ERR("%s, TSEN_CTRL0 value read 0x%x\n", __func__, value);
	temp = BIT_TSEN_CLK_SRC_SEL | BIT_TSEN_ADCLDO_EN;
	temp = value & (~temp);
	regmap_write(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL0), temp);
	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL0), &value);
	GNSSCOMM_ERR("%s, 2nd read 0x%x\n", __func__, value);

	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL1), &value);
	GNSSCOMM_ERR("%s, TSEN_CTRL1 value read 0x%x\n", __func__, value);
	temp = BIT_TSEN_SDADC_EN | BIT_TSEN_CLK_EN | BIT_TSEN_UGBUF_EN;
	temp = value & (~temp);
	regmap_write(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL1), temp);
	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL1), &value);
	GNSSCOMM_ERR("%s, 2nd read 0x%x\n", __func__, value);

	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL3), &value);
	GNSSCOMM_ERR("%s, TSEN_CTRL3 value read 0x%x\n", __func__, value);
	temp = value & ~BIT_TSEN_EN;
	regmap_write(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL3), temp);
	regmap_read(regmap, (REGS_ANA_APB_BASE + TSEN_CTRL3), &value);
	GNSSCOMM_ERR("%s, 2nd read 0x%x\n", __func__, value);

	return 0;
}
#endif
#endif

static void gnss_power_on(bool enable)
{
	int ret;

	GNSSCOMM_INFO("%s:enable=%d,current gnss_status=%d\n", __func__,
			enable, gnss_common_ctl_dev.gnss_status);
	if (enable && gnss_common_ctl_dev.gnss_status == GNSS_STATUS_POWEROFF) {
		gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWERON_GOING;
#ifdef CONFIG_UMW2652
		if (wcn_get_xtal_26m_clk_type() == WCN_CLOCK_TYPE_TSX)
			gnss_tsen_enable(TSEN_EXT);
#endif
		ret = start_marlin(gnss_common_ctl_dev.gnss_subsys);
		if (ret != 0)
			GNSSCOMM_INFO("%s: start marlin failed ret=%d\n",
					__func__, ret);
		else
			gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWERON;
	} else if (!enable && gnss_common_ctl_dev.gnss_status
			== GNSS_STATUS_POWERON) {
		gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWEROFF_GOING;
#ifdef CONFIG_UMW2652
		if (wcn_get_xtal_26m_clk_type() == WCN_CLOCK_TYPE_TSX)
			gnss_tsen_disable(TSEN_EXT);
#endif
		ret = stop_marlin(gnss_common_ctl_dev.gnss_subsys);
		if (ret != 0)
			GNSSCOMM_INFO("%s: stop marlin failed ret=%d\n",
				 __func__, ret);
		else
			gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWEROFF;
	} else {
		GNSSCOMM_INFO("%s: status is not match\n", __func__);
	}
}

static ssize_t gnss_power_enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;

	if (kstrtoul(buf, GNSS_MAX_STRING_LEN, &set_value)) {
		GNSSCOMM_ERR("%s, Maybe store string is too long\n", __func__);
		return -EINVAL;
	}
	GNSSCOMM_INFO("%s,%lu\n", __func__, set_value);
	if (set_value == 1)
		gnss_power_on(1);
	else if (set_value == 0)
		gnss_power_on(0);
	else {
		count = -EINVAL;
		GNSSCOMM_INFO("%s,unknown control\n", __func__);
	}

	return count;
}

static DEVICE_ATTR_WO(gnss_power_enable);

static ssize_t gnss_subsys_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;

	if (kstrtoul(buf, GNSS_MAX_STRING_LEN, &set_value))
		return -EINVAL;

	GNSSCOMM_INFO("%s,%lu\n", __func__, set_value);
#ifndef CONFIG_SC2342_INTEG
	gnss_common_ctl_dev.gnss_subsys = MARLIN_GNSS;
#else
	if (set_value == WCN_GNSS)
		gnss_common_ctl_dev.gnss_subsys = WCN_GNSS;
	else if (set_value == WCN_GNSS_BD)
		gnss_common_ctl_dev.gnss_subsys  = WCN_GNSS_BD;
	else if (set_value == WCN_GNSS_GAL)
		gnss_common_ctl_dev.gnss_subsys  = WCN_GNSS_GAL;
	else
		count = -EINVAL;
#endif
	return count;
}

void gnss_file_path_set(char *buf)
{
	strcpy(&gnss_common_ctl_dev.firmware_path[0], buf);
}

static ssize_t gnss_subsys_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;

	GNSSCOMM_INFO("%s\n", __func__);
	if (gnss_common_ctl_dev.gnss_status == GNSS_STATUS_POWERON) {
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d:%s\n",
				gnss_common_ctl_dev.gnss_subsys,
				&gnss_common_ctl_dev.firmware_path[0]);
	} else {
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				gnss_common_ctl_dev.gnss_subsys);
	}

	return i;
}

static DEVICE_ATTR_RW(gnss_subsys);

#ifdef CONFIG_SC2342_INTEG
static int gnss_status_get(void)
{
	phys_addr_t phy_addr;
	u32 magic_value = 0;

	phy_addr = wcn_get_gnss_base_addr() + GNSS_STATUS_OFFSET;
	wcn_read_data_from_phy_addr(phy_addr, &magic_value, sizeof(u32));
	GNSSCOMM_INFO("[%s] magic_value=%d\n", __func__, magic_value);

	return magic_value;
}

void gnss_dump_mem_ctrl_co(void)
{
	char flag = 0; /* 0: default, all, 1: only data, pmu, aon */
	unsigned int temp_status = 0;
	static char dump_flag;

	GNSSCOMM_INFO("[%s], flag is %d\n", __func__, dump_flag);
	if (dump_flag == 1)
		return;
	dump_flag = 1;
	temp_status = gnss_common_ctl_dev.gnss_status;
	if ((temp_status == GNSS_STATUS_POWERON_GOING) ||
		((temp_status == GNSS_STATUS_POWERON) &&
		(gnss_status_get() != GNSS_CP_STATUS_SLEEP))) {
		flag = (temp_status == GNSS_STATUS_POWERON) ? 0 : 1;
		gnss_dump_mem(flag);
		gnss_common_ctl_dev.gnss_status = GNSS_STATUS_ASSERT;
	}
	complete(&gnss_dump_complete);
}
#else
static int gnss_dump_mem_ctrl(void)
{
	int ret = -1;
	static char dump_flag;

	GNSSCOMM_INFO("[%s], flag is %d\n", __func__, dump_flag);
	if (dump_flag == 1)
		return 0;
	dump_flag = 1;
	if (gnss_common_ctl_dev.gnss_status == GNSS_STATUS_POWERON) {
		ret = gnss_dump_mem(0);
		gnss_common_ctl_dev.gnss_status = GNSS_STATUS_ASSERT;
	}

	return ret;
}
#endif
static ssize_t gnss_dump_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_value;
	int ret = -1;
	int temp = 0, strlen = 0;
	char triggerStr[64];

	set_value = buf[0] - '0';
	GNSSCOMM_INFO("%s, set_value: %lu\n", __func__, set_value);

	memset(triggerStr, 0, 64);
	strlen = ((count - 2) > 63) ? 63 : (count - 2);
	memcpy(triggerStr, &buf[2], strlen);
	GNSSCOMM_INFO("%s, triggerStr: %s\n", __func__, triggerStr);

	wcn_assert_interface(WCN_SOURCE_GNSS, triggerStr);

	if (set_value == 1) {
#ifdef CONFIG_SC2342_INTEG
		temp = wait_for_completion_timeout(&gnss_dump_complete,
						   msecs_to_jiffies(6000));
		GNSSCOMM_INFO("%s exit %d\n", __func__,
			      jiffies_to_msecs(temp));
		if (temp > 0)
			ret = GNSS_DUMP_DATA_SUCCESS;
		else
			gnss_dump_mem_ctrl_co();
#else
		temp = gnss_dump_mem_ctrl();
		GNSSCOMM_INFO("%s exit temp %d\n", __func__, temp);
		if (temp == 0)
			ret = GNSS_DUMP_DATA_SUCCESS;
#endif
	} else
		count = -EINVAL;

	return ret;
}

static DEVICE_ATTR_WO(gnss_dump);

static ssize_t gnss_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;
	unsigned int status = gnss_common_ctl_dev.gnss_status;

	GNSSCOMM_INFO("%s: %d\n", __func__, status);
	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", status);

	return i;
}
static DEVICE_ATTR_RO(gnss_status);

#ifdef CONFIG_UMW2652
static ssize_t gnss_clktype_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i = 0;
	enum wcn_clock_type clktype = WCN_CLOCK_TYPE_UNKNOWN;

	clktype = wcn_get_xtal_26m_clk_type();
	GNSSCOMM_INFO("%s: %d\n", __func__, clktype);
	i = scnprintf(buf, PAGE_SIZE, "%d\n", clktype);

	return i;
}
static DEVICE_ATTR_RO(gnss_clktype);
#endif

#ifndef CONFIG_SC2342_INTEG
static uint gnss_op_reg;
static uint gnss_indirect_reg_offset;
static ssize_t gnss_regr_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int i = 0;
	unsigned int op_reg = gnss_op_reg;
	unsigned int buffer;

	GNSSCOMM_INFO("%s, register is 0x%x\n", __func__, gnss_op_reg);
	if (op_reg == GNSS_INDIRECT_OP_REG) {
		int set_value;

		set_value = gnss_indirect_reg_offset + 0x80000000;
		sprdwcn_bus_reg_write(op_reg, &set_value, 4);
	}
	sprdwcn_bus_reg_read(op_reg, &buffer, 4);
	GNSSCOMM_INFO("%s,temp value is 0x%x\n", __func__, buffer);

	i += scnprintf((char *)buf + i, PAGE_SIZE - i, "show: 0x%x\n", buffer);

	return i;
}
static DEVICE_ATTR_RO(gnss_regr);

static ssize_t gnss_regaddr_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long set_addr;

	if (kstrtoul(buf, GNSS_DATA_BASE_TYPE_H, &set_addr)) {
		GNSSCOMM_ERR("%s, input error\n", __func__);
		return -EINVAL;
	}
	GNSSCOMM_INFO("%s,0x%lx\n", __func__, set_addr);
	gnss_op_reg = (uint)set_addr;

	return count;
}
static DEVICE_ATTR_WO(gnss_regaddr);

static ssize_t gnss_regspaddr_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned long set_addr;

	if (kstrtoul(buf, GNSS_DATA_BASE_TYPE_H, &set_addr)) {
		GNSSCOMM_ERR("%s, input error\n", __func__);
		return -EINVAL;
	}
	GNSSCOMM_INFO("%s,0x%lx\n", __func__, set_addr);
	gnss_op_reg = GNSS_INDIRECT_OP_REG;
	gnss_indirect_reg_offset = (uint)set_addr;
	return count;
}
static DEVICE_ATTR_WO(gnss_regspaddr);

static ssize_t gnss_regw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long set_value;
	unsigned int op_reg = gnss_op_reg;

	if (kstrtoul(buf, GNSS_DATA_BASE_TYPE_H, &set_value)) {
		GNSSCOMM_ERR("%s, input error\n", __func__);
		return -EINVAL;
	}
	if (op_reg == GNSS_INDIRECT_OP_REG)
		set_value = gnss_indirect_reg_offset + set_value;
	GNSSCOMM_INFO("%s,0x%lx\n", __func__, set_value);
	sprdwcn_bus_reg_write(op_reg, &set_value, 4);

	return count;
}
static DEVICE_ATTR_WO(gnss_regw);
#endif

bool gnss_delay_ctl(void)
{
	return (gnss_common_ctl_dev.gnss_status == GNSS_STATUS_POWERON);
}

static struct attribute *gnss_common_ctl_attrs[] = {
	&dev_attr_gnss_power_enable.attr,
	&dev_attr_gnss_dump.attr,
	&dev_attr_gnss_status.attr,
	&dev_attr_gnss_subsys.attr,
#ifdef CONFIG_UMW2652
	&dev_attr_gnss_clktype.attr,
#endif
#ifndef CONFIG_SC2342_INTEG
	&dev_attr_gnss_regr.attr,
	&dev_attr_gnss_regaddr.attr,
	&dev_attr_gnss_regspaddr.attr,
	&dev_attr_gnss_regw.attr,
#endif
	NULL,
};

static struct attribute_group gnss_common_ctl_group = {
	.name = NULL,
	.attrs = gnss_common_ctl_attrs,
};

static struct miscdevice gnss_common_ctl_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gnss_common_ctl",
	.fops = NULL,
};

#ifndef CONFIG_SC2342_INTEG
static struct sprdwcn_gnss_ops gnss_common_ctl_ops = {
	.backup_data = gnss_backup_data,
	.write_data = gnss_write_data,
	.set_file_path = gnss_file_path_set,
	.wait_gnss_boot = gnss_boot_wait
};
#endif

static int gnss_common_ctl_probe(struct platform_device *pdev)
{
	int ret;
	const struct of_device_id *of_id;

	GNSSCOMM_INFO("%s enter", __func__);
	gnss_common_ctl_dev.dev = &pdev->dev;

	gnss_common_ctl_dev.gnss_status = GNSS_STATUS_POWEROFF;
#ifndef CONFIG_SC2342_INTEG
	gnss_common_ctl_dev.gnss_subsys = MARLIN_GNSS;
	gnss_data_init();
#else
	gnss_common_ctl_dev.gnss_subsys = WCN_GNSS;
#endif
	/* considering backward compatibility, it's not use now  start */
	of_id = of_match_node(gnss_common_ctl_of_match,
		pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev,
			"get gnss_common_ctl of device id failed!\n");
		return -ENODEV;
	}
	gnss_common_ctl_dev.chip_ver = (unsigned long)(of_id->data);
	/* considering backward compatibility, it's not use now  end */

	platform_set_drvdata(pdev, &gnss_common_ctl_dev);
	ret = misc_register(&gnss_common_ctl_miscdev);
	if (ret) {
		GNSSCOMM_ERR("%s failed to register gnss_common_ctl.\n",
			__func__);
		return ret;
	}

	ret = sysfs_create_group(&gnss_common_ctl_miscdev.this_device->kobj,
			&gnss_common_ctl_group);
	if (ret) {
		GNSSCOMM_ERR("%s failed to create device attributes.\n",
			__func__);
		goto err_attr_failed;
	}

#ifdef CONFIG_SC2342_INTEG
	/* register dump callback func for mdbg */
	mdbg_dump_gnss_register(gnss_dump_mem_ctrl_co, NULL);
	init_completion(&gnss_dump_complete);
#else
	wcn_gnss_ops_register(&gnss_common_ctl_ops);
#endif

	return 0;

err_attr_failed:
	misc_deregister(&gnss_common_ctl_miscdev);
	return ret;
}

static int gnss_common_ctl_remove(struct platform_device *pdev)
{
#ifndef CONFIG_SC2342_INTEG
	wcn_gnss_ops_unregister();
#endif
	sysfs_remove_group(&gnss_common_ctl_miscdev.this_device->kobj,
				&gnss_common_ctl_group);

	misc_deregister(&gnss_common_ctl_miscdev);
	return 0;
}
static struct platform_driver gnss_common_ctl_drv = {
	.driver = {
		   .name = "gnss_common_ctl",
		   .of_match_table = of_match_ptr(gnss_common_ctl_of_match),
		   },
	.probe = gnss_common_ctl_probe,
	.remove = gnss_common_ctl_remove
};

int gnss_common_ctl_init(void)
{
	return platform_driver_register(&gnss_common_ctl_drv);
}

void gnss_common_ctl_exit(void)
{
	platform_driver_unregister(&gnss_common_ctl_drv);
}

#if 0
module_init(gnss_common_ctl_init);
module_exit(gnss_common_ctl_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum Gnss Driver");
MODULE_AUTHOR("Jun.an<jun.an@spreadtrum.com>");
#else
EXPORT_SYMBOL(gnss_common_ctl_init);
EXPORT_SYMBOL(gnss_common_ctl_exit);
#endif
