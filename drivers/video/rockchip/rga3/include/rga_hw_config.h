/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 *
 * Author: Huang Lee <Putin.li@rock-chips.com>
 */

#ifndef __LINUX_RGA_HW_CONFIG_H_
#define __LINUX_RGA_HW_CONFIG_H_

#include "rga_drv.h"

enum rga_mmu {
	RGA_NONE_MMU	= 0,
	RGA_MMU		= 1,
	RGA_IOMMU	= 2,
};

enum rga_hw_support_format_index {
	RGA_RASTER_INDEX,
	RGA_AFBC16x16_INDEX,
	RGA_TILE8x8_INDEX,
	RGA_TILE4x4_INDEX,
	RGA_RKFBC64x4_INDEX,
	RGA_AFBC32x8_INDEX,
	RGA_FORMAT_INDEX_BUTT,
};

enum rga_hw_issue {
	RGA_HW_ISSUE_DIS_AUTO_RST,
};

struct rga_win_data {
	const char *name;
	const uint32_t *formats[RGA_FORMAT_INDEX_BUTT];
	uint32_t formats_count[RGA_FORMAT_INDEX_BUTT];

	uint32_t supported_rotations;
	uint32_t scale_up_mode;
	uint32_t scale_down_mode;
	uint32_t rd_mode;
};

struct rga_rect {
	int width;
	int height;
};

struct rga_rect_range {
	struct rga_rect min;
	struct rga_rect max;
};

struct rga_hw_data {
	uint32_t version;
	uint32_t feature;

	uint32_t csc_r2y_mode;
	uint32_t csc_y2r_mode;

	struct rga_rect_range input_range;
	struct rga_rect_range output_range;

	unsigned int max_upscale_factor;
	unsigned int max_downscale_factor;

	uint32_t byte_stride_align;
	uint32_t max_byte_stride;

	const struct rga_win_data *win;
	unsigned int win_size;

	enum rga_mmu mmu;
};

extern const struct rga_hw_data rga3_data;
extern const struct rga_hw_data rga2e_data;
extern const struct rga_hw_data rga2e_1106_data;
extern const struct rga_hw_data rga2e_iommu_data;
extern const struct rga_hw_data rga2p_iommu_data;
extern const struct rga_hw_data rga2p_lite_1103b_data;

#define rga_hw_has_issue(scheduler, issue) test_bit(issue, &((scheduler)->hw_issues_mask))
#define rga_hw_set_issue_mask(scheduler, issue) set_bit(issue, &((scheduler)->hw_issues_mask))

/* Returns false if in range, true otherwise */
static inline bool rga_hw_out_of_range(const struct rga_rect_range *range, int width, int height)
{
	return (width > range->max.width || height > range->max.height ||
		width < range->min.width || height < range->min.height);
}

#endif /* __LINUX_RGA_HW_CONFIG_H_ */
