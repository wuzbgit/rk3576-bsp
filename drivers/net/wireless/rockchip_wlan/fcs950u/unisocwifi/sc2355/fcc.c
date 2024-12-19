/*
 * Copyright (C) 2021 Spreadtrum Communications Inc.
 *
 * Authors	:
 * Baolei.yuan <Baolei.yuan@unisoc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "sprdwl.h"
#include "fcc.h"

static struct sprdwl_fcc_priv fcc_info;

static struct fcc_power_bo g_fcc_power_table[MAX_FCC_COUNTRY_NUM] = {
	{
		.country = "UY",
		.num = 4,
		.power_backoff = {
			/* subtype, channel, bw, {mode(2.4g : b,g,n,ac; 5g : a,n,ac), value} */
			{0, 1, 0, { {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7} } },
			{1, 2, 0, { {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7} } },
			{0, 1, 0, { {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7} } },
			{1, 4, 0, { {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7} } },
		},
	},
	{
		.country = "MX",
		.num = 4,
		.power_backoff = {
			/* subtype, channel, bw, {mode(2.4g : b,g,n,ac; 5g : a,n,ac), value} */
			{0, 5, 0, { {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7} } },
			{1, 6, 0, { {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7} } },
			{0, 7, 0, { {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7} } },
			{1, 8, 0, { {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7} } },

		},
	},
	{
		.country = "CN",
		.num = 8,
		.power_backoff = {
			/* subtype, channel, bw, {mode(2.4g : b,g,n,ac; 5g : a,n,ac), value} */
			{1,   1, 0, { {0, 127}, {1,  13}, {2,  12}, {3, 127}, {4, 127}, {5, 127}, {6, 127} } },
			{1,   2, 0, { {0, 127}, {1,  13}, {2,  12}, {3, 127}, {4, 127}, {5, 127}, {6, 127} } },
			{1,  10, 0, { {0, 127}, {1,  13}, {2,  12}, {3, 127}, {4, 127}, {5, 127}, {6, 127} } },
			{1,  11, 0, { {0, 127}, {1,  13}, {2,  12}, {3, 127}, {4, 127}, {5, 127}, {6, 127} } },
			{1,  36, 0, { {0, 127}, {1, 127}, {2, 127}, {3, 127}, {4, 127}, {5,  11}, {6,  10} } },
			{1,  64, 0, { {0, 127}, {1, 127}, {2, 127}, {3, 127}, {4, 127}, {5,  11}, {6,  10} } },
			{1, 100, 0, { {0, 127}, {1, 127}, {2, 127}, {3, 127}, {4, 127}, {5,  11}, {6,  10} } },
			{1, 140, 0, { {0, 127}, {1, 127}, {2, 127}, {3, 127}, {4, 127}, {5,  11}, {6,  10} } },
		},
	},
};

static int sprdwl_fcc_fresh_bo(struct sprdwl_priv *priv, u8 channel, u8 bw, bool flag)
{
	struct sprd_power_backoff *p_backoff;
	struct fcc_power_bo *current_power_bo;
	int index;

	mutex_lock(&fcc_info.lock);
	fcc_info.flag = flag;
	if (flag) {
		fcc_info.channel = channel;
		fcc_info.bw = bw;
	}
	current_power_bo = fcc_info.cur_power_bo;
	mutex_unlock(&fcc_info.lock);

	if (!current_power_bo) {
		wl_info("current_power_bo is NULL, reset default!\n");
		p_backoff = NULL;
	} else {
		for (index = 0; index < current_power_bo->num; index++) {
			p_backoff = &current_power_bo->power_backoff[index];
			if (channel == p_backoff->channel &&
			    bw == p_backoff->bw) {
				wl_info("match channel : %hhu bw : %hhu\n",
					channel, bw);
				break;
			}
		}

		if (index == current_power_bo->num) {
			wl_info("do not match channel %hhu bw %hhu, reset default\n",
				channel, bw);
			p_backoff = NULL;
		}
	}

	atomic_set(&priv->power_back_off, 1);
	sprdwl_set_power_backoff(priv, 0, p_backoff);
	atomic_set(&priv->power_back_off, 0);
	return 0;
}

void sprdwl_fcc_fresh_bo_work(struct sprdwl_priv *priv, void *data, u16 len)
{
	struct fresh_bo_info *info = (struct fresh_bo_info *)data;
	sprdwl_fcc_fresh_bo(priv, info->pw_channel, info->pw_bw, true);
}

void sprdwl_fcc_match_country(struct sprdwl_priv *priv, const char *alpha2)
{
	bool found_country = false;
	bool need_refresh = false;
	struct fcc_power_bo *last_power_bo;
	int i, channel = 0, bw = 0;

	mutex_lock(&fcc_info.lock);
	for (i = 0; i < MAX_FCC_COUNTRY_NUM; i++) {
		if (g_fcc_power_table[i].country[0] == alpha2[0] &&
			g_fcc_power_table[i].country[1] == alpha2[1]) {
			wl_info("matched fcc country %s!\n", alpha2);
			found_country = true;
			last_power_bo = fcc_info.cur_power_bo;
			fcc_info.cur_power_bo = &g_fcc_power_table[i];
			/* handle alpha2 change after connected */
			if (last_power_bo && last_power_bo != fcc_info.cur_power_bo)
				fcc_info.flag = true;
			/* handle set regdom just after connected */
			if (fcc_info.flag) {
				need_refresh = true;
				channel = fcc_info.channel;
				bw = fcc_info.bw;
				wl_info("evt_fresh_backoff had came, now fresh it!\n");
			}
			break;
		}
	}

	if (!found_country) {
		wl_info("not fcc country, need reset fcc power\n");
		fcc_info.cur_power_bo = NULL;
	}
	mutex_unlock(&fcc_info.lock);

	if (!found_country || need_refresh) {
		sprdwl_fcc_fresh_bo(priv, channel, bw, false);
	}
}

void sprdwl_fcc_reset_bo(void)
{
	mutex_lock(&fcc_info.lock);
	fcc_info.flag = false;
	fcc_info.channel = 0;
	fcc_info.bw = 0;
	mutex_unlock(&fcc_info.lock);
}

void sprdwl_fcc_init(void)
{
	fcc_info.flag = false;
	mutex_init(&fcc_info.lock);
}

void sprdwl_fcc_deinit(void)
{
}
