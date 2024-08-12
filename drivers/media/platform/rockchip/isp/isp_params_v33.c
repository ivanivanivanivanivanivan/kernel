// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2024 Rockchip Electronics Co., Ltd. */

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include "dev.h"
#include "regs.h"
#include "isp_params_v33.h"

#define ISP33_MODULE_EN				BIT(0)
#define ISP33_SELF_FORCE_UPD			BIT(31)

static inline void
isp3_param_write_direct(struct rkisp_isp_params_vdev *params_vdev,
			u32 value, u32 addr)
{
	rkisp_write(params_vdev->dev, addr, value, true);
}

static inline void
isp3_param_write(struct rkisp_isp_params_vdev *params_vdev,
		 u32 value, u32 addr, u32 id)
{
	rkisp_idx_write(params_vdev->dev, addr, value, id, false);
}

static inline u32
isp3_param_read_direct(struct rkisp_isp_params_vdev *params_vdev, u32 addr)
{
	return rkisp_read(params_vdev->dev, addr, true);
}

static inline u32
isp3_param_read(struct rkisp_isp_params_vdev *params_vdev, u32 addr, u32 id)
{
	return rkisp_idx_read(params_vdev->dev, addr, id, false);
}

static inline u32
isp3_param_read_cache(struct rkisp_isp_params_vdev *params_vdev, u32 addr, u32 id)
{
	return rkisp_idx_read_reg_cache(params_vdev->dev, addr, id);
}

static inline void
isp3_param_set_bits(struct rkisp_isp_params_vdev *params_vdev,
		    u32 reg, u32 bit_mask, u32 id)
{
	rkisp_idx_set_bits(params_vdev->dev, reg, 0, bit_mask, id, false);
}

static inline void
isp3_param_clear_bits(struct rkisp_isp_params_vdev *params_vdev,
		      u32 reg, u32 bit_mask, u32 id)
{
	rkisp_idx_clear_bits(params_vdev->dev, reg, bit_mask, id, false);
}

static void
isp_dpcc_config(struct rkisp_isp_params_vdev *params_vdev,
		const struct isp39_dpcc_cfg *arg, u32 id)
{
	u32 value;
	int i;

	value = isp3_param_read(params_vdev, ISP3X_DPCC0_MODE, id);
	value &= ISP_DPCC_EN;

	value |= !!arg->stage1_enable << 2 |
		 !!arg->grayscale_mode << 1;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_MODE, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_MODE, id);

	value = !!arg->border_bypass_mode << 8 |
		(arg->sw_rk_out_sel & 0x03) << 5 |
		!!arg->sw_dpcc_output_sel << 4 |
		!!arg->stage1_rb_3x3 << 3 |
		!!arg->stage1_g_3x3 << 2 |
		!!arg->stage1_incl_rb_center << 1 |
		!!arg->stage1_incl_green_center;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_OUTPUT_MODE, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_OUTPUT_MODE, id);

	value = !!arg->stage1_use_fix_set << 3 |
		!!arg->stage1_use_set_3 << 2 |
		!!arg->stage1_use_set_2 << 1 |
		!!arg->stage1_use_set_1;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_SET_USE, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_SET_USE, id);

	value = !!arg->sw_rk_red_blue1_en << 13 |
		!!arg->rg_red_blue1_enable << 12 |
		!!arg->rnd_red_blue1_enable << 11 |
		!!arg->ro_red_blue1_enable << 10 |
		!!arg->lc_red_blue1_enable << 9 |
		!!arg->pg_red_blue1_enable << 8 |
		!!arg->sw_rk_green1_en << 5 |
		!!arg->rg_green1_enable << 4 |
		!!arg->rnd_green1_enable << 3 |
		!!arg->ro_green1_enable << 2 |
		!!arg->lc_green1_enable << 1 |
		!!arg->pg_green1_enable;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_METHODS_SET_1, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_METHODS_SET_1, id);

	value = !!arg->sw_rk_red_blue2_en << 13 |
		!!arg->rg_red_blue2_enable << 12 |
		!!arg->rnd_red_blue2_enable << 11 |
		!!arg->ro_red_blue2_enable << 10 |
		!!arg->lc_red_blue2_enable << 9 |
		!!arg->pg_red_blue2_enable << 8 |
		!!arg->sw_rk_green2_en << 5 |
		!!arg->rg_green2_enable << 4 |
		!!arg->rnd_green2_enable << 3 |
		!!arg->ro_green2_enable << 2 |
		!!arg->lc_green2_enable << 1 |
		!!arg->pg_green2_enable;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_METHODS_SET_2, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_METHODS_SET_2, id);

	value = !!arg->sw_rk_red_blue3_en << 13 |
		!!arg->rg_red_blue3_enable << 12 |
		!!arg->rnd_red_blue3_enable << 11 |
		!!arg->ro_red_blue3_enable << 10 |
		!!arg->lc_red_blue3_enable << 9 |
		!!arg->pg_red_blue3_enable << 8 |
		!!arg->sw_rk_green3_en << 5 |
		!!arg->rg_green3_enable << 4 |
		!!arg->rnd_green3_enable << 3 |
		!!arg->ro_green3_enable << 2 |
		!!arg->lc_green3_enable << 1 |
		!!arg->pg_green3_enable;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_METHODS_SET_3, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_METHODS_SET_3, id);

	value = ISP_PACK_4BYTE(arg->line_thr_1_g, arg->line_thr_1_rb,
				arg->sw_mindis1_g, arg->sw_mindis1_rb);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_THRESH_1, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_THRESH_1, id);

	value = ISP_PACK_4BYTE(arg->line_mad_fac_1_g, arg->line_mad_fac_1_rb,
				arg->sw_dis_scale_max1, arg->sw_dis_scale_min1);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_MAD_FAC_1, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_MAD_FAC_1, id);

	value = ISP_PACK_4BYTE(arg->pg_fac_1_g, arg->pg_fac_1_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PG_FAC_1, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PG_FAC_1, id);

	value = ISP_PACK_4BYTE(arg->rnd_thr_1_g, arg->rnd_thr_1_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RND_THRESH_1, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RND_THRESH_1, id);

	value = ISP_PACK_4BYTE(arg->rg_fac_1_g, arg->rg_fac_1_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RG_FAC_1, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RG_FAC_1, id);

	value = ISP_PACK_4BYTE(arg->line_thr_2_g, arg->line_thr_2_rb,
				arg->sw_mindis2_g, arg->sw_mindis2_rb);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_THRESH_2, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_THRESH_2, id);

	value = ISP_PACK_4BYTE(arg->line_mad_fac_2_g, arg->line_mad_fac_2_rb,
				arg->sw_dis_scale_max2, arg->sw_dis_scale_min2);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_MAD_FAC_2, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_MAD_FAC_2, id);

	value = ISP_PACK_4BYTE(arg->pg_fac_2_g, arg->pg_fac_2_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PG_FAC_2, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PG_FAC_2, id);

	value = ISP_PACK_4BYTE(arg->rnd_thr_2_g, arg->rnd_thr_2_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RND_THRESH_2, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RND_THRESH_2, id);

	value = ISP_PACK_4BYTE(arg->rg_fac_2_g, arg->rg_fac_2_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RG_FAC_2, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RG_FAC_2, id);

	value = ISP_PACK_4BYTE(arg->line_thr_3_g, arg->line_thr_3_rb,
				 arg->sw_mindis3_g, arg->sw_mindis3_rb);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_THRESH_3, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_THRESH_3, id);

	value = ISP_PACK_4BYTE(arg->line_mad_fac_3_g, arg->line_mad_fac_3_rb,
				arg->sw_dis_scale_max3, arg->sw_dis_scale_min3);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_MAD_FAC_3, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_MAD_FAC_3, id);

	value = ISP_PACK_4BYTE(arg->pg_fac_3_g, arg->pg_fac_3_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PG_FAC_3, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PG_FAC_3, id);

	value = ISP_PACK_4BYTE(arg->rnd_thr_3_g, arg->rnd_thr_3_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RND_THRESH_3, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RND_THRESH_3, id);

	value = ISP_PACK_4BYTE(arg->rg_fac_3_g, arg->rg_fac_3_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RG_FAC_3, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RG_FAC_3, id);

	value = (arg->ro_lim_3_rb & 0x03) << 10 |
		(arg->ro_lim_3_g & 0x03) << 8 |
		(arg->ro_lim_2_rb & 0x03) << 6 |
		(arg->ro_lim_2_g & 0x03) << 4 |
		(arg->ro_lim_1_rb & 0x03) << 2 |
		(arg->ro_lim_1_g & 0x03);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RO_LIMITS, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RO_LIMITS, id);

	value = (arg->rnd_offs_3_rb & 0x03) << 10 |
		(arg->rnd_offs_3_g & 0x03) << 8 |
		(arg->rnd_offs_2_rb & 0x03) << 6 |
		(arg->rnd_offs_2_g & 0x03) << 4 |
		(arg->rnd_offs_1_rb & 0x03) << 2 |
		(arg->rnd_offs_1_g & 0x03);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RND_OFFS, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RND_OFFS, id);

	value = !!arg->bpt_rb_3x3 << 11 |
		!!arg->bpt_g_3x3 << 10 |
		!!arg->bpt_incl_rb_center << 9 |
		!!arg->bpt_incl_green_center << 8 |
		!!arg->bpt_use_fix_set << 7 |
		!!arg->bpt_use_set_3 << 6 |
		!!arg->bpt_use_set_2 << 5 |
		!!arg->bpt_use_set_1 << 4 |
		!!arg->bpt_cor_en << 1 |
		!!arg->bpt_det_en;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_BPT_CTRL, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_BPT_CTRL, id);

	isp3_param_write(params_vdev, arg->bp_number, ISP3X_DPCC0_BPT_NUMBER, id);
	isp3_param_write(params_vdev, arg->bp_number, ISP3X_DPCC1_BPT_NUMBER, id);
	isp3_param_write(params_vdev, arg->bp_table_addr, ISP3X_DPCC0_BPT_ADDR, id);
	isp3_param_write(params_vdev, arg->bp_table_addr, ISP3X_DPCC1_BPT_ADDR, id);

	value = ISP_PACK_2SHORT(arg->bpt_h_addr, arg->bpt_v_addr);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_BPT_DATA, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_BPT_DATA, id);

	isp3_param_write(params_vdev, arg->bp_cnt, ISP3X_DPCC0_BP_CNT, id);
	isp3_param_write(params_vdev, arg->bp_cnt, ISP3X_DPCC1_BP_CNT, id);

	isp3_param_write(params_vdev, arg->sw_pdaf_en, ISP3X_DPCC0_PDAF_EN, id);
	isp3_param_write(params_vdev, arg->sw_pdaf_en, ISP3X_DPCC1_PDAF_EN, id);

	value = 0;
	for (i = 0; i < ISP32_DPCC_PDAF_POINT_NUM; i++)
		value |= !!arg->pdaf_point_en[i] << i;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PDAF_POINT_EN, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PDAF_POINT_EN, id);

	value = ISP_PACK_2SHORT(arg->pdaf_offsetx, arg->pdaf_offsety);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PDAF_OFFSET, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PDAF_OFFSET, id);

	value = ISP_PACK_2SHORT(arg->pdaf_wrapx, arg->pdaf_wrapy);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PDAF_WRAP, id);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PDAF_WRAP, id);

	value = ISP_PACK_2SHORT(arg->pdaf_wrapx_num, arg->pdaf_wrapy_num);
	isp3_param_write(params_vdev, value, ISP_DPCC0_PDAF_SCOPE, id);
	isp3_param_write(params_vdev, value, ISP_DPCC1_PDAF_SCOPE, id);

	for (i = 0; i < ISP32_DPCC_PDAF_POINT_NUM / 2; i++) {
		value = ISP_PACK_4BYTE(arg->point[2 * i].x, arg->point[2 * i].y,
					arg->point[2 * i + 1].x, arg->point[2 * i + 1].y);
		isp3_param_write(params_vdev, value, ISP3X_DPCC0_PDAF_POINT_0 + 4 * i, id);
		isp3_param_write(params_vdev, value, ISP3X_DPCC1_PDAF_POINT_0 + 4 * i, id);
	}

	isp3_param_write(params_vdev, arg->pdaf_forward_med, ISP3X_DPCC0_PDAF_FORWARD_MED, id);
	isp3_param_write(params_vdev, arg->pdaf_forward_med, ISP3X_DPCC1_PDAF_FORWARD_MED, id);
}

static void
isp_dpcc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_DPCC0_MODE, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_DPCC0_MODE, id);
	isp3_param_write(params_vdev, val, ISP3X_DPCC1_MODE, id);
}

static void
isp_bls_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp32_bls_cfg *arg, u32 id)
{
	const struct isp2x_bls_fixed_val *pval;
	u32 new_control, value;

	new_control = isp3_param_read(params_vdev, ISP3X_BLS_CTRL, id);
	new_control &= (ISP_BLS_ENA | ISP32_BLS_BLS2_EN);

	pval = &arg->bls1_val;
	if (arg->bls1_en)
		new_control |= ISP_BLS_BLS1_EN;
	switch (params_vdev->raw_type) {
	case RAW_BGGR:
		isp3_param_write(params_vdev, pval->r, ISP3X_BLS1_D_FIXED, id);
		isp3_param_write(params_vdev, pval->gr, ISP3X_BLS1_C_FIXED, id);
		isp3_param_write(params_vdev, pval->gb, ISP3X_BLS1_B_FIXED, id);
		isp3_param_write(params_vdev, pval->b, ISP3X_BLS1_A_FIXED, id);
		break;
	case RAW_GBRG:
		isp3_param_write(params_vdev, pval->r, ISP3X_BLS1_C_FIXED, id);
		isp3_param_write(params_vdev, pval->gr, ISP3X_BLS1_D_FIXED, id);
		isp3_param_write(params_vdev, pval->gb, ISP3X_BLS1_A_FIXED, id);
		isp3_param_write(params_vdev, pval->b, ISP3X_BLS1_B_FIXED, id);
		break;
	case RAW_GRBG:
		isp3_param_write(params_vdev, pval->r, ISP3X_BLS1_B_FIXED, id);
		isp3_param_write(params_vdev, pval->gr, ISP3X_BLS1_A_FIXED, id);
		isp3_param_write(params_vdev, pval->gb, ISP3X_BLS1_D_FIXED, id);
		isp3_param_write(params_vdev, pval->b, ISP3X_BLS1_C_FIXED, id);
		break;
	case RAW_RGGB:
	default:
		isp3_param_write(params_vdev, pval->r, ISP3X_BLS1_A_FIXED, id);
		isp3_param_write(params_vdev, pval->gr, ISP3X_BLS1_B_FIXED, id);
		isp3_param_write(params_vdev, pval->gb, ISP3X_BLS1_C_FIXED, id);
		isp3_param_write(params_vdev, pval->b, ISP3X_BLS1_D_FIXED, id);
		break;
	}

	/* fixed subtraction values */
	pval = &arg->fixed_val;
	if (!arg->enable_auto) {
		switch (params_vdev->raw_type) {
		case RAW_BGGR:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS_D_FIXED, id);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS_C_FIXED, id);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS_B_FIXED, id);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS_A_FIXED, id);
			break;
		case RAW_GBRG:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS_C_FIXED, id);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS_D_FIXED, id);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS_A_FIXED, id);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS_B_FIXED, id);
			break;
		case RAW_GRBG:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS_B_FIXED, id);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS_A_FIXED, id);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS_D_FIXED, id);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS_C_FIXED, id);
			break;
		case RAW_RGGB:
		default:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS_A_FIXED, id);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS_B_FIXED, id);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS_C_FIXED, id);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS_D_FIXED, id);
			break;
		}
	} else {
		if (arg->en_windows & BIT(1)) {
			value = arg->bls_window2.h_offs;
			isp3_param_write(params_vdev, value, ISP3X_BLS_H2_START, id);
			value = arg->bls_window2.h_offs + arg->bls_window2.h_size;
			isp3_param_write(params_vdev, value, ISP3X_BLS_H2_STOP, id);
			value = arg->bls_window2.v_offs;
			isp3_param_write(params_vdev, value, ISP3X_BLS_V2_START, id);
			value = arg->bls_window2.v_offs + arg->bls_window2.v_size;
			isp3_param_write(params_vdev, value, ISP3X_BLS_V2_STOP, id);
			new_control |= ISP_BLS_WINDOW_2;
		}

		if (arg->en_windows & BIT(0)) {
			value = arg->bls_window1.h_offs;
			isp3_param_write(params_vdev, value, ISP3X_BLS_H1_START, id);
			value = arg->bls_window1.h_offs + arg->bls_window1.h_size;
			isp3_param_write(params_vdev, value, ISP3X_BLS_H1_STOP, id);
			value = arg->bls_window1.v_offs;
			isp3_param_write(params_vdev, value, ISP3X_BLS_V1_START, id);
			value = arg->bls_window1.v_offs + arg->bls_window1.v_size;
			isp3_param_write(params_vdev, value, ISP3X_BLS_V1_STOP, id);
			new_control |= ISP_BLS_WINDOW_1;
		}

		isp3_param_write(params_vdev, arg->bls_samples, ISP3X_BLS_SAMPLES, id);

		new_control |= ISP_BLS_MODE_MEASURED;
	}
	isp3_param_write(params_vdev, new_control, ISP3X_BLS_CTRL, id);

	isp3_param_write(params_vdev, arg->isp_ob_offset, ISP32_BLS_ISP_OB_OFFSET, id);
	isp3_param_write(params_vdev, arg->isp_ob_predgain, ISP32_BLS_ISP_OB_PREDGAIN, id);
	isp3_param_write(params_vdev, arg->isp_ob_max, ISP32_BLS_ISP_OB_MAX, id);
}

static void
isp_bls_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_BLS_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_BLS_CTRL, id);
}

static void
isp_lsc_matrix_cfg_sram(struct rkisp_isp_params_vdev *params_vdev,
			const struct isp3x_lsc_cfg *pconfig,
			bool is_check, u32 id)
{
	u32 data = isp3_param_read(params_vdev, ISP3X_LSC_CTRL, id);
	int i, j;

	if (is_check && !(data & ISP33_MODULE_EN))
		return;

	isp3_param_write_direct(params_vdev, 0, ISP3X_LSC_R_TABLE_ADDR);
	isp3_param_write_direct(params_vdev, 0, ISP3X_LSC_GR_TABLE_ADDR);
	isp3_param_write_direct(params_vdev, 0, ISP3X_LSC_GB_TABLE_ADDR);
	isp3_param_write_direct(params_vdev, 0, ISP3X_LSC_B_TABLE_ADDR);

	/* program data tables (table size is 9 * 17 = 153) */
	for (i = 0; i < CIF_ISP_LSC_SECTORS_MAX * CIF_ISP_LSC_SECTORS_MAX;
	     i += CIF_ISP_LSC_SECTORS_MAX) {
		/*
		 * 17 sectors with 2 values in one DWORD = 9
		 * DWORDs (2nd value of last DWORD unused)
		 */
		for (j = 0; j < CIF_ISP_LSC_SECTORS_MAX - 1; j += 2) {
			data = ISP_ISP_LSC_TABLE_DATA(pconfig->r_data_tbl[i + j],
						      pconfig->r_data_tbl[i + j + 1]);
			isp3_param_write_direct(params_vdev, data, ISP3X_LSC_R_TABLE_DATA);

			data = ISP_ISP_LSC_TABLE_DATA(pconfig->gr_data_tbl[i + j],
						      pconfig->gr_data_tbl[i + j + 1]);
			isp3_param_write_direct(params_vdev, data, ISP3X_LSC_GR_TABLE_DATA);

			data = ISP_ISP_LSC_TABLE_DATA(pconfig->gb_data_tbl[i + j],
						      pconfig->gb_data_tbl[i + j + 1]);
			isp3_param_write_direct(params_vdev, data, ISP3X_LSC_GB_TABLE_DATA);

			data = ISP_ISP_LSC_TABLE_DATA(pconfig->b_data_tbl[i + j],
						      pconfig->b_data_tbl[i + j + 1]);
			isp3_param_write_direct(params_vdev, data, ISP3X_LSC_B_TABLE_DATA);
		}

		data = ISP_ISP_LSC_TABLE_DATA(pconfig->r_data_tbl[i + j], 0);
		isp3_param_write_direct(params_vdev, data, ISP3X_LSC_R_TABLE_DATA);

		data = ISP_ISP_LSC_TABLE_DATA(pconfig->gr_data_tbl[i + j], 0);
		isp3_param_write_direct(params_vdev, data, ISP3X_LSC_GR_TABLE_DATA);

		data = ISP_ISP_LSC_TABLE_DATA(pconfig->gb_data_tbl[i + j], 0);
		isp3_param_write_direct(params_vdev, data, ISP3X_LSC_GB_TABLE_DATA);

		data = ISP_ISP_LSC_TABLE_DATA(pconfig->b_data_tbl[i + j], 0);
		isp3_param_write_direct(params_vdev, data, ISP3X_LSC_B_TABLE_DATA);
	}
}

static void
isp_lsc_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp3x_lsc_cfg *arg, u32 id)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct isp33_isp_params_cfg *params_rec = params_vdev->isp33_params + id;
	u32 data, ctrl;
	int i;

	for (i = 0; i < ISP33_LSC_SIZE_TBL_SIZE / 4; i++) {
		/* program x size tables */
		data = CIF_ISP_LSC_SECT_SIZE(arg->x_size_tbl[i * 2],
					     arg->x_size_tbl[i * 2 + 1]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_XSIZE_01 + i * 4, id);
		data = CIF_ISP_LSC_SECT_SIZE(arg->x_size_tbl[i * 2 + 8],
					     arg->x_size_tbl[i * 2 + 9]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_XSIZE_89 + i * 4, id);

		/* program x grad tables */
		data = CIF_ISP_LSC_SECT_SIZE(arg->x_grad_tbl[i * 2],
					     arg->x_grad_tbl[i * 2 + 1]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_XGRAD_01 + i * 4, id);
		data = CIF_ISP_LSC_SECT_SIZE(arg->x_grad_tbl[i * 2 + 8],
					     arg->x_grad_tbl[i * 2 + 9]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_XGRAD_89 + i * 4, id);

		/* program y size tables */
		data = CIF_ISP_LSC_SECT_SIZE(arg->y_size_tbl[i * 2],
					     arg->y_size_tbl[i * 2 + 1]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_YSIZE_01 + i * 4, id);
		data = CIF_ISP_LSC_SECT_SIZE(arg->y_size_tbl[i * 2 + 8],
					     arg->y_size_tbl[i * 2 + 9]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_YSIZE_89 + i * 4, id);

		/* program y grad tables */
		data = CIF_ISP_LSC_SECT_SIZE(arg->y_grad_tbl[i * 2],
					     arg->y_grad_tbl[i * 2 + 1]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_YGRAD_01 + i * 4, id);
		data = CIF_ISP_LSC_SECT_SIZE(arg->y_grad_tbl[i * 2 + 8],
					     arg->y_grad_tbl[i * 2 + 9]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_YGRAD_89 + i * 4, id);
	}

	ctrl = isp3_param_read(params_vdev, ISP3X_LSC_CTRL, id);
	ctrl &= ISP33_MODULE_EN;
	ctrl |= !!arg->sector_16x16 << 2;
	isp3_param_write(params_vdev, ctrl, ISP3X_LSC_CTRL, id);

	if (dev->hw_dev->is_single)
		isp_lsc_matrix_cfg_sram(params_vdev, arg, false, id);
	params_rec->others.lsc_cfg = *arg;
}

static void
isp_lsc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_LSC_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;

	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_LSC_CTRL, id);
}

static void
isp_debayer_config(struct rkisp_isp_params_vdev *params_vdev,
		   const struct isp33_debayer_cfg *arg, u32 id)
{
	u32 val;
	int i;

	val = isp3_param_read(params_vdev, ISP3X_DEBAYER_CONTROL, id);
	val &= ISP_DEBAYER_EN;

	val |= !!arg->g_out_flt_en << 4 | !!arg->bypass << 1;
	isp3_param_write(params_vdev, val, ISP3X_DEBAYER_CONTROL, id);

	val = (arg->luma_dx[0] & 0xf) | (arg->luma_dx[1] & 0xf) << 4 |
	      (arg->luma_dx[2] & 0xf) << 8 | (arg->luma_dx[3] & 0xf) << 12 |
	      (arg->luma_dx[4] & 0xf) << 16 | (arg->luma_dx[5] & 0xf) << 20 |
	      (arg->luma_dx[6] & 0xf) << 24;
	isp3_param_write(params_vdev, val, ISP33_DEBAYER_LUMA_DX, id);

	val = !!arg->g_interp_clip_en << 0 |
	      (arg->hi_texture_thred & 0xf) << 4 |
	      (arg->hi_drct_thred & 0xf) << 8 |
	      (arg->lo_drct_thred & 0xf) << 12 |
	      arg->drct_method_thred << 16 |
	      arg->g_interp_sharp_strg_max_limit << 24;
	isp3_param_write(params_vdev, val, ISP33_DEBAYER_G_INTERP, id);

	val = (arg->lo_drct_flt_coeff1 & 0x1f) |
	      (arg->lo_drct_flt_coeff2 & 0x1f) << 8 |
	      (arg->lo_drct_flt_coeff3 & 0x1f) << 16 |
	      (arg->lo_drct_flt_coeff4 & 0x1f) << 24;
	isp3_param_write(params_vdev, val, ISP33_DEBAYER_G_INTERP_FILTER1, id);

	val = (arg->hi_drct_flt_coeff1 & 0x1f) |
	      (arg->hi_drct_flt_coeff2 & 0x1f) << 8 |
	      (arg->hi_drct_flt_coeff3 & 0x1f) << 16 |
	      (arg->hi_drct_flt_coeff4 & 0x1f) << 24;
	isp3_param_write(params_vdev, val, ISP33_DEBAYER_G_INTERP_FILTER2, id);

	val = arg->g_interp_sharp_strg_offset | arg->grad_lo_flt_alpha << 16;
	isp3_param_write(params_vdev, val, ISP33_DEBAYER_G_INTERP_OFFSET_ALPHA, id);

	for (i = 0; i < ISP33_DEBAYER_DRCT_OFFSET_NUM / 2; i++) {
		val = ISP_PACK_2SHORT(arg->drct_offset[i * 2], arg->drct_offset[i * 2 + 1]);
		isp3_param_write(params_vdev, val, ISP33_DEBAYER_G_INTERP_DRCT_OFFSET0 + i * 4, id);
	}

	val = !!arg->gflt_mode << 0 | (arg->gflt_ratio & 0x7ff) << 4 |
	      arg->gflt_offset << 16;
	isp3_param_write(params_vdev, val, ISP33_DEBAYER_G_FILTER_MODE_OFFSET, id);

	val = ISP_PACK_4BYTE(arg->gflt_coe0, arg->gflt_coe1, arg->gflt_coe2, 0);
	isp3_param_write(params_vdev, val, ISP33_DEBAYER_G_FILTER_FILTER, id);

	for (i = 0; i < ISP33_DEBAYER_VSIGMA_NUM / 2; i++) {
		val = ISP_PACK_2SHORT(arg->gflt_vsigma[i * 2], arg->gflt_vsigma[i * 2 + 1]);
		isp3_param_write(params_vdev, val, ISP33_DEBAYER_G_FILTER_VSIGMA0 + i * 4, id);
	}
}

static void
isp_debayer_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_DEBAYER_CONTROL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_DEBAYER_CONTROL, id);
}

static void
isp_awbgain_config(struct rkisp_isp_params_vdev *params_vdev,
		   const struct isp32_awb_gain_cfg *arg, u32 id)
{
	struct rkisp_device *dev = params_vdev->dev;

	if (!arg->gain0_red || !arg->gain0_blue ||
	    !arg->gain1_red || !arg->gain1_blue ||
	    !arg->gain2_red || !arg->gain2_blue ||
	    !arg->gain0_green_r || !arg->gain0_green_b ||
	    !arg->gain1_green_r || !arg->gain1_green_b ||
	    !arg->gain2_green_r || !arg->gain2_green_b) {
		dev_err(dev->dev, "awb gain is zero!\n");
		return;
	}

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain0_green_b, arg->gain0_green_r),
			 ISP3X_ISP_AWB_GAIN0_G, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain0_blue, arg->gain0_red),
			 ISP3X_ISP_AWB_GAIN0_RB, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain1_green_b, arg->gain1_green_r),
			 ISP3X_ISP_AWB_GAIN1_G, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain1_blue, arg->gain1_red),
			 ISP3X_ISP_AWB_GAIN1_RB, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain2_green_b, arg->gain2_green_r),
			 ISP3X_ISP_AWB_GAIN2_G, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain2_blue, arg->gain2_red),
			 ISP3X_ISP_AWB_GAIN2_RB, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->awb1_gain_gb, arg->awb1_gain_gr),
			 ISP32_ISP_AWB1_GAIN_G, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->awb1_gain_b, arg->awb1_gain_r),
			 ISP32_ISP_AWB1_GAIN_RB, id);
}

static void
isp_awbgain_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP3X_ISP_CTRL0, id);

	if (en == !!(val & CIF_ISP_CTRL_ISP_AWB_ENA))
		return;
	if (en)
		val |= CIF_ISP_CTRL_ISP_AWB_ENA;
	else
		val &= CIF_ISP_CTRL_ISP_AWB_ENA;
	isp3_param_write(params_vdev, val, ISP3X_ISP_CTRL0, id);
}

static void
isp_ccm_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp33_ccm_cfg *arg, u32 id)
{
	u32 value;
	u32 i;

	value = isp3_param_read(params_vdev, ISP3X_CCM_CTRL, id);
	value &= ISP_CCM_EN;

	value |= !!arg->sat_decay_en << 4 |
		 !!arg->asym_adj_en << 3 |
		 !!arg->enh_adj_en << 2 |
		 !!arg->highy_adjust_dis << 1;
	isp3_param_write(params_vdev, value, ISP3X_CCM_CTRL, id);

	value = ISP_PACK_2SHORT(arg->coeff0_r, arg->coeff1_r);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF0_R, id);

	value = ISP_PACK_2SHORT(arg->coeff2_r, arg->offset_r);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF1_R, id);

	value = ISP_PACK_2SHORT(arg->coeff0_g, arg->coeff1_g);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF0_G, id);

	value = ISP_PACK_2SHORT(arg->coeff2_g, arg->offset_g);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF1_G, id);

	value = ISP_PACK_2SHORT(arg->coeff0_b, arg->coeff1_b);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF0_B, id);

	value = ISP_PACK_2SHORT(arg->coeff2_b, arg->offset_b);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF1_B, id);

	value = ISP_PACK_2SHORT(arg->coeff0_y, arg->coeff1_y);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF0_Y, id);

	value = ISP_PACK_2SHORT(arg->coeff2_y, 0);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF1_Y, id);

	for (i = 0; i < ISP33_CCM_CURVE_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->alp_y[2 * i], arg->alp_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_CCM_ALP_Y0 + 4 * i, id);
	}

	value = (arg->right_bit & 0xf) << 4 | (arg->bound_bit & 0xf);
	isp3_param_write(params_vdev, value, ISP3X_CCM_BOUND_BIT, id);

	value = ISP_PACK_2SHORT(arg->color_coef0_r2y, arg->color_coef1_g2y);
	isp3_param_write(params_vdev, value, ISP32_CCM_ENHANCE0, id);

	value = ISP_PACK_2SHORT(arg->color_coef2_b2y, arg->color_enh_rat_max);
	isp3_param_write(params_vdev, value, ISP32_CCM_ENHANCE1, id);

	value = arg->hf_low | arg->hf_up << 8 | arg->hf_scale << 16;
	isp3_param_write(params_vdev, value, ISP33_CCM_HF_THD, id);

	for (i = 0; i < ISP33_CCM_HF_FACTOR_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->hf_factor[i * 2],
					arg->hf_factor[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_CCM_HF_FACTOR0 + i * 4, id);
	}
	value = arg->hf_factor[i * 2];
	isp3_param_write(params_vdev, value, ISP33_CCM_HF_FACTOR8, id);
}

static void
isp_ccm_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_CCM_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_CCM_CTRL, id);
}

static void
isp_goc_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp3x_gammaout_cfg *arg, u32 id)
{
	int i;
	u32 value;

	value = isp3_param_read(params_vdev, ISP3X_GAMMA_OUT_CTRL, id);
	value &= ISP3X_GAMMA_OUT_EN;
	value |= !!arg->equ_segm << 1 | !!arg->finalx4_dense_en << 2;
	isp3_param_write(params_vdev, value, ISP3X_GAMMA_OUT_CTRL, id);

	isp3_param_write(params_vdev, arg->offset, ISP3X_GAMMA_OUT_OFFSET, id);
	for (i = 0; i < ISP32_GAMMA_OUT_MAX_SAMPLES / 2; i++) {
		value = ISP_PACK_2SHORT(arg->gamma_y[2 * i],
					arg->gamma_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_GAMMA_OUT_Y0 + i * 4, id);
	}
	isp3_param_write(params_vdev, arg->gamma_y[2 * i], ISP3X_GAMMA_OUT_Y0 + i * 4, id);
}

static void
isp_goc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_GAMMA_OUT_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_GAMMA_OUT_CTRL, id);
}

static void
isp_cproc_config(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp2x_cproc_cfg *arg, u32 id)
{
	u32 quantization = params_vdev->quantization;

	isp3_param_write(params_vdev, arg->contrast, ISP3X_CPROC_CONTRAST, id);
	isp3_param_write(params_vdev, arg->hue, ISP3X_CPROC_HUE, id);
	isp3_param_write(params_vdev, arg->sat, ISP3X_CPROC_SATURATION, id);
	isp3_param_write(params_vdev, arg->brightness, ISP3X_CPROC_BRIGHTNESS, id);

	if (quantization != V4L2_QUANTIZATION_FULL_RANGE) {
		isp3_param_clear_bits(params_vdev, ISP3X_CPROC_CTRL,
				      CIF_C_PROC_YOUT_FULL |
				      CIF_C_PROC_YIN_FULL |
				      CIF_C_PROC_COUT_FULL, id);
	} else {
		isp3_param_set_bits(params_vdev, ISP3X_CPROC_CTRL,
				    CIF_C_PROC_YOUT_FULL |
				    CIF_C_PROC_YIN_FULL |
				    CIF_C_PROC_COUT_FULL, id);
	}
}

static void
isp_cproc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_CPROC_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_CPROC_CTRL, id);
}

static void
isp_ie_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_IMG_EFF_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val =  ISP33_MODULE_EN;
	else
		val = 0;
	isp3_param_write(params_vdev, val, ISP3X_IMG_EFF_CTRL, id);
}

static void
isp_rawaebig_config(struct rkisp_isp_params_vdev *params_vdev,
		    const struct isp2x_rawaebig_meas_cfg *arg,
		    u32 addr, u32 id)
{
	struct rkisp_device *ispdev = params_vdev->dev;
	struct v4l2_rect *out_crop = &ispdev->isp_sdev.out_crop;
	u32 width = out_crop->width, height = out_crop->height;
	u32 i, value, h_size, v_size, h_offs, v_offs;
	u32 block_hsize, block_vsize, wnd_num_idx = 0;
	const u32 ae_wnd_num[] = {
		1, 5, 15, 15
	};

	/* avoid to override the old enable value */
	value = isp3_param_read(params_vdev, addr + ISP3X_RAWAE_BIG_CTRL, id);
	value &= ISP3X_RAWAE_BIG_EN;

	wnd_num_idx = arg->wnd_num;
	if (wnd_num_idx >= ARRAY_SIZE(ae_wnd_num)) {
		wnd_num_idx = ARRAY_SIZE(ae_wnd_num) - 1;
		dev_err(params_vdev->dev->dev,
			"%s invalid wnd_num:%d, set to %d\n",
			__func__, arg->wnd_num, wnd_num_idx);
	}
	value |= ISP3X_RAWAE_BIG_WND0_NUM(wnd_num_idx);

	if (arg->subwin_en[0])
		value |= ISP3X_RAWAE_BIG_WND1_EN;
	if (arg->subwin_en[1])
		value |= ISP3X_RAWAE_BIG_WND2_EN;
	if (arg->subwin_en[2])
		value |= ISP3X_RAWAE_BIG_WND3_EN;
	if (arg->subwin_en[3])
		value |= ISP3X_RAWAE_BIG_WND4_EN;
	isp3_param_write(params_vdev, value, addr + ISP3X_RAWAE_BIG_CTRL, id);

	h_offs = arg->win.h_offs & ~0x1;
	v_offs = arg->win.v_offs & ~0x1;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(h_offs, v_offs),
			 addr + ISP3X_RAWAE_BIG_OFFSET, id);

	if (ispdev->unite_div > ISP_UNITE_DIV1)
		width = width / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;
	if (ispdev->unite_div == ISP_UNITE_DIV4)
		height = height / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;

	h_size = arg->win.h_size;
	v_size = arg->win.v_size;
	if (!h_size || h_size + h_offs + 1 > width)
		h_size = width - h_offs - 1;
	if (!v_size || v_size + v_offs + 2 > height)
		v_size = height - v_offs - 2;
	block_hsize = (h_size / ae_wnd_num[wnd_num_idx]) & ~0x1;
	block_vsize = (v_size / ae_wnd_num[wnd_num_idx]) & ~0x1;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(block_hsize, block_vsize),
			 addr + ISP3X_RAWAE_BIG_BLK_SIZE, id);

	for (i = 0; i < ISP33_RAWAEBIG_SUBWIN_NUM; i++) {
		h_offs = arg->subwin[i].h_offs & ~0x1;
		v_offs = arg->subwin[i].v_offs & ~0x1;
		isp3_param_write(params_vdev, ISP_PACK_2SHORT(h_offs, v_offs),
				 addr + ISP3X_RAWAE_BIG_WND1_OFFSET + 8 * i, id);

		v_size = arg->subwin[i].v_size;
		h_size = arg->subwin[i].h_size;
		if (!h_size || h_size + h_offs > width)
			h_size = width - h_offs;
		if (!v_size || v_size + v_offs > height)
			v_size = height - v_offs;
		h_size = (h_size + h_offs) & ~0x1;
		v_size = (v_size + v_offs) & ~0x1;
		isp3_param_write(params_vdev, ISP_PACK_2SHORT(h_size, v_size),
				 addr + ISP3X_RAWAE_BIG_WND1_SIZE + 8 * i, id);
	}

	value = isp3_param_read(params_vdev, ISP3X_VI_ISP_PATH, id);
	if (addr == ISP3X_RAWAE_BIG1_BASE) {
		value &= ~(ISP3X_RAWAE3_SEL(3) | BIT(29));
		value |= ISP3X_RAWAE3_SEL(arg->rawae_sel & 0xf);
		if (arg->rawae_sel & ISP33_BNR2AEBIG_SEL_EN)
			value |= BIT(29);
		isp3_param_write(params_vdev, value, ISP3X_VI_ISP_PATH, id);
	} else {
		value &= ~(ISP3X_RAWAE012_SEL(3) | BIT(30));
		value |= ISP3X_RAWAE012_SEL(arg->rawae_sel & 0xf);
		if (arg->rawae_sel & ISP33_BNR2AE0_SEL_EN)
			value |= BIT(30);
		isp3_param_write(params_vdev, value, ISP3X_VI_ISP_PATH, id);
	}
}

static void
isp_rawaebig_enable(struct rkisp_isp_params_vdev *params_vdev,
		    bool en, u32 addr, u32 id)
{
	u32 val = isp3_param_read(params_vdev, addr + ISP3X_RAWAE_BIG_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, addr + ISP3X_RAWAE_BIG_CTRL, id);
}

static void
isp_rawae0_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp2x_rawaebig_meas_cfg *arg, u32 id)
{
	isp_rawaebig_config(params_vdev, arg, ISP3X_RAWAE_LITE_BASE, id);
}

static void
isp_rawae0_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	isp_rawaebig_enable(params_vdev, en, ISP3X_RAWAE_LITE_BASE, id);
}

static void
isp_rawae3_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp2x_rawaebig_meas_cfg *arg, u32 id)
{
	isp_rawaebig_config(params_vdev, arg, ISP3X_RAWAE_BIG1_BASE, id);
}

static void
isp_rawae3_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	isp_rawaebig_enable(params_vdev, en, ISP3X_RAWAE_BIG1_BASE, id);
}

static void
isp_rawawb_cfg_sram(struct rkisp_isp_params_vdev *params_vdev,
		    const struct isp33_rawawb_meas_cfg *arg,
		    bool is_check, u32 id)
{
	u32 i, val = isp3_param_read(params_vdev, ISP3X_RAWAWB_CTRL, id);

	if (is_check && !(val & ISP33_MODULE_EN))
		return;

	isp3_param_write_direct(params_vdev, 0, ISP3X_RAWAWB_WRAM_CTRL);
	for (i = 0; i < ISP33_RAWAWB_WEIGHT_NUM / 5; i++) {
		val = (arg->wp_blk_wei_w[5 * i] & 0x3f) |
		      (arg->wp_blk_wei_w[5 * i + 1] & 0x3f) << 6 |
		      (arg->wp_blk_wei_w[5 * i + 2] & 0x3f) << 12 |
		      (arg->wp_blk_wei_w[5 * i + 3] & 0x3f) << 18 |
		      (arg->wp_blk_wei_w[5 * i + 4] & 0x3f) << 24;
		isp3_param_write_direct(params_vdev, val, ISP3X_RAWAWB_WRAM_DATA_BASE);
	}
}

static void
isp_rawawb_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp33_rawawb_meas_cfg *arg, u32 id)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct v4l2_rect *out_crop = &dev->isp_sdev.out_crop;
	struct isp33_isp_params_cfg *params_rec = params_vdev->isp33_params + id;
	struct isp33_rawawb_meas_cfg *arg_rec = &params_rec->meas.rawawb;
	const struct isp2x_bls_fixed_val *pval = &arg->bls2_val;
	u32 width = out_crop->width, height = out_crop->height;
	u32 value, val, mask, h_size, v_size, h_offs, v_offs;

	value = isp3_param_read(params_vdev, ISP3X_BLS_CTRL, id);
	value &= ~ISP32_BLS_BLS2_EN;
	if (arg->bls2_en)
		value |= ISP32_BLS_BLS2_EN;
	switch (params_vdev->raw_type) {
	case RAW_BGGR:
		isp3_param_write(params_vdev, pval->r, ISP32_BLS2_D_FIXED, id);
		isp3_param_write(params_vdev, pval->gr, ISP32_BLS2_C_FIXED, id);
		isp3_param_write(params_vdev, pval->gb, ISP32_BLS2_B_FIXED, id);
		isp3_param_write(params_vdev, pval->b, ISP32_BLS2_A_FIXED, id);
		break;
	case RAW_GBRG:
		isp3_param_write(params_vdev, pval->r, ISP32_BLS2_C_FIXED, id);
		isp3_param_write(params_vdev, pval->gr, ISP32_BLS2_D_FIXED, id);
		isp3_param_write(params_vdev, pval->gb, ISP32_BLS2_A_FIXED, id);
		isp3_param_write(params_vdev, pval->b, ISP32_BLS2_B_FIXED, id);
		break;
	case RAW_GRBG:
		isp3_param_write(params_vdev, pval->r, ISP32_BLS2_B_FIXED, id);
		isp3_param_write(params_vdev, pval->gr, ISP32_BLS2_A_FIXED, id);
		isp3_param_write(params_vdev, pval->gb, ISP32_BLS2_D_FIXED, id);
		isp3_param_write(params_vdev, pval->b, ISP32_BLS2_C_FIXED, id);
		break;
	case RAW_RGGB:
	default:
		isp3_param_write(params_vdev, pval->r, ISP32_BLS2_A_FIXED, id);
		isp3_param_write(params_vdev, pval->gr, ISP32_BLS2_B_FIXED, id);
		isp3_param_write(params_vdev, pval->gb, ISP32_BLS2_C_FIXED, id);
		isp3_param_write(params_vdev, pval->b, ISP32_BLS2_D_FIXED, id);
	}
	isp3_param_write(params_vdev, value, ISP3X_BLS_CTRL, id);

	value = arg->in_overexposure_threshold << 16 |
		!!arg->ovexp_2ddr_dis << 9 |
		!!arg->blk_with_luma_wei_en << 8 |
		!!arg->ds16x8_mode_en << 7 |
		(arg->blk_measure_illu_idx & 0x7) << 4 |
		!!arg->blk_rtdw_measure_en << 3 |
		!!arg->blk_measure_xytype << 2 |
		!!arg->blk_measure_mode << 1 |
		!!arg->blk_measure_enable;
	isp3_param_write(params_vdev, value, ISP3X_RAWAWB_BLK_CTRL, id);

	h_offs = arg->h_offs & ~0x1;
	v_offs = arg->v_offs & ~0x1;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(h_offs, v_offs),
			 ISP3X_RAWAWB_WIN_OFFS, id);

	if (dev->unite_div > ISP_UNITE_DIV1)
		width = width / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;
	if (dev->unite_div == ISP_UNITE_DIV4)
		height = height / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;

	h_size = arg->h_size;
	v_size = arg->v_size;
	if (!h_size || h_size + h_offs > width)
		h_size = width - h_offs;
	if (!v_size || v_size + v_offs > height)
		v_size = height - v_offs;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(h_size, v_size),
			 ISP3X_RAWAWB_WIN_SIZE, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->r_max, arg->g_max),
			 ISP3X_RAWAWB_LIMIT_RG_MAX, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->b_max, arg->y_max),
			 ISP3X_RAWAWB_LIMIT_BY_MAX, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->r_min, arg->g_min),
			 ISP3X_RAWAWB_LIMIT_RG_MIN, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->b_min, arg->y_min),
			 ISP3X_RAWAWB_LIMIT_BY_MIN, id);

	value = !!arg->wp_hist_xytype << 4 |
		!!arg->wp_blk_wei_en1 << 3 |
		!!arg->wp_blk_wei_en0 << 2 |
		!!arg->wp_luma_wei_en1 << 1 |
		!!arg->wp_luma_wei_en0;
	isp3_param_write(params_vdev, value, ISP3X_RAWAWB_WEIGHT_CURVE_CTRL, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->wp_luma_weicurve_y0,
					arg->wp_luma_weicurve_y1,
					arg->wp_luma_weicurve_y2,
					arg->wp_luma_weicurve_y3),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_XCOOR03, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->wp_luma_weicurve_y4,
					arg->wp_luma_weicurve_y5,
					arg->wp_luma_weicurve_y6,
					arg->wp_luma_weicurve_y7),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_XCOOR47, id);

	isp3_param_write(params_vdev,
			 arg->wp_luma_weicurve_y8,
			 ISP3X_RAWAWB_YWEIGHT_CURVE_XCOOR8, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->wp_luma_weicurve_w0,
					arg->wp_luma_weicurve_w1,
					arg->wp_luma_weicurve_w2,
					arg->wp_luma_weicurve_w3),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_YCOOR03, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->wp_luma_weicurve_w4,
					arg->wp_luma_weicurve_w5,
					arg->wp_luma_weicurve_w6,
					arg->wp_luma_weicurve_w7),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_YCOOR47, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->wp_luma_weicurve_w8,
					 arg->pre_wbgain_inv_r),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_YCOOR8, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->pre_wbgain_inv_g,
					 arg->pre_wbgain_inv_b),
			 ISP3X_RAWAWB_PRE_WBGAIN_INV, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex0_u_0, arg->vertex0_v_0),
			 ISP3X_RAWAWB_UV_DETC_VERTEX0_0, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex1_u_0, arg->vertex1_v_0),
			 ISP3X_RAWAWB_UV_DETC_VERTEX1_0, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex2_u_0, arg->vertex2_v_0),
			 ISP3X_RAWAWB_UV_DETC_VERTEX2_0, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex3_u_0, arg->vertex3_v_0),
			 ISP3X_RAWAWB_UV_DETC_VERTEX3_0, id);

	isp3_param_write(params_vdev, arg->islope01_0,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE01_0, id);

	isp3_param_write(params_vdev, arg->islope12_0,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE12_0, id);

	isp3_param_write(params_vdev, arg->islope23_0,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE23_0, id);

	isp3_param_write(params_vdev, arg->islope30_0,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE30_0, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex0_u_1,
					 arg->vertex0_v_1),
			 ISP3X_RAWAWB_UV_DETC_VERTEX0_1, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex1_u_1,
					 arg->vertex1_v_1),
			 ISP3X_RAWAWB_UV_DETC_VERTEX1_1, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex2_u_1,
					 arg->vertex2_v_1),
			 ISP3X_RAWAWB_UV_DETC_VERTEX2_1, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex3_u_1,
					 arg->vertex3_v_1),
			 ISP3X_RAWAWB_UV_DETC_VERTEX3_1, id);

	isp3_param_write(params_vdev, arg->islope01_1,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE01_1, id);

	isp3_param_write(params_vdev, arg->islope12_1,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE12_1, id);

	isp3_param_write(params_vdev, arg->islope23_1,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE23_1, id);

	isp3_param_write(params_vdev, arg->islope30_1,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE30_1, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex0_u_2,
					 arg->vertex0_v_2),
			 ISP3X_RAWAWB_UV_DETC_VERTEX0_2, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex1_u_2,
					 arg->vertex1_v_2),
			 ISP3X_RAWAWB_UV_DETC_VERTEX1_2, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex2_u_2,
					 arg->vertex2_v_2),
			 ISP3X_RAWAWB_UV_DETC_VERTEX2_2, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex3_u_2,
					 arg->vertex3_v_2),
			 ISP3X_RAWAWB_UV_DETC_VERTEX3_2, id);

	isp3_param_write(params_vdev, arg->islope01_2,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE01_2, id);

	isp3_param_write(params_vdev, arg->islope12_2,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE12_2, id);

	isp3_param_write(params_vdev, arg->islope23_2,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE23_2, id);

	isp3_param_write(params_vdev, arg->islope30_2,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE30_2, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex0_u_3,
					 arg->vertex0_v_3),
			 ISP3X_RAWAWB_UV_DETC_VERTEX0_3, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex1_u_3,
					 arg->vertex1_v_3),
			 ISP3X_RAWAWB_UV_DETC_VERTEX1_3, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex2_u_3,
					 arg->vertex2_v_3),
			 ISP3X_RAWAWB_UV_DETC_VERTEX2_3, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex3_u_3,
					 arg->vertex3_v_3),
			 ISP3X_RAWAWB_UV_DETC_VERTEX3_3, id);

	isp3_param_write(params_vdev, arg->islope01_3,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE01_3, id);

	isp3_param_write(params_vdev, arg->islope12_3,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE12_3, id);

	isp3_param_write(params_vdev, arg->islope23_3,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE23_3, id);

	isp3_param_write(params_vdev, arg->islope30_3,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE30_3, id);

	value = ISP_PACK_2SHORT(arg->ccm_coeff0_r, arg->ccm_coeff1_r);
	isp3_param_write(params_vdev, value, ISP33_RAWAWB_CCM_COEFF0_R, id);
	value = arg->ccm_coeff2_r;
	isp3_param_write(params_vdev, value, ISP33_RAWAWB_CCM_COEFF1_R, id);
	value = ISP_PACK_2SHORT(arg->ccm_coeff0_g, arg->ccm_coeff1_g);
	isp3_param_write(params_vdev, value, ISP33_RAWAWB_CCM_COEFF0_G, id);
	value = arg->ccm_coeff2_g;
	isp3_param_write(params_vdev, value, ISP33_RAWAWB_CCM_COEFF1_G, id);
	value = ISP_PACK_2SHORT(arg->ccm_coeff0_b, arg->ccm_coeff1_b);
	isp3_param_write(params_vdev, value, ISP33_RAWAWB_CCM_COEFF0_B, id);
	value = arg->ccm_coeff2_b;
	isp3_param_write(params_vdev, value, ISP33_RAWAWB_CCM_COEFF1_B, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->wt0, arg->wt1),
			 ISP3X_RAWAWB_RGB2XY_WT01, id);

	isp3_param_write(params_vdev, arg->wt2,
			 ISP3X_RAWAWB_RGB2XY_WT2, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->mat0_x, arg->mat0_y),
			 ISP3X_RAWAWB_RGB2XY_MAT0_XY, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->mat1_x, arg->mat1_y),
			 ISP3X_RAWAWB_RGB2XY_MAT1_XY, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->mat2_x, arg->mat2_y),
			 ISP3X_RAWAWB_RGB2XY_MAT2_XY, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_x0_0, arg->nor_x1_0),
			 ISP3X_RAWAWB_XY_DETC_NOR_X_0, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_y0_0, arg->nor_y1_0),
			 ISP3X_RAWAWB_XY_DETC_NOR_Y_0, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_x0_0, arg->big_x1_0),
			 ISP3X_RAWAWB_XY_DETC_BIG_X_0, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_y0_0, arg->big_y1_0),
			 ISP3X_RAWAWB_XY_DETC_BIG_Y_0, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_x0_1, arg->nor_x1_1),
			 ISP3X_RAWAWB_XY_DETC_NOR_X_1, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_y0_1, arg->nor_y1_1),
			 ISP3X_RAWAWB_XY_DETC_NOR_Y_1, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_x0_1, arg->big_x1_1),
			 ISP3X_RAWAWB_XY_DETC_BIG_X_1, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_y0_1, arg->big_y1_1),
			 ISP3X_RAWAWB_XY_DETC_BIG_Y_1, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_x0_2, arg->nor_x1_2),
			 ISP3X_RAWAWB_XY_DETC_NOR_X_2, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_y0_2, arg->nor_y1_2),
			 ISP3X_RAWAWB_XY_DETC_NOR_Y_2, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_x0_2, arg->big_x1_2),
			 ISP3X_RAWAWB_XY_DETC_BIG_X_2, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_y0_2, arg->big_y1_2),
			 ISP3X_RAWAWB_XY_DETC_BIG_Y_2, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_x0_3, arg->nor_x1_3),
			 ISP3X_RAWAWB_XY_DETC_NOR_X_3, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_y0_3, arg->nor_y1_3),
			 ISP3X_RAWAWB_XY_DETC_NOR_Y_3, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_x0_3, arg->big_x1_3),
			 ISP3X_RAWAWB_XY_DETC_BIG_X_3, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_y0_3, arg->big_y1_3),
			 ISP3X_RAWAWB_XY_DETC_BIG_Y_3, id);

	value = (arg->exc_wp_region0_excen & 0x3) |
		!!arg->exc_wp_region0_measen << 2 |
		!!arg->exc_wp_region0_domain << 3 |
		(arg->exc_wp_region1_excen & 0x3) << 4 |
		!!arg->exc_wp_region1_measen << 6 |
		!!arg->exc_wp_region1_domain << 7 |
		(arg->exc_wp_region2_excen & 0x3) << 8 |
		!!arg->exc_wp_region2_measen << 10 |
		!!arg->exc_wp_region2_domain << 11 |
		(arg->exc_wp_region3_excen & 0x3) << 12 |
		!!arg->exc_wp_region3_measen << 14 |
		!!arg->exc_wp_region3_domain << 15 |
		(arg->exc_wp_region4_excen & 0x3) << 16 |
		!!arg->exc_wp_region4_domain << 19 |
		(arg->exc_wp_region5_excen & 0x3) << 20 |
		!!arg->exc_wp_region5_domain << 23 |
		(arg->exc_wp_region6_excen & 0x3) << 24 |
		!!arg->exc_wp_region6_domain << 27 |
		!!arg->multiwindow_en << 31;
	isp3_param_write(params_vdev, value, ISP3X_RAWAWB_MULTIWINDOW_EXC_CTRL, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow0_h_offs,
					 arg->multiwindow0_v_offs),
			 ISP3X_RAWAWB_MULTIWINDOW0_OFFS, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow0_h_size,
					 arg->multiwindow0_v_size),
			 ISP3X_RAWAWB_MULTIWINDOW0_SIZE, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow1_h_offs,
					 arg->multiwindow1_v_offs),
			 ISP3X_RAWAWB_MULTIWINDOW1_OFFS, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow1_h_size,
					 arg->multiwindow1_v_size),
			 ISP3X_RAWAWB_MULTIWINDOW1_SIZE, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow2_h_offs,
					 arg->multiwindow2_v_offs),
			 ISP3X_RAWAWB_MULTIWINDOW2_OFFS, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow2_h_size,
					 arg->multiwindow2_v_size),
			 ISP3X_RAWAWB_MULTIWINDOW2_SIZE, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow3_h_offs,
					 arg->multiwindow3_v_offs),
			 ISP3X_RAWAWB_MULTIWINDOW3_OFFS, id);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow3_h_size,
					 arg->multiwindow3_v_size),
			 ISP3X_RAWAWB_MULTIWINDOW3_SIZE, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region0_xu0,
					 arg->exc_wp_region0_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION0_XU, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region0_yv0,
					 arg->exc_wp_region0_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION0_YV, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region1_xu0,
					 arg->exc_wp_region1_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION1_XU, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region1_yv0,
					 arg->exc_wp_region1_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION1_YV, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region2_xu0,
					 arg->exc_wp_region2_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION2_XU, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region2_yv0,
					 arg->exc_wp_region2_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION2_YV, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region3_xu0,
					 arg->exc_wp_region3_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION3_XU, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region3_yv0,
					 arg->exc_wp_region3_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION3_YV, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region4_xu0,
					 arg->exc_wp_region4_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION4_XU, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region4_yv0,
					 arg->exc_wp_region4_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION4_YV, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region5_xu0,
					 arg->exc_wp_region5_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION5_XU, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region5_yv0,
					 arg->exc_wp_region5_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION5_YV, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region6_xu0,
					 arg->exc_wp_region6_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION6_XU, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region6_yv0,
					 arg->exc_wp_region6_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION6_YV, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->exc_wp_region0_weight,
					arg->exc_wp_region1_weight,
					arg->exc_wp_region2_weight,
					arg->exc_wp_region3_weight),
			 ISP32_RAWAWB_EXC_WP_WEIGHT0_3, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->exc_wp_region4_weight,
					arg->exc_wp_region5_weight,
					arg->exc_wp_region6_weight, 0),
			 ISP32_RAWAWB_EXC_WP_WEIGHT4_6, id);

	if (dev->hw_dev->is_single)
		isp_rawawb_cfg_sram(params_vdev, arg, false, id);
	memcpy(arg_rec->wp_blk_wei_w, arg->wp_blk_wei_w, ISP33_RAWAWB_WEIGHT_NUM);

	/* avoid to override the old enable value */
	value = isp3_param_read_cache(params_vdev, ISP3X_RAWAWB_CTRL, id);
	value &= (ISP33_MODULE_EN |
		  ISP32_RAWAWB_2DDR_PATH_EN |
		  ISP32_RAWAWB_2DDR_PATH_DS);
	value |= !!arg->low12bit_val << 28 |
		 !!arg->yuv3d_en1 << 26 |
		 !!arg->xy_en1 << 25 |
		 !!arg->uv_en1 << 24 |
		 (arg->light_num & 0x7) << 20 |
		 !!arg->rawlsc_bypass_en << 19 |
		 !!arg->wind_size << 18 |
		 !!arg->in_overexposure_check_en << 17 |
		 !!arg->in_rshift_to_12bit_en << 16 |
		 (arg->yuv3d_ls_idx3 & 0x7) << 13 |
		 (arg->yuv3d_ls_idx2 & 0x7) << 10 |
		 (arg->yuv3d_ls_idx1 & 0x7) << 7 |
		 (arg->yuv3d_ls_idx0 & 0x7) << 4 |
		 !!arg->yuv3d_en0 << 3 |
		 !!arg->xy_en0 << 2 |
		 !!arg->uv_en0 << 1;
	isp3_param_write(params_vdev, value, ISP3X_RAWAWB_CTRL, id);

	mask = ISP32_DRC2AWB_SEL | ISP32_BNR2AWB_SEL | ISP3X_RAWAWB_SEL(3);
	val = ISP3X_RAWAWB_SEL(arg->rawawb_sel) |
	      (arg->bnr2awb_sel & 0x1) << 26 | (arg->drc2awb_sel & 0x1) << 27;
	value = isp3_param_read(params_vdev, ISP3X_VI_ISP_PATH, id);
	if ((value & mask) != val) {
		value &= ~mask;
		value |= val;
		isp3_param_write(params_vdev, value, ISP3X_VI_ISP_PATH, id);
	}
}

static void
isp_rawawb_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP3X_RAWAWB_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_RAWAWB_CTRL, id);
}

static void
isp_rawhstbig_cfg_sram(struct rkisp_isp_params_vdev *params_vdev,
		       const struct isp2x_rawhistbig_cfg *arg,
		       u32 addr, bool is_check, u32 id)
{
	u32 i, j, wnd_num_idx, value;
	u8 weight15x15[ISP33_RAWHISTBIG_WEIGHT_REG_SIZE];
	const u32 hist_wnd_num[] = {5, 5, 15, 15};

	value = isp3_param_read(params_vdev, addr + ISP3X_RAWHIST_BIG_CTRL, id);
	if (is_check && !(value & ISP3X_RAWHIST_EN))
		return;

	wnd_num_idx = arg->wnd_num;
	if (wnd_num_idx >= ARRAY_SIZE(hist_wnd_num)) {
		wnd_num_idx = ARRAY_SIZE(hist_wnd_num) - 1;
		dev_err(params_vdev->dev->dev,
			"%s invalid wnd_num:%d, set to %d\n",
			__func__, arg->wnd_num, wnd_num_idx);
	}
	memset(weight15x15, 0, sizeof(weight15x15));
	for (i = 0; i < hist_wnd_num[wnd_num_idx]; i++) {
		for (j = 0; j < hist_wnd_num[wnd_num_idx]; j++) {
			weight15x15[i * ISP33_RAWHISTBIG_ROW_NUM + j] =
				arg->weight[i * hist_wnd_num[wnd_num_idx] + j];
		}
	}

	for (i = 0; i < (ISP33_RAWHISTBIG_WEIGHT_REG_SIZE / 5); i++) {
		value = (weight15x15[5 * i + 0] & 0x3f) |
			(weight15x15[5 * i + 1] & 0x3f) << 6 |
			(weight15x15[5 * i + 2] & 0x3f) << 12 |
			(weight15x15[5 * i + 3] & 0x3f) << 18 |
			(weight15x15[5 * i + 4] & 0x3f) << 24;
		isp3_param_write_direct(params_vdev, value,
					addr + ISP3X_RAWHIST_BIG_WEIGHT_BASE);
	}
}

static void
isp_rawhstbig_config(struct rkisp_isp_params_vdev *params_vdev,
		     const struct isp2x_rawhistbig_cfg *arg, u32 addr, u32 id)
{
	struct isp33_isp_params_cfg *params_rec = params_vdev->isp33_params + id;
	struct rkisp_device *dev = params_vdev->dev;
	struct v4l2_rect *out_crop = &dev->isp_sdev.out_crop;
	u32 width = out_crop->width, height = out_crop->height;
	struct isp2x_rawhistbig_cfg *arg_rec;
	u32 hist_ctrl, block_hsize, block_vsize, wnd_num_idx;
	u32 h_size, v_size, h_offs, v_offs;
	const u32 hist_wnd_num[] = {5, 5, 15, 15};

	wnd_num_idx = arg->wnd_num;
	if (wnd_num_idx >= ARRAY_SIZE(hist_wnd_num)) {
		wnd_num_idx = ARRAY_SIZE(hist_wnd_num) - 1;
		dev_err(params_vdev->dev->dev,
			"%s invalid wnd_num:%d, set to %d\n",
			__func__, arg->wnd_num, wnd_num_idx);
	}
	/* avoid to override the old enable value */
	hist_ctrl = isp3_param_read(params_vdev, addr + ISP3X_RAWHIST_BIG_CTRL, id);
	hist_ctrl &= ISP3X_RAWHIST_EN;
	hist_ctrl = hist_ctrl |
		    ISP3X_RAWHIST_MODE(arg->mode) |
		    ISP3X_RAWHIST_WND_NUM(arg->wnd_num) |
		    ISP3X_RAWHIST_STEPSIZE(arg->stepsize) |
		    ISP3X_RAWHIST_DATASEL(arg->data_sel) |
		    ISP3X_RAWHIST_WATERLINE(arg->waterline);
	isp3_param_write(params_vdev, hist_ctrl, addr + ISP3X_RAWHIST_BIG_CTRL, id);

	h_offs = arg->win.h_offs & ~0x1;
	v_offs = arg->win.v_offs & ~0x1;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(h_offs, v_offs),
			 addr + ISP3X_RAWHIST_BIG_OFFS, id);

	if (dev->unite_div > ISP_UNITE_DIV1)
		width = width / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;
	if (dev->unite_div == ISP_UNITE_DIV4)
		height = height / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;

	h_size = arg->win.h_size;
	v_size = arg->win.v_size;
	if (!h_size || h_size + h_offs + 1 > width)
		h_size = width - h_offs - 1;
	if (!v_size || v_size + v_offs + 1 > height)
		v_size = height - v_offs - 1;
	block_hsize = (h_size / hist_wnd_num[wnd_num_idx]) & ~0x1;
	block_vsize = (v_size / hist_wnd_num[wnd_num_idx]) & ~0x1;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(block_hsize, block_vsize),
			 addr + ISP3X_RAWHIST_BIG_SIZE, id);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->rcc, arg->gcc, arg->bcc, arg->off),
			 addr + ISP3X_RAWHIST_BIG_RAW2Y_CC, id);

	if (dev->hw_dev->is_single)
		isp_rawhstbig_cfg_sram(params_vdev, arg, addr, false, id);

	arg_rec = (addr == ISP3X_RAWHIST_LITE_BASE) ?
		  &params_rec->meas.rawhist0 : &params_rec->meas.rawhist3;
	*arg_rec = *arg;
}

static void
isp_rawhstbig_enable(struct rkisp_isp_params_vdev *params_vdev,
		     bool en, u32 addr, u32 id)
{
	u32 val = isp3_param_read(params_vdev, addr + ISP3X_RAWHIST_BIG_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	val &= ~(ISP33_SELF_FORCE_UPD | ISP33_MODULE_EN);
	if (en)
		val |= ISP33_MODULE_EN;
	isp3_param_write(params_vdev, val, addr + ISP3X_RAWHIST_BIG_CTRL, id);
}

static void
isp_rawhst0_config(struct rkisp_isp_params_vdev *params_vdev,
		   const struct isp2x_rawhistbig_cfg *arg, u32 id)
{
	isp_rawhstbig_config(params_vdev, arg, ISP3X_RAWHIST_LITE_BASE, id);
}

static void
isp_rawhst0_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	isp_rawhstbig_enable(params_vdev, en, ISP3X_RAWHIST_LITE_BASE, id);
}

static void
isp_rawhst3_config(struct rkisp_isp_params_vdev *params_vdev,
		   const struct isp2x_rawhistbig_cfg *arg, u32 id)
{
	isp_rawhstbig_config(params_vdev, arg, ISP3X_RAWHIST_BIG1_BASE, id);
}

static void
isp_rawhst3_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	isp_rawhstbig_enable(params_vdev, en, ISP3X_RAWHIST_BIG1_BASE, id);
}

static void
isp_hdrmge_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp32_hdrmge_cfg *arg,
		  enum rkisp_params_type type, u32 id)
{
	u32 value;
	int i;

	if (type == RKISP_PARAMS_SHD || type == RKISP_PARAMS_ALL) {
		value = ISP_PACK_2SHORT(arg->gain0, arg->gain0_inv);
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_GAIN0, id);

		value = ISP_PACK_2SHORT(arg->gain1, arg->gain1_inv);
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_GAIN1, id);

		value = arg->gain2;
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_GAIN2, id);

		value = isp3_param_read_cache(params_vdev, ISP3X_HDRMGE_CTRL, id);
		if (arg->s_base)
			value |= BIT(1);
		else
			value &= ~BIT(1);
		if (arg->each_raw_en)
			value |= BIT(6);
		else
			value &= ~BIT(6);
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_CTRL, id);
	}

	if (type == RKISP_PARAMS_IMD || type == RKISP_PARAMS_ALL) {
		value = ISP_PACK_4BYTE(arg->ms_dif_0p8, arg->ms_diff_0p15,
				       arg->lm_dif_0p9, arg->lm_dif_0p15);
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_LIGHTZ, id);
		value = (arg->ms_scl & 0x7ff) |
			(arg->ms_thd0 & 0x3ff) << 12 |
			(arg->ms_thd1 & 0x3ff) << 22;
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_MS_DIFF, id);
		value = (arg->lm_scl & 0x7ff) |
			(arg->lm_thd0 & 0x3ff) << 12 |
			(arg->lm_thd1 & 0x3ff) << 22;
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_LM_DIFF, id);

		for (i = 0; i < ISP32_HDRMGE_L_CURVE_NUM; i++) {
			value = ISP_PACK_2SHORT(arg->curve.curve_0[i], arg->curve.curve_1[i]);
			isp3_param_write(params_vdev, value, ISP3X_HDRMGE_DIFF_Y0 + 4 * i, id);
		}

		for (i = 0; i < ISP32_HDRMGE_E_CURVE_NUM; i++) {
			value = (arg->l_raw1[i] & 0x3ff) << 20 |
				(arg->l_raw0[i] & 0x3ff) << 10 |
				(arg->e_y[i] & 0x3ff);
			isp3_param_write(params_vdev, value, ISP3X_HDRMGE_OVER_Y0 + 4 * i, id);
		}

		value = ISP_PACK_2SHORT(arg->each_raw_gain0, arg->each_raw_gain1);
		isp3_param_write(params_vdev, value, ISP32_HDRMGE_EACH_GAIN, id);
	}
}

static void
isp_hdrmge_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
}

static void
isp_hdrdrc_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp33_drc_cfg *arg,
		  enum rkisp_params_type type, u32 id)
{
	u32 i, value, ctrl;

	ctrl = isp3_param_read(params_vdev, ISP3X_DRC_CTRL0, id);
	ctrl &= ISP33_MODULE_EN;
	ctrl |= !!arg->bypass_en << 1 |
		!!arg->cmps_byp_en << 2 |
		!!arg->gainx32_en << 3 |
		!!arg->bf_lp_en << 4;
	isp3_param_write(params_vdev, ctrl, ISP3X_DRC_CTRL0, id);
	if (ctrl & BIT(29))
		dev_warn(params_vdev->dev->dev, "drc raw_dly_dis=1\n");
	value = isp3_param_read_cache(params_vdev, ISP3X_HDRMGE_CTRL, id);
	if (ctrl & BIT(2) && (value & ISP33_MODULE_EN))
		dev_warn(params_vdev->dev->dev, "drc cmps_byp_en=1 but hdr\n");

	if (type == RKISP_PARAMS_IMD)
		return;

	value = (arg->offset_pow2 & 0xF) << 28 |
		(arg->compres_scl & 0x1FFF) << 14 |
		(arg->position & 0x03FFF);
	isp3_param_write(params_vdev, value, ISP3X_DRC_CTRL1, id);

	value = arg->delta_scalein << 24 |
		(arg->hpdetail_ratio & 0xFFF) << 12 |
		(arg->lpdetail_ratio & 0xFFF);
	isp3_param_write(params_vdev, value, ISP3X_DRC_LPRATIO, id);

	value = arg->bilat_wt_off | (arg->thumb_thd_neg & 0x1ff) << 8 |
		!!arg->thumb_thd_enable << 23 | arg->weicur_pix << 24;
	isp3_param_write(params_vdev, value, ISP33_DRC_BILAT0, id);

	value = (arg->cmps_offset_bits_int & 0xf) |
		!!arg->cmps_fixbit_mode << 4 |
		arg->drc_gas_t << 16;
	isp3_param_write(params_vdev, value, ISP33_DRC_BILAT1, id);

	value = arg->thumb_clip | arg->thumb_scale << 16;
	isp3_param_write(params_vdev, value, ISP33_DRC_BILAT2, id);

	value = ISP_PACK_2SHORT(arg->range_sgm_inv0, arg->range_sgm_inv1);
	isp3_param_write(params_vdev, value, ISP33_DRC_BILAT3, id);

	value = arg->weig_bilat | arg->weight_8x8thumb << 8 |
		(arg->bilat_soft_thd & 0x7ff) << 16 |
		!!arg->enable_soft_thd << 31;
	isp3_param_write(params_vdev, value, ISP33_DRC_BILAT4, id);

	for (i = 0; i < ISP33_DRC_Y_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->gain_y[i * 2], arg->gain_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_DRC_GAIN_Y0 + i * 4, id);
		value = ISP_PACK_2SHORT(arg->compres_y[i * 2], arg->compres_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_DRC_COMPRES_Y0 + i * 4, id);
		value = ISP_PACK_2SHORT(arg->scale_y[i * 2], arg->scale_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_DRC_SCALE_Y0 + i * 4, id);
		value = ISP_PACK_2SHORT(arg->sfthd_y[i * 2], arg->sfthd_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_DRC_SFTHD_Y0 + i * 4, id);
	}
	value = arg->gain_y[i * 2];
	isp3_param_write(params_vdev, value, ISP33_DRC_GAIN_Y8, id);
	value = arg->compres_y[i * 2];
	isp3_param_write(params_vdev, value, ISP33_DRC_COMPRES_Y8, id);
	value = arg->scale_y[i * 2];
	isp3_param_write(params_vdev, value, ISP33_DRC_SCALE_Y8, id);
	value = arg->sfthd_y[i * 2];
	isp3_param_write(params_vdev, value, ISP33_DRC_SFTHD_Y8, id);

	value = arg->min_ogain;
	isp3_param_write(params_vdev, value, ISP33_DRC_IIRWG_GAIN, id);
}

static void
isp_hdrdrc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_DRC_CTRL0, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en) {
		val |= ISP33_MODULE_EN;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP3X_ADRC_FST_FRAME, id);
	} else {
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	}
	isp3_param_write(params_vdev, val, ISP3X_DRC_CTRL0, id);
}

static void
isp_gic_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp33_gic_cfg *arg, u32 id)
{
	u32 value;
	s32 i;

	value = isp3_param_read(params_vdev, ISP3X_GIC_CONTROL, id);
	value &= ISP33_MODULE_EN;
	value |= !!arg->bypass_en << 1 |
		 !!arg->pro_mode << 2 |
		 !!arg->manualnoisecurve_en << 3 |
		 !!arg->manualnoisethred_en << 4 |
		 !!arg->gain_bypass_en << 5;
	isp3_param_write(params_vdev, value, ISP3X_GIC_CONTROL, id);

	value = (arg->medflt_minthred & 0xf) |
		(arg->medflt_maxthred & 0xf) << 4 | arg->medflt_ratio << 16;
	isp3_param_write(params_vdev, value, ISP33_GIC_MEDFLT_PARA, id);

	value = (arg->medfltuv_minthred & 0xf) |
		(arg->medfltuv_maxthred & 0xf) << 4 | arg->medfltuv_ratio << 16;
	isp3_param_write(params_vdev, value, ISP33_GIC_MEDFLTUV_PARA, id);

	value = arg->noisecurve_scale;
	isp3_param_write(params_vdev, value, ISP33_GIC_NOISE_SCALE, id);

	value = arg->bffltwgt_offset | arg->bffltwgt_scale << 16;
	isp3_param_write(params_vdev, value, ISP33_GIC_BILAT_PARA1, id);

	value = arg->bfflt_ratio;
	isp3_param_write(params_vdev, value, ISP33_GIC_BILAT_PARA2, id);

	value = ISP_PACK_4BYTE(arg->bfflt_coeff0, arg->bfflt_coeff1,
			       arg->bfflt_coeff2, 0);
	isp3_param_write(params_vdev, value, ISP33_GIC_DISWGT_COEFF, id);

	for (i = 0; i < ISP33_GIC_SIGMA_Y_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->bfflt_vsigma_y[i * 2],
					arg->bfflt_vsigma_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_GIC_SIGMA_Y0 + i * 4, id);
	}
	value = arg->bfflt_vsigma_y[i * 2];
	isp3_param_write(params_vdev, value, ISP33_GIC_SIGMA_Y8, id);

	value = (arg->luma_dx[0] & 0xf) | (arg->luma_dx[1] & 0xf) << 4 |
		(arg->luma_dx[2] & 0xf) << 8 | (arg->luma_dx[3] & 0xf) << 12 |
		(arg->luma_dx[4] & 0xf) << 16 | (arg->luma_dx[5] & 0xf) << 20 |
		(arg->luma_dx[6] & 0xf) << 24;
	isp3_param_write(params_vdev, value, ISP33_GIC_LUMA_DX, id);

	for (i = 0; i < ISP33_GIC_THRED_Y_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->thred_y[i * 2],
					arg->thred_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_GIC_THRED_Y0 + i * 4, id);

		value = ISP_PACK_2SHORT(arg->minthred_y[i * 2],
					arg->minthred_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_GIC_MIN_THRED_Y0 + i * 4, id);
	}

	value = arg->autonoisethred_scale;
	isp3_param_write(params_vdev, value, ISP33_GIC_THRED_SCALE, id);

	value = ISP_PACK_4BYTE(arg->lofltgr_coeff0, arg->lofltgr_coeff1,
			       arg->lofltgr_coeff2, arg->lofltgr_coeff3);
	isp3_param_write(params_vdev, value, ISP33_GIC_LOFLTGR_COEFF, id);

	value = ISP_PACK_4BYTE(arg->lofltgb_coeff0, arg->lofltgb_coeff1, 0, 0);
	isp3_param_write(params_vdev, value, ISP33_GIC_LOFLTGB_COEFF, id);

	value = arg->sumlofltcoeff_inv;
	isp3_param_write(params_vdev, value, ISP33_GIC_SUM_LOFLT_INV, id);

	value = ISP_PACK_4BYTE(arg->lofltthred_coeff0, arg->lofltthred_coeff1, 0, 0);
	isp3_param_write(params_vdev, value, ISP33_GIC_LOFLTTHRED_COEFF, id);

	value = (arg->global_gain & 0x3ff) |
		(arg->globalgain_alpha & 0xf) << 12 |
		arg->globalgain_scale << 16;
	isp3_param_write(params_vdev, value, ISP33_GIC_GAIN, id);

	value = ISP_PACK_2SHORT(arg->gain_offset, arg->gain_scale);
	isp3_param_write(params_vdev, value, ISP33_GIC_GAIN_SLOPE, id);

	value = ISP_PACK_2SHORT(arg->gainadjflt_minthred,
				arg->gainadjflt_maxthred);
	isp3_param_write(params_vdev, value,  ISP33_GIC_GAIN_THRED, id);
}

static void
isp_gic_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP3X_GIC_CONTROL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en) {
		val |= ISP33_MODULE_EN;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP33_GIC_FST_FRAME, id);
	} else {
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	}
	isp3_param_write(params_vdev, val, ISP3X_GIC_CONTROL, id);
}

static void
isp_enh_cfg_sram(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp33_enh_cfg *arg, bool is_check, u32 id)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	u32 i, j, val, ctrl = isp3_param_read(params_vdev, ISP33_ENH_CTRL, id);

	if (is_check && (!(ctrl & ISP33_MODULE_EN) || !arg->iir_wr))
		return;

	val = (arg->pre_wet_frame_cnt0 & 0xf) |
	      (arg->pre_wet_frame_cnt1 & 0xf) << 4;
	isp3_param_write_direct(params_vdev, val, ISP33_ENH_PRE_FRAME);
	for (i = 0; i < priv_val->enh_row; i++) {
		val = ISP33_IIR_WR_ID(i) | ISP33_IIR_WR_CLEAR;
		isp3_param_write_direct(params_vdev, val, ISP33_ENH_IIR_RW);
		for (j = 0; j < priv_val->enh_col / 4; j++) {
			val = ISP_PACK_4BYTE(arg->iir[i][j * 4], arg->iir[i][j * 4 + 1],
					     arg->iir[i][j * 4 + 2], arg->iir[i][j * 4 + 3]);
			isp3_param_write_direct(params_vdev, val, ISP33_ENH_IIR0 + j * 4);
		}
	}
}

static void
isp_enh_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp33_enh_cfg *arg, u32 id)
{
	struct isp33_isp_params_cfg *params_rec = params_vdev->isp33_params + id;
	struct isp33_enh_cfg *arg_rec = &params_rec->others.enh_cfg;
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_device *dev = params_vdev->dev;
	struct v4l2_rect *out_crop = &dev->isp_sdev.out_crop;
	u32 w = out_crop->width, h = out_crop->height;
	u32 i, value, ctrl, het_aliquant;

	if (dev->unite_div > ISP_UNITE_DIV1)
		w = w / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;
	if (dev->unite_div == ISP_UNITE_DIV4)
		h = h / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;
	priv_val->enh_col = ALIGN((w + 127) / 128, 4);
	if (priv_val->enh_col > ISP33_ENH_IIR_COL_MAX)
		priv_val->enh_col = ISP33_ENH_IIR_COL_MAX;
	priv_val->enh_row = (h + 128) / 129;
	if (priv_val->enh_row > ISP33_ENH_IIR_ROW_MAX)
		priv_val->enh_row = ISP33_ENH_IIR_ROW_MAX;
	het_aliquant = h % 3;

	ctrl = isp3_param_read(params_vdev, ISP33_ENH_CTRL, id);
	ctrl &= ISP33_MODULE_EN;
	ctrl |= !!arg->bypass << 1 |
		!!arg->blf3_bypass << 2 |
		(het_aliquant & 0x3) << 4 |
		(priv_val->enh_row & 0x1f) << 8;
	isp3_param_write(params_vdev, ctrl, ISP33_ENH_CTRL, id);

	value = arg->iir_inv_sigma |
		arg->iir_soft_thed << 16 |
		arg->iir_cur_wgt << 24;
	isp3_param_write(params_vdev, value, ISP33_ENH_IIR_FLT, id);

	value = (arg->blf3_inv_sigma & 0x1ff) |
		(arg->blf3_cur_wgt & 0x1ff) << 16 |
		(arg->blf3_thumb_cur_wgt & 0xf) << 28;
	isp3_param_write(params_vdev, value, ISP33_ENH_BILAT_FLT3X3, id);

	value = arg->blf5_inv_sigma | arg->blf5_cur_wgt << 16;
	isp3_param_write(params_vdev, value, ISP33_ENH_BILAT_FLT5X5, id);

	value = arg->global_strg;
	isp3_param_write(params_vdev, value, ISP33_ENH_GLOBAL_STRG, id);

	for (i = 0; i < ISP33_ENH_LUMA_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->lum2strg[i * 2], arg->lum2strg[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_ENH_LUMA_LUT0 + i * 4, id);
	}
	value = arg->lum2strg[i * 2];
	isp3_param_write(params_vdev, value, ISP33_ENH_LUMA_LUT8, id);

	for (i = 0; i < ISP33_ENH_DETAIL_NUM / 3; i++) {
		value = (arg->detail2strg_idx[i * 3] & 0x3ff) |
			(arg->detail2strg_idx[i * 3 + 1] & 0x3ff) << 10 |
			(arg->detail2strg_idx[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_ENH_DETAIL_IDX0 + i * 4, id);
	}
	value = (arg->detail2strg_idx[i * 3] & 0x3ff) |
		(arg->detail2strg_idx[i * 3 + 1] & 0x7ff) << 10;
	isp3_param_write(params_vdev, value, ISP33_ENH_DETAIL_IDX2, id);

	value = (arg->detail2strg_power0 & 0xf) |
		(arg->detail2strg_power1 & 0xf) << 4 |
		(arg->detail2strg_power2 & 0xf) << 8 |
		(arg->detail2strg_power3 & 0xf) << 12 |
		(arg->detail2strg_power4 & 0xf) << 16 |
		(arg->detail2strg_power5 & 0xf) << 20 |
		(arg->detail2strg_power6 & 0xf) << 24;
	isp3_param_write(params_vdev, value, ISP33_ENH_DETAIL_POWER, id);

	for (i = 0; i < ISP33_ENH_DETAIL_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->detail2strg_val[i * 2],
					arg->detail2strg_val[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_ENH_DETAIL_VALUE0 + i * 4, id);
	}

	if (dev->hw_dev->is_single && arg->iir_wr)
		isp_enh_cfg_sram(params_vdev, arg, false, id);
	else if (arg->iir_wr)
		memcpy(arg_rec, arg, sizeof(struct isp33_enh_cfg));
}

static void
isp_enh_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP33_ENH_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en) {
		val |= ISP33_MODULE_EN;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP33_ENH_FST_FRAME, id);
	} else {
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	}
	isp3_param_write(params_vdev, val, ISP33_ENH_CTRL, id);
}

static void
isp_hist_cfg_sram(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp33_hist_cfg *arg, bool is_check, u32 id)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	u32 i, j, val, ctrl = isp3_param_read(params_vdev, ISP33_HIST_CTRL, id);

	if (is_check && (!(ctrl & ISP33_MODULE_EN) || !arg->iir_wr))
		return;

	val = (arg->stab_frame_cnt0 & 0xf) |
	      (arg->stab_frame_cnt1 & 0xf) << 4;
	isp3_param_write_direct(params_vdev, val, ISP33_HIST_STAB);
	for (i = 0; i < priv_val->hist_blk_num; i++) {
		val = ISP33_IIR_WR_ID(i) | ISP33_IIR_WR_CLEAR;
		isp3_param_write_direct(params_vdev, val, ISP33_HIST_RW);
		for (j = 0; j < ISP33_HIST_IIR_NUM / 2; j++) {
			val = ISP_PACK_2SHORT(arg->iir[i][2 * j], arg->iir[i][2 * j + 1]);
			isp3_param_write_direct(params_vdev, val, ISP33_HIST_IIR0 + 4 * j);
		}
	}
}

static void
isp_hist_config(struct rkisp_isp_params_vdev *params_vdev,
		const struct isp33_hist_cfg *arg, u32 id)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct v4l2_rect *out_crop = &dev->isp_sdev.out_crop;
	struct isp33_isp_params_cfg *params_rec = params_vdev->isp33_params + id;
	struct isp33_hist_cfg *arg_rec = &params_rec->others.hist_cfg;
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	u32 w = out_crop->width, h = out_crop->height;
	u32 value, ctrl, thumb_row, thumb_col, blk_het, blk_wid;
	int i;

	if (dev->unite_div > ISP_UNITE_DIV1)
		w = w / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;
	if (dev->unite_div == ISP_UNITE_DIV4)
		h = h / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;

	ctrl = isp3_param_read(params_vdev, ISP33_HIST_CTRL, id);
	ctrl &= ISP33_MODULE_EN;
	ctrl |= !!arg->bypass << 1 | !!arg->mem_mode << 4;
	isp3_param_write(params_vdev, ctrl, ISP33_HIST_CTRL, id);

	value = arg->count_scale | arg->count_offset << 8 |
		arg->count_min_limit << 16;
	isp3_param_write(params_vdev, value, ISP33_HIST_HF_STAT, id);

	value = ISP_PACK_2SHORT(arg->merge_alpha, arg->user_set);
	isp3_param_write(params_vdev, value, ISP33_HIST_MAP0, id);

	value = arg->map_count_scale | arg->gain_ref_wgt << 16;
	isp3_param_write(params_vdev, value, ISP33_HIST_MAP1, id);

	value = arg->flt_inv_sigma | arg->flt_cur_wgt << 16;
	isp3_param_write(params_vdev, value, ISP33_HIST_IIR, id);

	for (i = 0; i < ISP33_HIST_ALPHA_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->pos_alpha[i * 4],
				       arg->pos_alpha[i * 4 + 1],
				       arg->pos_alpha[i * 4 + 2],
				       arg->pos_alpha[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_HIST_POS_ALPHA0 + i * 4, id);
		value = ISP_PACK_4BYTE(arg->neg_alpha[i * 4],
				       arg->neg_alpha[i * 4 + 1],
				       arg->neg_alpha[i * 4 + 2],
				       arg->neg_alpha[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_HIST_NEG_ALPHA0 + i * 4, id);
	}
	value = arg->pos_alpha[i * 4];
	isp3_param_write(params_vdev, value, ISP33_HIST_POS_ALPHA4, id);
	value = arg->neg_alpha[i * 4];
	isp3_param_write(params_vdev, value, ISP33_HIST_NEG_ALPHA4, id);

	value = arg->saturate_scale;
	isp3_param_write(params_vdev, value, ISP33_HIST_UV_SCL, id);

	thumb_row = arg->thumb_row > ISP33_HIST_THUMB_ROW_MAX ?
		    ISP33_HIST_THUMB_ROW_MAX : arg->thumb_row & ~1;
	thumb_col = arg->thumb_col > ISP33_HIST_THUMB_COL_MAX ?
		    ISP33_HIST_THUMB_COL_MAX : arg->thumb_col & ~1;
	blk_het = ALIGN(h / thumb_row, 2);
	blk_wid = ALIGN(w / thumb_col, 2);
	priv_val->hist_blk_num = thumb_row * thumb_col;
	value = ISP_PACK_2SHORT(blk_het, blk_wid);
	isp3_param_write(params_vdev, value, ISP33_HIST_BLOCK_SIZE, id);
	value = ISP_PACK_4BYTE(thumb_row, thumb_col, 0, 0);
	isp3_param_write(params_vdev, value, ISP33_HIST_THUMB_SIZE, id);

	if (dev->hw_dev->is_single && arg->iir_wr)
		isp_hist_cfg_sram(params_vdev, arg, false, id);
	else if (arg->iir_wr)
		memcpy(arg_rec, arg, sizeof(struct isp33_hist_cfg));
}

static void
isp_hist_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP33_HIST_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en) {
		val |= ISP33_MODULE_EN;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP33_YHIST_FST_FRAME, id);
	} else {
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	}
	isp3_param_write(params_vdev, val, ISP33_HIST_CTRL, id);
}

static void
isp_hsv_cfg_sram(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp33_hsv_cfg *arg, bool is_check, u32 id)
{
	u32 ctrl = isp3_param_read(params_vdev, ISP3X_3DLUT_CTRL, id);
	u32 val, i, j;

	if (is_check && !(ctrl & ISP33_MODULE_EN))
		return;

	for (i = 0; i < ISP33_HSV_1DLUT_NUM; i++) {
		val = ISP_PACK_2SHORT(arg->lut0_1d[i], arg->lut1_1d[i]);
		isp3_param_write_direct(params_vdev, val, ISP33_HSV_1DLUT);
	}
	for (i = 0; i < ISP33_HSV_2DLUT_ROW; i++) {
		for (j = 0; j < ISP33_HSV_2DLUT_COL - 1; j += 2) {
			val = ISP_PACK_2SHORT(arg->lut_2d[i][j], arg->lut_2d[i][j + 1]);
			isp3_param_write_direct(params_vdev, val, ISP33_HSV_2DLUT);
		}
		val = arg->lut_2d[i][ISP33_HSV_2DLUT_COL - 1];
		isp3_param_write_direct(params_vdev, val, ISP33_HSV_2DLUT);
	}
}

static void
isp_hsv_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp33_hsv_cfg *arg, u32 id)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct isp33_isp_params_cfg *params_rec = params_vdev->isp33_params + id;
	u32 val = isp3_param_read(params_vdev, ISP3X_3DLUT_CTRL, id);

	val &= ISP33_MODULE_EN;
	val |= !!arg->hsv_1dlut0_en << 1 |
	       !!arg->hsv_1dlut1_en << 2 |
	       !!arg->hsv_2dlut_en << 3 |
	       (arg->hsv_1dlut0_idx_mode & 0x3) << 4 |
	       (arg->hsv_1dlut1_idx_mode & 0x3) << 6 |
	       (arg->hsv_2dlut_idx_mode & 0x3) << 8 |
	       (arg->hsv_1dlut0_item_mode & 0x7) << 10 |
	       (arg->hsv_1dlut1_item_mode & 0x7) << 13 |
	       (arg->hsv_2dlut_item_mode & 0x3) << 16;
	isp3_param_write(params_vdev, val, ISP3X_3DLUT_CTRL, id);

	if (dev->hw_dev->is_single)
		isp_hsv_cfg_sram(params_vdev, arg, false, id);
	params_rec->others.hsv_cfg = *arg;
}

static void
isp_hsv_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_3DLUT_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_3DLUT_CTRL, id);
}

static void
isp_ldch_config(struct rkisp_isp_params_vdev *params_vdev,
		const struct isp32_ldch_cfg *arg, u32 id)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_device *dev = params_vdev->dev;
	struct isp2x_mesh_head *head;
	int buf_idx, i;
	u32 value;

	value = isp3_param_read(params_vdev, ISP3X_LDCH_STS, id);
	value &= ISP33_MODULE_EN;
	value |= !!arg->map13p3_en << 7 |
		 !!arg->force_map_en << 6 |
		 !!arg->bic_mode_en << 4 |
		 !!arg->sample_avr_en << 3 |
		 !!arg->frm_end_dis << 1;
	isp3_param_write(params_vdev, value, ISP3X_LDCH_STS, id);
	if (arg->bic_mode_en) {
		for (i = 0; i < ISP33_LDCH_BIC_NUM / 4; i++) {
			value = ISP_PACK_4BYTE(arg->bicubic[i * 4], arg->bicubic[i * 4 + 1],
					arg->bicubic[i * 4 + 2], arg->bicubic[i * 4 + 3]);
			isp3_param_write(params_vdev, value, ISP32_LDCH_BIC_TABLE0 + i * 4, id);
		}
	}

	for (i = 0; i < ISP33_MESH_BUF_NUM; i++) {
		if (!priv_val->buf_ldch[id][i].mem_priv)
			continue;
		if (arg->buf_fd == priv_val->buf_ldch[id][i].dma_fd)
			break;
	}
	if (i == ISP33_MESH_BUF_NUM) {
		dev_err(dev->dev, "cannot find ldch buf fd(%d)\n", arg->buf_fd);
		return;
	}

	if (!priv_val->buf_ldch[id][i].vaddr) {
		dev_err(dev->dev, "no ldch buffer allocated\n");
		return;
	}

	buf_idx = priv_val->buf_ldch_idx[id];
	head = (struct isp2x_mesh_head *)priv_val->buf_ldch[id][buf_idx].vaddr;
	head->stat = MESH_BUF_INIT;

	buf_idx = i;
	head = (struct isp2x_mesh_head *)priv_val->buf_ldch[id][buf_idx].vaddr;
	head->stat = MESH_BUF_CHIPINUSE;
	priv_val->buf_ldch_idx[id] = buf_idx;
	rkisp_prepare_buffer(dev, &priv_val->buf_ldch[id][buf_idx]);
	value = priv_val->buf_ldch[id][buf_idx].dma_addr + head->data_oft;
	isp3_param_write(params_vdev, value, ISP3X_MI_LUT_LDCH_RD_BASE, id);
	isp3_param_write(params_vdev, arg->hsize, ISP3X_MI_LUT_LDCH_RD_H_WSIZE, id);
	isp3_param_write(params_vdev, arg->vsize, ISP3X_MI_LUT_LDCH_RD_V_SIZE, id);
}

static void
isp_ldch_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_device *dev = params_vdev->dev;
	u32 val = isp3_param_read(params_vdev, ISP3X_LDCH_STS, id);
	u32 buf_idx;

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en) {
		buf_idx = priv_val->buf_ldch_idx[id];
		if (!priv_val->buf_ldch[id][buf_idx].vaddr) {
			dev_err(dev->dev, "no ldch buffer allocated\n");
			return;
		}
		val |= ISP33_MODULE_EN;
	} else {
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	}
	isp3_param_write(params_vdev, val, ISP3X_LDCH_STS, id);
}

static void
isp_ynr_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp33_ynr_cfg *arg, u32 id)
{
	u32 i, value;

	value = isp3_param_read(params_vdev, ISP3X_YNR_GLOBAL_CTRL, id);
	value &= ISP33_MODULE_EN;

	value |= !!arg->hi_spnr_bypass << 1 |
		 !!arg->mi_spnr_bypass << 2 |
		 !!arg->lo_spnr_bypass << 3 |
		 !!arg->rnr_en << 4 |
		 !!arg->tex2lo_strg_en << 5 |
		 !!arg->hi_lp_en << 6;
	isp3_param_write(params_vdev, value, ISP3X_YNR_GLOBAL_CTRL, id);

	value = (arg->global_set_gain & 0x3ff) |
		(arg->gain_merge_alpha & 0xf) << 12 |
		arg->local_gain_scale << 16;
	isp3_param_write(params_vdev, value, ISP33_YNR_GAIN_CTRL, id);

	for (i = 0; i < ISP33_YNR_ADJ_NUM / 3; i++) {
		value = (arg->lo_spnr_gain2strg[i * 3] & 0x1ff) |
			(arg->lo_spnr_gain2strg[i * 3 + 1] & 0x1ff) << 10 |
			(arg->lo_spnr_gain2strg[i * 3 + 2] & 0x1ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_YNR_GAIN_ADJ_0_2 + i * 4, id);
	}

	value = arg->rnr_max_radius;
	isp3_param_write(params_vdev, value, ISP33_YNR_RNR_MAX_R, id);

	value = ISP_PACK_2SHORT(arg->rnr_center_h, arg->rnr_center_v);
	isp3_param_write(params_vdev, value, ISP33_YNR_RNR_CENTER_COOR, id);

	for (i = 0; i < ISP33_YNR_XY_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->radius2strg[i * 4],
				       arg->radius2strg[i * 4 + 1],
				       arg->radius2strg[i * 4 + 2],
				       arg->radius2strg[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_YNR_RNR_STRENGTH03 + i * 4, id);
	}
	value = arg->radius2strg[i * 4];
	isp3_param_write(params_vdev, value, ISP33_YNR_RNR_STRENGTH16, id);

	for (i = 0; i < ISP33_YNR_XY_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->luma2sima_x[i * 2],
					arg->luma2sima_x[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_YNR_SGM_DX_0_1 + i * 4, id);

		value = ISP_PACK_2SHORT(arg->luma2sima_y[i * 2],
					arg->luma2sima_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_YNR_SGM_Y_0_1 + i * 4, id);
	}
	value = arg->luma2sima_x[i * 2];
	isp3_param_write(params_vdev, value, ISP33_YNR_SGM_DX_16, id);
	value = arg->luma2sima_y[i * 2];
	isp3_param_write(params_vdev, value, ISP33_YNR_SGM_Y_16, id);

	value = (arg->hi_spnr_sigma_min_limit & 0x7ff) |
		(arg->hi_spnr_local_gain_alpha & 0x1f) << 11 |
		(arg->hi_spnr_strg & 0x3ff) << 16;
	isp3_param_write(params_vdev, value, ISP33_YNR_HI_SIGMA_GAIN, id);

	value = (arg->hi_spnr_filt_coeff[0] & 0x3f) |
		(arg->hi_spnr_filt_coeff[1] & 0x3f) << 6 |
		(arg->hi_spnr_filt_coeff[2] & 0x3f) << 12 |
		(arg->hi_spnr_filt_coeff[3] & 0x3f) << 18 |
		(arg->hi_spnr_filt_coeff[4] & 0x3f) << 24;
	isp3_param_write(params_vdev, value, ISP33_YNR_HI_GAUS_COE, id);

	value = (arg->hi_spnr_filt_wgt_offset & 0x3ff) |
		(arg->hi_spnr_filt_center_wgt & 0x1fff) << 10;
	isp3_param_write(params_vdev, value, ISP33_YNR_HI_WEIGHT, id);

	value = (arg->hi_spnr_filt1_coeff[0] & 0x1ff) |
		(arg->hi_spnr_filt1_coeff[1] & 0x1ff) << 10 |
		(arg->hi_spnr_filt1_coeff[2] & 0x1ff) << 20;
	isp3_param_write(params_vdev, value, ISP33_YNR_HI_GAUS1_COE_0_2, id);
	value = (arg->hi_spnr_filt1_coeff[3] & 0x1ff) |
		(arg->hi_spnr_filt1_coeff[4] & 0x1ff) << 10 |
		(arg->hi_spnr_filt1_coeff[5] & 0x1ff) << 20;
	isp3_param_write(params_vdev, value, ISP33_YNR_HI_GAUS1_COE_3_5, id);

	value = (arg->hi_spnr_filt1_tex_thred & 0x7ff) |
		(arg->hi_spnr_filt1_tex_scale & 0x3ff) << 12 |
		(arg->hi_spnr_filt1_wgt_alpha & 0x1ff) << 22;
	isp3_param_write(params_vdev, value, ISP33_YNR_HI_TEXT, id);

	value = (arg->mi_spnr_filt_coeff0 & 0x1ff) |
		(arg->mi_spnr_filt_coeff1 & 0x1ff) << 10 |
		(arg->mi_spnr_filt_coeff2 & 0x1ff) << 20;
	isp3_param_write(params_vdev, value, ISP33_YNR_MI_GAUS_COE, id);

	value = ISP_PACK_2SHORT(arg->mi_spnr_strg, arg->mi_spnr_soft_thred_scale);
	isp3_param_write(params_vdev, value, ISP33_YNR_MI_STRG_DETAIL, id);

	value = arg->mi_spnr_wgt |
		(arg->mi_spnr_filt_center_wgt & 0x7ff) << 10 |
		!!arg->mi_ehance_scale_en << 23 |
		arg->mi_ehance_scale << 24;
	isp3_param_write(params_vdev, value, ISP33_YNR_MI_WEIGHT, id);

	value = ISP_PACK_2SHORT(arg->lo_spnr_strg, arg->lo_spnr_soft_thred_scale);
	isp3_param_write(params_vdev, value, ISP33_YNR_LO_STRG_DETAIL, id);

	value = (arg->lo_spnr_thumb_thred_scale & 0x3ff) |
		(arg->tex2lo_strg_mantissa & 0x7ff) << 12 |
		(arg->tex2lo_strg_exponent & 0xf) << 24;
	isp3_param_write(params_vdev, value, ISP33_YNR_LO_LIMIT_SCALE, id);

	value = arg->lo_spnr_wgt |
		(arg->lo_spnr_filt_center_wgt & 0x1fff) << 10;
	isp3_param_write(params_vdev, value, ISP33_YNR_LO_WEIGHT, id);

	value = (arg->tex2lo_strg_upper_thred & 0x3ff) |
		(arg->tex2lo_strg_lower_thred & 0x3ff) << 12;
	isp3_param_write(params_vdev, value, ISP33_YNR_LO_TEXT_THRED, id);

	for (i = 0; i < ISP33_YNR_ADJ_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->lo_gain2wgt[i * 4],
				       arg->lo_gain2wgt[i * 4 + 1],
				       arg->lo_gain2wgt[i * 4 + 2],
				       arg->lo_gain2wgt[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_YNR_FUSION_WEIT_ADJ_0_3 + i * 4, id);
	}
	value = arg->lo_gain2wgt[i * 4];
	isp3_param_write(params_vdev, value, ISP33_YNR_FUSION_WEIT_ADJ_8, id);
}

static void
isp_ynr_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP3X_YNR_GLOBAL_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en) {
		val |= ISP33_MODULE_EN;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP3X_YNR_FST_FRAME, id);
	} else {
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	}
	isp3_param_write(params_vdev, val, ISP3X_YNR_GLOBAL_CTRL, id);
}

static void
isp_cnr_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp33_cnr_cfg *arg, u32 id)
{
	u32 i, value, ctrl, gain_ctrl;

	gain_ctrl = isp3_param_read(params_vdev, ISP3X_GAIN_CTRL, id);
	ctrl = isp3_param_read(params_vdev, ISP3X_CNR_CTRL, id);
	ctrl &= ISP33_MODULE_EN;

	ctrl |= (arg->loflt_coeff & 0x3f) << 12 |
		!!arg->local_alpha_dis << 11 |
		!!arg->hiflt_wgt0_mode << 8 |
		(arg->thumb_mode & 0x3) << 4 |
		!!arg->yuv422_mode << 2 |
		!!arg->exgain_bypass << 1;
	value = (arg->global_gain & 0x3ff) |
		(arg->global_gain_alpha & 0xf) << 12 |
		arg->local_gain_scale << 16;
	/* gain disable, using global gain for cnr */
	if (ctrl & ISP33_MODULE_EN && !(gain_ctrl & ISP33_MODULE_EN)) {
		ctrl |= BIT(1);
		value &= ~ISP3X_CNR_GLOBAL_GAIN_ALPHA_MAX;
		value |= BIT(15);
	}
	isp3_param_write(params_vdev, ctrl, ISP3X_CNR_CTRL, id);
	isp3_param_write(params_vdev, value, ISP3X_CNR_EXGAIN, id);

	value = ISP_PACK_2SHORT(arg->lobfflt_vsigma_uv, arg->lobfflt_vsigma_y);
	isp3_param_write(params_vdev, value, ISP32_CNR_THUMB1, id);

	value = arg->lobfflt_alpha;
	isp3_param_write(params_vdev, value, ISP32_CNR_THUMB_BF_RATIO, id);

	value = ISP_PACK_4BYTE(arg->thumb_bf_coeff0, arg->thumb_bf_coeff1,
			       arg->thumb_bf_coeff2, arg->thumb_bf_coeff3);
	isp3_param_write(params_vdev, value, ISP32_CNR_LBF_WEITD, id);

	value = (arg->loflt_uv_gain & 0xf) |
		arg->loflt_vsigma << 4 |
		(arg->exp_x_shift_bit & 0x3f) << 12 |
		(arg->loflt_wgt_slope & 0x3ff) << 20;
	isp3_param_write(params_vdev, value, ISP32_CNR_IIR_PARA1, id);

	value = ISP_PACK_4BYTE(arg->loflt_wgt_min_thred, arg->loflt_wgt_max_limit, 0, 0);
	isp3_param_write(params_vdev, value, ISP32_CNR_IIR_PARA2, id);

	value = ISP_PACK_4BYTE(arg->gaus_flt_coeff[0], arg->gaus_flt_coeff[1],
			       arg->gaus_flt_coeff[2], arg->gaus_flt_coeff[3]);
	isp3_param_write(params_vdev, value, ISP32_CNR_GAUS_COE1, id);

	value = ISP_PACK_4BYTE(arg->gaus_flt_coeff[4], arg->gaus_flt_coeff[5], 0, 0);
	isp3_param_write(params_vdev, value, ISP32_CNR_GAUS_COE2, id);

	value = (arg->gaus_flt_alpha & 0x7ff) |
		arg->hiflt_wgt_min_limit << 12 |
		(arg->hiflt_alpha & 0x7ff) << 20;
	isp3_param_write(params_vdev, value, ISP32_CNR_GAUS_RATIO, id);

	value = arg->hiflt_uv_gain |
		(arg->hiflt_global_vsigma & 0x3fff) << 8 |
		arg->hiflt_cur_wgt << 24;
	isp3_param_write(params_vdev, value, ISP32_CNR_BF_PARA1, id);

	value = ISP_PACK_2SHORT(arg->adj_offset, arg->adj_scale);
	isp3_param_write(params_vdev, value, ISP32_CNR_BF_PARA2, id);

	for (i = 0; i < ISP33_CNR_SIGMA_Y_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->sgm_ratio[i * 4], arg->sgm_ratio[i * 4 + 1],
				       arg->sgm_ratio[i * 4 + 2], arg->sgm_ratio[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP32_CNR_SIGMA0 + i * 4, id);
	}
	value = arg->sgm_ratio[i * 4] | arg->bf_merge_max_limit << 16;
	isp3_param_write(params_vdev, value, ISP32_CNR_SIGMA0 + i * 4, id);

	value = arg->loflt_global_sgm_ratio |
		arg->loflt_global_sgm_ratio_alpha << 8 |
		arg->bf_alpha_max_limit << 16;
	isp3_param_write(params_vdev, value, ISP32_CNR_IIR_GLOBAL_GAIN, id);

	for (i = 0; i < ISP33_CNR_WGT_SIGMA_Y_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->cur_wgt[i * 4], arg->cur_wgt[i * 4 + 1],
				       arg->cur_wgt[i * 4 + 2], arg->cur_wgt[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_CNR_WGT_SIGMA0 + i * 4, id);
	}
	value = arg->cur_wgt[i * 4];
	isp3_param_write(params_vdev, value, ISP33_CNR_WGT_SIGMA3, id);

	for (i = 0; i < ISP33_CNR_GAUS_SIGMAR_NUM / 3; i++) {
		value = (arg->hiflt_vsigma_idx[i * 3] & 0x3ff) |
			(arg->hiflt_vsigma_idx[i * 3 + 1] & 0x3ff) << 10 |
			(arg->hiflt_vsigma_idx[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_CNR_GAUS_X_SIGMAR0 + i * 4, id);
	}
	value = (arg->hiflt_vsigma_idx[i * 3] & 0x3ff) |
		(arg->hiflt_vsigma_idx[i * 3 + 1] & 0x3ff) << 10;
	isp3_param_write(params_vdev, value, ISP33_CNR_GAUS_X_SIGMAR2, id);

	for (i = 0; i < ISP33_CNR_GAUS_SIGMAR_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->hiflt_vsigma[i * 2], arg->hiflt_vsigma[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_CNR_GAUS_Y_SIGMAR0 + i * 4, id);
	}
}

static void
isp_cnr_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP3X_CNR_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en) {
		val |= ISP33_MODULE_EN;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP3X_CNR_FST_FRAME, id);
	} else {
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	}
	isp3_param_write(params_vdev, val, ISP3X_CNR_CTRL, id);
}

static void
isp_sharp_cfg_noise_curve(struct rkisp_isp_params_vdev *params_vdev,
			  const struct isp33_sharp_cfg *arg, u32 id, bool direct)
{
	struct rkisp_device *dev = params_vdev->dev;
	u32 i, value;

	for (i = 0; i < ISP33_SHARP_NOISE_CURVE_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->noise_curve_ext[i * 2],
					arg->noise_curve_ext[i * 2 + 1]);
		rkisp_idx_write(dev, ISP33_SHARP_NOISE_CURVE0 + i * 4, value, id, direct);
	}
	value = (arg->noise_curve_ext[i * 2] & 0x7ff) |
		arg->noise_count_thred_ratio << 12 |
		arg->noise_clip_scale << 20;
	rkisp_idx_write(dev, ISP33_SHARP_NOISE_CURVE8, value, id, direct);
}

static void
isp_sharp_config(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp33_sharp_cfg *arg, u32 id)
{
	struct isp33_isp_params_cfg *params_rec = params_vdev->isp33_params + id;
	u32 i, value;

	value = isp3_param_read(params_vdev, ISP3X_SHARP_EN, id);
	value &= ISP33_MODULE_EN;

	value |= !!arg->bypass << 1 |
		 !!arg->local_gain_bypass << 2 |
		 !!arg->tex_est_mode << 3 |
		 !!arg->max_min_flt_mode << 4 |
		 !!arg->detail_fusion_wgt_mode << 5 |
		 !!arg->noise_calc_mode << 6 |
		 !!arg->radius_step_mode << 7 |
		 !!arg->noise_curve_mode << 8 |
		 !!arg->gain_wgt_mode << 9 |
		 !!arg->detail_lp_en << 10 |
		 (arg->debug_mode & 0x7) << 12;
	isp3_param_write(params_vdev, value, ISP3X_SHARP_EN, id);

	value = ISP_PACK_2SHORT(arg->fst_noise_scale, arg->fst_sigma_scale);
	isp3_param_write(params_vdev, value, ISP33_SHARP_TEXTURE0, id);

	value = ISP_PACK_2SHORT(arg->fst_sigma_offset, arg->fst_wgt_scale);
	isp3_param_write(params_vdev, value, ISP33_SHARP_TEXTURE1, id);

	value = (arg->tex_wgt_mode & 0x3) << 8 |
		(arg->noise_est_alpha & 0x3f) << 12;
	isp3_param_write(params_vdev, value, ISP33_SHARP_TEXTURE2, id);

	value = ISP_PACK_2SHORT(arg->sec_noise_scale, arg->sec_sigma_scale);
	isp3_param_write(params_vdev, value, ISP33_SHARP_TEXTURE3, id);

	value = ISP_PACK_2SHORT(arg->sec_sigma_offset, arg->sec_wgt_scale);
	isp3_param_write(params_vdev, value, ISP33_SHARP_TEXTURE4, id);

	value = arg->img_hpf_coeff[0] << 24;
	isp3_param_write(params_vdev, value, ISP33_SHARP_HPF_KERNEL0, id);
	value = ISP_PACK_4BYTE(arg->img_hpf_coeff[1], arg->img_hpf_coeff[2],
			       arg->img_hpf_coeff[3], arg->img_hpf_coeff[4]);
	isp3_param_write(params_vdev, value, ISP33_SHARP_HPF_KERNEL1, id);

	value = ISP_PACK_4BYTE(arg->img_hpf_coeff[5], arg->texWgt_flt_coeff0,
			       arg->texWgt_flt_coeff1, arg->texWgt_flt_coeff2);
	isp3_param_write(params_vdev, value, ISP33_SHARP_TEXFLT_KERNEL, id);

	value = arg->detail_in_alpha |
		(arg->pre_bifilt_slope_fix & 0x7ff) << 8 |
		(arg->pre_bifilt_alpha & 0x3f) << 20 |
		!!arg->fusion_wgt_min_limit << 28 |
		!!arg->fusion_wgt_max_limit << 29;
	isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL0, id);
	value = arg->detail_fusion_slope_fix;
	isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL1, id);

	value = (arg->luma_dx[6] & 0x0F) << 24 |
		(arg->luma_dx[5] & 0x0F) << 20 |
		(arg->luma_dx[4] & 0x0F) << 16 |
		(arg->luma_dx[3] & 0x0F) << 12 |
		(arg->luma_dx[2] & 0x0F) << 8 |
		(arg->luma_dx[1] & 0x0F) << 4 |
		(arg->luma_dx[0] & 0x0F);
	isp3_param_write(params_vdev, value, ISP33_SHARP_LUMA_DX, id);

	for (i = 0; i < ISP33_SHARP_Y_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->pre_bifilt_vsigma_inv[i * 2],
					arg->pre_bifilt_vsigma_inv[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_SHARP_PBF_VSIGMA0 + i * 4, id);
	}

	value = (arg->pre_bifilt_coeff0 & 0x3f) |
		(arg->pre_bifilt_coeff1 & 0x3f) << 8 |
		(arg->pre_bifilt_coeff2 & 0x3f) << 16;
	isp3_param_write(params_vdev, value, ISP33_SHARP_PBF_KERNEL, id);

	value = ISP_PACK_4BYTE(arg->hi_detail_lpf_coeff[0], arg->hi_detail_lpf_coeff[1],
			       arg->hi_detail_lpf_coeff[2], arg->hi_detail_lpf_coeff[3]);
	isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_KERNEL0, id);
	value = ISP_PACK_4BYTE(arg->hi_detail_lpf_coeff[4], arg->hi_detail_lpf_coeff[5],
			       arg->mi_detail_lpf_coeff[0], arg->mi_detail_lpf_coeff[1]);
	isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_KERNEL1, id);
	value = ISP_PACK_4BYTE(arg->mi_detail_lpf_coeff[2], arg->mi_detail_lpf_coeff[3],
			       arg->mi_detail_lpf_coeff[4], arg->mi_detail_lpf_coeff[5]);
	isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_KERNEL2, id);

	value = arg->global_gain | arg->gain_merge_alpha << 16 | arg->local_gain_scale << 24;
	isp3_param_write(params_vdev, value, ISP33_SHARP_GAIN, id);

	value = ISP_PACK_4BYTE(arg->edge_gain_max_limit, arg->edge_gain_min_limit,
			       arg->detail_gain_max_limit, arg->detail_gain_min_limit);
	isp3_param_write(params_vdev, value, ISP33_SHARP_GAIN_ADJ0, id);

	value = ISP_PACK_4BYTE(arg->hitex_gain_max_limit, arg->hitex_gain_min_limit, 0, 0);
	isp3_param_write(params_vdev, value, ISP33_SHARP_GAIN_ADJ1, id);

	value = ISP_PACK_4BYTE(arg->edge_gain_slope, arg->detail_gain_slope,
			       arg->hitex_gain_slope, 0);
	isp3_param_write(params_vdev, value, ISP33_SHARP_GAIN_ADJ2, id);

	value = (arg->edge_gain_offset & 0x3ff) |
		(arg->detail_gain_offset & 0x3ff) << 10 |
		(arg->hitex_gain_offset & 0x3ff) << 20;
	isp3_param_write(params_vdev, value, ISP33_SHARP_GAIN_ADJ3, id);

	value = ISP_PACK_2SHORT(arg->edge_gain_sigma, arg->detail_gain_sigma);
	isp3_param_write(params_vdev, value, ISP33_SHARP_GAIN_ADJ4, id);

	value = ISP_PACK_2SHORT(arg->pos_edge_wgt_scale, arg->neg_edge_wgt_scale);
	isp3_param_write(params_vdev, value, ISP33_SHARP_EDGE0, id);

	value = ISP_PACK_4BYTE(arg->pos_edge_strg, arg->neg_edge_strg,
			       arg->overshoot_alpha, arg->undershoot_alpha);
	isp3_param_write(params_vdev, value, ISP33_SHARP_EDGE1, id);

	for (i = 0; i < ISP33_SHARP_EDGE_KERNEL_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->edge_lpf_coeff[i * 4],
				       arg->edge_lpf_coeff[i * 4 + 1],
				       arg->edge_lpf_coeff[i * 4 + 2],
				       arg->edge_lpf_coeff[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_SHARP_EDGE_KERNEL0 + i * 4, id);
	}
	value = ISP_PACK_4BYTE(arg->edge_lpf_coeff[i * 4], arg->edge_lpf_coeff[i * 4 + 1], 0, 0);
	isp3_param_write(params_vdev, value, ISP33_SHARP_EDGE_KERNEL2, id);

	for (i = 0; i < ISP33_SHARP_EDGE_WGT_NUM / 3; i++) {
		value = (arg->edge_wgt_val[i * 3] & 0x3ff) |
			(arg->edge_wgt_val[i * 3 + 1] & 0x3ff) << 10 |
			(arg->edge_wgt_val[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_SHARP_EDGE_WGT_VAL0 + i * 4, id);
	}
	value = (arg->edge_wgt_val[i * 3] & 0x3ff) |
		(arg->edge_wgt_val[i * 3 + 1] & 0x3ff) << 10;
	isp3_param_write(params_vdev, value, ISP33_SHARP_EDGE_WGT_VAL5, id);

	for (i = 0; i < ISP33_SHARP_LUMA_STRG_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->luma2strg[i * 4], arg->luma2strg[i * 4 + 1],
				       arg->luma2strg[i * 4 + 2], arg->luma2strg[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_SHARP_LUMA_ADJ_STRG0 + i * 4, id);
	}

	value = ISP_PACK_2SHORT(arg->center_x, arg->center_y);
	isp3_param_write(params_vdev, value, ISP33_SHARP_CENTER, id);

	value = ISP_PACK_2SHORT(arg->flat_max_limit, arg->edge_min_limit);
	isp3_param_write(params_vdev, value, ISP33_SHARP_OUT_LIMIT, id);

	value = arg->tex_x_inv_fix0;
	isp3_param_write(params_vdev, value, ISP33_SHARP_TEX_X_INV_FIX0, id);
	value = arg->tex_x_inv_fix1;
	isp3_param_write(params_vdev, value, ISP33_SHARP_TEX_X_INV_FIX1, id);
	value = arg->tex_x_inv_fix2;
	isp3_param_write(params_vdev, value, ISP33_SHARP_TEX_X_INV_FIX2, id);

	value = (arg->tex2detail_strg[0] & 0x3ff) |
		(arg->tex2detail_strg[1] & 0x3ff) << 10 |
		(arg->tex2detail_strg[2] & 0x3ff) << 20;
	isp3_param_write(params_vdev, value, ISP33_SHARP_LOCAL_STRG0, id);
	value = (arg->tex2detail_strg[3] & 0x3ff) |
		(arg->tex2loss_tex_in_hinr_strg[0] & 0x3ff) << 10 |
		(arg->tex2loss_tex_in_hinr_strg[1] & 0x3ff) << 20;
	isp3_param_write(params_vdev, value, ISP33_SHARP_LOCAL_STRG1, id);
	value = (arg->tex2loss_tex_in_hinr_strg[2] & 0x3ff) |
		(arg->tex2loss_tex_in_hinr_strg[3] & 0x3ff) << 10;
	isp3_param_write(params_vdev, value, ISP33_SHARP_LOCAL_STRG2, id);

	for (i = 0; i < ISP33_SHARP_CONTRAST_STRG_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->contrast2pos_strg[i * 4],
				       arg->contrast2pos_strg[i * 4 + 1],
				       arg->contrast2pos_strg[i * 4 + 2],
				       arg->contrast2pos_strg[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_SCALE_TAB0 + i * 4, id);
		value = ISP_PACK_4BYTE(arg->contrast2neg_strg[i * 4],
				       arg->contrast2neg_strg[i * 4 + 1],
				       arg->contrast2neg_strg[i * 4 + 2],
				       arg->contrast2neg_strg[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_SCALE_TAB3 + i * 4, id);
	}
	value = arg->contrast2pos_strg[i * 4] | arg->pos_detail_strg << 8;
	isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_SCALE_TAB2, id);
	value = arg->contrast2neg_strg[i * 4] | arg->neg_detail_strg << 8;
	isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_SCALE_TAB5, id);

	for (i = 0; i < ISP33_SHARP_TEX_CLIP_NUM / 3; i++) {
		value = (arg->tex2detail_pos_clip[i * 3] & 0x3ff) |
			(arg->tex2detail_pos_clip[i * 3 + 1] & 0x3ff) << 10 |
			(arg->tex2detail_pos_clip[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_TEX_CLIP0 + i * 4, id);
		value = (arg->tex2detail_neg_clip[i * 3] & 0x3ff) |
			(arg->tex2detail_neg_clip[i * 3 + 1] & 0x3ff) << 10 |
			(arg->tex2detail_neg_clip[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_TEX_CLIP3 + i * 4, id);

		value = (arg->tex2grain_pos_clip[i * 3] & 0x3ff) |
			(arg->tex2grain_pos_clip[i * 3 + 1] & 0x3ff) << 10 |
			(arg->tex2grain_pos_clip[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_SHARP_GRAIN_TEX_CLIP0 + i * 4, id);
		value = (arg->tex2grain_neg_clip[i * 3] & 0x3ff) |
			(arg->tex2grain_neg_clip[i * 3 + 1] & 0x3ff) << 10 |
			(arg->tex2grain_neg_clip[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_SHARP_GRAIN_TEX_CLIP3 + i * 4, id);
	}

	for (i = 0; i < ISP33_SHARP_LUM_CLIP_NUM / 3; i++) {
		value = (arg->luma2detail_pos_clip[i * 3] & 0x3ff) |
			(arg->luma2detail_pos_clip[i * 3 + 1] & 0x3ff) << 10 |
			(arg->luma2detail_pos_clip[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_LUMA_CLIP0 + i * 4, id);

		value = (arg->luma2detail_neg_clip[i * 3] & 0x3ff) |
			(arg->luma2detail_neg_clip[i * 3 + 1] & 0x3ff) << 10 |
			(arg->luma2detail_neg_clip[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_LUMA_CLIP3 + i * 4, id);
	}
	value = (arg->luma2detail_pos_clip[i * 3] & 0x3ff) |
		(arg->luma2detail_pos_clip[i * 3 + 1] & 0x3ff) << 10;
	isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_LUMA_CLIP2, id);
	value = (arg->luma2detail_neg_clip[i * 3] & 0x3ff) |
		(arg->luma2detail_neg_clip[i * 3 + 1] & 0x3ff) << 10;
	isp3_param_write(params_vdev, value, ISP33_SHARP_DETAIL_LUMA_CLIP5, id);

	value = arg->grain_strg;
	isp3_param_write(params_vdev, value, ISP33_SHARP_GRAIN_STRG, id);

	for (i = 0; i < ISP33_SHARP_HUE_NUM / 3; i++) {
		value = (arg->hue2strg[i * 3] & 0x3ff) |
			(arg->hue2strg[i * 3 + 1] & 0x3ff) << 10 |
			(arg->hue2strg[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_SHARP_HUE_ADJ_TAB0 + i * 4, id);
	}

	for (i = 0; i < ISP33_SHARP_DISATANCE_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->distance2strg[i * 4],
				       arg->distance2strg[i * 4 + 1],
				       arg->distance2strg[i * 4 + 2],
				       arg->distance2strg[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP33_SHARP_DISATANCE_ADJ0 + i * 4, id);
	}
	value = ISP_PACK_4BYTE(arg->distance2strg[i * 4],
			       arg->distance2strg[i * 4 + 1],
			       arg->distance2strg[i * 4 + 2], 0);
	isp3_param_write(params_vdev, value, ISP33_SHARP_DISATANCE_ADJ2, id);

	for (i = 0; i < ISP33_SHARP_HITEX_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->hi_tex_threshold[i * 2],
					arg->hi_tex_threshold[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_SHARP_NOISE_SIGMA0 + i * 4, id);
	}
	value = arg->hi_tex_threshold[i * 2];
	isp3_param_write(params_vdev, value, ISP33_SHARP_NOISE_SIGMA4, id);

	value = arg->loss_tex_in_hinr_strg;
	isp3_param_write(params_vdev, value, ISP33_SHARP_LOSSTEXINHINR_STRG, id);

	for (i = 0; i < ISP33_SHARP_NOISE_CURVE_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->noise_curve_ext[i * 2],
					arg->noise_curve_ext[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_SHARP_NOISE_CURVE0 + i * 4, id);
	}
	value = (arg->noise_curve_ext[i * 2] & 0x7ff) |
		arg->noise_count_thred_ratio << 12 |
		arg->noise_clip_scale << 20;
	isp3_param_write(params_vdev, value, ISP33_SHARP_NOISE_CURVE8, id);

	value = ISP_PACK_2SHORT(arg->noise_clip_min_limit, arg->noise_clip_max_limit);
	isp3_param_write(params_vdev, value, ISP33_SHARP_NOISE_CLIP, id);

	/* SHARP_NOISE_CURVE read back is not the config value, need to save */
	isp_sharp_cfg_noise_curve(params_vdev, arg, id, false);
	memcpy(&params_rec->others.sharp_cfg, arg, sizeof(struct isp33_sharp_cfg));
}

static void
isp_sharp_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP3X_SHARP_EN, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en) {
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1, ISP32_SHP_FST_FRAME, id);
		val |= ISP33_MODULE_EN;
	} else {
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	}
	isp3_param_write(params_vdev, val, ISP3X_SHARP_EN, id);
}

static void
isp_bay3d_config(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp33_bay3d_cfg *arg, u32 id)
{
	u32 i, value, ctrl;

	ctrl = isp3_param_read(params_vdev, ISP33_BAY3D_CTRL0, id);
	if (ctrl & BIT(1) && !arg->bypass_en)
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1, ISP3X_RAW3D_FST_FRAME, id);
	ctrl &= ISP33_MODULE_EN;

	ctrl |= !!arg->motion_est_en << 8 |
		 (arg->out_use_pre_mode & 0x7) << 5 |
		 !!arg->iirsparse_en << 2 |
		 !!arg->bypass_en << 1;
	isp3_param_write(params_vdev, ctrl, ISP33_BAY3D_CTRL0, id);

	value = !!arg->md_wgt_out_en << 25 |
		!!arg->cur_spnr_out_en << 22 |
		!!arg->md_only_lo_en << 21 |
		!!arg->pre_spnr_out_en << 20 |
		(arg->lo_mge_wgt_mode & 0x3) << 16 |
		!!arg->lo_detection_bypass_en << 15 |
		!!arg->sig_hfilt_en << 13 |
		!!arg->lo_diff_hfilt_en << 12 |
		!!arg->lo_wgt_hfilt_en << 11 |
		!!arg->lpf_lo_bypass_en << 10 |
		!!arg->lo_diff_vfilt_bypass_en << 9 |
		!!arg->lpf_hi_bypass_en << 8 |
		!!arg->motion_detect_bypass_en << 7 |
		!!arg->pre_pix_out_mode << 6 |
		!!arg->md_large_lo_md_wgt_bypass_en << 5 |
		!!arg->md_large_lo_gauss_filter_bypass_en << 4 |
		!!arg->md_large_lo_min_filter_bypass_en << 3 |
		!!arg->md_large_lo_use_mode << 2 |
		!!arg->tnrsigma_curve_double_en << 1 |
		!!arg->transf_bypass_en;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_CTRL1, id);

	value = !!arg->pre_lo_avg_lp_en << 22 |
		!!arg->pre_hi_bf_lp_en << 21 |
		!!arg->pre_hi_gic_lp_en << 20 |
		!!arg->pre_spnr_lo_filter_rb_wgt_mode << 15 |
		!!arg->pre_spnr_hi_filter_rb_wgt_mode << 14 |
		!!arg->pre_spnr_lo_filter_wgt_mode << 13 |
		!!arg->pre_spnr_hi_filter_wgt_mode << 12 |
		!!arg->pre_spnr_hi_noise_ctrl_en << 11 |
		!!arg->pre_spnr_sigma_idx_filt_mode << 10 |
		!!arg->pre_spnr_sigma_idx_filt_bypass_en << 9 |
		!!arg->pre_spnr_hi_guide_filter_bypass_en << 8 |
		!!arg->pre_spnr_sigma_curve_double_en << 7 |
		!!arg->pre_spnr_hi_filter_bypass_en << 6 |
		!!arg->pre_spnr_lo_filter_bypass_en << 5 |
		!!arg->spnr_presigma_use_en << 4 |
		!!arg->pre_spnr_hi_filter_gic_enhance_en << 3 |
		!!arg->pre_spnr_hi_filter_gic_en << 2 |
		!!arg->cur_spnr_filter_bypass_en;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_CTRL2, id);

	value = (arg->sigma_calc_mge_wgt_hdr_sht_thred & 0x3f) << 24 |
		(arg->mge_wgt_hdr_sht_thred & 0x3f) << 16 |
		(arg->wgt_last_mode & 0x3) << 3 |
		!!arg->ww_mode << 2 |
		!!arg->wgt_cal_mode << 1 |
		!!arg->transf_mode;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_CTRL3, id);

	value = arg->itransf_mode_offset << 16 |
		!!arg->transf_mode_scale << 15 |
		(arg->transf_mode_offset & 0x1fff);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_TRANS0, id);

	value = arg->transf_data_max_limit;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_TRANS1, id);

	value = arg->pre_sig_ctrl_scl;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_CURHI_SIGSCL, id);

	value = arg->pre_hi_guide_out_wgt;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_CURHI_SIGOF, id);

	value = arg->cur_spnr_filter_coeff[0] |
		arg->cur_spnr_filter_coeff[1] << 8 |
		arg->cur_spnr_filter_coeff[2] << 16;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_CURHISPW0, id);
	value = arg->cur_spnr_filter_coeff[3] |
		arg->cur_spnr_filter_coeff[4] << 8 |
		arg->cur_spnr_filter_coeff[5] << 16;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_CURHISPW1, id);

	for (i = 0; i < ISP33_BAY3D_XY_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->pre_spnr_luma2sigma_x[i * 2],
					arg->pre_spnr_luma2sigma_x[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_BAY3D_IIRSX0 + i * 4, id);
		value = ISP_PACK_2SHORT(arg->pre_spnr_luma2sigma_y[i * 2],
					arg->pre_spnr_luma2sigma_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_BAY3D_IIRSY0 + i * 4, id);
	}

	value = arg->pre_spnr_hi_sigma_scale;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PREHI_SIGSCL, id);

	value = arg->pre_spnr_hi_wgt_calc_scale;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PREHI_WSCL, id);

	value = arg->pre_spnr_hi_filter_wgt_min_limit;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PREHIWMM, id);

	value = arg->pre_spnr_sigma_hdr_sht_offset << 24 |
		arg->pre_spnr_sigma_offset << 16 |
		arg->pre_spnr_hi_filter_out_wgt;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PREHISIGOF, id);

	value = ISP_PACK_2SHORT(arg->pre_spnr_sigma_scale,
				arg->pre_spnr_sigma_hdr_sht_scale);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PREHISIGSCL, id);

	value = arg->pre_spnr_hi_filter_coeff[0] |
		arg->pre_spnr_hi_filter_coeff[1] << 8 |
		arg->pre_spnr_hi_filter_coeff[2] << 16;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PREHISPW0, id);
	value = arg->pre_spnr_hi_filter_coeff[3] |
		arg->pre_spnr_hi_filter_coeff[4] << 8 |
		arg->pre_spnr_hi_filter_coeff[5] << 16;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PREHISPW1, id);

	value = arg->pre_spnr_lo_sigma_scale;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PRELOSIGCSL, id);

	value = arg->pre_spnr_lo_wgt_calc_scale << 16;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PRELOSIGOF, id);

	value = arg->pre_spnr_hi_noise_ctrl_offset << 16 |
		arg->pre_spnr_hi_noise_ctrl_scale;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PREHI_NRCT, id);

	for (i = 0; i < ISP33_BAY3D_TNRSIG_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->tnr_luma2sigma_x[i * 2],
					arg->tnr_luma2sigma_x[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_BAY3D_TNRSX0 + i * 4, id);
		value = ISP_PACK_2SHORT(arg->tnr_luma2sigma_y[i * 2],
					arg->tnr_luma2sigma_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP33_BAY3D_TNRSY0 + i * 4, id);
	}

	for (i = 0; i < ISP33_BAY3D_LPF_COEFF_NUM / 3; i++) {
		value = (arg->lpf_hi_coeff[i * 3] & 0x3ff) |
			(arg->lpf_hi_coeff[i * 3 + 1] & 0x3ff) << 10 |
			(arg->lpf_hi_coeff[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_BAY3D_HIWD0 + i * 4, id);
		value = (arg->lpf_lo_coeff[i * 3] & 0x3ff) |
			(arg->lpf_lo_coeff[i * 3 + 1] & 0x3ff) << 10 |
			(arg->lpf_lo_coeff[i * 3 + 2] & 0x3ff) << 20;
		isp3_param_write(params_vdev, value, ISP33_BAY3D_LOWD0 + i * 4, id);
	}

	value = ISP_PACK_4BYTE(arg->sigma_idx_filt_coeff[0],
			       arg->sigma_idx_filt_coeff[1],
			       arg->sigma_idx_filt_coeff[2],
			       arg->sigma_idx_filt_coeff[3]);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_GF3, id);
	value = arg->sigma_idx_filt_coeff[4] |
		arg->sigma_idx_filt_coeff[5] << 8 |
		arg->lo_wgt_cal_first_line_sigma_scale << 16;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_GF4, id);

	value = (arg->lo_diff_first_line_scale & 0x3f) << 22 |
		(arg->sig_first_line_scale & 0x3f) << 16 |
		(arg->lo_wgt_vfilt_wgt & 0x1f) << 5 |
		(arg->lo_diff_vfilt_wgt & 0x1f);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_VIIR, id);

	value = ISP_PACK_2SHORT(arg->lo_wgt_cal_offset,
				arg->lo_wgt_cal_scale);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_LFSCL, id);

	value = ISP_PACK_2SHORT(arg->lo_wgt_cal_max_limit,
				arg->mode0_base_ratio);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_LFSCLTH, id);

	value = ISP_PACK_2SHORT(arg->lo_diff_wgt_cal_offset,
				arg->lo_diff_wgt_cal_scale);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_DSWGTSCL, id);

	value = ISP_PACK_2SHORT(arg->lo_mge_pre_wgt_offset,
				arg->lo_mge_pre_wgt_scale);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGTLASTSCL, id);

	value = ISP_PACK_2SHORT(arg->mode0_lo_wgt_scale,
				arg->mode0_lo_wgt_hdr_sht_scale);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGTSCL0, id);

	value = ISP_PACK_2SHORT(arg->mode1_lo_wgt_scale,
				arg->mode1_lo_wgt_hdr_sht_scale);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGTSCL1, id);

	value = ISP_PACK_2SHORT(arg->mode1_wgt_scale,
				arg->mode1_wgt_hdr_sht_scale);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGTSCL2, id);

	value = ISP_PACK_2SHORT(arg->mode1_lo_wgt_offset,
				arg->mode1_lo_wgt_hdr_sht_offset);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGTOFF, id);

	value = (arg->mode1_wgt_offset & 0xfff) << 20 |
		(arg->mode1_wgt_min_limit & 0x3ff) << 10 |
		(arg->auto_sigma_count_wgt_thred & 0x3ff);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGT1OFF, id);

	value = arg->tnr_out_sigma_sq;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_SIGORG, id);

	value = ISP_PACK_2SHORT(arg->lo_wgt_clip_min_limit,
				arg->lo_wgt_clip_hdr_sht_min_limit);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGTLO_L, id);

	value = ISP_PACK_2SHORT(arg->lo_wgt_clip_max_limit,
				arg->lo_wgt_clip_hdr_sht_max_limit);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGTLO_H, id);

	value = ISP_PACK_2SHORT(arg->lo_pre_gg_soft_thresh_scale,
				arg->lo_pre_rb_soft_thresh_scale);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_STH_SCL, id);

	value = ISP_PACK_2SHORT(arg->lo_pre_soft_thresh_max_limit,
				arg->lo_pre_soft_thresh_min_limit);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_STH_LIMIT, id);

	value = (arg->motion_est_lo_wgt_thred & 0x3ff) << 16 |
		arg->pre_spnr_hi_wgt_min_limit << 8 |
		arg->cur_spnr_hi_wgt_min_limit;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_HIKEEP, id);

	value = arg->pix_max_limit;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PIXMAX, id);

	value = arg->sigma_num_th;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_SIGNUMTH, id);

	value = arg->gain_out_max_limit << 24 |
		(arg->out_use_hi_noise_bal_nr_strg & 0x3fffff);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_MONR, id);

	value = ISP_PACK_2SHORT(arg->sigma_scale, arg->sigma_hdr_sht_scale);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_SIGSCL, id);

	value = arg->lo_wgt_cal_first_line_vfilt_wgt << 24 |
		(arg->lo_diff_vfilt_offset & 0xfff) << 10 |
		(arg->lo_wgt_vfilt_offset & 0x3ff);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_DSOFF, id);

	value = ISP_PACK_4BYTE(arg->lo_wgt_vfilt_scale,
			       arg->lo_diff_vfilt_scale_bit,
			       arg->lo_diff_vfilt_scale,
			       arg->lo_diff_first_line_vfilt_wgt);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_DSSCL, id);

	value = (arg->motion_est_sad_vert_wgt0 & 0x3) << 28 |
		(arg->motion_est_up_mvx_cost_scale & 0x7ff) << 16 |
		arg->motion_est_up_mvx_cost_offset;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_ME0, id);

	value = (arg->motion_est_sad_vert_wgt1 & 0x3) << 28 |
		(arg->motion_est_up_left_mvx_cost_scale & 0x7ff) << 16 |
		arg->motion_est_up_left_mvx_cost_offset;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_ME1, id);

	value = (arg->motion_est_sad_vert_wgt2 & 0x3) << 28 |
		(arg->motion_est_up_right_mvx_cost_scale & 0x7ff) << 16 |
		arg->motion_est_up_right_mvx_cost_offset;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_ME2, id);

	value = arg->lo_wgt_clip_motion_max_limit;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGTMAX, id);

	value = arg->mode1_wgt_max_limit;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGT1MAX, id);

	value = ISP_PACK_2SHORT(arg->mode0_wgt_out_max_limit, arg->mode0_wgt_out_offset);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_WGTM0, id);

	value = ISP_PACK_4BYTE(arg->pre_spnr_lo_val_wgt_out_wgt,
			       arg->pre_spnr_lo_filter_out_wgt,
			       arg->pre_spnr_lo_filter_wgt_min, 0);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_PRELOWGT, id);

	value = arg->md_large_lo_md_wgt_scale << 16 |
		arg->md_large_lo_md_wgt_offset;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_MIDBIG0, id);

	value = ISP_PACK_2SHORT(arg->md_large_lo_wgt_cut_offset,
				arg->md_large_lo_wgt_add_offset);
	isp3_param_write(params_vdev, value, ISP33_BAY3D_MIDBIG1, id);

	value = arg->md_large_lo_wgt_scale;
	isp3_param_write(params_vdev, value, ISP33_BAY3D_MIDBIG2, id);

	if (params_vdev->dev->hw_dev->is_single && ctrl & ISP33_MODULE_EN)
		isp3_param_write(params_vdev, ctrl | ISP33_SELF_FORCE_UPD, ISP33_BAY3D_CTRL0, id);
}

static void
isp_bay3d_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_device *ispdev = params_vdev->dev;
	u32 value, ctrl;

	ctrl = isp3_param_read_cache(params_vdev, ISP33_BAY3D_CTRL0, id);
	if (en == !!(ctrl & ISP33_MODULE_EN))
		return;

	if (en) {
		if (!priv_val->buf_3dnr_iir.mem_priv) {
			dev_err(ispdev->dev, "no bay3d buffer available\n");
			return;
		}

		value = priv_val->bay3d_iir_size;
		isp3_param_write(params_vdev, value, ISP3X_MI_BAY3D_IIR_WR_SIZE, id);
		value = priv_val->buf_3dnr_iir.dma_addr + value * id;
		isp3_param_write(params_vdev, value, ISP3X_MI_BAY3D_IIR_WR_BASE, id);
		isp3_param_write(params_vdev, value, ISP3X_MI_BAY3D_IIR_RD_BASE, id);
		isp3_param_write(params_vdev, value, ISP33_AIISP_RD_BASE, id);
		if (priv_val->buf_gain.mem_priv) {
			value = priv_val->gain_size;
			isp3_param_write(params_vdev, value, ISP3X_MI_GAIN_WR_SIZE, id);
			isp3_param_write(params_vdev, value, ISP32_MI_RAW0_RD_SIZE, id);
			value = priv_val->buf_gain.dma_addr + value * id;
			isp3_param_write(params_vdev, value, ISP3X_MI_GAIN_WR_BASE, id);
			isp3_param_write(params_vdev, value, ISP3X_MI_RAW0_RD_BASE, id);

			value = isp3_param_read_cache(params_vdev, ISP3X_GAIN_CTRL, id);
			value |= ISP3X_GAIN_2DDR_MODE(1) | ISP3X_GAIN_2DDR_EN;
			isp3_param_write(params_vdev, value, ISP3X_GAIN_CTRL, id);
		}

		value = priv_val->bay3d_ds_size;
		isp3_param_write(params_vdev, value, ISP3X_MI_BAY3D_DS_WR_SIZE, id);
		value = priv_val->buf_3dnr_ds.dma_addr + value * id;
		isp3_param_write(params_vdev, value, ISP3X_MI_BAY3D_DS_WR_BASE, id);
		isp3_param_write(params_vdev, value, ISP3X_MI_BAY3D_DS_RD_BASE, id);

		value = priv_val->bay3d_wgt_size;
		isp3_param_write(params_vdev, value, ISP3X_MI_BAY3D_CUR_WR_SIZE, id);
		isp3_param_write(params_vdev, value, ISP32_MI_BAY3D_CUR_RD_SIZE, id);
		value = priv_val->buf_3dnr_wgt.dma_addr + value * id;
		isp3_param_write(params_vdev, value, ISP3X_MI_BAY3D_CUR_WR_BASE, id);
		isp3_param_write(params_vdev, value, ISP3X_MI_BAY3D_CUR_RD_BASE, id);

		ctrl |= ISP33_MODULE_EN;
		isp3_param_write(params_vdev, ctrl, ISP33_BAY3D_CTRL0, id);

		value = ISP3X_BAY3D_IIR_WR_AUTO_UPD | ISP3X_BAY3D_CUR_WR_AUTO_UPD |
			ISP3X_BAY3D_DS_WR_AUTO_UPD | ISP3X_BAY3D_IIRSELF_UPD |
			ISP3X_BAY3D_CURSELF_UPD | ISP3X_BAY3D_DSSELF_UPD |
			ISP3X_BAY3D_RDSELF_UPD;
		if (priv_val->buf_gain.mem_priv)
			value |= ISP3X_GAIN_WR_AUTO_UPD | ISP3X_GAINSELF_UPD;
		isp3_param_set_bits(params_vdev, MI_WR_CTRL2, value, id);

		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1, ISP3X_RAW3D_FST_FRAME, id);
	} else {
		ctrl &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
		isp3_param_write(params_vdev, ctrl, ISP33_BAY3D_CTRL0, id);
	}
}

static void
isp_gain_config(struct rkisp_isp_params_vdev *params_vdev,
		const struct isp3x_gain_cfg *arg, u32 id)
{
	u32 val;

	val = arg->g0;
	isp3_param_write(params_vdev, val, ISP3X_GAIN_G0, id);
	val = ISP_PACK_2SHORT(arg->g1, arg->g2);
	isp3_param_write(params_vdev, val, ISP3X_GAIN_G1_G2, id);
}

static void
isp_gain_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP3X_GAIN_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_GAIN_CTRL, id);
}

static void
isp_cac_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp33_cac_cfg *arg, u32 id)
{
	u32 i, val, ctrl;

	ctrl = isp3_param_read(params_vdev, ISP3X_CAC_CTRL, id);
	ctrl &= ISP33_MODULE_EN;
	ctrl |= !!arg->bypass_en << 1 |
		!!arg->edge_detect_en << 2 |
		!!arg->neg_clip0_en << 3 |
		!!arg->wgt_color_en << 5;
	isp3_param_write(params_vdev, ctrl, ISP3X_CAC_CTRL, id);

	val = arg->psf_table_fix_bit;
	isp3_param_write(params_vdev, val, ISP3X_CAC_PSF_PARA, id);

	val = arg->hi_drct_ratio;
	isp3_param_write(params_vdev, val, ISP33_CAC_HIGH_DIRECT, id);

	val = arg->over_expo_thred;
	isp3_param_write(params_vdev, val, ISP33_CAC_OVER_EXPO0, id);

	val = arg->over_expo_adj;
	isp3_param_write(params_vdev, val, ISP33_CAC_OVER_EXPO1, id);

	val = arg->flat_thred | arg->flat_offset << 16;
	isp3_param_write(params_vdev, val, ISP33_CAC_FLAT, id);

	val = (arg->chroma_lo_flt_coeff0 & 0x7) |
	      (arg->chroma_lo_flt_coeff1 & 0x7) << 4 |
	      (arg->color_lo_flt_coeff0 & 0x7) << 8 |
	      (arg->color_lo_flt_coeff1 & 0x7) << 12;
	isp3_param_write(params_vdev, val, ISP33_CAC_GAUSS_COEFF, id);

	val = ISP_PACK_2SHORT(arg->search_range_ratio, arg->residual_chroma_ratio);
	isp3_param_write(params_vdev, val, ISP33_CAC_RATIO, id);

	val = arg->wgt_color_b_min_thred;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_COLOR_B, id);
	val = arg->wgt_color_r_min_thred;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_COLOR_R, id);

	val = arg->wgt_color_b_slope;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_COLOR_SLOPE_B, id);
	val = arg->wgt_color_r_slope;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_COLOR_SLOPE_R, id);

	val = arg->wgt_color_min_luma;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_COLOR_LUMA0, id);
	val = arg->wgt_color_luma_slope;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_COLOR_LUMA1, id);

	val = arg->wgt_over_expo_min_thred;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_OVER_EXPO0, id);
	val = arg->wgt_over_expo_slope;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_OVER_EXPO1, id);

	val = arg->wgt_contrast_min_thred;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_CONTRAST0, id);
	val = arg->wgt_contrast_slope;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_CONTRAST1, id);
	val = arg->wgt_contrast_offset;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_CONTRAST2, id);

	val = arg->wgt_dark_thed;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_DARK_AREA0, id);
	val = arg->wgt_dark_slope;
	isp3_param_write(params_vdev, val, ISP33_CAC_WGT_DARK_AREA1, id);

	for (i = 0; i < ISP33_CAC_PSF_NUM / 4; i++) {
		val = ISP_PACK_4BYTE(arg->psf_b_ker[i * 4], arg->psf_b_ker[i * 4 + 1],
				     arg->psf_b_ker[i * 4 + 2], arg->psf_b_ker[i * 4 + 3]);
		isp3_param_write(params_vdev, val, ISP33_CAC_PSF_B0 + i * 4, id);
		val = ISP_PACK_4BYTE(arg->psf_r_ker[i * 4], arg->psf_r_ker[i * 4 + 1],
				     arg->psf_r_ker[i * 4 + 2], arg->psf_r_ker[i * 4 + 3]);
		isp3_param_write(params_vdev, val, ISP33_CAC_PSF_R0 + i * 4, id);
	}
	val = ISP_PACK_4BYTE(arg->psf_b_ker[i * 4], arg->psf_b_ker[i * 4 + 1],
			     arg->psf_b_ker[i * 4 + 2], 0);
	isp3_param_write(params_vdev, val, ISP33_CAC_PSF_B2, id);
	val = ISP_PACK_4BYTE(arg->psf_r_ker[i * 4], arg->psf_r_ker[i * 4 + 1],
			     arg->psf_r_ker[i * 4 + 2], 0);
	isp3_param_write(params_vdev, val, ISP33_CAC_PSF_R2, id);
}

static void
isp_cac_enable(struct rkisp_isp_params_vdev *params_vdev, bool en, u32 id)
{
	u32 val = isp3_param_read(params_vdev, ISP3X_CAC_CTRL, id);

	if (en == !!(val & ISP33_MODULE_EN))
		return;
	if (en)
		val |= ISP33_MODULE_EN;
	else
		val &= ~(ISP33_MODULE_EN | ISP33_SELF_FORCE_UPD);
	isp3_param_write(params_vdev, val, ISP3X_CAC_CTRL, id);
}

static void
isp_csm_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp21_csm_cfg *arg, u32 id)
{
	u32 i, val;

	for (i = 0; i < ISP32_CSM_COEFF_NUM; i++) {
		if (i == 0)
			val = (arg->csm_y_offset & 0x3f) << 24 |
			      (arg->csm_c_offset & 0xff) << 16 |
			      (arg->csm_coeff[i] & 0x1ff);
		else
			val = arg->csm_coeff[i] & 0x1ff;
		isp3_param_write(params_vdev, val, ISP3X_ISP_CC_COEFF_0 + i * 4, id);
	}

	val = isp3_param_read_cache(params_vdev, ISP3X_ISP_CTRL0, id);
	val |= CIF_ISP_CTRL_ISP_CSM_Y_FULL_ENA | CIF_ISP_CTRL_ISP_CSM_C_FULL_ENA;
	isp3_param_write(params_vdev, val, ISP3X_ISP_CTRL0, id);
}

static void
isp_cgc_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp21_cgc_cfg *arg, u32 id)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP3X_ISP_CTRL0, id);
	u32 eff_ctrl, cproc_ctrl;

	params_vdev->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	val &= ~(ISP3X_SW_CGC_YUV_LIMIT | ISP3X_SW_CGC_RATIO_EN);
	if (arg->yuv_limit) {
		val |= ISP3X_SW_CGC_YUV_LIMIT;
		params_vdev->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	}
	if (arg->ratio_en)
		val |= ISP3X_SW_CGC_RATIO_EN;
	isp3_param_write(params_vdev, val, ISP3X_ISP_CTRL0, id);

	cproc_ctrl = isp3_param_read(params_vdev, ISP3X_CPROC_CTRL, id);
	if (cproc_ctrl & CIF_C_PROC_CTR_ENABLE) {
		val = CIF_C_PROC_YOUT_FULL | CIF_C_PROC_YIN_FULL | CIF_C_PROC_COUT_FULL;
		if (arg->yuv_limit)
			cproc_ctrl &= ~val;
		else
			cproc_ctrl |= val;
		isp3_param_write(params_vdev, cproc_ctrl, ISP3X_CPROC_CTRL, id);
	}

	eff_ctrl = isp3_param_read(params_vdev, ISP3X_IMG_EFF_CTRL, id);
	if (eff_ctrl & CIF_IMG_EFF_CTRL_ENABLE) {
		if (arg->yuv_limit)
			eff_ctrl &= ~CIF_IMG_EFF_CTRL_YCBCR_FULL;
		else
			eff_ctrl |= CIF_IMG_EFF_CTRL_YCBCR_FULL;
		isp3_param_write(params_vdev, eff_ctrl, ISP3X_IMG_EFF_CTRL, id);
	}
}

struct rkisp_isp_params_ops_v33 isp_params_ops_v33 = {
	.dpcc_config = isp_dpcc_config,
	.dpcc_enable = isp_dpcc_enable,
	.bls_config = isp_bls_config,
	.bls_enable = isp_bls_enable,
	.lsc_config = isp_lsc_config,
	.lsc_enable = isp_lsc_enable,
	.awbgain_config = isp_awbgain_config,
	.awbgain_enable = isp_awbgain_enable,
	.debayer_config = isp_debayer_config,
	.debayer_enable = isp_debayer_enable,
	.ccm_config = isp_ccm_config,
	.ccm_enable = isp_ccm_enable,
	.goc_config = isp_goc_config,
	.goc_enable = isp_goc_enable,
	.csm_config = isp_csm_config,
	.cproc_config = isp_cproc_config,
	.cproc_enable = isp_cproc_enable,
	.ie_enable = isp_ie_enable,
	.rawae0_config = isp_rawae0_config,
	.rawae0_enable = isp_rawae0_enable,
	.rawae3_config = isp_rawae3_config,
	.rawae3_enable = isp_rawae3_enable,
	.rawawb_config = isp_rawawb_config,
	.rawawb_enable = isp_rawawb_enable,
	.rawhst0_config = isp_rawhst0_config,
	.rawhst0_enable = isp_rawhst0_enable,
	.rawhst3_config = isp_rawhst3_config,
	.rawhst3_enable = isp_rawhst3_enable,
	.hdrmge_config = isp_hdrmge_config,
	.hdrmge_enable = isp_hdrmge_enable,
	.hdrdrc_config = isp_hdrdrc_config,
	.hdrdrc_enable = isp_hdrdrc_enable,
	.gic_config = isp_gic_config,
	.gic_enable = isp_gic_enable,
	.enh_config = isp_enh_config,
	.enh_enable = isp_enh_enable,
	.hist_config = isp_hist_config,
	.hist_enable = isp_hist_enable,
	.hsv_config = isp_hsv_config,
	.hsv_enable = isp_hsv_enable,
	.ldch_config = isp_ldch_config,
	.ldch_enable = isp_ldch_enable,
	.ynr_config = isp_ynr_config,
	.ynr_enable = isp_ynr_enable,
	.cnr_config = isp_cnr_config,
	.cnr_enable = isp_cnr_enable,
	.sharp_config = isp_sharp_config,
	.sharp_enable = isp_sharp_enable,
	.bay3d_config = isp_bay3d_config,
	.bay3d_enable = isp_bay3d_enable,
	.gain_config = isp_gain_config,
	.gain_enable = isp_gain_enable,
	.cac_config = isp_cac_config,
	.cac_enable = isp_cac_enable,
	.cgc_config = isp_cgc_config,
};

static __maybe_unused
void __isp_isr_other_config(struct rkisp_isp_params_vdev *params_vdev,
			    const struct isp33_isp_params_cfg *new_params,
			    enum rkisp_params_type type, u32 id)
{
	struct rkisp_isp_params_ops_v33 *ops = params_vdev->priv_ops;
	u64 module_cfg_update = new_params->module_cfg_update;

	if (type == RKISP_PARAMS_SHD) {
		if ((module_cfg_update & ISP33_MODULE_HDRMGE))
			ops->hdrmge_config(params_vdev, &new_params->others.hdrmge_cfg, type, id);

		if ((module_cfg_update & ISP33_MODULE_DRC))
			ops->hdrdrc_config(params_vdev, &new_params->others.drc_cfg, type, id);
		return;
	}

	v4l2_dbg(4, rkisp_debug, &params_vdev->dev->v4l2_dev,
		 "%s id:%d seq:%d module_cfg_update:0x%llx\n",
		 __func__, id, new_params->frame_id, module_cfg_update);

	if (module_cfg_update & ISP33_MODULE_DPCC)
		ops->dpcc_config(params_vdev, &new_params->others.dpcc_cfg, id);

	if (module_cfg_update & ISP33_MODULE_BLS)
		ops->bls_config(params_vdev, &new_params->others.bls_cfg, id);

	if (module_cfg_update & ISP33_MODULE_AWB_GAIN)
		ops->awbgain_config(params_vdev, &new_params->others.awb_gain_cfg, id);

	if (module_cfg_update & ISP33_MODULE_DEBAYER)
		ops->debayer_config(params_vdev, &new_params->others.debayer_cfg, id);

	if (module_cfg_update & ISP33_MODULE_CCM)
		ops->ccm_config(params_vdev, &new_params->others.ccm_cfg, id);

	if (module_cfg_update & ISP33_MODULE_GOC)
		ops->goc_config(params_vdev, &new_params->others.gammaout_cfg, id);

	/* range csm->cgc->cproc->ie */
	if (module_cfg_update & ISP33_MODULE_CSM)
		ops->csm_config(params_vdev, &new_params->others.csm_cfg, id);

	if (module_cfg_update & ISP33_MODULE_CGC)
		ops->cgc_config(params_vdev, &new_params->others.cgc_cfg, id);

	if (module_cfg_update & ISP33_MODULE_CPROC)
		ops->cproc_config(params_vdev, &new_params->others.cproc_cfg, id);

	if (module_cfg_update & ISP33_MODULE_HDRMGE)
		ops->hdrmge_config(params_vdev, &new_params->others.hdrmge_cfg, type, id);

	if (module_cfg_update & ISP33_MODULE_DRC)
		ops->hdrdrc_config(params_vdev, &new_params->others.drc_cfg, type, id);

	if (module_cfg_update & ISP33_MODULE_GIC)
		ops->gic_config(params_vdev, &new_params->others.gic_cfg, id);

	if (module_cfg_update & ISP33_MODULE_ENH)
		ops->enh_config(params_vdev, &new_params->others.enh_cfg, id);

	if (module_cfg_update & ISP33_MODULE_HIST)
		ops->hist_config(params_vdev, &new_params->others.hist_cfg, id);

	if (module_cfg_update & ISP33_MODULE_LDCH)
		ops->ldch_config(params_vdev, &new_params->others.ldch_cfg, id);

	if (module_cfg_update & ISP33_MODULE_YNR)
		ops->ynr_config(params_vdev, &new_params->others.ynr_cfg, id);

	if (module_cfg_update & ISP33_MODULE_CNR)
		ops->cnr_config(params_vdev, &new_params->others.cnr_cfg, id);

	if (module_cfg_update & ISP33_MODULE_SHARP)
		ops->sharp_config(params_vdev, &new_params->others.sharp_cfg, id);

	if (module_cfg_update & ISP33_MODULE_BAY3D)
		ops->bay3d_config(params_vdev, &new_params->others.bay3d_cfg, id);

	if (module_cfg_update & ISP33_MODULE_GAIN)
		ops->gain_config(params_vdev, &new_params->others.gain_cfg, id);

	if (module_cfg_update & ISP33_MODULE_LSC)
		ops->lsc_config(params_vdev, &new_params->others.lsc_cfg, id);

	if (module_cfg_update & ISP33_MODULE_CAC)
		ops->cac_config(params_vdev, &new_params->others.cac_cfg, id);

	if (module_cfg_update & ISP33_MODULE_HSV)
		ops->hsv_config(params_vdev, &new_params->others.hsv_cfg, id);
}

static __maybe_unused
void __isp_isr_other_en(struct rkisp_isp_params_vdev *params_vdev,
			const struct isp33_isp_params_cfg *new_params,
			enum rkisp_params_type type, u32 id)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_isp_params_ops_v33 *ops = params_vdev->priv_ops;
	u64 module_en_update = new_params->module_en_update;
	u64 mask, module_ens = new_params->module_ens;
	u32 gain_ctrl, cnr_ctrl, val;

	if (type == RKISP_PARAMS_SHD)
		return;

	mask = ISP33_MODULE_YNR | ISP33_MODULE_CNR | ISP33_MODULE_SHARP;
	if ((module_ens & mask) && ((module_ens & mask) != mask))
		dev_err(params_vdev->dev->dev, "ynr cnr sharp no enable together\n");
	v4l2_dbg(4, rkisp_debug, &params_vdev->dev->v4l2_dev,
		 "%s id:%d seq:%d module_en_update:0x%llx module_ens:0x%llx\n",
		 __func__, id, new_params->frame_id, module_en_update, module_ens);

	if (module_en_update & ISP33_MODULE_DPCC)
		ops->dpcc_enable(params_vdev, !!(module_ens & ISP33_MODULE_DPCC), id);

	if (module_en_update & ISP33_MODULE_BLS)
		ops->bls_enable(params_vdev, !!(module_ens & ISP33_MODULE_BLS), id);

	if (module_en_update & ISP33_MODULE_LSC)
		ops->lsc_enable(params_vdev, !!(module_ens & ISP33_MODULE_LSC), id);

	if (module_en_update & ISP33_MODULE_AWB_GAIN)
		ops->awbgain_enable(params_vdev, !!(module_ens & ISP33_MODULE_AWB_GAIN), id);

	if (module_en_update & ISP33_MODULE_DEBAYER)
		ops->debayer_enable(params_vdev, !!(module_ens & ISP33_MODULE_DEBAYER), id);

	if (module_en_update & ISP33_MODULE_CCM)
		ops->ccm_enable(params_vdev, !!(module_ens & ISP33_MODULE_CCM), id);

	if (module_en_update & ISP33_MODULE_GOC)
		ops->goc_enable(params_vdev, !!(module_ens & ISP33_MODULE_GOC), id);

	if (module_en_update & ISP33_MODULE_CPROC)
		ops->cproc_enable(params_vdev, !!(module_ens & ISP33_MODULE_CPROC), id);

	if (module_en_update & ISP33_MODULE_IE)
		ops->ie_enable(params_vdev, !!(module_ens & ISP33_MODULE_IE), id);

	if (module_en_update & ISP33_MODULE_HDRMGE)
		ops->hdrmge_enable(params_vdev, !!(module_ens & ISP33_MODULE_HDRMGE), id);

	if (module_en_update & ISP33_MODULE_DRC)
		ops->hdrdrc_enable(params_vdev, !!(module_ens & ISP33_MODULE_DRC), id);

	if (module_en_update & ISP33_MODULE_GIC)
		ops->gic_enable(params_vdev, !!(module_ens & ISP33_MODULE_GIC), id);

	if (module_en_update & ISP33_MODULE_ENH)
		ops->enh_enable(params_vdev, !!(module_ens & ISP33_MODULE_ENH), id);

	if (module_en_update & ISP33_MODULE_HIST)
		ops->hist_enable(params_vdev, !!(module_ens & ISP33_MODULE_HIST), id);

	if (module_en_update & ISP33_MODULE_HSV)
		ops->hsv_enable(params_vdev, !!(module_ens & ISP33_MODULE_HSV), id);

	if (module_en_update & ISP33_MODULE_LDCH)
		ops->ldch_enable(params_vdev, !!(module_ens & ISP33_MODULE_LDCH), id);

	if (module_en_update & ISP33_MODULE_YNR)
		ops->ynr_enable(params_vdev, !!(module_ens & ISP33_MODULE_YNR), id);

	if (module_en_update & ISP33_MODULE_CNR)
		ops->cnr_enable(params_vdev, !!(module_ens & ISP33_MODULE_CNR), id);

	if (module_en_update & ISP33_MODULE_SHARP)
		ops->sharp_enable(params_vdev, !!(module_ens & ISP33_MODULE_SHARP), id);

	if (module_en_update & ISP33_MODULE_BAY3D)
		ops->bay3d_enable(params_vdev, !!(module_ens & ISP33_MODULE_BAY3D), id);

	if (module_en_update & ISP33_MODULE_CAC)
		ops->cac_enable(params_vdev, !!(module_ens & ISP33_MODULE_CAC), id);

	if (module_en_update & ISP33_MODULE_GAIN ||
	    ((priv_val->buf_info_owner == RKISP_INFO2DRR_OWNER_GAIN) &&
	     !(isp3_param_read(params_vdev, ISP3X_GAIN_CTRL, id) & ISP3X_GAIN_2DDR_EN)))
		ops->gain_enable(params_vdev, !!(module_ens & ISP33_MODULE_GAIN), id);

	/* gain disable, using global gain for cnr */
	gain_ctrl = isp3_param_read_cache(params_vdev, ISP3X_GAIN_CTRL, id);
	cnr_ctrl = isp3_param_read_cache(params_vdev, ISP3X_CNR_CTRL, id);
	if (!(gain_ctrl & ISP33_MODULE_EN) && cnr_ctrl & ISP33_MODULE_EN) {
		cnr_ctrl |= BIT(1);
		isp3_param_write(params_vdev, cnr_ctrl, ISP3X_CNR_CTRL, id);
		val = isp3_param_read(params_vdev, ISP3X_CNR_EXGAIN, id) & 0x3ff;
		isp3_param_write(params_vdev, val | 0x8000, ISP3X_CNR_EXGAIN, id);
	}
}

static __maybe_unused
void __isp_isr_meas_config(struct rkisp_isp_params_vdev *params_vdev,
			   struct isp33_isp_params_cfg *new_params,
			   enum rkisp_params_type type, u32 id)
{
	struct rkisp_isp_params_ops_v33 *ops = params_vdev->priv_ops;
	u64 module_cfg_update = new_params->module_cfg_update;

	params_vdev->cur_frame_id = new_params->frame_id;
	if (type == RKISP_PARAMS_SHD)
		return;

	v4l2_dbg(4, rkisp_debug, &params_vdev->dev->v4l2_dev,
		 "%s id:%d seq:%d module_cfg_update:0x%llx\n",
		 __func__, id, new_params->frame_id, module_cfg_update);

	if (module_cfg_update & ISP33_MODULE_RAWAE0)
		ops->rawae0_config(params_vdev, &new_params->meas.rawae0, id);

	if (module_cfg_update & ISP33_MODULE_RAWAE3)
		ops->rawae3_config(params_vdev, &new_params->meas.rawae3, id);

	if (module_cfg_update & ISP33_MODULE_RAWHIST0)
		ops->rawhst0_config(params_vdev, &new_params->meas.rawhist0, id);

	if (module_cfg_update & ISP33_MODULE_RAWHIST3)
		ops->rawhst3_config(params_vdev, &new_params->meas.rawhist3, id);

	if (module_cfg_update & ISP33_MODULE_RAWAWB)
		ops->rawawb_config(params_vdev, &new_params->meas.rawawb, id);
}

static __maybe_unused
void __isp_isr_meas_en(struct rkisp_isp_params_vdev *params_vdev,
		       struct isp33_isp_params_cfg *new_params,
		       enum rkisp_params_type type, u32 id)
{
	struct rkisp_isp_params_ops_v33 *ops = params_vdev->priv_ops;
	u64 module_en_update = new_params->module_en_update;
	u64 module_ens = new_params->module_ens;

	if (type == RKISP_PARAMS_SHD)
		return;

	v4l2_dbg(4, rkisp_debug, &params_vdev->dev->v4l2_dev,
		 "%s id:%d seq:%d module_en_update:0x%llx module_ens:0x%llx\n",
		 __func__, id, new_params->frame_id, module_en_update, module_ens);

	if (module_en_update & ISP33_MODULE_RAWAE0)
		ops->rawae0_enable(params_vdev, !!(module_ens & ISP33_MODULE_RAWAE0), id);

	if (module_en_update & ISP33_MODULE_RAWAE3)
		ops->rawae3_enable(params_vdev, !!(module_ens & ISP33_MODULE_RAWAE3), id);

	if (module_en_update & ISP33_MODULE_RAWHIST0)
		ops->rawhst0_enable(params_vdev, !!(module_ens & ISP33_MODULE_RAWHIST0), id);

	if (module_en_update & ISP33_MODULE_RAWHIST3)
		ops->rawhst3_enable(params_vdev, !!(module_ens & ISP33_MODULE_RAWHIST3), id);

	if (module_en_update & ISP33_MODULE_RAWAWB)
		ops->rawawb_enable(params_vdev, !!(module_ens & ISP33_MODULE_RAWAWB), id);
}

static
void rkisp_params_cfgsram_v33(struct rkisp_isp_params_vdev *params_vdev, bool is_reset)
{
	u32 id = params_vdev->dev->unite_index;
	struct isp33_isp_params_cfg *params = params_vdev->isp33_params + id;

	if (is_reset) {
		isp_sharp_cfg_noise_curve(params_vdev, &params->others.sharp_cfg, id, true);
		params->others.enh_cfg.iir_wr = true;
		params->others.hist_cfg.iir_wr = true;
	}
	isp_enh_cfg_sram(params_vdev, &params->others.enh_cfg, true, id);
	isp_hist_cfg_sram(params_vdev, &params->others.hist_cfg, true, id);
	params->others.enh_cfg.iir_wr = false;
	params->others.hist_cfg.iir_wr = false;

	isp_lsc_matrix_cfg_sram(params_vdev, &params->others.lsc_cfg, true, id);
	isp_hsv_cfg_sram(params_vdev, &params->others.hsv_cfg, true, id);
	isp_rawawb_cfg_sram(params_vdev, &params->meas.rawawb, true, id);
	isp_rawhstbig_cfg_sram(params_vdev, &params->meas.rawhist0,
			       ISP3X_RAWHIST_LITE_BASE, true, id);
	isp_rawhstbig_cfg_sram(params_vdev, &params->meas.rawhist3,
			       ISP3X_RAWHIST_BIG1_BASE, true, id);
}

static int
rkisp_alloc_internal_buf(struct rkisp_isp_params_vdev *params_vdev,
			 const struct isp33_isp_params_cfg *new_params)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_isp_subdev *isp_sdev = &dev->isp_sdev;
	u64 module_en_update, module_ens;
	int ret;

	module_en_update = new_params->module_en_update;
	module_ens = new_params->module_ens;

	if ((module_en_update & ISP33_MODULE_BAY3D) &&
	    (module_ens & ISP33_MODULE_BAY3D)) {
		bool iirsparse_en = !!new_params->others.bay3d_cfg.iirsparse_en;
		u32 w = ALIGN(isp_sdev->in_crop.width, 16);
		u32 h = isp_sdev->in_crop.height;
		u32 hsize, vsize, val;
		bool is_alloc;

		if (dev->unite_div > ISP_UNITE_DIV1)
			w = ALIGN(isp_sdev->in_crop.width / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL, 16);
		if (dev->unite_div == ISP_UNITE_DIV4)
			h = isp_sdev->in_crop.height / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;

		if (!iirsparse_en) {
			hsize = w * 3 / 2 + w / 4;
			vsize = h / 2;
		} else {
			hsize = w;
			vsize = h;
		}
		val = ALIGN(hsize * vsize * 2, 16);
		priv_val->bay3d_iir_size = val;
		if (dev->unite_div > ISP_UNITE_DIV1)
			val *= dev->unite_div;
		is_alloc = true;
		if (priv_val->buf_3dnr_iir.mem_priv) {
			if (val > priv_val->buf_3dnr_iir.size)
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_iir);
			else
				is_alloc = false;
		}
		if (is_alloc) {
			priv_val->buf_3dnr_iir.size = val;
			ret = rkisp_alloc_buffer(dev, &priv_val->buf_3dnr_iir);
			if (ret) {
				dev_err(dev->dev, "alloc bay3d iir buf fail:%d\n", ret);
				goto err_3dnr;
			}
		}

		hsize = (w * 36 / 8 + 31) / 32 * 4;
		vsize = (h + 7) / 8;
		val = ALIGN(hsize * vsize, 16);
		priv_val->bay3d_ds_size = val;
		if (dev->unite_div > ISP_UNITE_DIV1)
			val *= dev->unite_div;
		is_alloc = true;
		if (priv_val->buf_3dnr_ds.mem_priv) {
			if (val > priv_val->buf_3dnr_ds.size)
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_ds);
			else
				is_alloc = false;
		}
		if (is_alloc) {
			priv_val->buf_3dnr_ds.size = val;
			ret = rkisp_alloc_buffer(dev, &priv_val->buf_3dnr_ds);
			if (ret) {
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_iir);
				dev_err(dev->dev, "alloc bay3d ds buf fail:%d\n", ret);
				goto err_3dnr;
			}
		}

		hsize = (((w + 31) / 32 + 1) / 2 * 2 + 3) / 4 * 4;
		vsize = (h + 31) / 32;
		val = ALIGN(hsize * vsize, 16);
		priv_val->bay3d_wgt_size = val;
		if (dev->unite_div > ISP_UNITE_DIV1)
			val *= dev->unite_div;
		is_alloc = true;
		if (priv_val->buf_3dnr_wgt.mem_priv) {
			if (val > priv_val->buf_3dnr_wgt.size)
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_wgt);
			else
				is_alloc = false;
		}
		if (is_alloc) {
			priv_val->buf_3dnr_wgt.size = val;
			ret = rkisp_alloc_buffer(dev, &priv_val->buf_3dnr_wgt);
			if (ret) {
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_iir);
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_ds);
				dev_err(dev->dev, "alloc bay3d wgt buf fail:%d\n", ret);
				goto err_3dnr;
			}
		}

		val = ALIGN(w * h / 4, 16);
		priv_val->gain_size = val;
		if (dev->unite_div > ISP_UNITE_DIV1)
			val *= dev->unite_div;
		is_alloc = dev->is_aiisp_en ? true : false;
		if (priv_val->buf_gain.mem_priv) {
			if (val > priv_val->buf_gain.size)
				rkisp_free_buffer(dev, &priv_val->buf_gain);
			else
				is_alloc = false;
		}
		if (is_alloc) {
			priv_val->buf_gain.size = val;
			if (ret) {
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_iir);
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_ds);
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_wgt);
				dev_err(dev->dev, "alloc gain buf fail:%d\n", ret);
				goto err_3dnr;
			}
		}
	}

	return 0;
err_3dnr:
	return ret;
}

static bool
rkisp_params_check_bigmode_v33(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_hw_dev *hw = params_vdev->dev->hw_dev;

	dev->multi_index = 0;
	dev->multi_mode = 0;
	if (!hw->is_single) {
		dev->is_frm_rd = true;
		dev->multi_index = dev->dev_id;
	}

	return dev->is_bigmode = false;
}

/* Not called when the camera active, thus not isr protection. */
static void
rkisp_params_first_cfg_v33(struct rkisp_isp_params_vdev *params_vdev)
{
	struct isp33_isp_params_cfg *params = params_vdev->isp33_params;
	struct rkisp_device *dev = params_vdev->dev;
	int i;

	rkisp_params_check_bigmode_v33(params_vdev);
	spin_lock(&params_vdev->config_lock);
	for (i = 0; i < dev->unite_div; i++) {
		u64 module_cfg_update = params->module_cfg_update;
		u64 module_en_update = params->module_en_update;
		u64 module_ens = params->module_ens;

		if (!module_cfg_update || !module_en_update || !module_ens)
			dev_warn(dev->dev,
				 "id:%d no first iq setting cfg_upd:%llx en_upd:%llx ens:%llx\n",
				 i, module_cfg_update, module_en_update, module_ens);

		__isp_isr_meas_config(params_vdev, params + i, RKISP_PARAMS_ALL, i);
		__isp_isr_other_config(params_vdev, params + i, RKISP_PARAMS_ALL, i);
		__isp_isr_other_en(params_vdev, params + i, RKISP_PARAMS_ALL, i);
		__isp_isr_meas_en(params_vdev, params + i, RKISP_PARAMS_ALL, i);
	}
	spin_unlock(&params_vdev->config_lock);

	if (dev->hw_dev->is_single && (dev->isp_state & ISP_START)) {
		rkisp_set_bits(dev, ISP3X_ISP_CTRL0, 0, CIF_ISP_CTRL_ISP_CFG_UPD, true);
		rkisp_clear_reg_cache_bits(dev, CIF_ISP_CTRL, CIF_ISP_CTRL_ISP_CFG_UPD);
	}
}

static void rkisp_save_first_param_v33(struct rkisp_isp_params_vdev *params_vdev, void *param)
{
	u32 size;

	if (!params_vdev->dev->is_rtt_first) {
		size = params_vdev->vdev_fmt.fmt.meta.buffersize;
		memcpy(params_vdev->isp33_params, param, size);
	} else {
		/* left and right params for unit fast case */
		size = sizeof(struct isp33_isp_params_cfg);
		memcpy(params_vdev->isp33_params, param, size);
		if (params_vdev->dev->unite_div == ISP_UNITE_DIV2)
			memcpy(params_vdev->isp33_params + 1, param, size);
	}
	rkisp_alloc_internal_buf(params_vdev, params_vdev->isp33_params);
}

static void rkisp_clear_first_param_v33(struct rkisp_isp_params_vdev *params_vdev)
{
	u32 mult = params_vdev->dev->hw_dev->unite ? ISP_UNITE_MAX : 1;
	u32 size = sizeof(struct isp33_isp_params_cfg) * mult;

	memset(params_vdev->isp33_params, 0, size);
}

static void rkisp_deinit_mesh_buf(struct rkisp_isp_params_vdev *params_vdev,
				  u64 module_id, u32 id)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_dummy_buffer *buf;
	int i;

	if (!priv_val)
		return;

	switch (module_id) {
	case ISP33_MODULE_LDCH:
		buf = priv_val->buf_ldch[id];
		break;
	default:
		return;
	}

	for (i = 0; i < ISP32_MESH_BUF_NUM; i++)
		rkisp_free_buffer(params_vdev->dev, buf + i);
}

static int rkisp_init_mesh_buf(struct rkisp_isp_params_vdev *params_vdev,
			       struct rkisp_meshbuf_size *meshsize)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_device *ispdev = params_vdev->dev;
	struct device *dev = ispdev->dev;
	struct isp2x_mesh_head *mesh_head;
	struct rkisp_dummy_buffer *buf;
	u32 mesh_w = meshsize->meas_width;
	u32 mesh_h = meshsize->meas_height;
	u32 mesh_size, buf_size;
	int i, ret, buf_cnt = meshsize->buf_cnt;
	int id = meshsize->unite_isp_id;
	bool is_alloc;

	if (!priv_val) {
		dev_err(dev, "priv_val is NULL\n");
		return -EINVAL;
	}

	switch (meshsize->module_id) {
	case ISP33_MODULE_LDCH:
		priv_val->buf_ldch_idx[id] = 0;
		buf = priv_val->buf_ldch[id];
		mesh_w = ((mesh_w + 15) / 16 + 2) / 2;
		mesh_h = (mesh_h + 7) / 8 + 1;
		mesh_size = mesh_w * 4 * mesh_h;
		break;
	default:
		return -EINVAL;
	}

	if (buf_cnt <= 0 || buf_cnt > ISP33_MESH_BUF_NUM)
		buf_cnt = ISP33_MESH_BUF_NUM;
	buf_size = PAGE_ALIGN(mesh_size + ALIGN(sizeof(struct isp2x_mesh_head), 16));
	for (i = 0; i < buf_cnt; i++) {
		buf->is_need_vaddr = true;
		buf->is_need_dbuf = true;
		buf->is_need_dmafd = true;
		is_alloc = true;
		if (buf->mem_priv) {
			if (buf_size > buf->size) {
				rkisp_free_buffer(params_vdev->dev, buf);
			} else {
				is_alloc = false;
				buf->dma_fd = dma_buf_fd(buf->dbuf, O_CLOEXEC);
				if (buf->dma_fd < 0)
					goto err;
			}
		}
		if (is_alloc) {
			buf->size = buf_size;
			ret = rkisp_alloc_buffer(params_vdev->dev, buf);
			if (ret) {
				dev_err(dev, "%s failed\n", __func__);
				goto err;
			}
			mesh_head = (struct isp2x_mesh_head *)buf->vaddr;
			mesh_head->stat = MESH_BUF_INIT;
			mesh_head->data_oft = ALIGN(sizeof(struct isp2x_mesh_head), 16);
		}
		buf++;
	}

	return 0;
err:
	rkisp_deinit_mesh_buf(params_vdev, meshsize->module_id, id);
	return -ENOMEM;
}

static void
rkisp_get_param_size_v33(struct rkisp_isp_params_vdev *params_vdev,
			 unsigned int sizes[])
{
	u32 mult = params_vdev->dev->unite_div;

	sizes[0] = sizeof(struct isp33_isp_params_cfg) * mult;
	params_vdev->vdev_fmt.fmt.meta.buffersize = sizes[0];
}

static void
rkisp_params_get_meshbuf_inf_v33(struct rkisp_isp_params_vdev *params_vdev,
				 void *meshbuf_inf)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_meshbuf_info *meshbuf = meshbuf_inf;
	struct rkisp_dummy_buffer *buf;
	int i, id = meshbuf->unite_isp_id;

	switch (meshbuf->module_id) {
	case ISP33_MODULE_LDCH:
		priv_val->buf_ldch_idx[id] = 0;
		buf = priv_val->buf_ldch[id];
		break;
	default:
		return;
	}

	for (i = 0; i < ISP33_MESH_BUF_NUM; i++) {
		if (!buf->mem_priv) {
			meshbuf->buf_fd[i] = -1;
			meshbuf->buf_size[i] = 0;
		} else {
			meshbuf->buf_fd[i] = buf->dma_fd;
			meshbuf->buf_size[i] = buf->size;
		}
		buf++;
	}
}

static int
rkisp_params_set_meshbuf_size_v33(struct rkisp_isp_params_vdev *params_vdev,
				  void *size)
{
	struct rkisp_meshbuf_size *meshsize = size;

	if (!params_vdev->dev->hw_dev->unite)
		meshsize->unite_isp_id = 0;
	return rkisp_init_mesh_buf(params_vdev, meshsize);
}

static void
rkisp_params_free_meshbuf_v33(struct rkisp_isp_params_vdev *params_vdev,
			      u64 module_id)
{
	int id;

	for (id = 0; id < params_vdev->dev->unite_div; id++)
		rkisp_deinit_mesh_buf(params_vdev, module_id, id);
}

static int
rkisp_params_info2ddr_cfg_v33(struct rkisp_isp_params_vdev *params_vdev, void *arg)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_info2ddr *cfg = arg;
	struct rkisp_dummy_buffer *buf;
	u32 reg, ctrl, mask, size, val, wsize = 0, vsize = 0;
	int i, ret;

	if (dev->is_aiisp_en) {
		dev_err(dev->dev, "%s no support for aiisp enable\n", __func__);
		return -EINVAL;
	}

	if (cfg->buf_cnt > RKISP_INFO2DDR_BUF_MAX)
		cfg->buf_cnt = RKISP_INFO2DDR_BUF_MAX;
	else if (cfg->buf_cnt == 0)
		cfg->buf_cnt = 1;
	for (val = 0; val < cfg->buf_cnt; val++)
		cfg->buf_fd[val] = -1;

	switch (cfg->owner) {
	case RKISP_INFO2DRR_OWNER_NULL:
		rkisp_clear_reg_cache_bits(dev, ISP3X_RAWAWB_CTRL,
					   ISP32_RAWAWB_2DDR_PATH_EN);
		rkisp_clear_reg_cache_bits(dev, ISP3X_GAIN_CTRL,
					   ISP3X_GAIN_2DDR_EN);
		priv_val->buf_info_owner = cfg->owner;
		return 0;
	case RKISP_INFO2DRR_OWNER_GAIN:
		ctrl = ISP3X_GAIN_2DDR_MODE(cfg->u.gain.gain2ddr_mode);
		ctrl |= ISP3X_GAIN_2DDR_EN;
		mask = ISP3X_GAIN_2DDR_MODE(3);
		reg = ISP3X_GAIN_CTRL;

		if (cfg->wsize)
			wsize = (cfg->wsize + 7) / 8;
		else
			wsize = (dev->isp_sdev.in_crop.width + 7) / 8;
		/* 0 or 3: 4x8mode, 1: 2x8 mode, 2: 1x8mode */
		val = cfg->u.gain.gain2ddr_mode;
		val = (val == 1) ? 2 : ((val == 2) ? 1 : 4);
		if (cfg->vsize)
			vsize = cfg->vsize;
		else
			vsize = dev->isp_sdev.in_crop.height / val;
		break;
	case RKISP_INFO2DRR_OWNER_AWB:
		ctrl = cfg->u.awb.awb2ddr_sel ? ISP32_RAWAWB_2DDR_PATH_DS : 0;
		ctrl |= ISP32_RAWAWB_2DDR_PATH_EN;
		mask = ISP32_RAWAWB_2DDR_PATH_DS;
		reg = ISP3X_RAWAWB_CTRL;

		val = cfg->u.awb.awb2ddr_sel ? 8 : 1;
		if (cfg->wsize)
			wsize = cfg->wsize;
		else
			wsize = dev->isp_sdev.in_crop.width * 4 / val;
		if (cfg->vsize)
			vsize = cfg->vsize;
		else
			vsize = dev->isp_sdev.in_crop.height / val;
		break;
	default:
		dev_err(dev->dev, "%s no support owner:%d\n", __func__, cfg->owner);
		return -EINVAL;
	}

	if (!wsize || !vsize) {
		dev_err(dev->dev, "%s inval wsize:%d vsize:%d\n", __func__, wsize, vsize);
		return -EINVAL;
	}

	wsize = ALIGN(wsize, 16);
	size = wsize * vsize;
	for (i = 0; i < cfg->buf_cnt; i++) {
		buf = &priv_val->buf_info[i];
		if (buf->mem_priv)
			rkisp_free_buffer(dev, buf);
		buf->size = size;
		buf->is_need_dbuf = true;
		buf->is_need_dmafd = true;
		buf->is_need_vaddr = true;
		ret = rkisp_alloc_buffer(dev, buf);
		if (ret) {
			dev_err(dev->dev, "%s alloc buf failed\n", __func__);
			goto err;
		}
		*(u32 *)buf->vaddr = RKISP_INFO2DDR_BUF_INIT;
		cfg->buf_fd[i] = buf->dma_fd;
	}
	buf = &priv_val->buf_info[0];
	isp3_param_write(params_vdev, buf->dma_addr, ISP3X_MI_GAIN_WR_BASE, 0);
	isp3_param_write(params_vdev, buf->size, ISP3X_MI_GAIN_WR_SIZE, 0);
	isp3_param_write(params_vdev, wsize, ISP3X_MI_GAIN_WR_LENGTH, 0);
	if (dev->hw_dev->is_single)
		rkisp_write(dev, ISP3X_MI_WR_CTRL2, ISP3X_GAINSELF_UPD, true);
	rkisp_set_reg_cache_bits(dev, reg, mask, ctrl);

	priv_val->buf_info_idx = 0;
	priv_val->buf_info_cnt = cfg->buf_cnt;
	priv_val->buf_info_owner = cfg->owner;

	cfg->wsize = wsize;
	cfg->vsize = vsize;
	return 0;
err:
	for (i -= 1; i >= 0; i--) {
		buf = &priv_val->buf_info[i];
		rkisp_free_buffer(dev, buf);
		cfg->buf_fd[i] = -1;
	}
	cfg->owner = RKISP_INFO2DRR_OWNER_NULL;
	cfg->buf_cnt = 0;
	return -ENOMEM;
}

static void
rkisp_params_get_bay3d_buffd_v33(struct rkisp_isp_params_vdev *params_vdev,
				 struct rkisp_bay3dbuf_info *bay3dbuf)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_dummy_buffer *buf;

	buf = &priv_val->buf_3dnr_iir;
	if (rkisp_buf_get_fd(params_vdev->dev, buf, true) < 0)
		return;
	bay3dbuf->iir_fd = buf->dma_fd;
	bay3dbuf->iir_size = buf->size;

	buf = &priv_val->buf_3dnr_ds;
	if (rkisp_buf_get_fd(params_vdev->dev, buf, true) < 0)
		return;
	bay3dbuf->u.v33.ds_fd = buf->dma_fd;
	bay3dbuf->u.v33.ds_size = buf->size;

	buf = &priv_val->buf_gain;
	if (rkisp_buf_get_fd(params_vdev->dev, buf, true) < 0)
		return;
	bay3dbuf->u.v33.gain_fd = buf->dma_fd;
	bay3dbuf->u.v33.gain_size = buf->size;
}

static void
rkisp_params_stream_stop_v33(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct rkisp_device *ispdev = params_vdev->dev;
	int i;

	rkisp_free_buffer(ispdev, &priv_val->buf_3dnr_iir);
	rkisp_free_buffer(ispdev, &priv_val->buf_3dnr_wgt);
	rkisp_free_buffer(ispdev, &priv_val->buf_3dnr_ds);
	rkisp_free_buffer(ispdev, &priv_val->buf_gain);
	for (i = 0; i < RKISP_STATS_DDR_BUF_NUM; i++)
		rkisp_free_buffer(ispdev, &ispdev->stats_vdev.stats_buf[i]);
	priv_val->buf_info_owner = 0;
	priv_val->buf_info_cnt = 0;
	priv_val->buf_info_idx = -1;
	for (i = 0; i < RKISP_INFO2DDR_BUF_MAX; i++)
		rkisp_free_buffer(ispdev, &priv_val->buf_info[i]);
}

static void
rkisp_params_fop_release_v33(struct rkisp_isp_params_vdev *params_vdev)
{
	int id;

	for (id = 0; id < params_vdev->dev->unite_div; id++)
		rkisp_deinit_mesh_buf(params_vdev, ISP33_MODULE_LDCH, id);
}

/* Not called when the camera active, thus not isr protection. */
static void
rkisp_params_disable_isp_v33(struct rkisp_isp_params_vdev *params_vdev)
{
	int i;

	params_vdev->isp33_params->module_ens = 0;
	params_vdev->isp33_params->module_en_update = ~ISP33_MODULE_FORCE;

	for (i = 0; i < params_vdev->dev->unite_div; i++) {
		__isp_isr_other_en(params_vdev, params_vdev->isp33_params, RKISP_PARAMS_ALL, i);
		__isp_isr_meas_en(params_vdev, params_vdev->isp33_params, RKISP_PARAMS_ALL, i);
	}
}

static void
module_data_abandon(struct rkisp_isp_params_vdev *params_vdev,
		    struct isp33_isp_params_cfg *params, u32 id)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;
	struct isp2x_mesh_head *mesh_head;
	int i;

	if (params->module_cfg_update & ISP33_MODULE_LDCH) {
		const struct isp32_ldch_cfg *arg = &params->others.ldch_cfg;

		for (i = 0; i < ISP32_MESH_BUF_NUM; i++) {
			if (priv_val->buf_ldch[id][i].vaddr &&
			    arg->buf_fd == priv_val->buf_ldch[id][i].dma_fd) {
				mesh_head = priv_val->buf_ldch[id][i].vaddr;
				mesh_head->stat = MESH_BUF_CHIPINUSE;
				break;
			}
		}
	}
}

static void
rkisp_params_cfg_v33(struct rkisp_isp_params_vdev *params_vdev,
		     u32 frame_id, enum rkisp_params_type type)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct isp33_isp_params_cfg *new_params = NULL;
	struct rkisp_buffer *cur_buf = params_vdev->cur_buf;
	int i;

	spin_lock(&params_vdev->config_lock);
	if (!params_vdev->streamon)
		goto unlock;

	/* get buffer by frame_id */
	while (!list_empty(&params_vdev->params) && !cur_buf) {
		cur_buf = list_first_entry(&params_vdev->params,
				struct rkisp_buffer, queue);

		new_params = (struct isp33_isp_params_cfg *)(cur_buf->vaddr[0]);
		if (new_params->frame_id < frame_id) {
			list_del(&cur_buf->queue);
			if (list_empty(&params_vdev->params))
				break;
			for (i = 0; i < dev->unite_div; i++) {
				/* update en immediately */
				if (new_params->module_en_update ||
				    (new_params->module_cfg_update & ISP33_MODULE_FORCE)) {
					__isp_isr_meas_config(params_vdev,
							      new_params, RKISP_PARAMS_ALL, i);
					__isp_isr_other_config(params_vdev,
							       new_params, RKISP_PARAMS_ALL, i);
					__isp_isr_other_en(params_vdev,
							   new_params, RKISP_PARAMS_ALL, i);
					__isp_isr_meas_en(params_vdev,
							  new_params, RKISP_PARAMS_ALL, i);
					new_params->module_cfg_update = 0;
				}
				if (new_params->module_cfg_update & ISP33_MODULE_LDCH)
					module_data_abandon(params_vdev, new_params, i);
				new_params++;
			}
			vb2_buffer_done(&cur_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
			cur_buf = NULL;
			continue;
		} else if (new_params->frame_id == frame_id) {
			list_del(&cur_buf->queue);
		} else {
			cur_buf = NULL;
		}
		break;
	}

	if (!cur_buf)
		goto unlock;

	new_params = (struct isp33_isp_params_cfg *)(cur_buf->vaddr[0]);
	for (i = 0; i < dev->unite_div; i++) {
		__isp_isr_meas_config(params_vdev, new_params, type, i);
		__isp_isr_other_config(params_vdev, new_params, type, i);
		__isp_isr_other_en(params_vdev, new_params, type, i);
		__isp_isr_meas_en(params_vdev, new_params, type, i);
		if (type != RKISP_PARAMS_IMD)
			new_params->module_cfg_update = 0;
		new_params++;
	}
	if (type != RKISP_PARAMS_IMD) {
		vb2_buffer_done(&cur_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		cur_buf = NULL;
	}

unlock:
	params_vdev->cur_buf = cur_buf;
	spin_unlock(&params_vdev->config_lock);
}

static void
rkisp_params_clear_fstflg(struct rkisp_isp_params_vdev *params_vdev)
{
	u32 value = isp3_param_read(params_vdev, ISP3X_ISP_CTRL1, 0);
	int i;

	if (params_vdev->dev->hw_dev->is_single)
		return;
	value &= (ISP3X_YNR_FST_FRAME | ISP3X_ADRC_FST_FRAME |
		  ISP3X_CNR_FST_FRAME | ISP33_GIC_FST_FRAME |
		  ISP33_ENH_FST_FRAME | ISP33_YHIST_FST_FRAME |
		  ISP3X_RAW3D_FST_FRAME | ISP32_SHP_FST_FRAME);
	for (i = 0; i < params_vdev->dev->unite_div && value; i++)
		isp3_param_clear_bits(params_vdev, ISP3X_ISP_CTRL1, value, i);
}

static void
rkisp_params_isr_v33(struct rkisp_isp_params_vdev *params_vdev,
		     u32 isp_mis)
{
	struct rkisp_device *dev = params_vdev->dev;
	u32 cur_frame_id;

	rkisp_dmarx_get_frame(dev, &cur_frame_id, NULL, NULL, true);
	if (isp_mis & CIF_ISP_V_START) {
		if (params_vdev->rdbk_times)
			params_vdev->rdbk_times--;
		if (!params_vdev->cur_buf)
			return;

		if (IS_HDR_RDBK(dev->rd_mode) && !params_vdev->rdbk_times) {
			rkisp_params_cfg_v33(params_vdev, cur_frame_id, RKISP_PARAMS_SHD);
			return;
		}
	}

	if ((isp_mis & CIF_ISP_FRAME) && !params_vdev->rdbk_times)
		rkisp_params_clear_fstflg(params_vdev);

	if ((isp_mis & CIF_ISP_FRAME) &&
	    !IS_HDR_RDBK(dev->rd_mode) && !params_vdev->rdbk_times)
		rkisp_params_cfg_v33(params_vdev, cur_frame_id + 1, RKISP_PARAMS_ALL);
}

static struct rkisp_isp_params_ops rkisp_isp_params_ops_tbl = {
	.save_first_param = rkisp_save_first_param_v33,
	.clear_first_param = rkisp_clear_first_param_v33,
	.get_param_size = rkisp_get_param_size_v33,
	.first_cfg = rkisp_params_first_cfg_v33,
	.disable_isp = rkisp_params_disable_isp_v33,
	.isr_hdl = rkisp_params_isr_v33,
	.param_cfg = rkisp_params_cfg_v33,
	.param_cfgsram = rkisp_params_cfgsram_v33,
	.get_meshbuf_inf = rkisp_params_get_meshbuf_inf_v33,
	.set_meshbuf_size = rkisp_params_set_meshbuf_size_v33,
	.free_meshbuf = rkisp_params_free_meshbuf_v33,
	.stream_stop = rkisp_params_stream_stop_v33,
	.fop_release = rkisp_params_fop_release_v33,
	.check_bigmode = rkisp_params_check_bigmode_v33,
	.info2ddr_cfg = rkisp_params_info2ddr_cfg_v33,
	.get_bay3d_buffd = rkisp_params_get_bay3d_buffd_v33,
};

int rkisp_init_params_vdev_v33(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_isp_params_val_v33 *priv_val;
	int size;

	priv_val = kzalloc(sizeof(*priv_val), GFP_KERNEL);
	if (!priv_val)
		return -ENOMEM;

	size = sizeof(struct isp33_isp_params_cfg);
	if (params_vdev->dev->hw_dev->unite)
		size *= ISP_UNITE_MAX;
	params_vdev->isp33_params = vmalloc(size);
	if (!params_vdev->isp33_params) {
		kfree(priv_val);
		return -ENOMEM;
	}

	params_vdev->priv_val = (void *)priv_val;
	params_vdev->ops = &rkisp_isp_params_ops_tbl;
	params_vdev->priv_ops = &isp_params_ops_v33;
	rkisp_clear_first_param_v33(params_vdev);
	priv_val->buf_info_owner = 0;
	priv_val->buf_info_cnt = 0;
	priv_val->buf_info_idx = -1;
	return 0;
}

void rkisp_uninit_params_vdev_v33(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_isp_params_val_v33 *priv_val = params_vdev->priv_val;

	if (params_vdev->isp33_params)
		vfree(params_vdev->isp33_params);
	kfree(priv_val);
	params_vdev->priv_val = NULL;
}
