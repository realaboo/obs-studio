/******************************************************************************
    Copyright (C) 2022 by Hugh Bailey <obs.jim@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include "obs-hevc.h"

#include "obs.h"
#include "obs-nal.h"
#include "util/array-serializer.h"
#include "util/golomb.h"


#define do_log(level, format, ...)                \
	blog(level, "[obs-hevc: ] " format, ##__VA_ARGS__)

#define warn(format, ...) do_log(LOG_WARNING, format, ##__VA_ARGS__)
#define info(format, ...) do_log(LOG_INFO, format, ##__VA_ARGS__)
#define debug(format, ...) do_log(LOG_DEBUG, format, ##__VA_ARGS__)

enum {
	OBS_HEVC_NAL_TRAIL_N = 0,
	OBS_HEVC_NAL_TRAIL_R = 1,
	OBS_HEVC_NAL_TSA_N = 2,
	OBS_HEVC_NAL_TSA_R = 3,
	OBS_HEVC_NAL_STSA_N = 4,
	OBS_HEVC_NAL_STSA_R = 5,
	OBS_HEVC_NAL_RADL_N = 6,
	OBS_HEVC_NAL_RADL_R = 7,
	OBS_HEVC_NAL_RASL_N = 8,
	OBS_HEVC_NAL_RASL_R = 9,
	OBS_HEVC_NAL_VCL_N10 = 10,
	OBS_HEVC_NAL_VCL_R11 = 11,
	OBS_HEVC_NAL_VCL_N12 = 12,
	OBS_HEVC_NAL_VCL_R13 = 13,
	OBS_HEVC_NAL_VCL_N14 = 14,
	OBS_HEVC_NAL_VCL_R15 = 15,
	OBS_HEVC_NAL_BLA_W_LP = 16,
	OBS_HEVC_NAL_BLA_W_RADL = 17,
	OBS_HEVC_NAL_BLA_N_LP = 18,
	OBS_HEVC_NAL_IDR_W_RADL = 19,
	OBS_HEVC_NAL_IDR_N_LP = 20,
	OBS_HEVC_NAL_CRA_NUT = 21,
	OBS_HEVC_NAL_RSV_IRAP_VCL22 = 22,
	OBS_HEVC_NAL_RSV_IRAP_VCL23 = 23,
	OBS_HEVC_NAL_RSV_VCL24 = 24,
	OBS_HEVC_NAL_RSV_VCL25 = 25,
	OBS_HEVC_NAL_RSV_VCL26 = 26,
	OBS_HEVC_NAL_RSV_VCL27 = 27,
	OBS_HEVC_NAL_RSV_VCL28 = 28,
	OBS_HEVC_NAL_RSV_VCL29 = 29,
	OBS_HEVC_NAL_RSV_VCL30 = 30,
	OBS_HEVC_NAL_RSV_VCL31 = 31,
	OBS_HEVC_NAL_VPS = 32,
	OBS_HEVC_NAL_SPS = 33,
	OBS_HEVC_NAL_PPS = 34,
	OBS_HEVC_NAL_AUD = 35,
	OBS_HEVC_NAL_EOS_NUT = 36,
	OBS_HEVC_NAL_EOB_NUT = 37,
	OBS_HEVC_NAL_FD_NUT = 38,
	OBS_HEVC_NAL_SEI_PREFIX = 39,
	OBS_HEVC_NAL_SEI_SUFFIX = 40,
};

#define FFMAX(a, b) ((a) > (b) ? (a) : (b))
#define FFMIN(a, b) ((a) > (b) ? (b) : (a))

#define MAX_SPATIAL_SEGMENTATION 4096 // max. value of u(12) field

typedef struct HEVCDecoderConfigurationRecord {
	uint8_t configurationVersion;
	uint8_t general_profile_space;
	uint8_t general_tier_flag;
	uint8_t general_profile_idc;
	uint32_t general_profile_compatibility_flags;
	uint64_t general_constraint_indicator_flags;
	uint8_t general_level_idc;
	uint16_t min_spatial_segmentation_idc;
	uint8_t parallelismType;
	uint8_t chromaFormat;
	uint8_t bitDepthLumaMinus8;
	uint8_t bitDepthChromaMinus8;
	uint16_t avgFrameRate;
	uint8_t constantFrameRate;
	uint8_t numTemporalLayers;
	uint8_t temporalIdNested;
	uint8_t lengthSizeMinusOne;
} HEVCDecoderConfigurationRecord;

typedef struct HVCCProfileTierLevel {
	uint8_t profile_space;
	uint8_t tier_flag;
	uint8_t profile_idc;
	uint32_t profile_compatibility_flags;
	uint64_t constraint_indicator_flags;
	uint8_t level_idc;
} HVCCProfileTierLevel;

enum {
	// 7.4.3.1: vps_max_layers_minus1 is in [0, 62].
	HEVC_MAX_LAYERS = 63,
	// 7.4.3.1: vps_max_sub_layers_minus1 is in [0, 6].
	HEVC_MAX_SUB_LAYERS = 7,
	// 7.4.3.1: vps_num_layer_sets_minus1 is in [0, 1023].
	HEVC_MAX_LAYER_SETS = 1024,

	// 7.4.2.1: vps_video_parameter_set_id is u(4).
	HEVC_MAX_VPS_COUNT = 16,
	// 7.4.3.2.1: sps_seq_parameter_set_id is in [0, 15].
	HEVC_MAX_SPS_COUNT = 16,
	// 7.4.3.3.1: pps_pic_parameter_set_id is in [0, 63].
	HEVC_MAX_PPS_COUNT = 64,

	// A.4.2: MaxDpbSize is bounded above by 16.
	HEVC_MAX_DPB_SIZE = 16,
	// 7.4.3.1: vps_max_dec_pic_buffering_minus1[i] is in [0, MaxDpbSize - 1].
	HEVC_MAX_REFS = HEVC_MAX_DPB_SIZE,

	// 7.4.3.2.1: num_short_term_ref_pic_sets is in [0, 64].
	HEVC_MAX_SHORT_TERM_REF_PIC_SETS = 64,
	// 7.4.3.2.1: num_long_term_ref_pics_sps is in [0, 32].
	HEVC_MAX_LONG_TERM_REF_PICS = 32,

	// A.3: all profiles require that CtbLog2SizeY is in [4, 6].
	HEVC_MIN_LOG2_CTB_SIZE = 4,
	HEVC_MAX_LOG2_CTB_SIZE = 6,

	// E.3.2: cpb_cnt_minus1[i] is in [0, 31].
	HEVC_MAX_CPB_CNT = 32,

	// A.4.1: in table A.6 the highest level allows a MaxLumaPs of 35 651 584.
	HEVC_MAX_LUMA_PS = 35651584,
	// A.4.1: pic_width_in_luma_samples and pic_height_in_luma_samples are
	// constrained to be not greater than sqrt(MaxLumaPs * 8).  Hence height/
	// width are bounded above by sqrt(8 * 35651584) = 16888.2 samples.
	HEVC_MAX_WIDTH = 16888,
	HEVC_MAX_HEIGHT = 16888,

	// A.4.1: table A.6 allows at most 22 tile rows for any level.
	HEVC_MAX_TILE_ROWS = 22,
	// A.4.1: table A.6 allows at most 20 tile columns for any level.
	HEVC_MAX_TILE_COLUMNS = 20,

	// A.4.2: table A.6 allows at most 600 slice segments for any level.
	HEVC_MAX_SLICE_SEGMENTS = 600,

	// 7.4.7.1: in the worst case (tiles_enabled_flag and
	// entropy_coding_sync_enabled_flag are both set), entry points can be
	// placed at the beginning of every Ctb row in every tile, giving an
	// upper bound of (num_tile_columns_minus1 + 1) * PicHeightInCtbsY - 1.
	// Only a stream with very high resolution and perverse parameters could
	// get near that, though, so set a lower limit here with the maximum
	// possible value for 4K video (at most 135 16x16 Ctb rows).
	HEVC_MAX_ENTRY_POINT_OFFSETS = HEVC_MAX_TILE_COLUMNS * 135,
};

bool obs_hevc_keyframe(const uint8_t *data, size_t size)
{
	const uint8_t *nal_start, *nal_end;
	const uint8_t *end = data + size;

	nal_start = obs_nal_find_startcode(data, end);
	while (true) {
		while (nal_start < end && !*(nal_start++))
			;

		if (nal_start == end)
			break;

		const uint8_t type = (nal_start[0] & 0x7F) >> 1;

		if (type <= OBS_HEVC_NAL_RSV_IRAP_VCL23)
			return type >= OBS_HEVC_NAL_BLA_W_LP;

		nal_end = obs_nal_find_startcode(nal_start, end);
		nal_start = nal_end;
	}

	return false;
}

static int compute_hevc_keyframe_priority(const uint8_t *nal_start,
					  bool *is_keyframe, int priority)
{
	// HEVC contains NAL unit specifier at [6..1] bits of
	// the byte next to the startcode 0x000001
	const int type = (nal_start[0] & 0x7F) >> 1;

	// Mark IDR slices as key-frames and set them to highest
	// priority if needed. Assume other slices are non-key
	// frames and set their priority as high
	if (type >= OBS_HEVC_NAL_BLA_W_LP &&
	    type <= OBS_HEVC_NAL_RSV_IRAP_VCL23) {
		*is_keyframe = 1;
		priority = OBS_NAL_PRIORITY_HIGHEST;
	} else if (type >= OBS_HEVC_NAL_TRAIL_N &&
		   type <= OBS_HEVC_NAL_RASL_R) {
		if (priority < OBS_NAL_PRIORITY_HIGH)
			priority = OBS_NAL_PRIORITY_HIGH;
	}

	return priority;
}

static void serialize_hevc_data(struct serializer *s, const uint8_t *data,
				size_t size, bool *is_keyframe, int *priority)
{
	const uint8_t *const end = data + size;
	const uint8_t *nal_start = obs_nal_find_startcode(data, end);
	while (true) {
		while (nal_start < end && !*(nal_start++))
			;

		if (nal_start == end)
			break;

		*priority = compute_hevc_keyframe_priority(
			nal_start, is_keyframe, *priority);

		const uint8_t *const nal_end =
			obs_nal_find_startcode(nal_start, end);
		const size_t nal_size = nal_end - nal_start;
		s_wb32(s, (uint32_t)nal_size);
		s_write(s, nal_start, nal_size);
		nal_start = nal_end;
	}
}

void obs_parse_hevc_packet(struct encoder_packet *hevc_packet,
			   const struct encoder_packet *src)
{
	struct array_output_data output;
	struct serializer s;
	long ref = 1;

	array_output_serializer_init(&s, &output);
	*hevc_packet = *src;

	serialize(&s, &ref, sizeof(ref));
	serialize_hevc_data(&s, src->data, src->size, &hevc_packet->keyframe,
			    &hevc_packet->priority);

	hevc_packet->data = output.bytes.array + sizeof(ref);
	hevc_packet->size = output.bytes.num - sizeof(ref);
	hevc_packet->drop_priority = hevc_packet->priority;
}

int obs_parse_hevc_packet_priority(const struct encoder_packet *packet)
{
	int priority = packet->priority;

	const uint8_t *const data = packet->data;
	const uint8_t *const end = data + packet->size;
	const uint8_t *nal_start = obs_nal_find_startcode(data, end);
	while (true) {
		while (nal_start < end && !*(nal_start++))
			;

		if (nal_start == end)
			break;

		bool unused;
		priority = compute_hevc_keyframe_priority(nal_start, &unused,
							  priority);

		nal_start = obs_nal_find_startcode(nal_start, end);
	}

	return priority;
}

static void hvcc_update_ptl(HEVCDecoderConfigurationRecord *hvcc,
			    HVCCProfileTierLevel *ptl)
{
	/*
     * The value of general_profile_space in all the parameter sets must be
     * identical.
     */
	hvcc->general_profile_space = ptl->profile_space;

	/*
     * The level indication general_level_idc must indicate a level of
     * capability equal to or greater than the highest level indicated for the
     * highest tier in all the parameter sets.
     */
	if (hvcc->general_tier_flag < ptl->tier_flag)
		hvcc->general_level_idc = ptl->level_idc;
	else
		hvcc->general_level_idc =
			FFMAX(hvcc->general_level_idc, ptl->level_idc);

	/*
     * The tier indication general_tier_flag must indicate a tier equal to or
     * greater than the highest tier indicated in all the parameter sets.
     */
	hvcc->general_tier_flag =
		FFMAX(hvcc->general_tier_flag, ptl->tier_flag);

	/*
     * The profile indication general_profile_idc must indicate a profile to
     * which the stream associated with this configuration record conforms.
     *
     * If the sequence parameter sets are marked with different profiles, then
     * the stream may need examination to determine which profile, if any, the
     * entire stream conforms to. If the entire stream is not examined, or the
     * examination reveals that there is no profile to which the entire stream
     * conforms, then the entire stream must be split into two or more
     * sub-streams with separate configuration records in which these rules can
     * be met.
     *
     * Note: set the profile to the highest value for the sake of simplicity.
     */
	hvcc->general_profile_idc =
		FFMAX(hvcc->general_profile_idc, ptl->profile_idc);

	/*
     * Each bit in general_profile_compatibility_flags may only be set if all
     * the parameter sets set that bit.
     */
	hvcc->general_profile_compatibility_flags &=
		ptl->profile_compatibility_flags;

	/*
     * Each bit in general_constraint_indicator_flags may only be set if all
     * the parameter sets set that bit.
     */
	hvcc->general_constraint_indicator_flags &=
		ptl->constraint_indicator_flags;
}

static void hvcc_parse_ptl(struct bitstream_reader *bs,
			   HEVCDecoderConfigurationRecord *hvcc,
			   unsigned int max_sub_layers_minus1)
{
	unsigned int i;
	HVCCProfileTierLevel general_ptl;
	uint8_t sub_layer_profile_present_flag[HEVC_MAX_SUB_LAYERS];
	uint8_t sub_layer_level_present_flag[HEVC_MAX_SUB_LAYERS];

	general_ptl.profile_space = bitstream_reader_read_bits(bs, 2);
	general_ptl.tier_flag = bitstream_reader_read_bit(bs);
	general_ptl.profile_idc = bitstream_reader_read_bits(bs, 5);
	general_ptl.profile_compatibility_flags = bitstream_reader_r32(bs, 32);
	general_ptl.constraint_indicator_flags = bitstream_reader_r64(bs, 48);
	general_ptl.level_idc = bitstream_reader_r8(bs);
	hvcc_update_ptl(hvcc, &general_ptl);

	for (i = 0; i < max_sub_layers_minus1; i++) {
		sub_layer_profile_present_flag[i] =
			bitstream_reader_read_bit(bs);
		sub_layer_level_present_flag[i] = bitstream_reader_read_bit(bs);
	}

	if (max_sub_layers_minus1 > 0)
		for (i = max_sub_layers_minus1; i < 8; i++)
			bitstream_reader_skip(bs, 2); // reserved_zero_2bits[i]

	for (i = 0; i < max_sub_layers_minus1; i++) {
		if (sub_layer_profile_present_flag[i]) {
			/*
			* sub_layer_profile_space[i]                     u(2)
			* sub_layer_tier_flag[i]                         u(1)
			* sub_layer_profile_idc[i]                       u(5)
			* sub_layer_profile_compatibility_flag[i][0..31] u(32)
			* sub_layer_progressive_source_flag[i]           u(1)
			* sub_layer_interlaced_source_flag[i]            u(1)
			* sub_layer_non_packed_constraint_flag[i]        u(1)
			* sub_layer_frame_only_constraint_flag[i]        u(1)
			* sub_layer_reserved_zero_44bits[i]              u(44)
			*/
			bitstream_reader_skip(bs, 88);
		}

		if (sub_layer_level_present_flag[i])
			bitstream_reader_skip(bs, 8);
	}
}

uint8_t *nal_unit_extract_rbsp(const uint8_t *src, size_t src_len,
			       size_t *dst_len, int header_len)
{
	uint8_t *dst;
	size_t i, len;

	dst = bmalloc(src_len);
	if (!dst) {
		warn("nal_unit_extract_rbsp: bmalloc failed");
		return NULL;
	}

	/* NAL unit header */
	i = len = 0;
	while (i < header_len && i < src_len)
		dst[len++] = src[i++];

	while (i + 2 < src_len)
		if (!src[i] && !src[i + 1] && src[i + 2] == 3) {
			dst[len++] = src[i++];
			dst[len++] = src[i++];
			i++; // remove emulation_prevention_three_byte
		} else
			dst[len++] = src[i++];

	while (i < src_len)
		dst[len++] = src[i++];

	memset(dst + len, 0, src_len - len);

	*dst_len = len;
	return dst;
}

static void hvcc_parse_vps(const uint8_t *rbsp_buf, const size_t rbsp_size,
			   HEVCDecoderConfigurationRecord *hvcc)
{
	struct bitstream_reader bs;
	bitstream_reader_init(&bs, rbsp_buf, rbsp_size);
	unsigned int vps_max_sub_layers_minus1;

	info("hvcc_parse_vps");

	/*
	* forbidden_zero_bit u(1)
	* nal_unit_type u(6)
	* nuh_layer_id u(6)
	* nuh_temporal_id_plus1 u(3)
	* vps_video_parameter_set_id u(4)
	* vps_reserved_three_2bits   u(2)
	* vps_max_layers_minus1      u(6)
	*/
	bitstream_reader_skip(&bs, 28);

	vps_max_sub_layers_minus1 = bitstream_reader_read_bits(&bs, 3);

	/*
	* numTemporalLayers greater than 1 indicates that the stream to which this
	* configuration record applies is temporally scalable and the contained
	* number of temporal layers (also referred to as temporal sub-layer or
	* sub-layer in ISO/IEC 23008-2) is equal to numTemporalLayers. Value 1
	* indicates that the stream is not temporally scalable. Value 0 indicates
	* that it is unknown whether the stream is temporally scalable.
	*/
	hvcc->numTemporalLayers =
		FFMAX(hvcc->numTemporalLayers, vps_max_sub_layers_minus1 + 1);

	/*
	* vps_temporal_id_nesting_flag u(1)
	 * vps_reserved_0xffff_16bits   u(16)
	*/
	bitstream_reader_skip(&bs, 17);

	hvcc_parse_ptl(&bs, hvcc, vps_max_sub_layers_minus1);

	/* nothing useful for hvcC past this point */
}

static int
parse_rps(struct bitstream_reader *bs, unsigned int rps_idx,
	  unsigned int num_rps,
	  unsigned int num_delta_pocs[HEVC_MAX_SHORT_TERM_REF_PIC_SETS])
{
	unsigned int i;

	if (rps_idx && bitstream_reader_read_bit(
			       bs)) { // inter_ref_pic_set_prediction_flag
		/* this should only happen for slice headers, and this isn't one */
		if (rps_idx >= num_rps)
			return -1;

		bitstream_reader_skip(bs, 1); // delta_rps_sign
		read_exp_golomb(bs);          // abs_delta_rps_minus1

		num_delta_pocs[rps_idx] = 0;

		/*
		* From libavcodec/hevc_ps.c:
		*
		* if (is_slice_header) {
		*    //foo
		* } else
		*     rps_ridx = &sps->st_rps[rps - sps->st_rps - 1];
		*
		* where:
		* rps:             &sps->st_rps[rps_idx]
		* sps->st_rps:     &sps->st_rps[0]
		* is_slice_header: rps_idx == num_rps
		*
		* thus:
		* if (num_rps != rps_idx)
		*     rps_ridx = &sps->st_rps[rps_idx - 1];
		*
		* NumDeltaPocs[RefRpsIdx]: num_delta_pocs[rps_idx - 1]
		*/
		for (i = 0; i <= num_delta_pocs[rps_idx - 1]; i++) {
			uint8_t use_delta_flag = 0;
			uint8_t used_by_curr_pic_flag =
				bitstream_reader_read_bit(bs);
			if (!used_by_curr_pic_flag)
				use_delta_flag = bitstream_reader_read_bit(bs);

			if (used_by_curr_pic_flag || use_delta_flag)
				num_delta_pocs[rps_idx]++;
		}
	} else {
		unsigned int num_negative_pics = read_exp_golomb(bs);
		unsigned int num_positive_pics = read_exp_golomb(bs);

		if ((num_positive_pics + (uint64_t)num_negative_pics) * 2 >
		    bitstream_reader_get_bits_left(bs))
			return -1;

		num_delta_pocs[rps_idx] = num_negative_pics + num_positive_pics;

		for (i = 0; i < num_negative_pics; i++) {
			read_exp_golomb(bs); // delta_poc_s0_minus1[rps_idx]
			bitstream_reader_skip(
				bs, 1); // used_by_curr_pic_s0_flag[rps_idx]
		}

		for (i = 0; i < num_positive_pics; i++) {
			read_exp_golomb(bs); // delta_poc_s1_minus1[rps_idx]
			bitstream_reader_skip(
				bs, 1); // used_by_curr_pic_s1_flag[rps_idx]
		}
	}

	return 0;
}

static void skip_timing_info(struct bitstream_reader *bs)
{
	bitstream_reader_skip(bs, 32); // num_units_in_tick
	bitstream_reader_skip(bs, 32); // time_scale

	if (bitstream_reader_read_bit(bs)) // poc_proportional_to_timing_flag
		read_exp_golomb(bs);       // num_ticks_poc_diff_one_minus1
}

static void
skip_sub_layer_hrd_parameters(struct bitstream_reader *bs,
			      unsigned int cpb_cnt_minus1,
			      uint8_t sub_pic_hrd_params_present_flag)
{
	unsigned int i;

	for (i = 0; i <= cpb_cnt_minus1; i++) {
		read_exp_golomb(bs); // bit_rate_value_minus1
		read_exp_golomb(bs); // cpb_size_value_minus1

		if (sub_pic_hrd_params_present_flag) {
			read_exp_golomb(bs); // cpb_size_du_value_minus1
			read_exp_golomb(bs); // bit_rate_du_value_minus1
		}

		bitstream_reader_skip(bs, 1); // cbr_flag
	}
}

static void skip_hrd_parameters(struct bitstream_reader *bs,
				uint8_t cprms_present_flag,
				unsigned int max_sub_layers_minus1)
{
	unsigned int i;
	uint8_t sub_pic_hrd_params_present_flag = 0;
	uint8_t nal_hrd_parameters_present_flag = 0;
	uint8_t vcl_hrd_parameters_present_flag = 0;

	if (cprms_present_flag) {
		nal_hrd_parameters_present_flag = bitstream_reader_read_bit(bs);
		vcl_hrd_parameters_present_flag = bitstream_reader_read_bit(bs);

		if (nal_hrd_parameters_present_flag ||
		    vcl_hrd_parameters_present_flag) {
			sub_pic_hrd_params_present_flag =
				bitstream_reader_read_bit(bs);

			if (sub_pic_hrd_params_present_flag)
				/*
				 * tick_divisor_minus2                          u(8)
				 * du_cpb_removal_delay_increment_length_minus1 u(5)
				 * sub_pic_cpb_params_in_pic_timing_sei_flag    u(1)
				 * dpb_output_delay_du_length_minus1            u(5)
				 */
				bitstream_reader_skip(bs, 19);

			/*
			* bit_rate_scale u(4)
			* cpb_size_scale u(4)
			*/
			bitstream_reader_skip(bs, 8);

			if (sub_pic_hrd_params_present_flag)
				bitstream_reader_skip(bs,
						      4); // cpb_size_du_scale

			/*
			* initial_cpb_removal_delay_length_minus1 u(5)
			* au_cpb_removal_delay_length_minus1      u(5)
			* dpb_output_delay_length_minus1          u(5)
			*/
			bitstream_reader_skip(bs, 15);
		}
	}

	for (i = 0; i <= max_sub_layers_minus1; i++) {
		unsigned int cpb_cnt_minus1 = 0;
		uint8_t low_delay_hrd_flag = 0;
		uint8_t fixed_pic_rate_within_cvs_flag = 0;
		uint8_t fixed_pic_rate_general_flag =
			bitstream_reader_read_bit(bs);

		if (!fixed_pic_rate_general_flag)
			fixed_pic_rate_within_cvs_flag =
				bitstream_reader_read_bit(bs);

		if (fixed_pic_rate_within_cvs_flag)
			read_exp_golomb(bs); // elemental_duration_in_tc_minus1
		else
			low_delay_hrd_flag = bitstream_reader_read_bit(bs);

		if (!low_delay_hrd_flag) {
			cpb_cnt_minus1 = read_exp_golomb(bs);
			if (cpb_cnt_minus1 > 31)
				return;
		}

		if (nal_hrd_parameters_present_flag)
			skip_sub_layer_hrd_parameters(
				bs, cpb_cnt_minus1,
				sub_pic_hrd_params_present_flag);

		if (vcl_hrd_parameters_present_flag)
			skip_sub_layer_hrd_parameters(
				bs, cpb_cnt_minus1,
				sub_pic_hrd_params_present_flag);
	}
}

static void hvcc_parse_vui(struct bitstream_reader *bs,
			   HEVCDecoderConfigurationRecord *hvcc,
			   unsigned int max_sub_layers_minus1)
{
	unsigned int min_spatial_segmentation_idc;

	if (bitstream_reader_read_bit(bs)) // aspect_ratio_info_present_flag
		if (bitstream_reader_read_bits(bs, 8) ==
		    255) // aspect_ratio_idc
			bitstream_reader_skip(
				bs, 32); // sar_width u(16), sar_height u(16)

	if (bitstream_reader_read_bit(bs))    // overscan_info_present_flag
		bitstream_reader_skip(bs, 1); // overscan_appropriate_flag

	if (bitstream_reader_read_bit(bs)) { // video_signal_type_present_flag
		bitstream_reader_skip(
			bs, 4); // video_format u(3), video_full_range_flag u(1)

		if (bitstream_reader_read_bit(
			    bs)) // colour_description_present_flag
			/*
			* colour_primaries         u(8)
			* transfer_characteristics u(8)
			* matrix_coeffs            u(8)
			*/
			bitstream_reader_skip(bs, 24);
	}

	if (bitstream_reader_read_bit(bs)) { // chroma_loc_info_present_flag
		read_exp_golomb(bs);         // chroma_sample_loc_type_top_field
		read_exp_golomb(bs); // chroma_sample_loc_type_bottom_field
	}

	/*
	* neutral_chroma_indication_flag u(1)
	* field_seq_flag                 u(1)
	* frame_field_info_present_flag  u(1)
	*/
	bitstream_reader_skip(bs, 3);

	if (bitstream_reader_read_bit(bs)) { // default_display_window_flag
		read_exp_golomb(bs);         // def_disp_win_left_offset
		read_exp_golomb(bs);         // def_disp_win_right_offset
		read_exp_golomb(bs);         // def_disp_win_top_offset
		read_exp_golomb(bs);         // def_disp_win_bottom_offset
	}

	if (bitstream_reader_read_bit(bs)) { // vui_timing_info_present_flag
		skip_timing_info(bs);

		if (bitstream_reader_read_bit(
			    bs)) // vui_hrd_parameters_present_flag
			skip_hrd_parameters(bs, 1, max_sub_layers_minus1);
	}

	if (bitstream_reader_read_bit(bs)) { // bitstream_restriction_flag
		/*
		* tiles_fixed_structure_flag              u(1)
		* motion_vectors_over_pic_boundaries_flag u(1)
		* restricted_ref_pic_lists_flag           u(1)
		*/
		bitstream_reader_skip(bs, 3);

		min_spatial_segmentation_idc = read_exp_golomb(bs);

		/*
		 * unsigned int(12) min_spatial_segmentation_idc;
		 *
		 * The min_spatial_segmentation_idc indication must indicate a level of
		 * spatial segmentation equal to or less than the lowest level of
		 * spatial segmentation indicated in all the parameter sets.
		 */
		hvcc->min_spatial_segmentation_idc =
			FFMIN(hvcc->min_spatial_segmentation_idc,
			      min_spatial_segmentation_idc);

		read_exp_golomb(bs); // max_bytes_per_pic_denom
		read_exp_golomb(bs); // max_bits_per_min_cu_denom
		read_exp_golomb(bs); // log2_max_mv_length_horizontal
		read_exp_golomb(bs); // log2_max_mv_length_vertical
	}
}

static void skip_sub_layer_ordering_info(struct bitstream_reader *bs)
{
	read_exp_golomb(bs); // max_dec_pic_buffering_minus1
	read_exp_golomb(bs); // max_num_reorder_pics
	read_exp_golomb(bs); // max_latency_increase_plus1
}

static void skip_scaling_list_data(struct bitstream_reader *bs)
{
	int i, j, k, num_coeffs;

	for (i = 0; i < 4; i++)
		for (j = 0; j < (i == 3 ? 2 : 6); j++)
			if (!bitstream_reader_read_bit(
				    bs)) // scaling_list_pred_mode_flag[i][j]
				read_exp_golomb(
					bs); // scaling_list_pred_matrix_id_delta[i][j]
			else {
				num_coeffs = FFMIN(64, 1 << (4 + (i << 1)));

				if (i > 1)
					read_sexp_golomb(
						bs); // scaling_list_dc_coef_minus8[i-2][j]

				for (k = 0; k < num_coeffs; k++)
					read_sexp_golomb(
						bs); // scaling_list_delta_coef
			}
}

static void hvcc_parse_sps(const uint8_t *rbsp_buf, const size_t rbsp_size,
			   HEVCDecoderConfigurationRecord *hvcc)
{
	struct bitstream_reader bs;
	bitstream_reader_init(&bs, rbsp_buf, rbsp_size);
	unsigned int i, sps_max_sub_layers_minus1,
		log2_max_pic_order_cnt_lsb_minus4;
	unsigned int num_short_term_ref_pic_sets,
		num_delta_pocs[HEVC_MAX_SHORT_TERM_REF_PIC_SETS];

	info("hvcc_parse_sps");

	/*
	* forbidden_zero_bit u(1)
	* nal_unit_type u(6)
	* nuh_layer_id u(6)
	* nuh_temporal_id_plus1 u(3)
	*/
	bitstream_reader_skip(&bs, 16);
	bitstream_reader_read_bits(&bs, 4); // sps_video_parameter_set_id

	sps_max_sub_layers_minus1 = bitstream_reader_read_bits(&bs, 3);

	/*
	* numTemporalLayers greater than 1 indicates that the stream to which this
	* configuration record applies is temporally scalable and the contained
	* number of temporal layers (also referred to as temporal sub-layer or
	* sub-layer in ISO/IEC 23008-2) is equal to numTemporalLayers. Value 1
	* indicates that the stream is not temporally scalable. Value 0 indicates
	* that it is unknown whether the stream is temporally scalable.
	*/
	hvcc->numTemporalLayers =
		FFMAX(hvcc->numTemporalLayers, sps_max_sub_layers_minus1 + 1);

	hvcc->temporalIdNested = bitstream_reader_read_bit(&bs);

	hvcc_parse_ptl(&bs, hvcc, sps_max_sub_layers_minus1);

	read_exp_golomb(&bs); // sps_seq_parameter_set_id

	hvcc->chromaFormat = read_exp_golomb(&bs);

	if (hvcc->chromaFormat == 3)
		bitstream_reader_skip(&bs, 1); // separate_colour_plane_flag

	read_exp_golomb(&bs); // pic_width_in_luma_samples
	read_exp_golomb(&bs); // pic_height_in_luma_samples

	if (bitstream_reader_read_bit(&bs)) { // conformance_window_flag
		read_exp_golomb(&bs);         // conf_win_left_offset
		read_exp_golomb(&bs);         // conf_win_right_offset
		read_exp_golomb(&bs);         // conf_win_top_offset
		read_exp_golomb(&bs);         // conf_win_bottom_offset
	}

	hvcc->bitDepthLumaMinus8 = read_exp_golomb(&bs);
	hvcc->bitDepthChromaMinus8 = read_exp_golomb(&bs);
	log2_max_pic_order_cnt_lsb_minus4 = read_exp_golomb(&bs);

	/* sps_sub_layer_ordering_info_present_flag */
	i = bitstream_reader_read_bit(&bs) ? 0 : sps_max_sub_layers_minus1;
	for (; i <= sps_max_sub_layers_minus1; i++)
		skip_sub_layer_ordering_info(&bs);

	read_exp_golomb(&bs); // log2_min_luma_coding_block_size_minus3
	read_exp_golomb(&bs); // log2_diff_max_min_luma_coding_block_size
	read_exp_golomb(&bs); // log2_min_transform_block_size_minus2
	read_exp_golomb(&bs); // log2_diff_max_min_transform_block_size
	read_exp_golomb(&bs); // max_transform_hierarchy_depth_inter
	read_exp_golomb(&bs); // max_transform_hierarchy_depth_intra

	if (bitstream_reader_read_bit(&bs) && // scaling_list_enabled_flag
	    bitstream_reader_read_bit(
		    &bs)) // sps_scaling_list_data_present_flag
		skip_scaling_list_data(&bs);

	bitstream_reader_skip(&bs, 1); // amp_enabled_flag
	bitstream_reader_skip(&bs, 1); // sample_adaptive_offset_enabled_flag

	if (bitstream_reader_read_bit(&bs)) { // pcm_enabled_flag
		bitstream_reader_skip(&bs,
				      4); // pcm_sample_bit_depth_luma_minus1
		bitstream_reader_skip(&bs,
				      4); // pcm_sample_bit_depth_chroma_minus1
		read_exp_golomb(
			&bs); // log2_min_pcm_luma_coding_block_size_minus3
		read_exp_golomb(
			&bs); // log2_diff_max_min_pcm_luma_coding_block_size
		bitstream_reader_skip(&bs, 1); // pcm_loop_filter_disabled_flag
	}

	num_short_term_ref_pic_sets = read_exp_golomb(&bs);
	if (num_short_term_ref_pic_sets > HEVC_MAX_SHORT_TERM_REF_PIC_SETS)
		return;

	for (i = 0; i < num_short_term_ref_pic_sets; i++) {
		if (parse_rps(&bs, i, num_short_term_ref_pic_sets,
			      num_delta_pocs) < 0)
			return;
	}

	if (bitstream_reader_read_bit(&bs)) { // long_term_ref_pics_present_flag
		unsigned num_long_term_ref_pics_sps = read_exp_golomb(&bs);
		if (num_long_term_ref_pics_sps > 31U)
			return;
		for (i = 0; i < num_long_term_ref_pics_sps;
		     i++) { // num_long_term_ref_pics_sps
			int len = FFMIN(log2_max_pic_order_cnt_lsb_minus4 + 4,
					16);
			bitstream_reader_skip(&bs,
					      len); // lt_ref_pic_poc_lsb_sps[i]
			bitstream_reader_skip(
				&bs, 1); // used_by_curr_pic_lt_sps_flag[i]
		}
	}

	bitstream_reader_skip(&bs, 1); // sps_temporal_mvp_enabled_flag
	bitstream_reader_skip(&bs, 1); // strong_intra_smoothing_enabled_flag

	if (bitstream_reader_read_bit(&bs)) // vui_parameters_present_flag
		hvcc_parse_vui(&bs, hvcc, sps_max_sub_layers_minus1);

	/* nothing useful for hvcC past this point */
}

static void hvcc_parse_pps(const uint8_t *rbsp_buf, const size_t rbsp_size,
			   HEVCDecoderConfigurationRecord *hvcc)
{
	struct bitstream_reader bs;
	bitstream_reader_init(&bs, rbsp_buf, rbsp_size);
	uint8_t tiles_enabled_flag, entropy_coding_sync_enabled_flag;

	info("hvcc_parse_pps");

	/*
	* forbidden_zero_bit u(1)
	* nal_unit_type u(6)
	* nuh_layer_id u(6)
	* nuh_temporal_id_plus1 u(3)
	*/
	bitstream_reader_skip(&bs, 16);
	read_exp_golomb(&bs); // pps_pic_parameter_set_id
	read_exp_golomb(&bs); // pps_seq_parameter_set_id

	/*
	* dependent_slice_segments_enabled_flag u(1)
	* output_flag_present_flag              u(1)
	* num_extra_slice_header_bits           u(3)
	* sign_data_hiding_enabled_flag         u(1)
	* cabac_init_present_flag               u(1)
	*/
	bitstream_reader_skip(&bs, 7);

	read_exp_golomb(&bs);  // num_ref_idx_l0_default_active_minus1
	read_exp_golomb(&bs);  // num_ref_idx_l1_default_active_minus1
	read_sexp_golomb(&bs); // init_qp_minus26

	/*
	* constrained_intra_pred_flag u(1)
	* transform_skip_enabled_flag u(1)
	*/
	bitstream_reader_skip(&bs, 2);

	if (bitstream_reader_read_bit(&bs)) // cu_qp_delta_enabled_flag
		read_exp_golomb(&bs);       // diff_cu_qp_delta_depth

	read_sexp_golomb(&bs); // pps_cb_qp_offset
	read_sexp_golomb(&bs); // pps_cr_qp_offset

	/*
	* pps_slice_chroma_qp_offsets_present_flag u(1)
	* weighted_pred_flag               u(1)
	* weighted_bipred_flag             u(1)
	* transquant_bypass_enabled_flag   u(1)
	*/
	bitstream_reader_skip(&bs, 4);

	tiles_enabled_flag = bitstream_reader_read_bit(&bs);
	entropy_coding_sync_enabled_flag = bitstream_reader_read_bit(&bs);

	if (entropy_coding_sync_enabled_flag && tiles_enabled_flag)
		hvcc->parallelismType = 0; // mixed-type parallel decoding
	else if (entropy_coding_sync_enabled_flag)
		hvcc->parallelismType = 3; // wavefront-based parallel decoding
	else if (tiles_enabled_flag)
		hvcc->parallelismType = 2; // tile-based parallel decoding
	else
		hvcc->parallelismType = 1; // slice-based parallel decoding

	/* nothing useful for hvcC past this point */
}

static void hevc_parse_nal_units(const uint8_t *data, size_t size,
				 struct serializer *s, const uint8_t **vps,
				 size_t *vps_size, const uint8_t **sps,
				 size_t *sps_size, const uint8_t **pps,
				 size_t *pps_size,
				 HEVCDecoderConfigurationRecord *hvcc)
{
	const uint8_t *nal_start, *nal_end;
	const uint8_t *end = data + size;
	int type;
	uint8_t *rbsp_buf = NULL;
	size_t rbsp_size;

	info("hevc_parse_nal_units");
	nal_start = obs_nal_find_startcode(data, end);
	while (true) {
		while (nal_start < end && !*(nal_start++))
			;

		if (nal_start == end)
			break;

		nal_end = obs_nal_find_startcode(nal_start, end);

		type = (nal_start[0] & 0x7F) >> 1;
		if (type == OBS_HEVC_NAL_VPS || type == OBS_HEVC_NAL_SPS || type == OBS_HEVC_NAL_PPS) {
			rbsp_buf = nal_unit_extract_rbsp(
				nal_start, nal_end - nal_start, &rbsp_size, 2);
			if (!rbsp_buf)
				return;
		}
		if (type == OBS_HEVC_NAL_VPS) {
			*vps = nal_start;
			*vps_size = nal_end - nal_start;
			debug("hvcc_parse_vps, nal size %ld, rbsp size %d",
				nal_end - nal_start, rbsp_size);
			hvcc_parse_vps(rbsp_buf, rbsp_size, hvcc);
		} else if (type == OBS_HEVC_NAL_SPS) {
			*sps = nal_start;
			*sps_size = nal_end - nal_start;
			debug("hvcc_parse_sps, nal size %ld, rbsp size %d",
			      nal_end - nal_start, rbsp_size);
			hvcc_parse_sps(rbsp_buf, rbsp_size, hvcc);
		} else if (type == OBS_HEVC_NAL_PPS) {
			*pps = nal_start;
			*pps_size = nal_end - nal_start;
			debug("hvcc_parse_pps, nal size %ld, rbsp size %d",
			      nal_end - nal_start, rbsp_size);
			hvcc_parse_pps(rbsp_buf, rbsp_size, hvcc);
		}

		if (rbsp_buf) {
			bfree(rbsp_buf);
			rbsp_buf = NULL;
		}

		nal_start = nal_end;
	}
}

static inline bool has_start_code(const uint8_t *data)
{
	if (data[0] != 0 || data[1] != 0)
		return false;

	return data[2] == 1 || (data[2] == 0 && data[3] == 1);
}

static void hvcc_init(HEVCDecoderConfigurationRecord *hvcc)
{
	memset(hvcc, 0, sizeof(HEVCDecoderConfigurationRecord));
	hvcc->configurationVersion = 1;
	hvcc->lengthSizeMinusOne = 3; // 4 bytes

	/*
	* The following fields have all their valid bits set by default,
	* the ProfileTierLevel parsing code will unset them when needed.
	*/
	hvcc->general_profile_compatibility_flags = 0xffffffff;
	hvcc->general_constraint_indicator_flags = 0xffffffffffff;

	/*
	* Initialize this field with an invalid value which can be used to detect
	* whether we didn't see any VUI (in which case it should be reset to zero).
	*/
	hvcc->min_spatial_segmentation_idc = MAX_SPATIAL_SEGMENTATION + 1;
}

static void hvcc_write(struct serializer *s, const uint8_t *vps,
		       const size_t vps_size, const uint8_t *sps,
		       const size_t sps_size, const uint8_t *pps,
		       const size_t pps_size,
		       HEVCDecoderConfigurationRecord *hvcc)
{
	/*
	* We only support writing HEVCDecoderConfigurationRecord version 1.
	*/
	hvcc->configurationVersion = 1;

	/*
	* If min_spatial_segmentation_idc is invalid, reset to 0 (unspecified).
	*/
	if (hvcc->min_spatial_segmentation_idc > MAX_SPATIAL_SEGMENTATION)
		hvcc->min_spatial_segmentation_idc = 0;

	/*
	* parallelismType indicates the type of parallelism that is used to meet
	* the restrictions imposed by min_spatial_segmentation_idc when the value
	* of min_spatial_segmentation_idc is greater than 0.
	*/
	if (!hvcc->min_spatial_segmentation_idc)
		hvcc->parallelismType = 0;

	/*
	* It's unclear how to properly compute these fields, so
	* let's always set them to values meaning 'unspecified'.
	*/
	hvcc->avgFrameRate = 0;
	hvcc->constantFrameRate = 0;

	/* unsigned int(8) configurationVersion = 1; */
	s_w8(s, hvcc->configurationVersion);

	/*
	* unsigned int(2) general_profile_space;
	* unsigned int(1) general_tier_flag;
	* unsigned int(5) general_profile_idc;
	*/
	s_w8(s, hvcc->general_profile_space << 6 |
			hvcc->general_tier_flag << 5 |
			hvcc->general_profile_idc);

	/* unsigned int(32) general_profile_compatibility_flags; */
	s_wb32(s, hvcc->general_profile_compatibility_flags);

	/* unsigned int(48) general_constraint_indicator_flags; */
	s_wb32(s, (uint32_t)hvcc->general_constraint_indicator_flags >> 16);
	s_wb16(s, (uint16_t)hvcc->general_constraint_indicator_flags);

	/* unsigned int(8) general_level_idc; */
	s_w8(s, hvcc->general_level_idc);

	/*
	* bit(4) reserved = '1111'b;
	* unsigned int(12) min_spatial_segmentation_idc;
	*/
	s_wb16(s, hvcc->min_spatial_segmentation_idc | 0xf000);

	/*
	* bit(6) reserved = '111111'b;
	* unsigned int(2) parallelismType;
	*/
	s_w8(s, hvcc->parallelismType | 0xfc);

	/*
	* bit(6) reserved = '111111'b;
	* unsigned int(2) chromaFormat;
	*/
	s_w8(s, hvcc->chromaFormat | 0xfc);

	/*
	* bit(5) reserved = '11111'b;
	* unsigned int(3) bitDepthLumaMinus8;
	*/
	s_w8(s, hvcc->bitDepthLumaMinus8 | 0xf8);

	/*
	* bit(5) reserved = '11111'b;
	* unsigned int(3) bitDepthChromaMinus8;
	*/
	s_w8(s, hvcc->bitDepthChromaMinus8 | 0xf8);

	/* bit(16) avgFrameRate; */
	s_wb16(s, hvcc->avgFrameRate);

	/*
	* bit(2) constantFrameRate;
	* bit(3) numTemporalLayers;
	* bit(1) temporalIdNested;
	* unsigned int(2) lengthSizeMinusOne;
	*/
	s_w8(s, hvcc->constantFrameRate << 6 | hvcc->numTemporalLayers << 3 |
			hvcc->temporalIdNested << 2 | hvcc->lengthSizeMinusOne);

	/* unsigned int(8) numOfArrays; */
	s_w8(s, 3); // 1 vps, 1 sps, 1 pps

	/*
	* bit(1) array_completeness;
	* unsigned int(1) reserved = 0;
	* unsigned int(6) NAL_unit_type;
	*/
	s_w8(s, OBS_HEVC_NAL_VPS & 0x3f);
	/* unsigned int(16) numNalus; */
	s_wb16(s, 1);
	/* unsigned int(16) nalUnitLength; */
	s_wb16(s, (uint16_t)vps_size);
	/* bit(8*nalUnitLength) nalUnit; */
	s_write(s, vps, vps_size);

	s_w8(s, OBS_HEVC_NAL_SPS & 0x3f);
	s_wb16(s, 1);
	s_wb16(s, (uint16_t)sps_size);
	s_write(s, sps, sps_size);

	s_w8(s, OBS_HEVC_NAL_PPS & 0x3f);
	s_wb16(s, 1);
	s_wb16(s, (uint16_t)pps_size);
	s_write(s, pps, pps_size);
}

size_t obs_parse_hevc_header(uint8_t **header, const uint8_t *data, size_t size)
{
	struct array_output_data output;
	struct serializer s;
	HEVCDecoderConfigurationRecord hvcc;
	const uint8_t *vps = NULL, *sps = NULL, *pps = NULL;
	size_t vps_size = 0, sps_size = 0, pps_size = 0;

	info("obs_parse_hevc_header");
	if (size <= 6) {
		warn("size<=6");
		return 0;
	}

	if (data[0] == 1) {
		/* Data is already hvcC-formatted */
		info("Data is already hvcC-formatted");
		*header = bmemdup(data, size);
		return size;
	}

	if (!has_start_code(data)) {
		/* Not a valid Annex B start code prefix */
		warn("Not a valid Annex B start code prefix");
		return 0;
	}

	array_output_serializer_init(&s, &output);
	hvcc_init(&hvcc);

	hevc_parse_nal_units(data, size, &s, &vps, &vps_size, &sps, &sps_size,
			     &pps, &pps_size, &hvcc);
	if (!vps || !sps || !pps) {
		warn("vps/sps/pps not present");
		return 0;
	}

	hvcc_write(&s, vps, vps_size, sps, sps_size, pps, pps_size, &hvcc);
	info("Will write %ld bytes", output.bytes.num);

	*header = output.bytes.array;
	return output.bytes.num;
}

void obs_extract_hevc_headers(const uint8_t *packet, size_t size,
			      uint8_t **new_packet_data,
			      size_t *new_packet_size, uint8_t **header_data,
			      size_t *header_size, uint8_t **sei_data,
			      size_t *sei_size)
{
	DARRAY(uint8_t) new_packet;
	DARRAY(uint8_t) header;
	DARRAY(uint8_t) sei;
	const uint8_t *nal_start, *nal_end, *nal_codestart;
	const uint8_t *end = packet + size;

	da_init(new_packet);
	da_init(header);
	da_init(sei);

	nal_start = obs_nal_find_startcode(packet, end);
	nal_end = NULL;
	while (nal_end != end) {
		nal_codestart = nal_start;

		while (nal_start < end && !*(nal_start++))
			;

		if (nal_start == end)
			break;

		const uint8_t type = (nal_start[0] & 0x7F) >> 1;

		nal_end = obs_nal_find_startcode(nal_start, end);
		if (!nal_end)
			nal_end = end;

		if (type == OBS_HEVC_NAL_VPS || type == OBS_HEVC_NAL_SPS ||
		    type == OBS_HEVC_NAL_PPS) {
			da_push_back_array(header, nal_codestart,
					   nal_end - nal_codestart);
		} else if (type == OBS_HEVC_NAL_SEI_PREFIX ||
			   type == OBS_HEVC_NAL_SEI_SUFFIX) {
			da_push_back_array(sei, nal_codestart,
					   nal_end - nal_codestart);

		} else {
			da_push_back_array(new_packet, nal_codestart,
					   nal_end - nal_codestart);
		}

		nal_start = nal_end;
	}

	*new_packet_data = new_packet.array;
	*new_packet_size = new_packet.num;
	*header_data = header.array;
	*header_size = header.num;
	*sei_data = sei.array;
	*sei_size = sei.num;
}

