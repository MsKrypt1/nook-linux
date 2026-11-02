// SPDX-License-Identifier: GPL-2.0-only
/*
 * The picture's size and cropping from an H.264 sequence parameter set
 * (ITU-T H.264 7.3.2.1.1). The firmware reports the output crop only after
 * the first picture, when the client has set up its CAPTURE buffers: the
 * visible size for the source change event comes from here.
 */
#include <linux/kernel.h>
#include <linux/string.h>

#include "ducati.h"

struct ducati_bits {
	const u8 *p, *end;
	u32 zeros;	/* zero bytes in a row, for emulation prevention */
	u32 cur;	/* the current byte */
	int left;	/* its bits not read yet */
	bool err;
};

static void bits_next_byte(struct ducati_bits *b)
{
	while (b->p < b->end) {
		u8 c = *b->p++;

		/* 0x000003: the 3 is not data */
		if (b->zeros >= 2 && c == 3) {
			b->zeros = 0;
			continue;
		}
		b->zeros = c ? 0 : b->zeros + 1;
		b->cur = c;
		b->left = 8;
		return;
	}
	b->err = true;
	b->cur = 0;
	b->left = 8;
}

static u32 bits_u(struct ducati_bits *b, int n)
{
	u32 v = 0;

	while (n--) {
		if (!b->left)
			bits_next_byte(b);
		b->left--;
		v = v << 1 | ((b->cur >> b->left) & 1);
	}
	return v;
}

static u32 bits_ue(struct ducati_bits *b)
{
	int lz = 0;

	while (!bits_u(b, 1)) {
		if (++lz > 31 || b->err) {
			b->err = true;
			return 0;
		}
	}
	return lz ? (1U << lz) - 1 + bits_u(b, lz) : 0;
}

static s32 bits_se(struct ducati_bits *b)
{
	u32 v = bits_ue(b);

	return v & 1 ? (s32)((v + 1) / 2) : -(s32)(v / 2);
}

static void skip_scaling_list(struct ducati_bits *b, int size)
{
	int last = 8, next = 8, j;

	for (j = 0; j < size && !b->err; j++) {
		if (next)
			next = (last + bits_se(b) + 256) % 256;
		if (next)
			last = next;
	}
}

/* the NAL unit's payload, after its one-byte header */
static int parse_sps(const u8 *p, const u8 *end, struct ducati_sps *sps)
{
	struct ducati_bits b = { .p = p, .end = end };
	u32 profile, chroma = 1, w, h, frame_mbs_only, i, n;
	u32 cl = 0, cr = 0, ct = 0, cb = 0, ux, uy;
	bool sep = false;

	profile = bits_u(&b, 8);
	bits_u(&b, 16);			/* constraint flags, level */
	bits_ue(&b);			/* seq_parameter_set_id */
	switch (profile) {
	case 100: case 110: case 122: case 244: case 44: case 83: case 86:
	case 118: case 128: case 138: case 139: case 134: case 135:
		chroma = bits_ue(&b);
		if (chroma > 3)
			return -EINVAL;
		if (chroma == 3)
			sep = bits_u(&b, 1);
		bits_ue(&b);		/* bit depths */
		bits_ue(&b);
		bits_u(&b, 1);		/* qpprime_y_zero_transform_bypass */
		if (bits_u(&b, 1)) {	/* seq_scaling_matrix_present */
			n = chroma != 3 ? 8 : 12;
			for (i = 0; i < n && !b.err; i++)
				if (bits_u(&b, 1))
					skip_scaling_list(&b, i < 6 ? 16 : 64);
		}
		break;
	}
	bits_ue(&b);			/* log2_max_frame_num_minus4 */
	switch (bits_ue(&b)) {		/* pic_order_cnt_type */
	case 0:
		bits_ue(&b);
		break;
	case 1:
		bits_u(&b, 1);
		bits_se(&b);
		bits_se(&b);
		n = bits_ue(&b);
		if (n > 255)
			return -EINVAL;
		for (i = 0; i < n && !b.err; i++)
			bits_se(&b);
		break;
	}
	bits_ue(&b);			/* max_num_ref_frames */
	bits_u(&b, 1);			/* gaps_in_frame_num_value_allowed */
	w = bits_ue(&b) + 1;
	h = bits_ue(&b) + 1;
	frame_mbs_only = bits_u(&b, 1);
	if (!frame_mbs_only)
		bits_u(&b, 1);		/* mb_adaptive_frame_field */
	bits_u(&b, 1);			/* direct_8x8_inference */
	if (bits_u(&b, 1)) {		/* frame_cropping */
		cl = bits_ue(&b);
		cr = bits_ue(&b);
		ct = bits_ue(&b);
		cb = bits_ue(&b);
	}
	if (b.err || w > 512 || h > 512)
		return -EINVAL;

	w *= 16;
	h *= 16 * (2 - frame_mbs_only);
	/* CropUnitX/Y: by chroma subsampling, field pairs count twice */
	if (chroma == 0 || sep) {
		ux = 1;
		uy = 2 - frame_mbs_only;
	} else {
		ux = chroma == 3 ? 1 : 2;
		uy = (chroma == 1 ? 2 : 1) * (2 - frame_mbs_only);
	}
	if ((u64)(cl + cr) * ux >= w || (u64)(ct + cb) * uy >= h)
		return -EINVAL;

	sps->width = w;
	sps->height = h;
	sps->left = cl * ux;
	sps->top = ct * uy;
	sps->vis_width = w - (cl + cr) * ux;
	sps->vis_height = h - (ct + cb) * uy;
	sps->valid = true;
	return 0;
}

/*
 * Look through an Annex B access unit up to its first slice: an SPS there
 * updates @sps. Returns true if one was found.
 */
bool ducati_h264_find_sps(const u8 *p, size_t len, struct ducati_sps *sps)
{
	const u8 *end = p + len, *nal = NULL, *q;
	bool found = false;
	u8 type;

	for (q = p; q + 3 <= end; ) {
		if (q[0] || q[1] || q[2] != 1) {
			q += q[2] > 1 ? 3 : 1;
			continue;
		}
		q += 3;
		if (nal && (nal[0] & 0x1f) == 7)
			found |= !parse_sps(nal + 1, q - 3, sps);
		nal = q;
		if (q >= end)
			break;
		type = *q & 0x1f;
		if (type >= 1 && type <= 5)	/* a slice: no SPS after it */
			return found;
	}
	if (nal && nal < end && (nal[0] & 0x1f) == 7)
		found |= !parse_sps(nal + 1, end, sps);
	return found;
}

/* an access unit whose first slice is an IDR picture's */
bool ducati_h264_is_idr(const u8 *p, size_t len)
{
	const u8 *end = p + len, *q;
	u8 type;

	for (q = p; q + 3 < end; ) {
		if (q[0] || q[1] || q[2] != 1) {
			q += q[2] > 1 ? 3 : 1;
			continue;
		}
		q += 3;
		type = *q & 0x1f;
		if (type >= 1 && type <= 5)
			return type == 5;
	}
	return false;
}

/*
 * The SPS and PPS NAL units of an access unit (up to its first slice), each
 * after a 4-byte start code, into @out: their size, 0 (and @out untouched)
 * if there are none or they do not fit.
 */
size_t ducati_h264_copy_headers(const u8 *p, size_t len, u8 *out, size_t max)
{
	const u8 *end = p + len, *nal = NULL, *q;
	size_t n = 0;
	int pass;
	u8 type;

	for (pass = 0; pass < 2; pass++) {
		n = 0;
		nal = NULL;
		for (q = p; ; ) {
			const u8 *next = NULL;

			while (q + 3 <= end) {
				if (!q[0] && !q[1] && q[2] == 1) {
					next = q;
					break;
				}
				q += q[2] > 1 ? 3 : 1;
			}
			if (nal) {
				const u8 *e = next ? next : end;

				/* trailing zeros belong to the next start code */
				while (e > nal && !e[-1])
					e--;
				type = *nal & 0x1f;
				if (type == 7 || type == 8) {
					if (pass) {
						memcpy(out + n, "\0\0\0\1", 4);
						memcpy(out + n + 4, nal, e - nal);
					}
					n += 4 + (e - nal);
				}
			}
			if (!next || next + 3 >= end)
				break;
			nal = next + 3;
			q = nal;
			type = *nal & 0x1f;
			if (type >= 1 && type <= 5)	/* a slice: no more */
				break;
		}
		if (!n || n > max)
			return 0;
	}
	return n;
}
