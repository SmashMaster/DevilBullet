/*
 * DevilBullet modifications (c) 2019 Sam Johnson https://github.com/SmashMaster
 * 
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.extras.gimpact;

import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;

/**
 *
 * @author jezek2
 */
class Quantization {

	public static void bt_calc_quantization_parameters(Vec3 outMinBound, Vec3 outMaxBound, Vec3 bvhQuantization, Vec3 srcMinBound, Vec3 srcMaxBound, float quantizationMargin) {
		// enlarge the AABB to avoid division by zero when initializing the quantization values
		Vec3 clampValue = new Vec3();
		clampValue.set(quantizationMargin, quantizationMargin, quantizationMargin);
		VectorUtil.sub(outMinBound, srcMinBound, clampValue);
		VectorUtil.add(outMaxBound, srcMaxBound, clampValue);
		Vec3 aabbSize = new Vec3();
		VectorUtil.sub(aabbSize, outMaxBound, outMinBound);
		bvhQuantization.set(65535.0f, 65535.0f, 65535.0f);
		VectorUtil.div(bvhQuantization, bvhQuantization, aabbSize);
	}

	public static void bt_quantize_clamp(short[] out, Vec3 point, Vec3 min_bound, Vec3 max_bound, Vec3 bvhQuantization) {
		Vec3 clampedPoint = new Vec3(point);
		VectorUtil.setMax(clampedPoint, min_bound);
		VectorUtil.setMin(clampedPoint, max_bound);

		Vec3 v = new Vec3();
		VectorUtil.sub(v, clampedPoint, min_bound);
		VectorUtil.mul(v, v, bvhQuantization);

		out[0] = (short) (v.x + 0.5f);
		out[1] = (short) (v.y + 0.5f);
		out[2] = (short) (v.z + 0.5f);
	}

	public static Vec3 bt_unquantize(short[] vecIn, Vec3 offset, Vec3 bvhQuantization, Vec3 out) {
		out.set((float)(vecIn[0] & 0xFFFF) / (bvhQuantization.x),
		        (float)(vecIn[1] & 0xFFFF) / (bvhQuantization.y),
		        (float)(vecIn[2] & 0xFFFF) / (bvhQuantization.z));
		out.add(offset);
		return out;
	}
	
}
