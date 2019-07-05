/*
 * DevilBullet modifications (c) 2019 Sam Johnson https://github.com/SmashMaster
 * 
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
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

package com.bulletphysics.demos.opengl;

import static com.bulletphysics.demos.opengl.IGL.*;
import com.bulletphysics.linearmath.DebugDrawModes;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;

/**
 *
 * @author jezek2
 */
public class GLDebugDrawer extends IDebugDraw {
	
	// JAVA NOTE: added
	private static final boolean DEBUG_NORMALS = false;
	
	private IGL gl;
	private int debugMode;
	
	private final Vec3 tmpVec = new Vec3();

	public GLDebugDrawer(IGL gl) {
		this.gl = gl;
	}

	@Override
	public void drawLine(Vec3 from, Vec3 to, Vec3 color) {
		if (debugMode > 0) {
			gl.glBegin(GL_LINES);
			gl.glColor3f(color.x, color.y, color.z);
			gl.glVertex3f(from.x, from.y, from.z);
			gl.glVertex3f(to.x, to.y, to.z);
			gl.glEnd();
		}
	}

	@Override
	public void setDebugMode(int debugMode) {
		this.debugMode = debugMode;
	}

	@Override
	public void draw3dText(Vec3 location, String textString) {
		//glRasterPos3f(location.x,  location.y,  location.z);
		// TODO: BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),textString);
	}

	@Override
	public void reportErrorWarning(String warningString) {
		System.err.println(warningString);
	}

	@Override
	public void drawContactPoint(Vec3 pointOnB, Vec3 normalOnB, float distance, int lifeTime, Vec3 color) {
		if ((debugMode & DebugDrawModes.DRAW_CONTACT_POINTS) != 0) {
			Vec3 to = tmpVec;
			VectorUtil.scaleAdd(to, distance*100f, normalOnB, pointOnB);
			Vec3 from = pointOnB;

			// JAVA NOTE: added
			if (DEBUG_NORMALS) {
				VectorUtil.normalize(to, normalOnB);
				to.mult(10f);
				to.add(pointOnB);
				gl.glLineWidth(3f);
				gl.glPointSize(6f);
				gl.glBegin(GL_POINTS);
				gl.glColor3f(color.x, color.y, color.z);
				gl.glVertex3f(from.x, from.y, from.z);
				gl.glEnd();
			}

			gl.glBegin(GL_LINES);
			gl.glColor3f(color.x, color.y, color.z);
			gl.glVertex3f(from.x, from.y, from.z);
			gl.glVertex3f(to.x, to.y, to.z);
			gl.glEnd();

			// JAVA NOTE: added
			if (DEBUG_NORMALS) {
				gl.glLineWidth(1f);
				gl.glPointSize(1f);
			}

			//glRasterPos3f(from.x, from.y, from.z);
			//char buf[12];
			//sprintf(buf," %d",lifeTime);
			// TODO: BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		}
	}

	@Override
	public int getDebugMode() {
		return debugMode;
	}

}
