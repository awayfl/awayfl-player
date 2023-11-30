/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

import { b2ShapeDef } from './b2ShapeDef';
import { b2Shape } from './b2Shape';
import { b2Settings } from '../../Common/b2Settings';
import { b2Vec2, b2Mat22 } from '../../Common/Math';

export class b2PolygonDef extends b2ShapeDef {
	constructor() {
		super();

		this.type = b2Shape.e_polygonShape;
		this.vertexCount = 0;

		for (let i: number /** int */ = 0; i < b2Settings.b2_maxPolygonVertices; i++) {
			this.vertices[i] = new b2Vec2();
		}
	}

	/// Build vertices to represent an axis-aligned box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	public SetAsBox(hx: number, hy: number): void {
		this.vertexCount = 4;
		this.vertices[0].Set(-hx, -hy);
		this.vertices[1].Set(hx, -hy);
		this.vertices[2].Set(hx,  hy);
		this.vertices[3].Set(-hx,  hy);
	}

	/// Build vertices to represent an oriented box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	/// @param center the center of the box in local coordinates.
	/// @param angle the rotation of the box in local coordinates.
	private static s_mat: b2Mat22 = new b2Mat22();
	public SetAsOrientedBox(hx: number, hy: number, center: b2Vec2 = null, angle: number = 0.0): void {
		//SetAsBox(hx, hy);
		{
			this.vertexCount = 4;
			this.vertices[0].Set(-hx, -hy);
			this.vertices[1].Set(hx, -hy);
			this.vertices[2].Set(hx,  hy);
			this.vertices[3].Set(-hx,  hy);
		}

		if (center) {
			//b2XForm xf;
			//xf.position = center;
			const xfPosition: b2Vec2 = center;
			//xf.R.Set(angle);
			const xfR: b2Mat22 = b2PolygonDef.s_mat;
			xfR.Set(angle);

			for (let i: number /** int */ = 0; i < this.vertexCount; ++i) {
				//vertices[i] = b2Mul(xf, vertices[i]);
				//var a:b2Vec2 = b2MulMV(T.R, v);
				center = this.vertices[i];
				hx = xfPosition.x + (xfR.col1.x * center.x + xfR.col2.x * center.y);
				center.y = xfPosition.y + (xfR.col1.y * center.x + xfR.col2.y * center.y);
				center.x = hx;
			}
		}
	}

	/// The polygon vertices in local coordinates.
	public vertices: b2Vec2[] = new Array(b2Settings.b2_maxPolygonVertices);
	/// The number of polygon vertices.
	public vertexCount: number /** int */;
}