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
import { b2Vec2 } from '../Math';

/// A 2-by-2 matrix. Stored in column-major order.
export class b2Mat22 {
	constructor(angle: number = 0, c1: b2Vec2 = null, c2: b2Vec2 = null) {
		if (c1 != null && c2 != null) {
			this.col1.SetV(c1);
			this.col2.SetV(c2);
		} else {
			const c: number = Math.cos(angle);
			const s: number = Math.sin(angle);
			this.col1.x = c; this.col2.x = -s;
			this.col1.y = s; this.col2.y = c;
		}
	}

	public Set(angle: number): void {
		const c: number = Math.cos(angle);
		const s: number = Math.sin(angle);
		this.col1.x = c; this.col2.x = -s;
		this.col1.y = s; this.col2.y = c;
	}

	public SetVV(c1: b2Vec2, c2: b2Vec2): void {
		this.col1.SetV(c1);
		this.col2.SetV(c2);
	}

	public Copy(): b2Mat22 {
		return new b2Mat22(0, this.col1, this.col2);
	}

	public SetM(m: b2Mat22): void {
		this.col1.SetV(m.col1);
		this.col2.SetV(m.col2);
	}

	public AddM(m: b2Mat22): void {
		this.col1.x += m.col1.x;
		this.col1.y += m.col1.y;
		this.col2.x += m.col2.x;
		this.col2.y += m.col2.y;
	}

	public SetIdentity(): void {
		this.col1.x = 1.0; this.col2.x = 0.0;
		this.col1.y = 0.0; this.col2.y = 1.0;
	}

	public SetZero(): void {
		this.col1.x = 0.0; this.col2.x = 0.0;
		this.col1.y = 0.0; this.col2.y = 0.0;
	}

	public GetAngle(): number {
		return Math.atan2(this.col1.y, this.col1.x);
	}

	public Invert(out: b2Mat22): b2Mat22 {
		const a: number = this.col1.x;
		const b: number = this.col2.x;
		const c: number = this.col1.y;
		const d: number = this.col2.y;
		//var B:b2Mat22 = new b2Mat22();
		let det: number = a * d - b * c;
		//b2Settings.b2Assert(det != 0.0);
		det = 1.0 / det;
		out.col1.x =  det * d;	out.col2.x = -det * b;
		out.col1.y = -det * c;	out.col2.y =  det * a;
		return out;
	}

	// Solve A * x = b
	public Solve(out: b2Vec2, bX: number, bY: number): b2Vec2 {
		//float32 a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
		const a11: number = this.col1.x;
		const a12: number = this.col2.x;
		const a21: number = this.col1.y;
		const a22: number = this.col2.y;
		//float32 det = a11 * a22 - a12 * a21;
		let det: number = a11 * a22 - a12 * a21;
		//b2Settings.b2Assert(det != 0.0);
		det = 1.0 / det;
		out.x = det * (a22 * bX - a12 * bY);
		out.y = det * (a11 * bY - a21 * bX);

		return out;
	}

	public Abs(): void {
		this.col1.Abs();
		this.col2.Abs();
	}

	public col1: b2Vec2 = new b2Vec2();
	public col2: b2Vec2 = new b2Vec2();
}