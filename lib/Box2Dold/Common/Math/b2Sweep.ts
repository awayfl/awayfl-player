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
import { b2XForm, b2Mat22, b2Vec2 } from '../Math';

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
export class b2Sweep {
	__fast__ = true;

	/// Get the interpolated transform at a specific time.
	/// @param t the normalized time in [0,1].
	public GetXForm(xf: b2XForm, t: number): void {

		// center = p + R * localCenter
		if (1.0 - this.t0 > Number.MIN_VALUE) {
			const alpha: number = (t - this.t0) / (1.0 - this.t0);
			xf.position.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
			xf.position.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
			const angle: number = (1.0 - alpha) * this.a0 + alpha * this.a;
			xf.R.Set(angle);
		} else {
			xf.position.SetV(this.c);
			xf.R.Set(this.a);
		}

		// Shift to origin
		//xf->position -= b2Mul(xf->R, localCenter);
		const tMat: b2Mat22 = xf.R;
		xf.position.x -= (tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y);
		xf.position.y -= (tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y);

	}

	/// Advance the sweep forward, yielding a new initial state.
	/// @param t the new initial time.
	public Advance(t: number): void {
		if (this.t0 < t && 1.0 - this.t0 > Number.MIN_VALUE) {
			const alpha: number = (t - this.t0) / (1.0 - this.t0);
			//c0 = (1.0f - alpha) * c0 + alpha * c;
			this.c0.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
			this.c0.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
			this.a0 = (1.0 - alpha) * this.a0 + alpha * this.a;
			this.t0 = t;
		}
	}

	public localCenter: b2Vec2 = new b2Vec2();	///< local center of mass position
	public c0: b2Vec2 = new b2Vec2();				///< center world positions
	public c: b2Vec2 = new b2Vec2();
	public a0: number
	public a: number;					///< world angles
	public t0: number;							///< time interval = [t0,1], where t0 is in [0,1]
}