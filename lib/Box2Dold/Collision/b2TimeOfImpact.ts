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

import { b2Vec2 } from '../Common/Math';
import { b2XForm } from '../Common/Math';
import { b2Shape } from './Shapes/b2Shape';
import { b2Sweep } from '../Common/Math';
import { b2Settings } from '../Common/b2Settings';
import { b2Distance } from './b2Distance';

export class b2TimeOfImpact {

	// This algorithm uses conservative advancement to compute the time of
	// impact (TOI) of two shapes.
	// Refs: Bullet, Young Kim
	//
	public static s_p1: b2Vec2 = new b2Vec2();
	public static s_p2: b2Vec2 = new b2Vec2();
	public static s_xf1: b2XForm = new b2XForm();
	public static s_xf2: b2XForm = new b2XForm();
	//
	public static TimeOfImpact(shape1: b2Shape, sweep1: b2Sweep,
		shape2: b2Shape, sweep2: b2Sweep): number {
		let math1: number;
		let math2: number;

		const r1: number = shape1.m_sweepRadius;
		const r2: number = shape2.m_sweepRadius;

		//b2Settings.b2Assert(sweep1.t0 == sweep2.t0);
		//b2Settings.b2Assert(1.0 - sweep1.t0 > Number.MIN_VALUE);

		const t0: number = sweep1.t0;
		//b2Vec2 v1 = sweep1.c - sweep1.c0;
		const v1X: number = sweep1.c.x - sweep1.c0.x;
		const v1Y: number = sweep1.c.y - sweep1.c0.y;
		//b2Vec2 v2 = sweep2.c - sweep2.c0;
		const v2X: number = sweep2.c.x - sweep2.c0.x;
		const v2Y: number = sweep2.c.y - sweep2.c0.y;
		const omega1: number = sweep1.a - sweep1.a0;
		const omega2: number = sweep2.a - sweep2.a0;

		let alpha: number = 0.0;

		const p1: b2Vec2 = this.s_p1;
		const p2: b2Vec2 = this.s_p2;
		const k_maxIterations: number /** int */ = 20;	// TODO_ERIN b2Settings
		let iter: number /** int */ = 0;
		//b2Vec2 normal = b2Vec2_zero;
		let normalX: number = 0.0;
		let normalY: number = 0.0;
		let distance: number = 0.0;
		let targetDistance: number = 0.0;
		for (;;) {
			const t: number = (1.0 - alpha) * t0 + alpha;
			//b2XForm xf1, xf2;
			const xf1: b2XForm = this.s_xf1;
			const xf2: b2XForm = this.s_xf2;
			sweep1.GetXForm(xf1, t);
			sweep2.GetXForm(xf2, t);

			// Get the distance between shapes.
			distance = b2Distance.Distance(p1, p2, shape1, xf1, shape2, xf2);

			if (iter == 0) {
				// Compute a reasonable target distance to give some breathing room
				// for conservative advancement.
				if (distance > 2.0 * b2Settings.b2_toiSlop) {
					targetDistance = 1.5 * b2Settings.b2_toiSlop;
				} else {
					//targetDistance = Math.max(0.05 * b2Settings.b2_toiSlop, distance - 0.5 * b2Settings.b2_toiSlop);
					math1 = 0.05 * b2Settings.b2_toiSlop;
					math2 = distance - 0.5 * b2Settings.b2_toiSlop;
					targetDistance = math1 > math2 ? math1 : math2;
				}
			}

			if (distance - targetDistance < 0.05 * b2Settings.b2_toiSlop || iter == k_maxIterations) {
				break;
			}

			//normal = p2 - p1;
			normalX = p2.x - p1.x;
			normalY = p2.y - p1.y;
			//normal.Normalize();
			const nLen: number = Math.sqrt(normalX * normalX + normalY * normalY);
			normalX /= nLen;
			normalY /= nLen;

			// Compute upper bound on remaining movement.
			//float32 approachVelocityBound = b2Dot(normal, v1 - v2) + b2Abs(omega1) * r1 + b2Abs(omega2) * r2;
			const approachVelocityBound: number = 	(normalX * (v1X - v2X) + normalY * (v1Y - v2Y))
												+ (omega1 < 0 ? -omega1 : omega1) * r1
												+ (omega2 < 0 ? -omega2 : omega2) * r2;
			//if (Math.abs(approachVelocityBound) < Number.MIN_VALUE)
			if (approachVelocityBound == 0) {
				alpha = 1.0;
				break;
			}

			// Get the conservative time increment. Don't advance all the way.
			const dAlpha: number = (distance - targetDistance) / approachVelocityBound;
			//float32 dt = (distance - 0.5f * b2_linearSlop) / approachVelocityBound;
			const newAlpha: number = alpha + dAlpha;

			// The shapes may be moving apart or a safe distance apart.
			if (newAlpha < 0.0 || 1.0 < newAlpha) {
				alpha = 1.0;
				break;
			}

			// Ensure significant advancement.
			if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha) {
				break;
			}

			alpha = newAlpha;

			++iter;
		}

		return alpha;
	}
}