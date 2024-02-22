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

import { b2Shape } from './b2Shape';
import { b2XForm, b2Vec2, b2Mat22 } from '../../Common/Math';
import { b2Segment } from '../b2Segment';
import { b2AABB } from '../b2AABB';
import { b2Settings } from '../../Common/b2Settings';
import { b2MassData } from './b2MassData';
import { b2ShapeDef } from './b2ShapeDef';
import { b2CircleDef } from './b2CircleDef';

export class b2CircleShape extends b2Shape {
	/// @see b2Shape::TestPoint
	public TestPoint(transform: b2XForm, p: b2Vec2): boolean {
		//b2Vec2 center = transform.position + b2Mul(transform.R, m_localPosition);
		const tMat: b2Mat22 = transform.R;
		let dX: number = transform.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		let dY: number = transform.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);
		//b2Vec2 d = p - center;
		dX = p.x - dX;
		dY = p.y - dY;
		//return b2Dot(d, d) <= m_radius * m_radius;
		return (dX * dX + dY * dY) <= this.m_radius * this.m_radius;
	}

	/// @see b2Shape::TestSegment
	public TestSegment(transform: b2XForm,
		lambda: number[], // float pointer
		normal: b2Vec2, // pointer
		segment: b2Segment,
		maxLambda: number
	): number {
		//b2Vec2 position = transform.position + b2Mul(transform.R, m_localPosition);
		const tMat: b2Mat22 = transform.R;
		const positionX: number = transform.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		const positionY: number = transform.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);

		//b2Vec2 s = segment.p1 - position;
		const sX: number = segment.p1.x - positionX;
		const sY: number = segment.p1.y - positionY;
		//float32 b = b2Dot(s, s) - m_radius * m_radius;
		const b: number = (sX * sX + sY * sY) - this.m_radius * this.m_radius;

		// Does the segment start inside the circle?
		if (b < 0.0) {
			lambda[0] = 0;
			return b2Shape.e_startsInsideCollide;
		}

		// Solve quadratic equation.
		//b2Vec2 r = segment.p2 - segment.p1;
		const rX: number = segment.p2.x - segment.p1.x;
		const rY: number = segment.p2.y - segment.p1.y;
		//float32 c =  b2Dot(s, r);
		const c: number =  (sX * rX + sY * rY);
		//float32 rr = b2Dot(r, r);
		const rr: number = (rX * rX + rY * rY);
		const sigma: number = c * c - rr * b;

		// Check for negative discriminant and short segment.
		if (sigma < 0.0 || rr < Number.MIN_VALUE) {
			return b2Shape.e_missCollide;
		}

		// Find the point of intersection of the line with the circle.
		let a: number = -(c + Math.sqrt(sigma));

		// Is the intersection point on the segment?
		if (0.0 <= a && a <= maxLambda * rr) {
			a /= rr;
			//*lambda = a;
			lambda[0] = a;
			//*normal = s + a * r;
			normal.x = sX + a * rX;
			normal.y = sY + a * rY;
			normal.Normalize();
			return b2Shape.e_hitCollide;
		}

		return b2Shape.e_missCollide;
	}

	/// @see b2Shape::ComputeAABB
	public ComputeAABB(aabb: b2AABB, transform: b2XForm): void {
		//b2Vec2 p = transform.position + b2Mul(transform.R, m_localPosition);
		const tMat: b2Mat22 = transform.R;
		const pX: number = transform.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		const pY: number = transform.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);
		aabb.lowerBound.Set(pX - this.m_radius, pY - this.m_radius);
		aabb.upperBound.Set(pX + this.m_radius, pY + this.m_radius);
	}

	/// @see b2Shape::ComputeSweptAABB
	public ComputeSweptAABB(aabb: b2AABB,
		transform1: b2XForm,
		transform2: b2XForm): void {
		let tMat: b2Mat22;
		//b2Vec2 p1 = transform1.position + b2Mul(transform1.R, m_localPosition);
		tMat = transform1.R;
		const p1X: number = transform1.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		const p1Y: number = transform1.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);
		//b2Vec2 p2 = transform2.position + b2Mul(transform2.R, m_localPosition);
		tMat = transform2.R;
		const p2X: number = transform2.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		const p2Y: number = transform2.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);

		//b2Vec2 lower = b2Min(p1, p2);
		//b2Vec2 upper = b2Max(p1, p2);

		//aabb->lowerBound.Set(lower.x - m_radius, lower.y - m_radius);
		aabb.lowerBound.Set((p1X < p2X ? p1X : p2X) - this.m_radius, (p1Y < p2Y ? p1Y : p2Y) - this.m_radius);
		//aabb->upperBound.Set(upper.x + m_radius, upper.y + m_radius);
		aabb.upperBound.Set((p1X > p2X ? p1X : p2X) + this.m_radius, (p1Y > p2Y ? p1Y : p2Y) + this.m_radius);
	}

	/// @see b2Shape::ComputeMass
	public ComputeMass(massData: b2MassData): void {
		massData.mass = this.m_density * b2Settings.b2_pi * this.m_radius * this.m_radius;
		massData.center.SetV(this.m_localPosition);

		// inertia about the local origin
		//massData.I = massData.mass * (0.5 * m_radius * m_radius + b2Dot(m_localPosition, m_localPosition));
		massData.I = massData.mass * (0.5 * this.m_radius * this.m_radius + (this.m_localPosition.x * this.m_localPosition.x + this.m_localPosition.y * this.m_localPosition.y));
	}

	/// Get the local position of this circle in its parent body.
	public GetLocalPosition(): b2Vec2 {
		return this.m_localPosition;
	}

	/// Get the radius of this circle.
	public GetRadius(): number {
		return this.m_radius;
	}

	//--------------- Internals Below -------------------

	constructor(def: b2ShapeDef) {
		super(def);

		//b2Settings.b2Assert(def.type == e_circleShape);
		const circleDef: b2CircleDef = def as b2CircleDef;

		this.m_type = b2CircleShape.e_circleShape;
		this.m_localPosition.SetV(circleDef.localPosition);
		this.m_radius = circleDef.radius;

	}

	public UpdateSweepRadius(center: b2Vec2): void {
		// Update the sweep radius (maximum radius) as measured from
		// a local center point.
		//b2Vec2 d = m_localPosition - center;
		let dX: number = this.m_localPosition.x - center.x;
		const dY: number = this.m_localPosition.y - center.y;
		dX = Math.sqrt(dX * dX + dY * dY); // length
		//m_sweepRadius = d.Length() + m_radius - b2_toiSlop;
		this.m_sweepRadius = dX + this.m_radius - b2Settings.b2_toiSlop;
	}

	// Local position in parent body
	public m_localPosition: b2Vec2 = new b2Vec2();
	public m_radius: number;

}