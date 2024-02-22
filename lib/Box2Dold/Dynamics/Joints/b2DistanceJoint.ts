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

import { b2Joint, b2DistanceJointDef } from '../Joints';
import { b2Mat22, b2Math, b2Vec2 } from '../../Common/Math';
import { b2TimeStep } from '../b2TimeStep';
import { b2Body } from '../b2Body';
import { b2Settings } from '../../Common/b2Settings';

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.

export class b2DistanceJoint extends b2Joint {
	//--------------- Internals Below -------------------

	constructor(def: b2DistanceJointDef) {
		super(def);

		let tMat: b2Mat22;
		let tX: number;
		let tY: number;
		//this.m_localAnchor1 = def->localAnchor1;
		this.m_localAnchor1.SetV(def.localAnchor1);
		//this.m_localAnchor2 = def->localAnchor2;
		this.m_localAnchor2.SetV(def.localAnchor2);

		this.m_length = def.length;
		this.m_frequencyHz = def.frequencyHz;
		this.m_dampingRatio = def.dampingRatio;
		this.m_impulse = 0.0;
		this.m_gamma = 0.0;
		this.m_bias = 0.0;
		this.m_inv_dt = 0.0;
	}

	public InitVelocityConstraints(step: b2TimeStep): void {

		let tMat: b2Mat22;
		let tX: number;

		this.m_inv_dt = step.inv_dt;

		const b1: b2Body = this.m_body1;
		const b2: b2Body = this.m_body2;

		// Compute the effective mass matrix.
		//b2Vec2 r1 = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
		tMat = b1.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
		tMat = b2.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		//m_u = b2->m_sweep.c + r2 - b1->m_sweep.c - r1;
		this.m_u.x = b2.m_sweep.c.x + r2X - b1.m_sweep.c.x - r1X;
		this.m_u.y = b2.m_sweep.c.y + r2Y - b1.m_sweep.c.y - r1Y;

		// Handle singularity.
		//float32 length = m_u.Length();
		const length: number = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);
		if (length > b2Settings.b2_linearSlop) {
			//m_u *= 1.0 / length;
			this.m_u.Multiply(1.0 / length);
		} else {
			this.m_u.SetZero();
		}

		//float32 cr1u = b2Cross(r1, m_u);
		const cr1u: number = (r1X * this.m_u.y - r1Y * this.m_u.x);
		//float32 cr2u = b2Cross(r2, m_u);
		const cr2u: number = (r2X * this.m_u.y - r2Y * this.m_u.x);
		//m_mass = b1->m_invMass + b1->m_invI * cr1u * cr1u + b2->m_invMass + b2->m_invI * cr2u * cr2u;
		const invMass: number = b1.m_invMass + b1.m_invI * cr1u * cr1u + b2.m_invMass + b2.m_invI * cr2u * cr2u;
		//b2Settings.b2Assert(invMass > Number.MIN_VALUE);
		this.m_mass = 1.0 / invMass;

		if (this.m_frequencyHz > 0.0) {
			const C: number = length - this.m_length;

			// Frequency
			const omega: number = 2.0 * Math.PI * this.m_frequencyHz;

			// Damping coefficient
			const d: number = 2.0 * this.m_mass * this.m_dampingRatio * omega;

			// Spring stiffness
			const k: number = this.m_mass * omega * omega;

			// magic formulas
			this.m_gamma = 1.0 / (step.dt * (d + step.dt * k));
			this.m_bias = C * step.dt * k * this.m_gamma;

			this.m_mass = 1.0 / (invMass + this.m_gamma);
		}

		if (step.warmStarting) {
			this.m_impulse *= step.dtRatio;
			//b2Vec2 P = m_impulse * m_u;
			const PX: number = this.m_impulse * this.m_u.x;
			const PY: number = this.m_impulse * this.m_u.y;
			//b1->m_linearVelocity -= b1->m_invMass * P;
			b1.m_linearVelocity.x -= b1.m_invMass * PX;
			b1.m_linearVelocity.y -= b1.m_invMass * PY;
			//b1->m_angularVelocity -= b1->m_invI * b2Cross(r1, P);
			b1.m_angularVelocity -= b1.m_invI * (r1X * PY - r1Y * PX);
			//b2->m_linearVelocity += b2->m_invMass * P;
			b2.m_linearVelocity.x += b2.m_invMass * PX;
			b2.m_linearVelocity.y += b2.m_invMass * PY;
			//b2->m_angularVelocity += b2->m_invI * b2Cross(r2, P);
			b2.m_angularVelocity += b2.m_invI * (r2X * PY - r2Y * PX);
		} else {
			this.m_impulse = 0.0;
		}
	}

	public SolveVelocityConstraints(step: b2TimeStep): void {

		let tMat: b2Mat22;

		const b1: b2Body = this.m_body1;
		const b2: b2Body = this.m_body2;

		//b2Vec2 r1 = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
		tMat = b1.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
		let tX: number =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
		tMat = b2.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		// Cdot = dot(u, v + cross(w, r))
		//b2Vec2 v1 = b1->m_linearVelocity + b2Cross(b1->m_angularVelocity, r1);
		const v1X: number = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
		const v1Y: number = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
		//b2Vec2 v2 = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2);
		const v2X: number = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
		const v2Y: number = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);
		//float32 Cdot = b2Dot(m_u, v2 - v1);
		const Cdot: number = (this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y));

		const impulse: number = -this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
		this.m_impulse += impulse;

		//b2Vec2 P = impulse * m_u;
		const PX: number = impulse * this.m_u.x;
		const PY: number = impulse * this.m_u.y;
		//b1->m_linearVelocity -= b1->m_invMass * P;
		b1.m_linearVelocity.x -= b1.m_invMass * PX;
		b1.m_linearVelocity.y -= b1.m_invMass * PY;
		//b1->m_angularVelocity -= b1->m_invI * b2Cross(r1, P);
		b1.m_angularVelocity -= b1.m_invI * (r1X * PY - r1Y * PX);
		//b2->m_linearVelocity += b2->m_invMass * P;
		b2.m_linearVelocity.x += b2.m_invMass * PX;
		b2.m_linearVelocity.y += b2.m_invMass * PY;
		//b2->m_angularVelocity += b2->m_invI * b2Cross(r2, P);
		b2.m_angularVelocity += b2.m_invI * (r2X * PY - r2Y * PX);
	}

	public SolvePositionConstraints(): boolean {

		let tMat: b2Mat22;

		if (this.m_frequencyHz > 0.0) {
			return true;
		}

		const b1: b2Body = this.m_body1;
		const b2: b2Body = this.m_body2;

		//b2Vec2 r1 = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
		tMat = b1.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
		let tX: number =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
		tMat = b2.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		//b2Vec2 d = b2->m_sweep.c + r2 - b1->m_sweep.c - r1;
		let dX: number = b2.m_sweep.c.x + r2X - b1.m_sweep.c.x - r1X;
		let dY: number = b2.m_sweep.c.y + r2Y - b1.m_sweep.c.y - r1Y;

		//float32 length = d.Normalize();
		const length: number = Math.sqrt(dX * dX + dY * dY);
		dX /= length;
		dY /= length;
		//float32 C = length - m_length;
		let C: number = length - this.m_length;
		C = b2Math.b2Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);

		const impulse: number = -this.m_mass * C;
		//m_u = d;
		this.m_u.Set(dX, dY);
		//b2Vec2 P = impulse * m_u;
		const PX: number = impulse * this.m_u.x;
		const PY: number = impulse * this.m_u.y;

		//b1->m_sweep.c -= b1->m_invMass * P;
		b1.m_sweep.c.x -= b1.m_invMass * PX;
		b1.m_sweep.c.y -= b1.m_invMass * PY;
		//b1->m_sweep.a -= b1->m_invI * b2Cross(r1, P);
		b1.m_sweep.a -= b1.m_invI * (r1X * PY - r1Y * PX);
		//b2->m_sweep.c += b2->m_invMass * P;
		b2.m_sweep.c.x += b2.m_invMass * PX;
		b2.m_sweep.c.y += b2.m_invMass * PY;
		//b2->m_sweep.a -= b2->m_invI * b2Cross(r2, P);
		b2.m_sweep.a += b2.m_invI * (r2X * PY - r2Y * PX);

		b1.SynchronizeTransform();
		b2.SynchronizeTransform();

		return b2Math.b2Abs(C) < b2Settings.b2_linearSlop;

	}

	public GetAnchor1(): b2Vec2 {
		return this.m_body1.GetWorldPoint(this.m_localAnchor1);
	}

	public GetAnchor2(): b2Vec2 {
		return this.m_body2.GetWorldPoint(this.m_localAnchor2);
	}

	public GetReactionForce(): b2Vec2 {
		//b2Vec2 F = (m_inv_dt * m_impulse) * m_u;
		const F: b2Vec2 = new b2Vec2();
		F.SetV(this.m_u);
		F.Multiply(this.m_inv_dt * this.m_impulse);
		return F;
	}

	public GetReactionTorque(): number {
		//NOT_USED(invTimeStep);
		return 0.0;
	}

	public m_localAnchor1: b2Vec2 = new b2Vec2();
	public m_localAnchor2: b2Vec2 = new b2Vec2();
	public m_u: b2Vec2 = new b2Vec2();
	public m_frequencyHz: number;
	public m_dampingRatio: number;
	public m_gamma: number;
	public m_bias: number;
	public m_impulse: number;
	public m_mass: number;	// effective mass for the constraint.
	public m_length: number;
}
