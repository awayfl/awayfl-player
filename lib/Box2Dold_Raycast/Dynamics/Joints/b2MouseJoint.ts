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

import { b2Joint, b2MouseJointDef } from '../Joints';
import { b2Vec2, b2Mat22 } from '../../Common/Math';
import { b2Settings } from '../../Common/b2Settings';
import { b2TimeStep } from '../b2TimeStep';
import { b2Body } from '../b2Body';

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.

export class b2MouseJoint extends b2Joint {
	/// Implements b2Joint.
	public GetAnchor1(): b2Vec2 {
		return this.m_target;
	}

	/// Implements b2Joint.
	public GetAnchor2(): b2Vec2 {
		return this.m_body2.GetWorldPoint(this.m_localAnchor);
	}

	/// Implements b2Joint.
	public GetReactionForce(): b2Vec2 {
		return this.m_impulse;
	}

	/// Implements b2Joint.
	public GetReactionTorque(): number {
		return 0.0;
	}

	/// Use this to update the target point.
	public SetTarget(target: b2Vec2): void {
		if (this.m_body2.IsSleeping()) {
			this.m_body2.WakeUp();
		}
		this.m_target = target;
	}

	//--------------- Internals Below -------------------

	constructor(def: b2MouseJointDef) {
		super(def);

		this.m_target.SetV(def.target);
		//m_localAnchor = b2MulT(m_body2.m_xf, m_target);
		const tX: number = this.m_target.x - this.m_body2.m_xf.position.x;
		const tY: number = this.m_target.y - this.m_body2.m_xf.position.y;
		const tMat: b2Mat22 = this.m_body2.m_xf.R;
		this.m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
		this.m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);

		this.m_maxForce = def.maxForce;
		this.m_impulse.SetZero();

		const mass: number = this.m_body2.m_mass;

		// Frequency
		const omega: number = 2.0 * b2Settings.b2_pi * def.frequencyHz;

		// Damping coefficient
		const d: number = 2.0 * mass * def.dampingRatio * omega;

		// Spring stiffness
		const k: number = (def.timeStep * mass) * (omega * omega);

		// magic formulas
		//b2Assert(d + k > B2_FLT_EPSILON);
		this.m_gamma = 1.0 / (d + k);
		this.m_beta = k / (d + k);
	}

	// Presolve vars
	private K: b2Mat22 = new b2Mat22();
	private K1: b2Mat22 = new b2Mat22();
	private K2: b2Mat22 = new b2Mat22();
	public InitVelocityConstraints(step: b2TimeStep): void {
		const b: b2Body = this.m_body2;

		let tMat: b2Mat22;

		// Compute the effective mass matrix.
		//b2Vec2 r = b2Mul(b->m_xf.R, m_localAnchor - b->GetLocalCenter());
		tMat = b.m_xf.R;
		let rX: number = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		let rY: number = this.m_localAnchor.y - b.m_sweep.localCenter.y;
		const tX: number = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;

		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		const invMass: number = b.m_invMass;
		const invI: number = b.m_invI;

		//b2Mat22 K1;
		this.K1.col1.x = invMass;	this.K1.col2.x = 0.0;
		this.K1.col1.y = 0.0;		this.K1.col2.y = invMass;

		//b2Mat22 K2;
		this.K2.col1.x =  invI * rY * rY;	this.K2.col2.x = -invI * rX * rY;
		this.K2.col1.y = -invI * rX * rY;	this.K2.col2.y =  invI * rX * rX;

		//b2Mat22 K = K1 + K2;
		this.K.SetM(this.K1);
		this.K.AddM(this.K2);
		this.K.col1.x += this.m_gamma;
		this.K.col2.y += this.m_gamma;

		//m_ptpMass = K.Invert();
		this.K.Invert(this.m_mass);

		//m_C = b.m_position + r - m_target;
		this.m_C.x = b.m_sweep.c.x + rX - this.m_target.x;
		this.m_C.y = b.m_sweep.c.y + rY - this.m_target.y;

		// Cheat with some damping
		b.m_angularVelocity *= 0.98;

		// Warm starting.
		//b2Vec2 P = m_impulse;
		const PX: number = step.dt * this.m_impulse.x;
		const PY: number = step.dt * this.m_impulse.y;
		//b.m_linearVelocity += invMass * P;
		b.m_linearVelocity.x += invMass * PX;
		b.m_linearVelocity.y += invMass * PY;
		//b.m_angularVelocity += invI * b2Cross(r, P);
		b.m_angularVelocity += invI * (rX * PY - rY * PX);
	}

	public SolveVelocityConstraints(step: b2TimeStep): void {
		const b: b2Body = this.m_body2;

		let tMat: b2Mat22;
		let tX: number;
		let tY: number;

		// Compute the effective mass matrix.
		//b2Vec2 r = b2Mul(b->m_xf.R, m_localAnchor - b->GetLocalCenter());
		tMat = b.m_xf.R;
		let rX: number = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		let rY: number = this.m_localAnchor.y - b.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;

		// Cdot = v + cross(w, r)
		//b2Vec2 Cdot = b->m_linearVelocity + b2Cross(b->m_angularVelocity, r);
		const CdotX: number = b.m_linearVelocity.x + (-b.m_angularVelocity * rY);
		const CdotY: number = b.m_linearVelocity.y + (b.m_angularVelocity * rX);
		//b2Vec2 force = -step.inv_dt * b2Mul(m_mass, Cdot + (m_beta * step.inv_dt) * m_C + m_gamma * step.dt * m_force);
		tMat = this.m_mass;
		tX = CdotX + (this.m_beta * step.inv_dt) * this.m_C.x + this.m_gamma * step.dt * this.m_impulse.x;
		tY = CdotY + (this.m_beta * step.inv_dt) * this.m_C.y + this.m_gamma * step.dt * this.m_impulse.y;
		let forceX: number = -step.inv_dt * (tMat.col1.x * tX + tMat.col2.x * tY);
		let forceY: number = -step.inv_dt * (tMat.col1.y * tX + tMat.col2.y * tY);

		const oldForceX: number = this.m_impulse.x;
		const oldForceY: number = this.m_impulse.y;
		//m_force += force;
		this.m_impulse.x += forceX;
		this.m_impulse.y += forceY;
		const forceMagnitude: number = this.m_impulse.Length();
		if (forceMagnitude > this.m_maxForce) {
			//m_impulse *= m_maxForce / forceMagnitude;
			this.m_impulse.Multiply(this.m_maxForce / forceMagnitude);
		}
		//force = m_impulse - oldForce;
		forceX = this.m_impulse.x - oldForceX;
		forceY = this.m_impulse.y - oldForceY;

		//b2Vec2 P = step.dt * force;
		const PX: number = step.dt * forceX;
		const PY: number = step.dt * forceY;
		//b->m_linearVelocity += b->m_invMass * P;
		b.m_linearVelocity.x += b.m_invMass * PX;
		b.m_linearVelocity.y += b.m_invMass * PY;
		//b->m_angularVelocity += b->m_invI * b2Cross(r, P);
		b.m_angularVelocity += b.m_invI * (rX * PY - rY * PX);
	}

	public SolvePositionConstraints(): boolean {
		return true;
	}

	public m_localAnchor: b2Vec2 = new b2Vec2();
	public m_target: b2Vec2 = new b2Vec2();
	public m_impulse: b2Vec2 = new b2Vec2();

	public m_mass: b2Mat22 = new b2Mat22();	// effective mass for point-to-point constraint.
	public m_C: b2Vec2 = new b2Vec2();			// position error
	public m_maxForce: number;
	public m_beta: number;						// bias factor
	public m_gamma: number;						// softness
}
