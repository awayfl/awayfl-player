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

import { b2Joint, b2PulleyJointDef } from '../Joints';
import { b2Vec2, b2Math, b2Mat22 } from '../../Common/Math';
import { b2Body } from '../b2Body';
import { b2TimeStep } from '../b2TimeStep';
import { b2Settings } from '../../Common/b2Settings';

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// The pulley also enforces a maximum length limit on both sides. This is
/// useful to prevent one side of the pulley hitting the top.

export class b2PulleyJoint extends b2Joint {
	public GetAnchor1(): b2Vec2 {
		return this.m_body1.GetWorldPoint(this.m_localAnchor1);
	}

	public GetAnchor2(): b2Vec2 {
		return this.m_body2.GetWorldPoint(this.m_localAnchor2);
	}

	public GetReactionForce(): b2Vec2 {
		//b2Vec2 F = this.m_force * this.m_u2;
		const F: b2Vec2 = this.m_u2.Copy();
		F.Multiply(this.m_force);
		return F;
	}

	public GetReactionTorque(): number {
		return 0.0;
	}

	public GetGroundAnchor1(): b2Vec2 {
		//return this.m_ground.m_xf.position + this.m_groundAnchor1;
		const a: b2Vec2 = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor1);
		return a;
	}

	public GetGroundAnchor2(): b2Vec2 {
		//return this.m_ground.m_xf.position + this.m_groundAnchor2;
		const a: b2Vec2 = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor2);
		return a;
	}

	public GetLength1(): number {
		const p: b2Vec2 = this.m_body1.GetWorldPoint(this.m_localAnchor1);
		//b2Vec2 s = this.m_ground->this.m_xf.position + this.m_groundAnchor1;
		const sX: number = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		const sY: number = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		//b2Vec2 d = p - s;
		const dX: number = p.x - sX;
		const dY: number = p.y - sY;
		//return d.Length();
		return Math.sqrt(dX * dX + dY * dY);
	}

	public GetLength2(): number {
		const p: b2Vec2 = this.m_body2.GetWorldPoint(this.m_localAnchor2);
		//b2Vec2 s = this.m_ground->this.m_xf.position + this.m_groundAnchor2;
		const sX: number = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		const sY: number = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		//b2Vec2 d = p - s;
		const dX: number = p.x - sX;
		const dY: number = p.y - sY;
		//return d.Length();
		return Math.sqrt(dX * dX + dY * dY);
	}

	public GetRatio(): number {
		return this.m_ratio;
	}

	//--------------- Internals Below -------------------

	constructor(def: b2PulleyJointDef) {

		// parent
		super(def);

		let tMat: b2Mat22;
		let tX: number;
		let tY: number;

		this.m_ground = this.m_body1.m_world.m_groundBody;
		//this.m_groundAnchor1 = def->groundAnchor1 - this.m_ground->this.m_xf.position;
		this.m_groundAnchor1.x = def.groundAnchor1.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor1.y = def.groundAnchor1.y - this.m_ground.m_xf.position.y;
		//this.m_groundAnchor2 = def->groundAnchor2 - this.m_ground->this.m_xf.position;
		this.m_groundAnchor2.x = def.groundAnchor2.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor2.y = def.groundAnchor2.y - this.m_ground.m_xf.position.y;
		//this.m_localAnchor1 = def->localAnchor1;
		this.m_localAnchor1.SetV(def.localAnchor1);
		//this.m_localAnchor2 = def->localAnchor2;
		this.m_localAnchor2.SetV(def.localAnchor2);

		//b2Settings.b2Assert(def.ratio != 0.0);
		this.m_ratio = def.ratio;

		this.m_constant = def.length1 + this.m_ratio * def.length2;

		this.m_maxLength1 = b2Math.b2Min(def.maxLength1, this.m_constant - this.m_ratio * b2PulleyJoint.b2_minPulleyLength);
		this.m_maxLength2 = b2Math.b2Min(def.maxLength2, (this.m_constant - b2PulleyJoint.b2_minPulleyLength) / this.m_ratio);

		this.m_force = 0.0;
		this.m_limitForce1 = 0.0;
		this.m_limitForce2 = 0.0;

	}

	public InitVelocityConstraints(step: b2TimeStep): void {
		const b1: b2Body = this.m_body1;
		const b2: b2Body = this.m_body2;

		let tMat: b2Mat22;

		//b2Vec2 r1 = b2Mul(b1->this.m_xf.R, this.m_localAnchor1 - b1->GetLocalCenter());
		tMat = b1.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
		let tX: number =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(b2->this.m_xf.R, this.m_localAnchor2 - b2->GetLocalCenter());
		tMat = b2.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		//b2Vec2 p1 = b1->this.m_sweep.c + r1;
		const p1X: number = b1.m_sweep.c.x + r1X;
		const p1Y: number = b1.m_sweep.c.y + r1Y;
		//b2Vec2 p2 = b2->this.m_sweep.c + r2;
		const p2X: number = b2.m_sweep.c.x + r2X;
		const p2Y: number = b2.m_sweep.c.y + r2Y;

		//b2Vec2 s1 = this.m_ground->this.m_xf.position + this.m_groundAnchor1;
		const s1X: number = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		const s1Y: number = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		//b2Vec2 s2 = this.m_ground->this.m_xf.position + this.m_groundAnchor2;
		const s2X: number = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		const s2Y: number = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;

		// Get the pulley axes.
		//this.m_u1 = p1 - s1;
		this.m_u1.Set(p1X - s1X, p1Y - s1Y);
		//this.m_u2 = p2 - s2;
		this.m_u2.Set(p2X - s2X, p2Y - s2Y);

		const length1: number = this.m_u1.Length();
		const length2: number = this.m_u2.Length();

		if (length1 > b2Settings.b2_linearSlop) {
			//this.m_u1 *= 1.0f / length1;
			this.m_u1.Multiply(1.0 / length1);
		} else {
			this.m_u1.SetZero();
		}

		if (length2 > b2Settings.b2_linearSlop) {
			//this.m_u2 *= 1.0f / length2;
			this.m_u2.Multiply(1.0 / length2);
		} else {
			this.m_u2.SetZero();
		}

		const C: number = this.m_constant - length1 - this.m_ratio * length2;
		if (C > 0.0) {
			this.m_state = b2PulleyJoint.e_inactiveLimit;
			this.m_force = 0.0;
		} else {
			this.m_state = b2PulleyJoint.e_atUpperLimit;
			this.m_positionImpulse = 0.0;
		}

		if (length1 < this.m_maxLength1) {
			this.m_limitState1 = b2PulleyJoint.e_inactiveLimit;
			this.m_limitForce1 = 0.0;
		} else {
			this.m_limitState1 = b2PulleyJoint.e_atUpperLimit;
			this.m_limitPositionImpulse1 = 0.0;
		}

		if (length2 < this.m_maxLength2) {
			this.m_limitState2 = b2PulleyJoint.e_inactiveLimit;
			this.m_limitForce2 = 0.0;
		} else {
			this.m_limitState2 = b2PulleyJoint.e_atUpperLimit;
			this.m_limitPositionImpulse2 = 0.0;
		}

		// Compute effective mass.
		//var cr1u1:number = b2Cross(r1, this.m_u1);
		const cr1u1: number = r1X * this.m_u1.y - r1Y * this.m_u1.x;
		//var cr2u2:number = b2Cross(r2, this.m_u2);
		const cr2u2: number = r2X * this.m_u2.y - r2Y * this.m_u2.x;

		this.m_limitMass1 = b1.m_invMass + b1.m_invI * cr1u1 * cr1u1;
		this.m_limitMass2 = b2.m_invMass + b2.m_invI * cr2u2 * cr2u2;
		this.m_pulleyMass = this.m_limitMass1 + this.m_ratio * this.m_ratio * this.m_limitMass2;
		//b2Settings.b2Assert(this.m_limitMass1 > Number.MIN_VALUE);
		//b2Settings.b2Assert(this.m_limitMass2 > Number.MIN_VALUE);
		//b2Settings.b2Assert(this.m_pulleyMass > Number.MIN_VALUE);
		this.m_limitMass1 = 1.0 / this.m_limitMass1;
		this.m_limitMass2 = 1.0 / this.m_limitMass2;
		this.m_pulleyMass = 1.0 / this.m_pulleyMass;

		if (step.warmStarting) {
			// Warm starting.
			//b2Vec2 P1 = step.dt * (-this.m_force - this.m_limitForce1) * this.m_u1;
			//b2Vec2 P1 = step.dt * (-this.m_force - this.m_limitForce1) * this.m_u1;
			const P1X: number = step.dt * (-this.m_force - this.m_limitForce1) * this.m_u1.x;
			const P1Y: number = step.dt * (-this.m_force - this.m_limitForce1) * this.m_u1.y;
			//b2Vec2 P2 = step.dt * (-this.m_ratio * this.m_force - this.m_limitForce2) * this.m_u2;
			//b2Vec2 P2 = step.dt * (-this.m_ratio * this.m_force - this.m_limitForce2) * this.m_u2;
			const P2X: number = step.dt * (-this.m_ratio * this.m_force - this.m_limitForce2) * this.m_u2.x;
			const P2Y: number = step.dt * (-this.m_ratio * this.m_force - this.m_limitForce2) * this.m_u2.y;
			//b1.m_linearVelocity += b1.m_invMass * P1;
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			//b1.m_angularVelocity += b1.m_invI * b2Cross(r1, P1);
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
			//b2.m_linearVelocity += b2.m_invMass * P2;
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			//b2.m_angularVelocity += b2.m_invI * b2Cross(r2, P2);
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		} else {
			this.m_force = 0.0;
			this.m_limitForce1 = 0.0;
			this.m_limitForce2 = 0.0;
		}
	}

	public SolveVelocityConstraints(step: b2TimeStep): void {
		const b1: b2Body = this.m_body1;
		const b2: b2Body = this.m_body2;

		let tMat: b2Mat22;

		//b2Vec2 r1 = b2Mul(b1->this.m_xf.R, this.m_localAnchor1 - b1->GetLocalCenter());
		tMat = b1.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
		let tX: number =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(b2->this.m_xf.R, this.m_localAnchor2 - b2->GetLocalCenter());
		tMat = b2.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		// temp vars
		let v1X: number;
		let v1Y: number;
		let v2X: number;
		let v2Y: number;
		let P1X: number;
		let P1Y: number;
		let P2X: number;
		let P2Y: number;
		let Cdot: number;
		let force: number;
		let oldForce: number;

		if (this.m_state == b2PulleyJoint.e_atUpperLimit) {
			//b2Vec2 v1 = b1->this.m_linearVelocity + b2Cross(b1->this.m_angularVelocity, r1);
			v1X = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
			v1Y = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
			//b2Vec2 v2 = b2->this.m_linearVelocity + b2Cross(b2->this.m_angularVelocity, r2);
			v2X = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
			v2Y = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);

			//Cdot = -b2Dot(this.m_u1, v1) - this.m_ratio * b2Dot(this.m_u2, v2);
			Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y) - this.m_ratio * (this.m_u2.x * v2X + this.m_u2.y * v2Y);
			force = -step.inv_dt * this.m_pulleyMass * Cdot;
			oldForce = this.m_force;
			this.m_force = b2Math.b2Max(0.0, this.m_force + force);
			force = this.m_force - oldForce;

			//b2Vec2 P1 = -step.dt * force * this.m_u1;
			P1X = -step.dt * force * this.m_u1.x;
			P1Y = -step.dt * force * this.m_u1.y;
			//b2Vec2 P2 = -step.dt * this.m_ratio * force * this.m_u2;
			P2X = -step.dt * this.m_ratio * force * this.m_u2.x;
			P2Y = -step.dt * this.m_ratio * force * this.m_u2.y;
			//b1.m_linearVelocity += b1.m_invMass * P1;
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			//b1.m_angularVelocity += b1.m_invI * b2Cross(r1, P1);
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
			//b2.m_linearVelocity += b2.m_invMass * P2;
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			//b2.m_angularVelocity += b2.m_invI * b2Cross(r2, P2);
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		}

		if (this.m_limitState1 == b2PulleyJoint.e_atUpperLimit) {
			//b2Vec2 v1 = b1->this.m_linearVelocity + b2Cross(b1->this.m_angularVelocity, r1);
			v1X = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
			v1Y = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);

			//float32 Cdot = -b2Dot(this.m_u1, v1);
			Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y);
			force = -step.inv_dt * this.m_limitMass1 * Cdot;
			oldForce = this.m_limitForce1;
			this.m_limitForce1 = b2Math.b2Max(0.0, this.m_limitForce1 + force);
			force = this.m_limitForce1 - oldForce;

			//b2Vec2 P1 = -step.dt * force * this.m_u1;
			P1X = -step.dt * force * this.m_u1.x;
			P1Y = -step.dt * force * this.m_u1.y;
			//b1.m_linearVelocity += b1->this.m_invMass * P1;
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			//b1.m_angularVelocity += b1->this.m_invI * b2Cross(r1, P1);
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
		}

		if (this.m_limitState2 == b2PulleyJoint.e_atUpperLimit) {
			//b2Vec2 v2 = b2->this.m_linearVelocity + b2Cross(b2->this.m_angularVelocity, r2);
			v2X = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
			v2Y = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);

			//float32 Cdot = -b2Dot(this.m_u2, v2);
			Cdot = -(this.m_u2.x * v2X + this.m_u2.y * v2Y);
			force = -step.inv_dt * this.m_limitMass2 * Cdot;
			oldForce = this.m_limitForce2;
			this.m_limitForce2 = b2Math.b2Max(0.0, this.m_limitForce2 + force);
			force = this.m_limitForce2 - oldForce;

			//b2Vec2 P2 = -step.dt * force * this.m_u2;
			P2X = -step.dt * force * this.m_u2.x;
			P2Y = -step.dt * force * this.m_u2.y;
			//b2->this.m_linearVelocity += b2->this.m_invMass * P2;
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			//b2->this.m_angularVelocity += b2->this.m_invI * b2Cross(r2, P2);
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		}
	}

	public SolvePositionConstraints(): boolean {
		const b1: b2Body = this.m_body1;
		const b2: b2Body = this.m_body2;

		let tMat: b2Mat22;

		//b2Vec2 s1 = this.m_ground->this.m_xf.position + this.m_groundAnchor1;
		const s1X: number = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		const s1Y: number = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		//b2Vec2 s2 = this.m_ground->this.m_xf.position + this.m_groundAnchor2;
		const s2X: number = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		const s2Y: number = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;

		// temp vars
		let r1X: number;
		let r1Y: number;
		let r2X: number;
		let r2Y: number;
		let p1X: number;
		let p1Y: number;
		let p2X: number;
		let p2Y: number;
		let length1: number;
		let length2: number;
		let C: number;
		let impulse: number;
		let oldImpulse: number;
		let oldLimitPositionImpulse: number;

		let tX: number;

		let linearError: number = 0.0;

		if (this.m_state == b2PulleyJoint.e_atUpperLimit) {
			//b2Vec2 r1 = b2Mul(b1->this.m_xf.R, this.m_localAnchor1 - b1->GetLocalCenter());
			tMat = b1.m_xf.R;
			r1X = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			//b2Vec2 r2 = b2Mul(b2->this.m_xf.R, this.m_localAnchor2 - b2->GetLocalCenter());
			tMat = b2.m_xf.R;
			r2X = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;

			//b2Vec2 p1 = b1->this.m_sweep.c + r1;
			p1X = b1.m_sweep.c.x + r1X;
			p1Y = b1.m_sweep.c.y + r1Y;
			//b2Vec2 p2 = b2->this.m_sweep.c + r2;
			p2X = b2.m_sweep.c.x + r2X;
			p2Y = b2.m_sweep.c.y + r2Y;

			// Get the pulley axes.
			//this.m_u1 = p1 - s1;
			this.m_u1.Set(p1X - s1X, p1Y - s1Y);
			//this.m_u2 = p2 - s2;
			this.m_u2.Set(p2X - s2X, p2Y - s2Y);

			length1 = this.m_u1.Length();
			length2 = this.m_u2.Length();

			if (length1 > b2Settings.b2_linearSlop) {
				//this.m_u1 *= 1.0f / length1;
				this.m_u1.Multiply(1.0 / length1);
			} else {
				this.m_u1.SetZero();
			}

			if (length2 > b2Settings.b2_linearSlop) {
				//this.m_u2 *= 1.0f / length2;
				this.m_u2.Multiply(1.0 / length2);
			} else {
				this.m_u2.SetZero();
			}

			C = this.m_constant - length1 - this.m_ratio * length2;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -this.m_pulleyMass * C;

			oldImpulse = this.m_positionImpulse;
			this.m_positionImpulse = b2Math.b2Max(0.0, this.m_positionImpulse + impulse);
			impulse = this.m_positionImpulse - oldImpulse;

			p1X = -impulse * this.m_u1.x;
			p1Y = -impulse * this.m_u1.y;
			p2X = -this.m_ratio * impulse * this.m_u2.x;
			p2Y = -this.m_ratio * impulse * this.m_u2.y;

			b1.m_sweep.c.x += b1.m_invMass * p1X;
			b1.m_sweep.c.y += b1.m_invMass * p1Y;
			b1.m_sweep.a += b1.m_invI * (r1X * p1Y - r1Y * p1X);
			b2.m_sweep.c.x += b2.m_invMass * p2X;
			b2.m_sweep.c.y += b2.m_invMass * p2Y;
			b2.m_sweep.a += b2.m_invI * (r2X * p2Y - r2Y * p2X);

			b1.SynchronizeTransform();
			b2.SynchronizeTransform();
		}

		if (this.m_limitState1 == b2PulleyJoint.e_atUpperLimit) {
			//b2Vec2 r1 = b2Mul(b1->this.m_xf.R, this.m_localAnchor1 - b1->GetLocalCenter());
			tMat = b1.m_xf.R;
			r1X = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			//b2Vec2 p1 = b1->this.m_sweep.c + r1;
			p1X = b1.m_sweep.c.x + r1X;
			p1Y = b1.m_sweep.c.y + r1Y;

			//this.m_u1 = p1 - s1;
			this.m_u1.Set(p1X - s1X, p1Y - s1Y);

			length1 = this.m_u1.Length();

			if (length1 > b2Settings.b2_linearSlop) {
				//this.m_u1 *= 1.0 / length1;
				this.m_u1.x *= 1.0 / length1;
				this.m_u1.y *= 1.0 / length1;
			} else {
				this.m_u1.SetZero();
			}

			C = this.m_maxLength1 - length1;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -this.m_limitMass1 * C;
			oldLimitPositionImpulse = this.m_limitPositionImpulse1;
			this.m_limitPositionImpulse1 = b2Math.b2Max(0.0, this.m_limitPositionImpulse1 + impulse);
			impulse = this.m_limitPositionImpulse1 - oldLimitPositionImpulse;

			//P1 = -impulse * this.m_u1;
			p1X = -impulse * this.m_u1.x;
			p1Y = -impulse * this.m_u1.y;

			b1.m_sweep.c.x += b1.m_invMass * p1X;
			b1.m_sweep.c.y += b1.m_invMass * p1Y;
			//b1.m_rotation += b1.m_invI * b2Cross(r1, P1);
			b1.m_sweep.a += b1.m_invI * (r1X * p1Y - r1Y * p1X);

			b1.SynchronizeTransform();
		}

		if (this.m_limitState2 == b2PulleyJoint.e_atUpperLimit) {
			//b2Vec2 r2 = b2Mul(b2->this.m_xf.R, this.m_localAnchor2 - b2->GetLocalCenter());
			tMat = b2.m_xf.R;
			r2X = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			//b2Vec2 p2 = b2->this.m_position + r2;
			p2X = b2.m_sweep.c.x + r2X;
			p2Y = b2.m_sweep.c.y + r2Y;

			//this.m_u2 = p2 - s2;
			this.m_u2.Set(p2X - s2X, p2Y - s2Y);

			length2 = this.m_u2.Length();

			if (length2 > b2Settings.b2_linearSlop) {
				//this.m_u2 *= 1.0 / length2;
				this.m_u2.x *= 1.0 / length2;
				this.m_u2.y *= 1.0 / length2;
			} else {
				this.m_u2.SetZero();
			}

			C = this.m_maxLength2 - length2;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -this.m_limitMass2 * C;
			oldLimitPositionImpulse = this.m_limitPositionImpulse2;
			this.m_limitPositionImpulse2 = b2Math.b2Max(0.0, this.m_limitPositionImpulse2 + impulse);
			impulse = this.m_limitPositionImpulse2 - oldLimitPositionImpulse;

			//P2 = -impulse * this.m_u2;
			p2X = -impulse * this.m_u2.x;
			p2Y = -impulse * this.m_u2.y;

			//b2.m_sweep.c += b2.m_invMass * P2;
			b2.m_sweep.c.x += b2.m_invMass * p2X;
			b2.m_sweep.c.y += b2.m_invMass * p2Y;
			//b2.m_sweep.a += b2.m_invI * b2Cross(r2, P2);
			b2.m_sweep.a += b2.m_invI * (r2X * p2Y - r2Y * p2X);

			b2.SynchronizeTransform();
		}

		return linearError < b2Settings.b2_linearSlop;
	}

	public m_ground: b2Body;
	public m_groundAnchor1: b2Vec2 = new b2Vec2();
	public m_groundAnchor2: b2Vec2 = new b2Vec2();
	public m_localAnchor1: b2Vec2 = new b2Vec2();
	public m_localAnchor2: b2Vec2 = new b2Vec2();

	public m_u1: b2Vec2 = new b2Vec2();
	public m_u2: b2Vec2 = new b2Vec2();

	public m_constant: number;
	public m_ratio: number;

	public m_maxLength1: number;
	public m_maxLength2: number;

	// Effective masses
	public m_pulleyMass: number;
	public m_limitMass1: number;
	public m_limitMass2: number;

	// Impulses for accumulation/warm starting.
	public m_force: number;
	public m_limitForce1: number;
	public m_limitForce2: number;

	// Position impulses for accumulation.
	public m_positionImpulse: number;
	public m_limitPositionImpulse1: number;
	public m_limitPositionImpulse2: number;

	public m_state: number /** int */;
	public m_limitState1: number /** int */;
	public m_limitState2: number /** int */;

	// static
	public static readonly b2_minPulleyLength: number = 2.0;
}