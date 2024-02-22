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

import { b2Joint, b2GearJointDef } from '../Joints';
import { b2Vec2, b2Mat22 } from '../../Common/Math';
import { b2Body } from '../b2Body';
import { b2TimeStep } from '../b2TimeStep';
import { b2Settings } from '../../Common/b2Settings';
import { b2Jacobian } from './b2Jacobian';
import { b2RevoluteJoint } from './b2RevoluteJoint';
import { b2PrismaticJoint } from './b2PrismaticJoint';

/// A gear joint is used to connect two joints together. Either joint
/// can be a revolute or prismatic joint. You specify a gear ratio
/// to bind the motions together:
/// coordinate1 + ratio * coordinate2 = constant
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
/// @warning The revolute and prismatic joints must be attached to
/// fixed bodies (which must be body1 on those joints).

export class b2GearJoint extends b2Joint {
	public GetAnchor1(): b2Vec2 {
		//return this.m_body1->GetWorldPoint(this.m_localAnchor1);
		return this.m_body1.GetWorldPoint(this.m_localAnchor1);
	}

	public GetAnchor2(): b2Vec2 {
		//return this.m_body2->GetWorldPoint(this.m_localAnchor2);
		return this.m_body2.GetWorldPoint(this.m_localAnchor2);
	}

	public GetReactionForce(): b2Vec2 {
		// TODO_ERIN not tested
		const F: b2Vec2 = new b2Vec2(this.m_force * this.m_J.linear2.x, this.m_force * this.m_J.linear2.y);
		return F;
	}

	public GetReactionTorque(): number {
		// TODO_ERIN not tested
		//b2Vec2 r = b2Mul(m_body2->m_xf.R, m_localAnchor2 - m_body2->GetLocalCenter());
		const tMat: b2Mat22 = this.m_body2.m_xf.R;
		let rX: number = this.m_localAnchor1.x - this.m_body2.m_sweep.localCenter.x;
		let rY: number = this.m_localAnchor1.y - this.m_body2.m_sweep.localCenter.y;
		let tX: number = tMat.col1.x * rX + tMat.col2.x * rY;
		rY = tMat.col1.y * rX + tMat.col2.y * rY;
		rX = tX;
		//b2Vec2 F = m_force * m_J.linear2;
		//float32 T = m_force * m_J.angular2 - b2Cross(r, F);
		tX = this.m_force * this.m_J.angular2 - (rX * (this.m_force * this.m_J.linear2.y) - rY * (this.m_force * this.m_J.linear2.x));
		return tX;
	}

	public GetRatio(): number {
		return this.m_ratio;
	}

	//--------------- Internals Below -------------------

	constructor(def: b2GearJointDef) {
		// parent constructor
		super(def);

		const type1: number /** int */ = def.joint1.m_type;
		const type2: number /** int */ = def.joint2.m_type;

		//b2Settings.b2Assert(type1 == b2Joint.e_revoluteJoint || type1 == b2Joint.e_prismaticJoint);
		//b2Settings.b2Assert(type2 == b2Joint.e_revoluteJoint || type2 == b2Joint.e_prismaticJoint);
		//b2Settings.b2Assert(def.joint1.m_body1.IsStatic());
		//b2Settings.b2Assert(def.joint2.m_body1.IsStatic());

		this.m_revolute1 = null;
		this.m_prismatic1 = null;
		this.m_revolute2 = null;
		this.m_prismatic2 = null;

		let coordinate1: number;
		let coordinate2: number;

		this.m_ground1 = def.joint1.m_body1;
		this.m_body1 = def.joint1.m_body2;
		if (type1 == b2Joint.e_revoluteJoint) {
			this.m_revolute1 = def.joint1 as b2RevoluteJoint;
			this.m_groundAnchor1.SetV(this.m_revolute1.m_localAnchor1);
			this.m_localAnchor1.SetV(this.m_revolute1.m_localAnchor2);
			coordinate1 = this.m_revolute1.GetJointAngle();
		} else {
			this.m_prismatic1 = def.joint1 as b2PrismaticJoint;
			this.m_groundAnchor1.SetV(this.m_prismatic1.m_localAnchor1);
			this.m_localAnchor1.SetV(this.m_prismatic1.m_localAnchor2);
			coordinate1 = this.m_prismatic1.GetJointTranslation();
		}

		this.m_ground2 = def.joint2.m_body1;
		this.m_body2 = def.joint2.m_body2;
		if (type2 == b2Joint.e_revoluteJoint) {
			this.m_revolute2 = def.joint2 as b2RevoluteJoint;
			this.m_groundAnchor2.SetV(this.m_revolute2.m_localAnchor1);
			this.m_localAnchor2.SetV(this.m_revolute2.m_localAnchor2);
			coordinate2 = this.m_revolute2.GetJointAngle();
		} else {
			this.m_prismatic2 = def.joint2 as b2PrismaticJoint;
			this.m_groundAnchor2.SetV(this.m_prismatic2.m_localAnchor1);
			this.m_localAnchor2.SetV(this.m_prismatic2.m_localAnchor2);
			coordinate2 = this.m_prismatic2.GetJointTranslation();
		}

		this.m_ratio = def.ratio;

		this.m_constant = coordinate1 + this.m_ratio * coordinate2;

		this.m_force = 0.0;

	}

	public InitVelocityConstraints(step: b2TimeStep): void {
		const g1: b2Body = this.m_ground1;
		const g2: b2Body = this.m_ground2;
		const b1: b2Body = this.m_body1;
		const b2: b2Body = this.m_body2;

		// temp vars
		let ugX: number;
		let ugY: number;
		let rX: number;
		let rY: number;
		let tMat: b2Mat22;
		let tVec: b2Vec2;
		let crug: number;
		let tX: number;

		let K: number = 0.0;
		this.m_J.SetZero();

		if (this.m_revolute1) {
			this.m_J.angular1 = -1.0;
			K += b1.m_invI;
		} else {
			//b2Vec2 ug = b2MulMV(g1->m_xf.R, m_prismatic1->m_localXAxis1);
			tMat = g1.m_xf.R;
			tVec = this.m_prismatic1.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//b2Vec2 r = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
			tMat = b1.m_xf.R;
			rX = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
			rY = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;

			//var crug:number = b2Cross(r, ug);
			crug = rX * ugY - rY * ugX;
			//m_J.linear1 = -ug;
			this.m_J.linear1.Set(-ugX, -ugY);
			this.m_J.angular1 = -crug;
			K += b1.m_invMass + b1.m_invI * crug * crug;
		}

		if (this.m_revolute2) {
			this.m_J.angular2 = -this.m_ratio;
			K += this.m_ratio * this.m_ratio * b2.m_invI;
		} else {
			//b2Vec2 ug = b2Mul(g2->m_xf.R, m_prismatic2->m_localXAxis1);
			tMat = g2.m_xf.R;
			tVec = this.m_prismatic2.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//b2Vec2 r = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
			tMat = b2.m_xf.R;
			rX = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
			rY = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;

			//float32 crug = b2Cross(r, ug);
			crug = rX * ugY - rY * ugX;
			//m_J.linear2 = -m_ratio * ug;
			this.m_J.linear2.Set(-this.m_ratio * ugX, -this.m_ratio * ugY);
			this.m_J.angular2 = -this.m_ratio * crug;
			K += this.m_ratio * this.m_ratio * (b2.m_invMass + b2.m_invI * crug * crug);
		}

		// Compute effective mass.
		//b2Settings.b2Assert(K > 0.0);
		this.m_mass = 1.0 / K;

		if (step.warmStarting) {
			// Warm starting.
			const P: number = step.dt * this.m_force;
			//b1.m_linearVelocity += b1.m_invMass * P * m_J.linear1;
			b1.m_linearVelocity.x += b1.m_invMass * P * this.m_J.linear1.x;
			b1.m_linearVelocity.y += b1.m_invMass * P * this.m_J.linear1.y;
			b1.m_angularVelocity += b1.m_invI * P * this.m_J.angular1;
			//b2.m_linearVelocity += b2.m_invMass * P * m_J.linear2;
			b2.m_linearVelocity.x += b2.m_invMass * P * this.m_J.linear2.x;
			b2.m_linearVelocity.y += b2.m_invMass * P * this.m_J.linear2.y;
			b2.m_angularVelocity += b2.m_invI * P * this.m_J.angular2;
		} else {
			this.m_force = 0.0;
		}
	}

	public SolveVelocityConstraints(step: b2TimeStep): void {
		const b1: b2Body = this.m_body1;
		const b2: b2Body = this.m_body2;

		const Cdot: number = this.m_J.Compute(b1.m_linearVelocity, b1.m_angularVelocity,
			b2.m_linearVelocity, b2.m_angularVelocity);

		const force: number = -step.inv_dt * this.m_mass * Cdot;
		this.m_force += force;

		const P: number = step.dt * force;
		b1.m_linearVelocity.x += b1.m_invMass * P * this.m_J.linear1.x;
		b1.m_linearVelocity.y += b1.m_invMass * P * this.m_J.linear1.y;
		b1.m_angularVelocity  += b1.m_invI * P * this.m_J.angular1;
		b2.m_linearVelocity.x += b2.m_invMass * P * this.m_J.linear2.x;
		b2.m_linearVelocity.y += b2.m_invMass * P * this.m_J.linear2.y;
		b2.m_angularVelocity  += b2.m_invI * P * this.m_J.angular2;
	}

	public SolvePositionConstraints(): boolean {
		const linearError: number = 0.0;

		const b1: b2Body = this.m_body1;
		const b2: b2Body = this.m_body2;

		let coordinate1: number;
		let coordinate2: number;
		if (this.m_revolute1) {
			coordinate1 = this.m_revolute1.GetJointAngle();
		} else {
			coordinate1 = this.m_prismatic1.GetJointTranslation();
		}

		if (this.m_revolute2) {
			coordinate2 = this.m_revolute2.GetJointAngle();
		} else {
			coordinate2 = this.m_prismatic2.GetJointTranslation();
		}

		const C: number = this.m_constant - (coordinate1 + this.m_ratio * coordinate2);

		const impulse: number = -this.m_mass * C;

		b1.m_sweep.c.x += b1.m_invMass * impulse * this.m_J.linear1.x;
		b1.m_sweep.c.y += b1.m_invMass * impulse * this.m_J.linear1.y;
		b1.m_sweep.a += b1.m_invI * impulse * this.m_J.angular1;
		b2.m_sweep.c.x += b2.m_invMass * impulse * this.m_J.linear2.x;
		b2.m_sweep.c.y += b2.m_invMass * impulse * this.m_J.linear2.y;
		b2.m_sweep.a += b2.m_invI * impulse * this.m_J.angular2;

		b1.SynchronizeTransform();
		b2.SynchronizeTransform();

		return linearError < b2Settings.b2_linearSlop;
	}

	public m_ground1: b2Body;
	public m_ground2: b2Body;

	// One of these is NULL.
	public m_revolute1: b2RevoluteJoint;
	public m_prismatic1: b2PrismaticJoint;

	// One of these is NULL.
	public m_revolute2: b2RevoluteJoint;
	public m_prismatic2: b2PrismaticJoint;

	public m_groundAnchor1: b2Vec2 = new b2Vec2();
	public m_groundAnchor2: b2Vec2 = new b2Vec2();

	public m_localAnchor1: b2Vec2 = new b2Vec2();
	public m_localAnchor2: b2Vec2 = new b2Vec2();

	public m_J: b2Jacobian = new b2Jacobian();

	public m_constant: number;
	public m_ratio: number;

	// Effective mass
	public m_mass: number;

	// Impulse for accumulation/warm starting.
	public m_force: number;
}