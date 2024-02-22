import { b2Vec2, b2Mat22 } from '../../Common/Math';
import { b2Body } from '../b2Body';
import { b2Settings } from '../../Common/b2Settings';
import { b2TimeStep } from '../b2TimeStep';
import { b2Joint, b2RevoluteJoint, b2PrismaticJoint, b2GearJointDef, b2Jacobian } from '../Joints';

/**
* A gear joint is used to connect two joints together. Either joint
* can be a revolute or prismatic joint. You specify a gear ratio
* to bind the motions together:
* coordinate1 + ratio * coordinate2 = constant
* The ratio can be negative or positive. If one joint is a revolute joint
* and the other joint is a prismatic joint, then the ratio will have units
* of length or units of 1/length.
* @warning The revolute and prismatic joints must be attached to
* fixed bodies (which must be body1 on those joints).
* @see b2GearJointDef
*/
export class b2GearJoint extends b2Joint {
	/** @inheritDoc */
	public GetAnchorA(): b2Vec2 {
		//return this.m_bodyA->GetWorldPoint(this.m_localAnchor1);
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}

	/** @inheritDoc */
	public GetAnchorB(): b2Vec2 {
		//return this.m_bodyB->GetWorldPoint(this.m_localAnchor2);
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}

	/** @inheritDoc */
	public GetReactionForce(inv_dt: number): b2Vec2 {
		// TODO_ERIN not tested
		// b2Vec2 P = this.m_impulse * this.m_J.linear2;
		//return inv_dt * P;
		return new b2Vec2(inv_dt * this.m_impulse * this.m_J.linearB.x, inv_dt * this.m_impulse * this.m_J.linearB.y);
	}

	/** @inheritDoc */
	public GetReactionTorque(inv_dt: number): number {
		// TODO_ERIN not tested
		//b2Vec2 r = b2Mul(m_bodyB->m_xf.R, m_localAnchor2 - m_bodyB->GetLocalCenter());
		const tMat: b2Mat22 = this.m_bodyB.m_xf.R;
		let rX: number = this.m_localAnchor1.x - this.m_bodyB.m_sweep.localCenter.x;
		let rY: number = this.m_localAnchor1.y - this.m_bodyB.m_sweep.localCenter.y;
		const tX: number = tMat.col1.x * rX + tMat.col2.x * rY;
		rY = tMat.col1.y * rX + tMat.col2.y * rY;
		rX = tX;
		//b2Vec2 P = m_impulse * m_J.linearB;
		const PX: number = this.m_impulse * this.m_J.linearB.x;
		const PY: number = this.m_impulse * this.m_J.linearB.y;
		//float32 L = this.m_impulse * this.m_J.angularB - b2Cross(r, P);
		//return inv_dt * L;
		return inv_dt * (this.m_impulse * this.m_J.angularB - rX * PY + rY * PX);
	}

	/**
	 * Get the gear ratio.
	 */
	public GetRatio(): number {
		return this.m_ratio;
	}

	/**
	 * Set the gear ratio.
	 */
	public SetRatio(ratio: number): void {
		//b2Settings.b2Assert(b2Math.b2IsValid(ratio));
		this.m_ratio = ratio;
	}

	//--------------- Internals Below -------------------

	/** @private */
	constructor(def: b2GearJointDef) {
		// parent constructor
		super(def);

		const type1: number /** int */ = def.joint1.m_type;
		const type2: number /** int */ = def.joint2.m_type;

		//b2Settings.b2Assert(type1 == b2Joint.e_revoluteJoint || type1 == b2Joint.e_prismaticJoint);
		//b2Settings.b2Assert(type2 == b2Joint.e_revoluteJoint || type2 == b2Joint.e_prismaticJoint);
		//b2Settings.b2Assert(def.joint1.GetBodyA().GetType() == b2Body.b2_staticBody);
		//b2Settings.b2Assert(def.joint2.GetBodyA().GetType() == b2Body.b2_staticBody);

		this.m_revolute1 = null;
		this.m_prismatic1 = null;
		this.m_revolute2 = null;
		this.m_prismatic2 = null;

		let coordinate1: number;
		let coordinate2: number;

		this.m_ground1 = def.joint1.GetBodyA();
		this.m_bodyA = def.joint1.GetBodyB();
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

		this.m_ground2 = def.joint2.GetBodyA();
		this.m_bodyB = def.joint2.GetBodyB();
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

		this.m_impulse = 0.0;

	}

	public InitVelocityConstraints(step: b2TimeStep): void {
		const g1: b2Body = this.m_ground1;
		const g2: b2Body = this.m_ground2;
		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

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
			this.m_J.angularA = -1.0;
			K += bA.m_invI;
		} else {
			//b2Vec2 ug = b2MulMV(g1->m_xf.R, m_prismatic1->m_localXAxis1);
			tMat = g1.m_xf.R;
			tVec = this.m_prismatic1.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//b2Vec2 r = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
			tMat = bA.m_xf.R;
			rX = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			rY = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;

			//var crug:number = b2Cross(r, ug);
			crug = rX * ugY - rY * ugX;
			//this.m_J.linearA = -ug;
			this.m_J.linearA.Set(-ugX, -ugY);
			this.m_J.angularA = -crug;
			K += bA.m_invMass + bA.m_invI * crug * crug;
		}

		if (this.m_revolute2) {
			this.m_J.angularB = -this.m_ratio;
			K += this.m_ratio * this.m_ratio * bB.m_invI;
		} else {
			//b2Vec2 ug = b2Mul(g2->m_xf.R, m_prismatic2->m_localXAxis1);
			tMat = g2.m_xf.R;
			tVec = this.m_prismatic2.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//b2Vec2 r = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
			tMat = bB.m_xf.R;
			rX = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			rY = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;

			//float32 crug = b2Cross(r, ug);
			crug = rX * ugY - rY * ugX;
			//this.m_J.linearB = -this.m_ratio * ug;
			this.m_J.linearB.Set(-this.m_ratio * ugX, -this.m_ratio * ugY);
			this.m_J.angularB = -this.m_ratio * crug;
			K += this.m_ratio * this.m_ratio * (bB.m_invMass + bB.m_invI * crug * crug);
		}

		// Compute effective mass.
		this.m_mass = K > 0.0 ? 1.0 / K : 0.0;

		if (step.warmStarting) {
			// Warm starting.
			//bA.m_linearVelocity += bA.m_invMass * this.m_impulse * this.m_J.linearA;
			bA.m_linearVelocity.x += bA.m_invMass * this.m_impulse * this.m_J.linearA.x;
			bA.m_linearVelocity.y += bA.m_invMass * this.m_impulse * this.m_J.linearA.y;
			bA.m_angularVelocity += bA.m_invI * this.m_impulse * this.m_J.angularA;
			//bB.m_linearVelocity += bB.m_invMass * this.m_impulse * this.m_J.linearB;
			bB.m_linearVelocity.x += bB.m_invMass * this.m_impulse * this.m_J.linearB.x;
			bB.m_linearVelocity.y += bB.m_invMass * this.m_impulse * this.m_J.linearB.y;
			bB.m_angularVelocity += bB.m_invI * this.m_impulse * this.m_J.angularB;
		} else {
			this.m_impulse = 0.0;
		}
	}

	public SolveVelocityConstraints(step: b2TimeStep): void {
		//B2_NOT_USED(step);

		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		const Cdot: number = this.m_J.Compute(bA.m_linearVelocity, bA.m_angularVelocity,
			bB.m_linearVelocity, bB.m_angularVelocity);

		const impulse: number = -this.m_mass * Cdot;
		this.m_impulse += impulse;

		bA.m_linearVelocity.x += bA.m_invMass * impulse * this.m_J.linearA.x;
		bA.m_linearVelocity.y += bA.m_invMass * impulse * this.m_J.linearA.y;
		bA.m_angularVelocity  += bA.m_invI * impulse * this.m_J.angularA;
		bB.m_linearVelocity.x += bB.m_invMass * impulse * this.m_J.linearB.x;
		bB.m_linearVelocity.y += bB.m_invMass * impulse * this.m_J.linearB.y;
		bB.m_angularVelocity  += bB.m_invI * impulse * this.m_J.angularB;
	}

	public SolvePositionConstraints(baumgarte: number): boolean {
		//B2_NOT_USED(baumgarte);

		const linearError: number = 0.0;

		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

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

		bA.m_sweep.c.x += bA.m_invMass * impulse * this.m_J.linearA.x;
		bA.m_sweep.c.y += bA.m_invMass * impulse * this.m_J.linearA.y;
		bA.m_sweep.a += bA.m_invI * impulse * this.m_J.angularA;
		bB.m_sweep.c.x += bB.m_invMass * impulse * this.m_J.linearB.x;
		bB.m_sweep.c.y += bB.m_invMass * impulse * this.m_J.linearB.y;
		bB.m_sweep.a += bB.m_invI * impulse * this.m_J.angularB;

		bA.SynchronizeTransform();
		bB.SynchronizeTransform();

		// TODO_ERIN not implemented
		return linearError < b2Settings.b2_linearSlop;
	}

	private m_ground1: b2Body;
	private m_ground2: b2Body;

	// One of these is NULL.
	private m_revolute1: b2RevoluteJoint;
	private m_prismatic1: b2PrismaticJoint;

	// One of these is NULL.
	private m_revolute2: b2RevoluteJoint;
	private m_prismatic2: b2PrismaticJoint;

	private m_groundAnchor1: b2Vec2 = new b2Vec2();
	private m_groundAnchor2: b2Vec2 = new b2Vec2();

	private m_localAnchor1: b2Vec2 = new b2Vec2();
	private m_localAnchor2: b2Vec2 = new b2Vec2();

	private m_J: b2Jacobian = new b2Jacobian();

	private m_constant: number;
	private m_ratio: number;

	// Effective mass
	private m_mass: number;

	// Impulse for accumulation/warm starting.
	private m_impulse: number;
}