import { b2Vec2, b2Mat22, b2Math } from '../../Common/Math';
import { b2Joint, b2FrictionJointDef } from '../Joints';
import { b2Body } from '../b2Body';
import { b2TimeStep } from '../b2TimeStep';

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

/**
 * Friction joint. This is used for top-down friction.
 * It provides 2D translational friction and angular friction.
 * @see b2FrictionJointDef
 */
export class b2FrictionJoint extends b2Joint {
	/** @inheritDoc */
	public GetAnchorA(): b2Vec2 {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	}

	/** @inheritDoc */
	public GetAnchorB(): b2Vec2 {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	}

	/** @inheritDoc */
	public GetReactionForce(inv_dt: number): b2Vec2 {
		return new b2Vec2(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
	}

	/** @inheritDoc */
	public GetReactionTorque(inv_dt: number): number {
		//B2_NOT_USED(inv_dt);
		return inv_dt * this.m_angularImpulse;
	}

	public SetMaxForce(force: number): void {
		this.m_maxForce = force;
	}

	public GetMaxForce(): number {
		return this.m_maxForce;
	}

	public SetMaxTorque(torque: number): void {
		this.m_maxTorque = torque;
	}

	public GetMaxTorque(): number {
		return this.m_maxTorque;
	}

	//--------------- Internals Below -------------------

	/** @private */
	constructor(def: b2FrictionJointDef) {
		super(def);

		this.m_localAnchorA.SetV(def.localAnchorA);
		this.m_localAnchorB.SetV(def.localAnchorB);

		this.m_linearMass.SetZero();
		this.m_angularMass = 0.0;

		this.m_linearImpulse.SetZero();
		this.m_angularImpulse = 0.0;

		this.m_maxForce = def.maxForce;
		this.m_maxTorque = def.maxTorque;
	}

	public InitVelocityConstraints(step: b2TimeStep): void {
		let tMat: b2Mat22;
		let tX: number;

		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		// Compute the effective mass matrix.
		//b2Vec2 rA = b2Mul(bA->m_xf.R, m_localAnchorA - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		let rAX: number = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		let rAY: number = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		//b2Vec2 rB = b2Mul(bB->m_xf.R, m_localAnchorB - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		let rBX: number = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		let rBY: number = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		const mA: number = bA.m_invMass;
		const mB: number = bB.m_invMass;
		const iA: number = bA.m_invI;
		const iB: number = bB.m_invI;

		const K: b2Mat22 = new b2Mat22();
		K.col1.x = mA + mB;	K.col2.x = 0.0;
		K.col1.y = 0.0;		K.col2.y = mA + mB;

		K.col1.x +=  iA * rAY * rAY;	K.col2.x += -iA * rAX * rAY;
		K.col1.y += -iA * rAX * rAY;	K.col2.y +=  iA * rAX * rAX;

		K.col1.x +=  iB * rBY * rBY;	K.col2.x += -iB * rBX * rBY;
		K.col1.y += -iB * rBX * rBY;	K.col2.y +=  iB * rBX * rBX;

		K.GetInverse(this.m_linearMass);

		this.m_angularMass = iA + iB;
		if (this.m_angularMass > 0.0) {
			this.m_angularMass = 1.0 / this.m_angularMass;
		}

		if (step.warmStarting) {
			// Scale impulses to support a variable time step.
			this.m_linearImpulse.x *= step.dtRatio;
			this.m_linearImpulse.y *= step.dtRatio;
			this.m_angularImpulse *= step.dtRatio;

			const P: b2Vec2 = this.m_linearImpulse;

			bA.m_linearVelocity.x -= mA * P.x;
			bA.m_linearVelocity.y -= mA * P.y;
			bA.m_angularVelocity -= iA * (rAX * P.y - rAY * P.x + this.m_angularImpulse);

			bB.m_linearVelocity.x += mB * P.x;
			bB.m_linearVelocity.y += mB * P.y;
			bB.m_angularVelocity += iB * (rBX * P.y - rBY * P.x + this.m_angularImpulse);
		} else {
			this.m_linearImpulse.SetZero();
			this.m_angularImpulse = 0.0;
		}

	}

	public SolveVelocityConstraints(step: b2TimeStep): void {
		//B2_NOT_USED(step);
		let tMat: b2Mat22;
		let tX: number;

		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		const vA: b2Vec2 = bA.m_linearVelocity;
		let wA: number = bA.m_angularVelocity;
		const vB: b2Vec2 = bB.m_linearVelocity;
		let wB: number = bB.m_angularVelocity;

		const mA: number = bA.m_invMass;
		const mB: number = bB.m_invMass;
		const iA: number = bA.m_invI;
		const iB: number = bB.m_invI;

		//b2Vec2 rA = b2Mul(bA->m_xf.R, m_localAnchorA - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		let rAX: number = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		let rAY: number = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		//b2Vec2 rB = b2Mul(bB->m_xf.R, m_localAnchorB - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		let rBX: number = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		let rBY: number = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;

		let maxImpulse: number;

		// Solve angular friction
		{
			const Cdot: number = wB - wA;
			let impulse: number = -this.m_angularMass * Cdot;

			const oldImpulse: number = this.m_angularImpulse;
			maxImpulse = step.dt * this.m_maxTorque;
			this.m_angularImpulse = b2Math.Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.m_angularImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Solve linear friction
		{
			//b2Vec2 Cdot = vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA);
			const CdotX: number = vB.x - wB * rBY - vA.x + wA * rAY;
			const CdotY: number = vB.y + wB * rBX - vA.y - wA * rAX;

			let impulseV: b2Vec2 = b2Math.MulMV(this.m_linearMass, new b2Vec2(-CdotX, -CdotY));
			const oldImpulseV: b2Vec2 = this.m_linearImpulse.Copy();

			this.m_linearImpulse.Add(impulseV);

			maxImpulse = step.dt * this.m_maxForce;

			if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
				this.m_linearImpulse.Normalize();
				this.m_linearImpulse.Multiply(maxImpulse);
			}

			impulseV = b2Math.SubtractVV(this.m_linearImpulse, oldImpulseV);

			vA.x -= mA * impulseV.x;
			vA.y -= mA * impulseV.y;
			wA -= iA * (rAX * impulseV.y - rAY * impulseV.x);

			vB.x += mB * impulseV.x;
			vB.y += mB * impulseV.y;
			wB += iB * (rBX * impulseV.y - rBY * impulseV.x);
		}

		// References has made some sets unnecessary
		//bA->m_linearVelocity = vA;
		bA.m_angularVelocity = wA;
		//bB->m_linearVelocity = vB;
		bB.m_angularVelocity = wB;

	}

	public SolvePositionConstraints(baumgarte: number): boolean {
		//B2_NOT_USED(baumgarte);

		return true;

	}

	private m_localAnchorA: b2Vec2 = new b2Vec2();
	private m_localAnchorB: b2Vec2 = new b2Vec2();

	public m_linearMass: b2Mat22 = new b2Mat22();
	public m_angularMass: number;

	private m_linearImpulse: b2Vec2 = new b2Vec2();
	private m_angularImpulse: number;

	private m_maxForce: number;
	private m_maxTorque: number;
}
