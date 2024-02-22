import { b2Joint, b2RevoluteJointDef } from '../Joints';
import { b2Vec2, b2Mat33, b2Vec3, b2Mat22, b2Math } from '../../Common/Math';
import { b2Settings } from '../../Common/b2Settings';
import { b2Body } from '../b2Body';
import { b2TimeStep } from '../b2TimeStep';

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

/**
* A revolute joint constrains to bodies to share a common point while they
* are free to rotate about the point. The relative rotation about the shared
* point is the joint angle. You can limit the relative rotation with
* a joint limit that specifies a lower and upper angle. You can use a motor
* to drive the relative rotation about the shared point. A maximum motor torque
* is provided so that infinite forces are not generated.
* @see b2RevoluteJointDef
*/
export class b2RevoluteJoint extends b2Joint {
	/** @inheritDoc */
	public GetAnchorA(): b2Vec2 {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}

	/** @inheritDoc */
	public GetAnchorB(): b2Vec2 {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}

	/** @inheritDoc */
	public GetReactionForce(inv_dt: number): b2Vec2 {
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	}

	/** @inheritDoc */
	public GetReactionTorque(inv_dt: number): number {
		return inv_dt * this.m_impulse.z;
	}

	/**
	* Get the current joint angle in radians.
	*/
	public GetJointAngle(): number {
		//b2Body* bA = this.m_bodyA;
		//b2Body* bB = this.m_bodyB;
		return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
	}

	/**
	* Get the current joint angle speed in radians per second.
	*/
	public GetJointSpeed(): number {
		//b2Body* bA = this.m_bodyA;
		//b2Body* bB = this.m_bodyB;
		return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
	}

	/**
	* Is the joint limit enabled?
	*/
	public IsLimitEnabled(): boolean {
		return this.m_enableLimit;
	}

	/**
	* Enable/disable the joint limit.
	*/
	public EnableLimit(flag: boolean): void {
		this.m_enableLimit = flag;
	}

	/**
	* Get the lower joint limit in radians.
	*/
	public GetLowerLimit(): number {
		return this.m_lowerAngle;
	}

	/**
	* Get the upper joint limit in radians.
	*/
	public GetUpperLimit(): number {
		return this.m_upperAngle;
	}

	/**
	* Set the joint limits in radians.
	*/
	public SetLimits(lower: number, upper: number): void {
		//b2Settings.b2Assert(lower <= upper);
		this.m_lowerAngle = lower;
		this.m_upperAngle = upper;
	}

	/**
	* Is the joint motor enabled?
	*/
	public IsMotorEnabled(): boolean {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		return this.m_enableMotor;
	}

	/**
	* Enable/disable the joint motor.
	*/
	public EnableMotor(flag: boolean): void {
		this.m_enableMotor = flag;
	}

	/**
	* Set the motor speed in radians per second.
	*/
	public SetMotorSpeed(speed: number): void {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	}

	/**
	* Get the motor speed in radians per second.
	*/
	public GetMotorSpeed(): number {
		return this.m_motorSpeed;
	}

	/**
	* Set the maximum motor torque, usually in N-m.
	*/
	public SetMaxMotorTorque(torque: number): void {
		this.m_maxMotorTorque = torque;
	}

	/**
	* Get the current motor torque, usually in N-m.
	*/
	public GetMotorTorque(): number {
		return this.m_maxMotorTorque;
	}

	//--------------- Internals Below -------------------

	/** @private */
	constructor(def: b2RevoluteJointDef) {
		super(def);

		//this.m_localAnchor1 = def->localAnchorA;
		this.m_localAnchor1.SetV(def.localAnchorA);
		//this.m_localAnchor2 = def->localAnchorB;
		this.m_localAnchor2.SetV(def.localAnchorB);

		this.m_referenceAngle = def.referenceAngle;

		this.m_impulse.SetZero();
		this.m_motorImpulse = 0.0;

		this.m_lowerAngle = def.lowerAngle;
		this.m_upperAngle = def.upperAngle;
		this.m_maxMotorTorque = def.maxMotorTorque;
		this.m_motorSpeed = def.motorSpeed;
		this.m_enableLimit = def.enableLimit;
		this.m_enableMotor = def.enableMotor;
		this.m_limitState = b2Joint.e_inactiveLimit;
	}

	// internal vars
	private K: b2Mat22 = new b2Mat22();
	private K1: b2Mat22 = new b2Mat22();
	private K2: b2Mat22 = new b2Mat22();
	private K3: b2Mat22 = new b2Mat22();
	public InitVelocityConstraints(step: b2TimeStep): void {
		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		let tMat: b2Mat22;
		let tX: number;

		if (this.m_enableMotor || this.m_enableLimit) {
			// You cannot create prismatic joint between bodies that
			// both have fixed rotation.
			//b2Settings.b2Assert(bA.m_invI > 0.0 || bB.m_invI > 0.0);
		}

		// Compute the effective mass matrix.

		//b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		// J = [-I -r1_skew I r2_skew]
		// [ 0 -1 0 1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ m1+r1y^2*i1+m2+r2y^2*i2, -r1y*i1*r1x-r2y*i2*r2x, -r1y*i1-r2y*i2]
		//     [ -r1y*i1*r1x-r2y*i2*r2x, m1+r1x^2*i1+m2+r2x^2*i2, r1x*i1+r2x*i2]
		//     [ -r1y*i1-r2y*i2, r1x*i1+r2x*i2, i1+i2]

		const m1: number = bA.m_invMass;
		const m2: number = bB.m_invMass;
		const i1: number = bA.m_invI;
		const i2: number = bB.m_invI;

		this.m_mass.col1.x = m1 + m2 + r1Y * r1Y * i1 + r2Y * r2Y * i2;
		this.m_mass.col2.x = -r1Y * r1X * i1 - r2Y * r2X * i2;
		this.m_mass.col3.x = -r1Y * i1 - r2Y * i2;
		this.m_mass.col1.y = this.m_mass.col2.x;
		this.m_mass.col2.y = m1 + m2 + r1X * r1X * i1 + r2X * r2X * i2;
		this.m_mass.col3.y = r1X * i1 + r2X * i2;
		this.m_mass.col1.z = this.m_mass.col3.x;
		this.m_mass.col2.z = this.m_mass.col3.y;
		this.m_mass.col3.z = i1 + i2;

		this.m_motorMass = 1.0 / (i1 + i2);

		if (this.m_enableMotor == false) {
			this.m_motorImpulse = 0.0;
		}

		if (this.m_enableLimit) {
			//float32 jointAngle = bB->m_sweep.a - bA->m_sweep.a - m_referenceAngle;
			const jointAngle: number = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
			if (b2Math.Abs(this.m_upperAngle - this.m_lowerAngle) < 2.0 * b2Settings.b2_angularSlop) {
				this.m_limitState = b2Joint.e_equalLimits;
			} else if (jointAngle <= this.m_lowerAngle) {
				if (this.m_limitState != b2Joint.e_atLowerLimit) {
					this.m_impulse.z = 0.0;
				}
				this.m_limitState = b2Joint.e_atLowerLimit;
			} else if (jointAngle >= this.m_upperAngle) {
				if (this.m_limitState != b2Joint.e_atUpperLimit) {
					this.m_impulse.z = 0.0;
				}
				this.m_limitState = b2Joint.e_atUpperLimit;
			} else {
				this.m_limitState = b2Joint.e_inactiveLimit;
				this.m_impulse.z = 0.0;
			}
		} else {
			this.m_limitState = b2Joint.e_inactiveLimit;
		}

		// Warm starting.
		if (step.warmStarting) {
			//Scale impulses to support a variable time step
			this.m_impulse.x *= step.dtRatio;
			this.m_impulse.y *= step.dtRatio;
			this.m_motorImpulse *= step.dtRatio;

			const PX: number = this.m_impulse.x;
			const PY: number = this.m_impulse.y;

			//bA->m_linearVelocity -= m1 * P;
			bA.m_linearVelocity.x -= m1 * PX;
			bA.m_linearVelocity.y -= m1 * PY;
			//bA->m_angularVelocity -= i1 * (b2Cross(r1, P) + m_motorImpulse + m_impulse.z);
			bA.m_angularVelocity -= i1 * ((r1X * PY - r1Y * PX) + this.m_motorImpulse + this.m_impulse.z);

			//bB->m_linearVelocity += m2 * P;
			bB.m_linearVelocity.x += m2 * PX;
			bB.m_linearVelocity.y += m2 * PY;
			//bB->m_angularVelocity += i2 * (b2Cross(r2, P) + m_motorImpulse + m_impulse.z);
			bB.m_angularVelocity += i2 * ((r2X * PY - r2Y * PX) + this.m_motorImpulse + this.m_impulse.z);
		} else {
			this.m_impulse.SetZero();
			this.m_motorImpulse = 0.0;
		}
	}

	private impulse3: b2Vec3 = new b2Vec3();
	private impulse2: b2Vec2 = new b2Vec2();
	private reduced: b2Vec2 = new b2Vec2();
	public SolveVelocityConstraints(step: b2TimeStep): void {
		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		let tMat: b2Mat22;
		let tX: number;

		let newImpulse: number;
		let r1X: number;
		let r1Y: number;
		let r2X: number;
		let r2Y: number;

		const v1: b2Vec2 = bA.m_linearVelocity;
		let w1: number = bA.m_angularVelocity;
		const v2: b2Vec2 = bB.m_linearVelocity;
		let w2: number = bB.m_angularVelocity;

		const m1: number = bA.m_invMass;
		const m2: number = bB.m_invMass;
		const i1: number = bA.m_invI;
		const i2: number = bB.m_invI;

		// Solve motor constraint.
		if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
			const Cdot: number = w2 - w1 - this.m_motorSpeed;
			let impulse: number = this.m_motorMass * (-Cdot);
			const oldImpulse: number = this.m_motorImpulse;
			const maxImpulse: number = step.dt * this.m_maxMotorTorque;

			this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.m_motorImpulse - oldImpulse;

			w1 -= i1 * impulse;
			w2 += i2 * impulse;
		}

		// Solve limit constraint.
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
			//b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			//b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;

			// Solve point-to-point constraint
			//b2Vec2 Cdot1 = v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1);
			const Cdot1X: number = v2.x + (-w2 * r2Y) - v1.x - (-w1 * r1Y);
			const Cdot1Y: number = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
			const Cdot2: number  = w2 - w1;

			this.m_mass.Solve33(this.impulse3, -Cdot1X, -Cdot1Y, -Cdot2);

			if (this.m_limitState == b2Joint.e_equalLimits) {
				this.m_impulse.Add(this.impulse3);
			} else if (this.m_limitState == b2Joint.e_atLowerLimit) {
				newImpulse = this.m_impulse.z + this.impulse3.z;
				if (newImpulse < 0.0) {
					this.m_mass.Solve22(this.reduced, -Cdot1X, -Cdot1Y);
					this.impulse3.x = this.reduced.x;
					this.impulse3.y = this.reduced.y;
					this.impulse3.z = -this.m_impulse.z;
					this.m_impulse.x += this.reduced.x;
					this.m_impulse.y += this.reduced.y;
					this.m_impulse.z = 0.0;
				}
			} else if (this.m_limitState == b2Joint.e_atUpperLimit) {
				newImpulse = this.m_impulse.z + this.impulse3.z;
				if (newImpulse > 0.0) {
					this.m_mass.Solve22(this.reduced, -Cdot1X, -Cdot1Y);
					this.impulse3.x = this.reduced.x;
					this.impulse3.y = this.reduced.y;
					this.impulse3.z = -this.m_impulse.z;
					this.m_impulse.x += this.reduced.x;
					this.m_impulse.y += this.reduced.y;
					this.m_impulse.z = 0.0;
				}
			}

			v1.x -= m1 * this.impulse3.x;
			v1.y -= m1 * this.impulse3.y;
			w1 -= i1 * (r1X * this.impulse3.y - r1Y * this.impulse3.x + this.impulse3.z);

			v2.x += m2 * this.impulse3.x;
			v2.y += m2 * this.impulse3.y;
			w2 += i2 * (r2X * this.impulse3.y - r2Y * this.impulse3.x + this.impulse3.z);
		} else {
			//b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			//b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;

			//b2Vec2 Cdot = v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1);
			const CdotX: number = v2.x + (-w2 * r2Y) - v1.x - (-w1 * r1Y);
			const CdotY: number = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);

			this.m_mass.Solve22(this.impulse2, -CdotX, -CdotY);

			this.m_impulse.x += this.impulse2.x;
			this.m_impulse.y += this.impulse2.y;

			v1.x -= m1 * this.impulse2.x;
			v1.y -= m1 * this.impulse2.y;
			//w1 -= i1 * b2Cross(r1, impulse2);
			w1 -= i1 * (r1X * this.impulse2.y - r1Y * this.impulse2.x);

			v2.x += m2 * this.impulse2.x;
			v2.y += m2 * this.impulse2.y;
			//w2 += i2 * b2Cross(r2, impulse2);
			w2 += i2 * (r2X * this.impulse2.y - r2Y * this.impulse2.x);
		}

		bA.m_linearVelocity.SetV(v1);
		bA.m_angularVelocity = w1;
		bB.m_linearVelocity.SetV(v2);
		bB.m_angularVelocity = w2;
	}

	private static tImpulse: b2Vec2 = new b2Vec2();
	public SolvePositionConstraints(baumgarte: number): boolean {

		// TODO_ERIN block solve with limit

		let oldLimitImpulse: number;
		let C: number;

		let tMat: b2Mat22;

		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		let angularError: number = 0.0;
		let positionError: number = 0.0;

		let tX: number;

		let impulseX: number;
		let impulseY: number;

		// Solve angular limit constraint.
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
			const angle: number = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
			let limitImpulse: number = 0.0;

			if (this.m_limitState == b2Joint.e_equalLimits) {
				// Prevent large angular corrections
				C = b2Math.Clamp(angle - this.m_lowerAngle, -b2Settings.b2_maxAngularCorrection, b2Settings.b2_maxAngularCorrection);
				limitImpulse = -this.m_motorMass * C;
				angularError = b2Math.Abs(C);
			} else if (this.m_limitState == b2Joint.e_atLowerLimit) {
				C = angle - this.m_lowerAngle;
				angularError = -C;

				// Prevent large angular corrections and allow some slop.
				C = b2Math.Clamp(C + b2Settings.b2_angularSlop, -b2Settings.b2_maxAngularCorrection, 0.0);
				limitImpulse = -this.m_motorMass * C;
			} else if (this.m_limitState == b2Joint.e_atUpperLimit) {
				C = angle - this.m_upperAngle;
				angularError = C;

				// Prevent large angular corrections and allow some slop.
				C = b2Math.Clamp(C - b2Settings.b2_angularSlop, 0.0, b2Settings.b2_maxAngularCorrection);
				limitImpulse = -this.m_motorMass * C;
			}

			bA.m_sweep.a -= bA.m_invI * limitImpulse;
			bB.m_sweep.a += bB.m_invI * limitImpulse;

			bA.SynchronizeTransform();
			bB.SynchronizeTransform();
		}

		// Solve point-to-point constraint
		{
			//b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
			tMat = bA.m_xf.R;
			let r1X: number = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			let r1Y: number = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			//b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
			tMat = bB.m_xf.R;
			let r2X: number = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			let r2Y: number = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;

			//b2Vec2 C = bB->m_sweep.c + r2 - bA->m_sweep.c - r1;
			let CX: number = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
			let CY: number = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
			const CLengthSquared: number = CX * CX + CY * CY;
			const CLength: number = Math.sqrt(CLengthSquared);
			positionError = CLength;

			const invMass1: number = bA.m_invMass;
			const invMass2: number = bB.m_invMass;
			const invI1: number = bA.m_invI;
			const invI2: number = bB.m_invI;

			//Handle large detachment.
			const k_allowedStretch: number = 10.0 * b2Settings.b2_linearSlop;
			if (CLengthSquared > k_allowedStretch * k_allowedStretch) {
				// Use a particle solution (no rotation)
				//b2Vec2 u = C; u.Normalize();
				const uX: number = CX / CLength;
				const uY: number = CY / CLength;
				const k: number = invMass1 + invMass2;
				//b2Settings.b2Assert(k>Number.MIN_VALUE)
				const m: number = 1.0 / k;
				impulseX = m * (-CX);
				impulseY = m * (-CY);
				const k_beta: number = 0.5;
				bA.m_sweep.c.x -= k_beta * invMass1 * impulseX;
				bA.m_sweep.c.y -= k_beta * invMass1 * impulseY;
				bB.m_sweep.c.x += k_beta * invMass2 * impulseX;
				bB.m_sweep.c.y += k_beta * invMass2 * impulseY;

				//C = bB->m_sweep.c + r2 - bA->m_sweep.c - r1;
				CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
				CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
			}

			//b2Mat22 K1;
			this.K1.col1.x = invMass1 + invMass2;	this.K1.col2.x = 0.0;
			this.K1.col1.y = 0.0;					this.K1.col2.y = invMass1 + invMass2;

			//b2Mat22 K2;
			this.K2.col1.x =  invI1 * r1Y * r1Y;	this.K2.col2.x = -invI1 * r1X * r1Y;
			this.K2.col1.y = -invI1 * r1X * r1Y;	this.K2.col2.y =  invI1 * r1X * r1X;

			//b2Mat22 K3;
			this.K3.col1.x =  invI2 * r2Y * r2Y;		this.K3.col2.x = -invI2 * r2X * r2Y;
			this.K3.col1.y = -invI2 * r2X * r2Y;		this.K3.col2.y =  invI2 * r2X * r2X;

			//b2Mat22 K = K1 + K2 + K3;
			this.K.SetM(this.K1);
			this.K.AddM(this.K2);
			this.K.AddM(this.K3);
			//b2Vec2 impulse = K.Solve(-C);
			this.K.Solve(b2RevoluteJoint.tImpulse, -CX, -CY);
			impulseX = b2RevoluteJoint.tImpulse.x;
			impulseY = b2RevoluteJoint.tImpulse.y;

			//bA.m_sweep.c -= bA.m_invMass * impulse;
			bA.m_sweep.c.x -= bA.m_invMass * impulseX;
			bA.m_sweep.c.y -= bA.m_invMass * impulseY;
			//bA.m_sweep.a -= bA.m_invI * b2Cross(r1, impulse);
			bA.m_sweep.a -= bA.m_invI * (r1X * impulseY - r1Y * impulseX);

			//bB.m_sweep.c += bB.m_invMass * impulse;
			bB.m_sweep.c.x += bB.m_invMass * impulseX;
			bB.m_sweep.c.y += bB.m_invMass * impulseY;
			//bB.m_sweep.a += bB.m_invI * b2Cross(r2, impulse);
			bB.m_sweep.a += bB.m_invI * (r2X * impulseY - r2Y * impulseX);

			bA.SynchronizeTransform();
			bB.SynchronizeTransform();
		}

		return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
	}

	public m_localAnchor1: b2Vec2 = new b2Vec2(); // relative
	public m_localAnchor2: b2Vec2 = new b2Vec2();
	private m_impulse: b2Vec3 = new b2Vec3();
	private m_motorImpulse: number;

	private m_mass: b2Mat33 = new b2Mat33();		// effective mass for point-to-point constraint.
	private m_motorMass: number;	// effective mass for motor/limit angular constraint.
	private m_enableMotor: boolean;
	private m_maxMotorTorque: number;
	private m_motorSpeed: number;

	private m_enableLimit: boolean;
	private m_referenceAngle: number;
	private m_lowerAngle: number;
	private m_upperAngle: number;
	private m_limitState: number /** int */;
}