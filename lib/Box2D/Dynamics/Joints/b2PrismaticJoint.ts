import { b2Mat33, b2Vec2, b2Mat22, b2Vec3, b2Math, b2Transform } from '../../Common/Math';
import { b2Body } from '../b2Body';
import { b2Settings } from '../../Common/b2Settings';
import { b2TimeStep } from '../b2TimeStep';
import { b2Joint, b2PrismaticJointDef } from '../Joints';

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

/**
* A prismatic joint. This joint provides one degree of freedom: translation
* along an axis fixed in body1. Relative rotation is prevented. You can
* use a joint limit to restrict the range of motion and a joint motor to
* drive the motion or to model joint friction.
* @see b2PrismaticJointDef
*/
export class b2PrismaticJoint extends b2Joint {
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
		//return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
		return new b2Vec2(inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x),
			inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y));
	}

	/** @inheritDoc */
	public GetReactionTorque(inv_dt: number): number {
		return inv_dt * this.m_impulse.y;
	}

	/**
	* Get the current joint translation, usually in meters.
	*/
	public GetJointTranslation(): number {
		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		let tMat: b2Mat22;

		const p1: b2Vec2 = bA.GetWorldPoint(this.m_localAnchor1);
		const p2: b2Vec2 = bB.GetWorldPoint(this.m_localAnchor2);
		//var d:b2Vec2 = b2Math.SubtractVV(p2, p1);
		const dX: number = p2.x - p1.x;
		const dY: number = p2.y - p1.y;
		//b2Vec2 axis = bA->GetWorldVector(this.m_localXAxis1);
		const axis: b2Vec2 = bA.GetWorldVector(this.m_localXAxis1);

		//float32 translation = b2Dot(d, axis);
		const translation: number = axis.x * dX + axis.y * dY;
		return translation;
	}

	/**
	* Get the current joint translation speed, usually in meters per second.
	*/
	public GetJointSpeed(): number {
		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		let tMat: b2Mat22;

		//b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		let tX: number =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		//b2Vec2 p1 = bA->m_sweep.c + r1;
		const p1X: number = bA.m_sweep.c.x + r1X;
		const p1Y: number = bA.m_sweep.c.y + r1Y;
		//b2Vec2 p2 = bB->m_sweep.c + r2;
		const p2X: number = bB.m_sweep.c.x + r2X;
		const p2Y: number = bB.m_sweep.c.y + r2Y;
		//var d:b2Vec2 = b2Math.SubtractVV(p2, p1);
		const dX: number = p2X - p1X;
		const dY: number = p2Y - p1Y;
		//b2Vec2 axis = bA->GetWorldVector(this.m_localXAxis1);
		const axis: b2Vec2 = bA.GetWorldVector(this.m_localXAxis1);

		const v1: b2Vec2 = bA.m_linearVelocity;
		const v2: b2Vec2 = bB.m_linearVelocity;
		const w1: number = bA.m_angularVelocity;
		const w2: number = bB.m_angularVelocity;

		//var speed:number = b2Math.b2Dot(d, b2Math.b2CrossFV(w1, ax1)) + b2Math.b2Dot(ax1, b2Math.SubtractVV( b2Math.SubtractVV( b2Math.AddVV( v2 , b2Math.b2CrossFV(w2, r2)) , v1) , b2Math.b2CrossFV(w1, r1)));
		//var b2D:number = (dX*(-w1 * ax1Y) + dY*(w1 * ax1X));
		//var b2D2:number = (ax1X * ((( v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + ax1Y * ((( v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
		const speed: number = (dX * (-w1 * axis.y) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));

		return speed;
	}

	/**
	* Is the joint limit enabled?
	*/
	public IsLimitEnabled(): Boolean {
		return this.m_enableLimit;
	}

	/**
	* Enable/disable the joint limit.
	*/
	public EnableLimit(flag: boolean): void {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableLimit = flag;
	}

	/**
	* Get the lower joint limit, usually in meters.
	*/
	public GetLowerLimit(): number {
		return this.m_lowerTranslation;
	}

	/**
	* Get the upper joint limit, usually in meters.
	*/
	public GetUpperLimit(): number {
		return this.m_upperTranslation;
	}

	/**
	* Set the joint limits, usually in meters.
	*/
	public SetLimits(lower: number, upper: number): void {
		//b2Settings.b2Assert(lower <= upper);
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_lowerTranslation = lower;
		this.m_upperTranslation = upper;
	}

	/**
	* Is the joint motor enabled?
	*/
	public IsMotorEnabled(): Boolean {
		return this.m_enableMotor;
	}

	/**
	* Enable/disable the joint motor.
	*/
	public EnableMotor(flag: boolean): void {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableMotor = flag;
	}

	/**
	* Set the motor speed, usually in meters per second.
	*/
	public SetMotorSpeed(speed: number): void {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	}

	/**
	* Get the motor speed, usually in meters per second.
	*/
	public GetMotorSpeed(): number {
		return this.m_motorSpeed;
	}

	/**
	* Set the maximum motor force, usually in N.
	*/
	public SetMaxMotorForce(force: number): void {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_maxMotorForce = force;
	}

	/**
	* Get the current motor force, usually in N.
	*/
	public GetMotorForce(): number {
		return this.m_motorImpulse;
	}

	//--------------- Internals Below -------------------

	/** @private */
	constructor(def: b2PrismaticJointDef) {
		super(def);

		let tMat: b2Mat22;
		let tX: number;
		let tY: number;

		this.m_localAnchor1.SetV(def.localAnchorA);
		this.m_localAnchor2.SetV(def.localAnchorB);
		this.m_localXAxis1.SetV(def.localAxisA);

		//this.m_localYAxisA = b2Cross(1.0f, this.m_localXAxisA);
		this.m_localYAxis1.x = -this.m_localXAxis1.y;
		this.m_localYAxis1.y = this.m_localXAxis1.x;

		this.m_refAngle = def.referenceAngle;

		this.m_impulse.SetZero();
		this.m_motorMass = 0.0;
		this.m_motorImpulse = 0.0;

		this.m_lowerTranslation = def.lowerTranslation;
		this.m_upperTranslation = def.upperTranslation;
		this.m_maxMotorForce = def.maxMotorForce;
		this.m_motorSpeed = def.motorSpeed;
		this.m_enableLimit = def.enableLimit;
		this.m_enableMotor = def.enableMotor;
		this.m_limitState = b2Joint.e_inactiveLimit;

		this.m_axis.SetZero();
		this.m_perp.SetZero();
	}

	public InitVelocityConstraints(step: b2TimeStep): void {
		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		let tMat: b2Mat22;
		let tX: number;

		this.m_localCenterA.SetV(bA.GetLocalCenter());
		this.m_localCenterB.SetV(bB.GetLocalCenter());

		const xf1: b2Transform = bA.GetTransform();
		const xf2: b2Transform = bB.GetTransform();

		// Compute the effective masses.
		//b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - this.m_localCenterA.x;
		let r1Y: number = this.m_localAnchor1.y - this.m_localCenterA.y;
		tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - this.m_localCenterB.x;
		let r2Y: number = this.m_localAnchor2.y - this.m_localCenterB.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		//b2Vec2 d = bB->m_sweep.c + r2 - bA->m_sweep.c - r1;
		const dX: number = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		const dY: number = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;

		this.m_invMassA = bA.m_invMass;
		this.m_invMassB = bB.m_invMass;
		this.m_invIA = bA.m_invI;
		this.m_invIB = bB.m_invI;

		// Compute motor Jacobian and effective mass.
		{
			this.m_axis.SetV(b2Math.MulMV(xf1.R, this.m_localXAxis1));
			//m_a1 = b2Math.b2Cross(d + r1, m_axis);
			this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
			//m_a2 = b2Math.b2Cross(r2, m_axis);
			this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;

			this.m_motorMass = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_a1 * this.m_a1 + this.m_invIB * this.m_a2 * this.m_a2;
			if (this.m_motorMass > Number.MIN_VALUE)
				this.m_motorMass = 1.0 / this.m_motorMass;
		}

		// Prismatic constraint.
		{
			this.m_perp.SetV(b2Math.MulMV(xf1.R, this.m_localYAxis1));
			//m_s1 = b2Math.b2Cross(d + r1, m_perp);
			this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
			//this.m_s2 = b2Math.b2Cross(r2, this.m_perp);
			this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;

			const m1: number = this.m_invMassA;
			const m2: number = this.m_invMassB;
			const i1: number = this.m_invIA;
			const i2: number = this.m_invIB;

			this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
 	  	  	this.m_K.col1.y = i1 * this.m_s1 + i2 * this.m_s2;
 	  	  	this.m_K.col1.z = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
			this.m_K.col2.x = this.m_K.col1.y;
 	  	  	this.m_K.col2.y = i1 + i2;
 	  	  	this.m_K.col2.z = i1 * this.m_a1 + i2 * this.m_a2;
			this.m_K.col3.x = this.m_K.col1.z;
			this.m_K.col3.y = this.m_K.col2.z;
 	  	  	this.m_K.col3.z = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
		}

		// Compute motor and limit terms
		if (this.m_enableLimit) {
			//float32 jointTranslation = b2Dot(m_axis, d);
			const jointTransition: number = this.m_axis.x * dX + this.m_axis.y * dY;
			if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
				this.m_limitState = b2Joint.e_equalLimits;
			} else if (jointTransition <= this.m_lowerTranslation) {
				if (this.m_limitState != b2Joint.e_atLowerLimit) {
					this.m_limitState = b2Joint.e_atLowerLimit;
					this.m_impulse.z = 0.0;
				}
			} else if (jointTransition >= this.m_upperTranslation) {
				if (this.m_limitState != b2Joint.e_atUpperLimit) {
					this.m_limitState = b2Joint.e_atUpperLimit;
					this.m_impulse.z = 0.0;
				}
			} else {
				this.m_limitState = b2Joint.e_inactiveLimit;
				this.m_impulse.z = 0.0;
			}
		} else {
			this.m_limitState = b2Joint.e_inactiveLimit;
		}

		if (this.m_enableMotor == false) {
			this.m_motorImpulse = 0.0;
		}

		if (step.warmStarting) {
			// Account for variable time step.
			this.m_impulse.x *= step.dtRatio;
			this.m_impulse.y *= step.dtRatio;
			this.m_motorImpulse *= step.dtRatio;

			//b2Vec2 P = m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
			const PX: number = this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x;
			const PY: number = this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y;
			const L1: number = this.m_impulse.x * this.m_s1 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
			const L2: number = this.m_impulse.x * this.m_s2 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;

			//bA->m_linearVelocity -= m_invMassA * P;
			bA.m_linearVelocity.x -= this.m_invMassA * PX;
			bA.m_linearVelocity.y -= this.m_invMassA * PY;
			//bA->m_angularVelocity -= m_invIA * L1;
			bA.m_angularVelocity -= this.m_invIA * L1;

			//bB->m_linearVelocity += m_invMassB * P;
			bB.m_linearVelocity.x += this.m_invMassB * PX;
			bB.m_linearVelocity.y += this.m_invMassB * PY;
			//bB->m_angularVelocity += m_invIB * L2;
			bB.m_angularVelocity += this.m_invIB * L2;
		} else {
			this.m_impulse.SetZero();
			this.m_motorImpulse = 0.0;
		}
	}

	public SolveVelocityConstraints(step: b2TimeStep): void {
		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		const v1: b2Vec2 = bA.m_linearVelocity;
		let w1: number = bA.m_angularVelocity;
		const v2: b2Vec2 = bB.m_linearVelocity;
		let w2: number = bB.m_angularVelocity;

		let PX: number;
		let PY: number;
		let L1: number;
		let L2: number;

		// Solve linear motor constraint
		if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
			//float32 Cdot = b2Dot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
			const Cdot: number = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;
			let impulse: number = this.m_motorMass * (this.m_motorSpeed - Cdot);
			const oldImpulse: number = this.m_motorImpulse;
			const maxImpulse: number = step.dt * this.m_maxMotorForce;
			this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.m_motorImpulse - oldImpulse;

			PX = impulse * this.m_axis.x;
			PY = impulse * this.m_axis.y;
			L1 = impulse * this.m_a1;
			L2 = impulse * this.m_a2;

			v1.x -= this.m_invMassA * PX;
			v1.y -= this.m_invMassA * PY;
			w1 -= this.m_invIA * L1;

			v2.x += this.m_invMassB * PX;
			v2.y += this.m_invMassB * PY;
			w2 += this.m_invIB * L2;
		}

		//Cdot1.x = b2Dot(m_perp, v2 - v1) + m_s2 * w2 - m_s1 * w1;
		const Cdot1X: number = this.m_perp.x * (v2.x - v1.x) + this.m_perp.y * (v2.y - v1.y) + this.m_s2 * w2 - this.m_s1 * w1;
		const Cdot1Y: number = w2 - w1;

		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
			// Solve prismatic and limit constraint in block form
			//Cdot2 = b2Dot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
			const Cdot2: number = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;

			const f1: b2Vec3 = this.m_impulse.Copy();
			const df: b2Vec3 = this.m_K.Solve33(new b2Vec3(), -Cdot1X, -Cdot1Y, -Cdot2);

			this.m_impulse.Add(df);

			if (this.m_limitState == b2Joint.e_atLowerLimit) {
				this.m_impulse.z = b2Math.Max(this.m_impulse.z, 0.0);
			} else if (this.m_limitState == b2Joint.e_atUpperLimit) {
				this.m_impulse.z = b2Math.Min(this.m_impulse.z, 0.0);
			}

			// f2(1:2) = invK(1:2,1:2) * (-Cdot3\(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
			//b2Vec2 b = -Cdot1 - (m_impulse.z - f1.z) * b2Vec2(m_K.col3.x, m_K.col3.y);
			const bX: number = -Cdot1X - (this.m_impulse.z - f1.z) * this.m_K.col3.x;
			const bY: number = -Cdot1Y - (this.m_impulse.z - f1.z) * this.m_K.col3.y;
			const f2r: b2Vec2 = this.m_K.Solve22(new b2Vec2(), bX, bY);
			f2r.x += f1.x;
			f2r.y += f1.y;
			this.m_impulse.x = f2r.x;
			this.m_impulse.y = f2r.y;

			df.x = this.m_impulse.x - f1.x;
			df.y = this.m_impulse.y - f1.y;
			df.z = this.m_impulse.z - f1.z;

			PX = df.x * this.m_perp.x + df.z * this.m_axis.x;
			PY = df.x * this.m_perp.y + df.z * this.m_axis.y;
			L1 = df.x * this.m_s1 + df.y + df.z * this.m_a1;
			L2 = df.x * this.m_s2 + df.y + df.z * this.m_a2;

			v1.x -= this.m_invMassA * PX;
			v1.y -= this.m_invMassA * PY;
			w1 -= this.m_invIA * L1;

			v2.x += this.m_invMassB * PX;
			v2.y += this.m_invMassB * PY;
			w2 += this.m_invIB * L2;
		} else {
			// Limit is inactive, just solve the prismatic constraint in block form.
			const df2: b2Vec2 = this.m_K.Solve22(new b2Vec2(), -Cdot1X, -Cdot1Y);
			this.m_impulse.x += df2.x;
			this.m_impulse.y += df2.y;

			PX = df2.x * this.m_perp.x;
			PY = df2.x * this.m_perp.y;
			L1 = df2.x * this.m_s1 + df2.y;
			L2 = df2.x * this.m_s2 + df2.y;

			v1.x -= this.m_invMassA * PX;
			v1.y -= this.m_invMassA * PY;
			w1 -= this.m_invIA * L1;

			v2.x += this.m_invMassB * PX;
			v2.y += this.m_invMassB * PY;
			w2 += this.m_invIB * L2;
		}

		bA.m_linearVelocity.SetV(v1);
		bA.m_angularVelocity = w1;
		bB.m_linearVelocity.SetV(v2);
		bB.m_angularVelocity = w2;
	}

	public SolvePositionConstraints(baumgarte: number): boolean {
		//B2_NOT_USED(baumgarte);

		let limitC: number;
		let oldLimitImpulse: number;

		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		const c1: b2Vec2 = bA.m_sweep.c;
		let a1: number = bA.m_sweep.a;

		const c2: b2Vec2 = bB.m_sweep.c;
		let a2: number = bB.m_sweep.a;

		let tMat: b2Mat22;
		let tX: number;

		let m1: number;
		let m2: number;
		let i1: number;
		let i2: number;

		// Solve linear limit constraint
		let linearError: number = 0.0;
		let angularError: number = 0.0;
		let active: boolean = false;
		let C2: number = 0.0;

		const R1: b2Mat22 = b2Mat22.FromAngle(a1);
		const R2: b2Mat22 = b2Mat22.FromAngle(a2);

		//b2Vec2 r1 = b2Mul(R1, m_localAnchor1 - m_localCenterA);
		tMat = R1;
		let r1X: number = this.m_localAnchor1.x - this.m_localCenterA.x;
		let r1Y: number = this.m_localAnchor1.y - this.m_localCenterA.y;
		tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(R2, m_localAnchor2 - m_localCenterB);
		tMat = R2;
		let r2X: number = this.m_localAnchor2.x - this.m_localCenterB.x;
		let r2Y: number = this.m_localAnchor2.y - this.m_localCenterB.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		const dX: number = c2.x + r2X - c1.x - r1X;
		const dY: number = c2.y + r2Y - c1.y - r1Y;

		if (this.m_enableLimit) {
			this.m_axis = b2Math.MulMV(R1, this.m_localXAxis1);

			//m_a1 = b2Math.b2Cross(d + r1, m_axis);
			this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
			//m_a2 = b2Math.b2Cross(r2, m_axis);
			this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;

			const translation: number = this.m_axis.x * dX + this.m_axis.y * dY;
			if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
				// Prevent large angular corrections.
				C2 = b2Math.Clamp(translation, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
				linearError = b2Math.Abs(translation);
				active = true;
			} else if (translation <= this.m_lowerTranslation) {
				// Prevent large angular corrections and allow some slop.
				C2 = b2Math.Clamp(translation - this.m_lowerTranslation + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
				linearError = this.m_lowerTranslation - translation;
				active = true;
			} else if (translation >= this.m_upperTranslation) {
				// Prevent large angular corrections and allow some slop.
				C2 = b2Math.Clamp(translation - this.m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection);
				linearError = translation - this.m_upperTranslation;
				active = true;
			}
		}

		this.m_perp = b2Math.MulMV(R1, this.m_localYAxis1);

		//m_s1 = b2Cross(d + r1, m_perp);
		this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
		//m_s2 = b2Cross(r2, m_perp);
		this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;

		const impulse: b2Vec3 = new b2Vec3();
		const C1X: number = this.m_perp.x * dX + this.m_perp.y * dY;
		const C1Y: number = a2 - a1 - this.m_refAngle;

		linearError = b2Math.Max(linearError, b2Math.Abs(C1X));
		angularError = b2Math.Abs(C1Y);

		if (active) {
			m1 = this.m_invMassA;
			m2 = this.m_invMassB;
			i1 = this.m_invIA;
			i2 = this.m_invIB;

			this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
 	  	  	this.m_K.col1.y = i1 * this.m_s1 + i2 * this.m_s2;
 	  	  	this.m_K.col1.z = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
			this.m_K.col2.x = this.m_K.col1.y;
 	  	  	this.m_K.col2.y = i1 + i2;
 	  	  	this.m_K.col2.z = i1 * this.m_a1 + i2 * this.m_a2;
			this.m_K.col3.x = this.m_K.col1.z;
			this.m_K.col3.y = this.m_K.col2.z;
 	  	  	this.m_K.col3.z = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;

			this.m_K.Solve33(impulse, -C1X, -C1Y, -C2);
		} else {
			m1 = this.m_invMassA;
			m2 = this.m_invMassB;
			i1 = this.m_invIA;
			i2 = this.m_invIB;

			const k11: number  = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
			const k12: number = i1 * this.m_s1 + i2 * this.m_s2;
			const k22: number = i1 + i2;

			this.m_K.col1.Set(k11, k12, 0.0);
			this.m_K.col2.Set(k12, k22, 0.0);

			const impulse1: b2Vec2 = this.m_K.Solve22(new b2Vec2(), -C1X, -C1Y);
			impulse.x = impulse1.x;
			impulse.y = impulse1.y;
			impulse.z = 0.0;
		}

		const PX: number = impulse.x * this.m_perp.x + impulse.z * this.m_axis.x;
		const PY: number = impulse.x * this.m_perp.y + impulse.z * this.m_axis.y;
		const L1: number = impulse.x * this.m_s1 + impulse.y + impulse.z * this.m_a1;
		const L2: number = impulse.x * this.m_s2 + impulse.y + impulse.z * this.m_a2;

		c1.x -= this.m_invMassA * PX;
		c1.y -= this.m_invMassA * PY;
		a1 -= this.m_invIA * L1;

		c2.x += this.m_invMassB * PX;
		c2.y += this.m_invMassB * PY;
		a2 += this.m_invIB * L2;

		// TODO_ERIN remove need for this
		//bA.m_sweep.c = c1;	//Already done by reference
		bA.m_sweep.a = a1;
		//bB.m_sweep.c = c2;	//Already done by reference
		bB.m_sweep.a = a2;
		bA.SynchronizeTransform();
		bB.SynchronizeTransform();

		return linearError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;

	}

	public m_localAnchor1: b2Vec2 = new b2Vec2();
	public m_localAnchor2: b2Vec2 = new b2Vec2();
	public m_localXAxis1: b2Vec2 = new b2Vec2();
	private m_localYAxis1: b2Vec2 = new b2Vec2();
	private m_refAngle: number;

	private m_axis: b2Vec2 = new b2Vec2();
	private m_perp: b2Vec2 = new b2Vec2();
	private m_s1: number;
	private m_s2: number;
	private m_a1: number;
	private m_a2: number;

	private m_K: b2Mat33 = new b2Mat33();
	private m_impulse: b2Vec3 = new b2Vec3();

	private m_motorMass: number;			// effective mass for motor/limit translational constraint.
	private m_motorImpulse: number;

	private m_lowerTranslation: number;
	private m_upperTranslation: number;
	private m_maxMotorForce: number;
	private m_motorSpeed: number;

	private m_enableLimit: boolean;
	private m_enableMotor: boolean;
	private m_limitState: number /**int */;
}