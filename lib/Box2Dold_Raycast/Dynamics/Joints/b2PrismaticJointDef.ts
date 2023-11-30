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

import { b2JointDef, b2Joint } from '../Joints';
import { b2Body } from '../b2Body';
import { b2Vec2 } from '../../Common/Math';

export class b2PrismaticJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_prismaticJoint;
		//localAnchor1.SetZero();
		//localAnchor2.SetZero();
		this.localAxis1.Set(1.0, 0.0);
		this.referenceAngle = 0.0;
		this.enableLimit = false;
		this.lowerTranslation = 0.0;
		this.upperTranslation = 0.0;
		this.enableMotor = false;
		this.maxMotorForce = 0.0;
		this.motorSpeed = 0.0;
	}

	public Initialize(b1: b2Body, b2: b2Body, anchor: b2Vec2, axis: b2Vec2): void {
		this.body1 = b1;
		this.body2 = b2;
		this.localAnchor1 = this.body1.GetLocalPoint(anchor);
		this.localAnchor2 = this.body2.GetLocalPoint(anchor);
		this.localAxis1 = this.body1.GetLocalVector(axis);
		this.referenceAngle = this.body2.GetAngle() - this.body1.GetAngle();
	}

	/// The local anchor point relative to body1's origin.
	public localAnchor1: b2Vec2 = new b2Vec2();

	/// The local anchor point relative to body2's origin.
	public localAnchor2: b2Vec2 = new b2Vec2();

	/// The local translation axis in body1.
	public localAxis1: b2Vec2 = new b2Vec2();

	/// The constrained angle between the bodies: body2_angle - body1_angle.
	public referenceAngle: number;

	/// Enable/disable the joint limit.
	public enableLimit: boolean;

	/// The lower translation limit, usually in meters.
	public lowerTranslation: number;

	/// The upper translation limit, usually in meters.
	public upperTranslation: number;

	/// Enable/disable the joint motor.
	public enableMotor: boolean;

	/// The maximum motor torque, usually in N-m.
	public maxMotorForce: number;

	/// The desired motor speed in radians per second.
	public motorSpeed: number;
}