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

/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.

export class b2RevoluteJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_revoluteJoint;
		this.localAnchor1.Set(0.0, 0.0);
		this.localAnchor2.Set(0.0, 0.0);
		this.referenceAngle = 0.0;
		this.lowerAngle = 0.0;
		this.upperAngle = 0.0;
		this.maxMotorTorque = 0.0;
		this.motorSpeed = 0.0;
		this.enableLimit = false;
		this.enableMotor = false;
	}

	/// Initialize the bodies, anchors, and reference angle using the world
	/// anchor.
	public Initialize(b1: b2Body, b2: b2Body, anchor: b2Vec2): void {
		this.body1 = b1;
		this.body2 = b2;
		this.localAnchor1 = this.body1.GetLocalPoint(anchor);
		this.localAnchor2 = this.body2.GetLocalPoint(anchor);
		this.referenceAngle = this.body2.GetAngle() - this.body1.GetAngle();
	}

	/// The local anchor point relative to body1's origin.
	public localAnchor1: b2Vec2 = new b2Vec2();

	/// The local anchor point relative to body2's origin.
	public localAnchor2: b2Vec2 = new b2Vec2();

	/// The body2 angle minus body1 angle in the reference state (radians).
	public referenceAngle: number;

	/// A flag to enable joint limits.
	public enableLimit: boolean;

	/// The lower angle for the joint limit (radians).
	public lowerAngle: number;

	/// The upper angle for the joint limit (radians).
	public upperAngle: number;

	/// A flag to enable the joint motor.
	public enableMotor: boolean;

	/// The desired motor speed. Usually in radians per second.
	public motorSpeed: number;

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in N-m.
	public maxMotorTorque: number;

}