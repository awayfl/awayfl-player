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
import { b2Vec2 } from '../../Common/Math/b2Vec2';

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
export class b2DistanceJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_distanceJoint;
		//localAnchor1.Set(0.0, 0.0);
		//localAnchor2.Set(0.0, 0.0);
		this.length = 1.0;
		this.frequencyHz = 0.0;
		this.dampingRatio = 0.0;
	}

	/// Initialize the bodies, anchors, and length using the world
	/// anchors.
	public Initialize(b1: b2Body, b2: b2Body,
		anchor1: b2Vec2, anchor2: b2Vec2): void {
		this.body1 = b1;
		this.body2 = b2;
		this.localAnchor1.SetV(this.body1.GetLocalPoint(anchor1));
		this.localAnchor2.SetV(this.body2.GetLocalPoint(anchor2));
		const dX: number = anchor2.x - anchor1.x;
		const dY: number = anchor2.y - anchor1.y;
		this.length = Math.sqrt(dX * dX + dY * dY);
		this.frequencyHz = 0.0;
		this.dampingRatio = 0.0;
	}

	/// The local anchor point relative to body1's origin.
	public localAnchor1: b2Vec2 = new b2Vec2();

	/// The local anchor point relative to body2's origin.
	public localAnchor2: b2Vec2 = new b2Vec2();

	/// The equilibrium length between the anchor points.
	public length: number;

	/// The response speed.
	public frequencyHz: number;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	public dampingRatio: number;
}