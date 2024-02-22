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

import { b2JointDef, b2PulleyJoint, b2Joint } from '../Joints';
import { b2Body } from '../b2Body';
import { b2Vec2 } from '../../Common/Math';

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, max lengths for each side,
/// and a pulley ratio.

export class b2PulleyJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_pulleyJoint;
		this.groundAnchor1.Set(-1.0, 1.0);
		this.groundAnchor2.Set(1.0, 1.0);
		this.localAnchor1.Set(-1.0, 0.0);
		this.localAnchor2.Set(1.0, 0.0);
		this.length1 = 0.0;
		this.maxLength1 = 0.0;
		this.length2 = 0.0;
		this.maxLength2 = 0.0;
		this.ratio = 1.0;
		this.collideConnected = true;
	}

	public Initialize(b1: b2Body, b2: b2Body,
		ga1: b2Vec2, ga2: b2Vec2,
		anchor1: b2Vec2, anchor2: b2Vec2,
		r: number): void {
		this.body1 = b1;
		this.body2 = b2;
		this.groundAnchor1.SetV(ga1);
		this.groundAnchor2.SetV(ga2);
		this.localAnchor1 = this.body1.GetLocalPoint(anchor1);
		this.localAnchor2 = this.body2.GetLocalPoint(anchor2);
		//b2Vec2 d1 = anchor1 - ga1;
		const d1X: number = anchor1.x - ga1.x;
		const d1Y: number = anchor1.y - ga1.y;
		//length1 = d1.Length();
		this.length1 = Math.sqrt(d1X * d1X + d1Y * d1Y);

		//b2Vec2 d2 = anchor2 - ga2;
		const d2X: number = anchor2.x - ga2.x;
		const d2Y: number = anchor2.y - ga2.y;
		//length2 = d2.Length();
		this.length2 = Math.sqrt(d2X * d2X + d2Y * d2Y);

		this.ratio = r;
		//b2Settings.b2Assert(ratio > Number.MIN_VALUE);
		const C: number = this.length1 + this.ratio * this.length2;
		this.maxLength1 = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
		this.maxLength2 = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio;
	}

	/// The first ground anchor in world coordinates. This point never moves.
	public groundAnchor1: b2Vec2 = new b2Vec2();

	/// The second ground anchor in world coordinates. This point never moves.
	public groundAnchor2: b2Vec2 = new b2Vec2();

	/// The local anchor point relative to body1's origin.
	public localAnchor1: b2Vec2 = new b2Vec2();

	/// The local anchor point relative to body2's origin.
	public localAnchor2: b2Vec2 = new b2Vec2();

	/// The a reference length for the segment attached to body1.
	public length1: number;

	/// The maximum length of the segment attached to body1.
	public maxLength1: number;

	/// The a reference length for the segment attached to body2.
	public length2: number;

	/// The maximum length of the segment attached to body2.
	public maxLength2: number;

	/// The pulley ratio, used to simulate a block-and-tackle.
	public ratio: number;

}