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

import { b2Vec2 } from '../Common/Math/b2Vec2';
import { b2MassData } from '../Collision/Shapes/b2MassData';

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions.
export class b2BodyDef {
	__fast__ = true;

	/// This constructor sets the body definition default values.
	constructor() {
		this.massData.center.SetZero();
		this.massData.mass = 0.0;
		this.massData.I = 0.0;
		this.userData = null;
		this.position.Set(0.0, 0.0);
		this.angle = 0.0;
		this.linearDamping = 0.0;
		this.angularDamping = 0.0;
		this.allowSleep = true;
		this.isSleeping = false;
		this.fixedRotation = false;
		this.isBullet = false;
	}

	/// You can use this to initialized the mass properties of the body.
	/// If you prefer, you can set the mass properties after the shapes
	/// have been added using b2Body::SetMassFromShapes.
	public massData: b2MassData = new b2MassData();

	/// Use this to store application specific body data.
	public userData: any;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	public position: b2Vec2 = new b2Vec2();

	/// The world angle of the body in radians.
	public angle: number;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	public linearDamping: number;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	public angularDamping: number;

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	public allowSleep: boolean;

	/// Is this body initially sleeping?
	public isSleeping: boolean;

	/// Should this body be prevented from rotating? Useful for characters.
	public fixedRotation: boolean;

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// static bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	public isBullet: boolean;
}