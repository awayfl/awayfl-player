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

import { b2ContactID } from './b2ContactID';
import { b2Vec2 } from '../Common/Math';

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The point is stored in local coordinates because CCD
/// requires sub-stepping in which the separation is stale.
export class b2ManifoldPoint {
	public Reset(): void {
		this.localPoint1.SetZero();
		this.localPoint2.SetZero();
		this.separation = 0.0;
		this.normalImpulse = 0.0;
		this.tangentImpulse = 0.0;
		this.id.key = 0;
	}

	public Set(m: b2ManifoldPoint): void {
		this.localPoint1.SetV(m.localPoint1);
		this.localPoint2.SetV(m.localPoint2);
		this.separation = m.separation;
		this.normalImpulse = m.normalImpulse;
		this.tangentImpulse = m.tangentImpulse;
		this.id.key = m.id.key;
	}

	public localPoint1: b2Vec2 = new b2Vec2();
	public localPoint2: b2Vec2 = new b2Vec2();
	public separation: number;
	public normalImpulse: number;
	public tangentImpulse: number;
	public id: b2ContactID = new b2ContactID();
}