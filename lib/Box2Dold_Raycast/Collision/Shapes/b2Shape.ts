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

import { b2XForm } from '../../Common/Math/b2XForm';
import { b2Vec2 } from '../../Common/Math/b2Vec2';
import { b2FilterData } from './b2FilterData';
import { b2Pair } from '../b2Pair';
import { b2AABB } from '../b2AABB';
import { b2BroadPhase } from '../b2BroadPhase';
import { b2ShapeDef } from './b2ShapeDef';
import { b2Segment } from '../b2Segment';
import { b2MassData } from './b2MassData';
import { b2Body } from '../../Dynamics/b2Body';

/// A shape is used for collision detection. Shapes are created in b2World.
/// You can use shape for collision detection before they are attached to the world.
/// @warning you cannot reuse shapes.
export class b2Shape {
	readonly __fast__ = true;
	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	public GetType(): number /** int */{
		return this.m_type;
	}

	/// Is this shape a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
	public IsSensor(): boolean {
		return this.m_isSensor;
	}

	/// Set the contact filtering data. You must call b2World::Refilter to correct
	/// existing contacts/non-contacts.
	public SetFilterData(filter: b2FilterData): void {
		this.m_filter = filter.Copy();
	}

	/// Get the contact filtering data.
	public GetFilterData(): b2FilterData {
		return this.m_filter.Copy();
	}

	/// Get the parent body of this shape. This is NULL if the shape is not attached.
	/// @return the parent body.
	public GetBody(): b2Body {
		return this.m_body;
	}

	/// Get the next shape in the parent body's shape list.
	/// @return the next shape.
	public GetNext(): b2Shape {
		return this.m_next;
	}

	/// Get the user data that was assigned in the shape definition. Use this to
	/// store your application specific data.
	public GetUserData(): any {
		return this.m_userData;
	}

	/// Set the user data. Use this to store your application specific data.
	public SetUserData(data: any): void {
		this.m_userData = data;
	}

	/// Test a point for containment in this shape. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	public TestPoint(xf: b2XForm, p: b2Vec2): boolean {return false;}

	/// Perform a ray cast against this shape.
	/// @param xf the shape world transform.
	/// @param lambda returns the hit fraction. You can use this to compute the contact point
	/// p = (1 - lambda) * segment.p1 + lambda * segment.p2.
	/// @param normal returns the normal at the contact point. If there is no intersection, the normal
	/// is not set.
	/// @param segment defines the begin and end point of the ray cast.
	/// @param maxLambda a number typically in the range [0,1].
	/// @return true if there was an intersection.
	public  TestSegment(xf: b2XForm,
		lambda: any[], // float pointer
		normal: b2Vec2, // pointer
		segment: b2Segment,
		maxLambda: number): number {return b2Shape.e_missCollide;}

	/// Given a transform, compute the associated axis aligned bounding box for this shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf the world transform of the shape.
	public  ComputeAABB(aabb: b2AABB, xf: b2XForm): void {}

	/// Given two transforms, compute the associated swept axis aligned bounding box for this shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf1 the starting shape world transform.
	/// @param xf2 the ending shape world transform.
	public  ComputeSweptAABB(aabb: b2AABB,
		xf1: b2XForm,
		xf2: b2XForm): void {}

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin, not the centroid.
	/// @param massData returns the mass data for this shape.
	public  ComputeMass(massData: b2MassData): void {}

	/// Get the maximum radius about the parent body's center of mass.
	public GetSweepRadius(): number {
		return this.m_sweepRadius;
	}

	/// Get the coefficient of friction.
	public GetFriction(): number {
		return this.m_friction;
	}

	/// Get the coefficient of restitution.
	public GetRestitution(): number {
		return this.m_restitution;
	}

	//--------------- Internals Below -------------------

	public static Destroy(shape: b2Shape, allocator: any): void {
		/*switch (s.m_type)
		{
		case e_circleShape:
			//s->~b2Shape();
			//allocator->Free(s, sizeof(b2CircleShape));
			break;

		case e_polygonShape:
			//s->~b2Shape();
			//allocator->Free(s, sizeof(b2PolygonShape));
			break;

		default:
			//b2Settings.b2Assert(false);
		}*/
	}

	constructor(def: b2ShapeDef) {

		this.m_userData = def.userData;
		this.m_friction = def.friction;
		this.m_restitution = def.restitution;
		this.m_density = def.density;
		this.m_body = null;
		this.m_sweepRadius = 0.0;

		this.m_next = null;

		this.m_proxyId = b2Pair.b2_nullProxy;

		this.m_filter = def.filter.Copy();

		this.m_isSensor = def.isSensor;

	}

	//virtual ~b2Shape();

	//
	private static s_proxyAABB: b2AABB = new b2AABB();
	public CreateProxy(broadPhase: b2BroadPhase, transform: b2XForm): void {

		//b2Settings.b2Assert(this.m_proxyId == b2_nullProxy);

		const aabb: b2AABB = b2Shape.s_proxyAABB;
		this.ComputeAABB(aabb, transform);

		const inRange: boolean = broadPhase.InRange(aabb);

		// You are creating a shape outside the world box.
		//b2Settings.b2Assert(inRange);

		if (inRange) {
			this.m_proxyId = broadPhase.CreateProxy(aabb, this);
		} else {
			this.m_proxyId = b2Pair.b2_nullProxy;
		}

	}

	public DestroyProxy(broadPhase: b2BroadPhase): void {

		if (this.m_proxyId != b2Pair.b2_nullProxy) {
			broadPhase.DestroyProxy(this.m_proxyId);
			this.m_proxyId = b2Pair.b2_nullProxy;
		}

	}

	//
	private static s_syncAABB: b2AABB = new b2AABB();
	//
	public Synchronize(broadPhase: b2BroadPhase, transform1: b2XForm, transform2: b2XForm): boolean {

		if (this.m_proxyId == b2Pair.b2_nullProxy) {
			return false;
		}

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		const aabb: b2AABB = b2Shape.s_syncAABB;
		this.ComputeSweptAABB(aabb, transform1, transform2);

		if (broadPhase.InRange(aabb)) {
			broadPhase.MoveProxy(this.m_proxyId, aabb);
			return true;
		} else {
			return false;
		}

	}

	private static s_resetAABB: b2AABB = new b2AABB();
	public RefilterProxy(broadPhase: b2BroadPhase, transform: b2XForm): void {

		if (this.m_proxyId == b2Pair.b2_nullProxy) {
			return;
		}

		broadPhase.DestroyProxy(this.m_proxyId);

		const aabb: b2AABB = b2Shape.s_resetAABB;
		this.ComputeAABB(aabb, transform);

		const inRange: boolean = broadPhase.InRange(aabb);

		if (inRange) {
			this.m_proxyId = broadPhase.CreateProxy(aabb, this);
		} else {
			this.m_proxyId = b2Pair.b2_nullProxy;
		}

	}

	public UpdateSweepRadius(center: b2Vec2): void {}

	public m_type: number /** int */;
	public m_next: b2Shape;
	public m_body: b2Body;

	// Sweep radius relative to the parent body's center of mass.
	public m_sweepRadius: number;

	public m_density: number;
	public m_friction: number;
	public m_restitution: number;

	public m_proxyId: number /** uint */;
	public m_filter: b2FilterData;

	public m_isSensor: boolean;

	public m_userData: any;

	/// The various collision shape types supported by Box2D.
	//enum b2ShapeType
	//{
	public static readonly e_unknownShape: number /** int */ = 	-1;
	public static readonly e_circleShape: number /** int */ = 	0;
	public static readonly e_polygonShape: number /** int */ = 	1;
	public static readonly e_shapeTypeCount: number /** int */ = 	2;
	/// Possible return values for TestSegment
	public static readonly e_hitCollide: number /** int */ = 1;
	public static readonly e_missCollide: number /** int */ = 0;
	public static readonly e_startsInsideCollide: number /** int */ = -1;
	//};

}