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

import { b2ShapeDef } from '../Collision/Shapes/b2ShapeDef';
import { b2Shape } from '../Collision/Shapes/b2Shape';
import { b2MassData } from '../Collision/Shapes/b2MassData';
import { b2XForm, b2Sweep, b2Vec2, b2Mat22, b2Math } from '../Common/Math';
import { b2JointEdge } from './Joints';
import { b2ContactEdge } from './Contacts/b2ContactEdge';
import { b2BodyDef } from './b2BodyDef';
import { b2World } from './b2World';
import { b2CircleShape } from '../Collision/Shapes/b2CircleShape';
import { b2PolygonShape } from '../Collision/Shapes/b2PolygonShape';
import { b2ConvexArcShape } from '../Collision/Shapes/b2ConvexArcShape';
import { b2ConcaveArcShape } from '../Collision/Shapes/b2ConcaveArcShape';

/// A rigid body.
export class b2Body {
	__fast__ = true;

	/// Creates a shape and attach it to this body.
	/// @param shapeDef the shape definition.
	/// @warning This function is locked during callbacks.
	public CreateShape(def: b2ShapeDef): b2Shape {
		//b2Settings.b2Assert(this.m_world.m_lock == false);
		if (this.m_world.m_lock == true) {
			return null;
		}

		let s: b2Shape;

		switch (def.type) {
			case b2Shape.e_circleShape:
			{
				//void* mem = allocator->Allocate(sizeof(b2CircleShape));
				s = new b2CircleShape(def);
				break;
			}

			case b2Shape.e_polygonShape:
			{
				//void* mem = allocator->Allocate(sizeof(b2PolygonShape));
				s = new b2PolygonShape(def);
				break;
			}

			case b2Shape.e_convexArcShape:
				{
					return new b2ConvexArcShape(def);
				}

			case b2Shape.e_concaveArcShape:
				{
					return new b2ConcaveArcShape(def);
				}		

			default:
				//b2Settings.b2Assert(false);
				throw new Error("Shape type not found or cannot be added to Dynamic Bodies.");
				return null;
		}

		s.m_next = this.m_shapeList;
		this.m_shapeList = s;
		++this.m_shapeCount;

		s.m_body = this;

		// Add the shape to the world's broad-phase.
		s.CreateProxy(this.m_world.m_broadPhase, this.m_xf);

		// Compute the sweep radius for CCD.
		s.UpdateSweepRadius(this.m_sweep.localCenter);

		return s;
	}

	/// Destroy a shape. This removes the shape from the broad-phase and
	/// therefore destroys any contacts associated with this shape. All shapes
	/// attached to a body are implicitly destroyed when the body is destroyed.
	/// @param shape the shape to be removed.
	/// @warning This function is locked during callbacks.
	public DestroyShape(s: b2Shape): void {
		//b2Settings.b2Assert(this.m_world.m_lock == false);
		if (this.m_world.m_lock == true) {
			return;
		}

		//b2Settings.b2Assert(s.m_body == this);
		s.DestroyProxy(this.m_world.m_broadPhase);

		//b2Settings.b2Assert(this.m_shapeCount > 0);
		//b2Shape** node = &this.m_shapeList;
		let node: b2Shape = this.m_shapeList;
		let ppS: b2Shape = null; // Fix pointer-pointer stuff
		let found: boolean = false;
		while (node != null) {
			if (node == s) {
				if (ppS)
					ppS.m_next = s.m_next;
				else
					this.m_shapeList = s.m_next;
				//node = s.m_next;
				found = true;
				break;
			}

			ppS = node;
			node = node.m_next;
		}

		// You tried to remove a shape that is not attached to this body.
		//b2Settings.b2Assert(found);

		s.m_body = null;
		s.m_next = null;

		--this.m_shapeCount;

		b2Shape.Destroy(s, this.m_world.m_blockAllocator);
	}

	/// Set the mass properties. Note that this changes the center of mass position.
	/// If you are not sure how to compute mass properties, use SetMassFromShapes.
	/// The inertia tensor is assumed to be relative to the center of mass.
	/// @param massData the mass properties.
	public SetMass(massData: b2MassData): void {
		let s: b2Shape;

		//b2Settings.b2Assert(this.m_world.m_lock == false);
		if (this.m_world.m_lock == true) {
			return;
		}

		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;

		this.m_mass = massData.mass;

		if (this.m_mass > 0.0) {
			this.m_invMass = 1.0 / this.m_mass;
		}

		if ((this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
			this.m_I = massData.I;
		}

		if (this.m_I > 0.0) {
			this.m_invI = 1.0 / this.m_I;
		}

		// Move center of mass.
		this.m_sweep.localCenter.SetV(massData.center);
		//this.m_sweep.c0 = this.m_sweep.c = b2Mul(this.m_xf, this.m_sweep.localCenter);
		//b2MulMV(this.m_xf.R, this.m_sweep.localCenter);
		const tMat: b2Mat22 = this.m_xf.R;
		const tVec: b2Vec2 = this.m_sweep.localCenter;
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//return T.position + b2Mul(T.R, v);
		this.m_sweep.c.x += this.m_xf.position.x;
		this.m_sweep.c.y += this.m_xf.position.y;
		//this.m_sweep.c0 = this.m_sweep.c
		this.m_sweep.c0.SetV(this.m_sweep.c);

		// Update the sweep radii of all child shapes.
		for (s = this.m_shapeList; s; s = s.m_next) {
			s.UpdateSweepRadius(this.m_sweep.localCenter);
		}

		const oldType: number /** int */ = this.m_type;
		if (this.m_invMass == 0.0 && this.m_invI == 0.0) {
			this.m_type = b2Body.e_staticType;
		} else {
			this.m_type = b2Body.e_dynamicType;
		}

		// If the body type changed, we need to refilter the broad-phase proxies.
		if (oldType != this.m_type) {
			for (s = this.m_shapeList; s; s = s.m_next) {
				s.RefilterProxy(this.m_world.m_broadPhase, this.m_xf);
			}
		}
	}

	/// Compute the mass properties from the attached shapes. You typically call this
	/// after adding all the shapes. If you add or remove shapes later, you may want
	/// to call this again. Note that this changes the center of mass position.
	private static s_massData: b2MassData = new b2MassData();
	public SetMassFromShapes(): void {

		let s: b2Shape;

		//b2Settings.b2Assert(this.m_world.m_lock == false);
		if (this.m_world.m_lock == true) {
			return;
		}

		// Compute mass data from shapes. Each shape has its own density.
		this.m_mass = 0.0;
		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;

		//b2Vec2 center = b2Vec2_zero;
		let centerX: number = 0.0;
		let centerY: number = 0.0;
		const massData: b2MassData = b2Body.s_massData;
		for (s = this.m_shapeList; s; s = s.m_next) {
			s.ComputeMass(massData);
			this.m_mass += massData.mass;
			//center += massData.mass * massData.center;
			centerX += massData.mass * massData.center.x;
			centerY += massData.mass * massData.center.y;
			this.m_I += massData.I;
		}

		// Compute center of mass, and shift the origin to the COM.
		if (this.m_mass > 0.0) {
			this.m_invMass = 1.0 / this.m_mass;
			centerX *= this.m_invMass;
			centerY *= this.m_invMass;
		}

		if (this.m_I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
			// Center the inertia about the center of mass.
			//this.m_I -= this.m_mass * b2Dot(center, center);
			this.m_I -= this.m_mass * (centerX * centerX + centerY * centerY);
			//b2Settings.b2Assert(this.m_I > 0.0);
			this.m_invI = 1.0 / this.m_I;
		} else {
			this.m_I = 0.0;
			this.m_invI = 0.0;
		}

		// Move center of mass.
		this.m_sweep.localCenter.Set(centerX, centerY);
		//this.m_sweep.c0 = this.m_sweep.c = b2Mul(this.m_xf, this.m_sweep.localCenter);
		//b2MulMV(this.m_xf.R, this.m_sweep.localCenter);
		const tMat: b2Mat22 = this.m_xf.R;
		const tVec: b2Vec2 = this.m_sweep.localCenter;
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//return T.position + b2Mul(T.R, v);
		this.m_sweep.c.x += this.m_xf.position.x;
		this.m_sweep.c.y += this.m_xf.position.y;
		//this.m_sweep.c0 = this.m_sweep.c
		this.m_sweep.c0.SetV(this.m_sweep.c);

		// Update the sweep radii of all child shapes.
		for (s = this.m_shapeList; s; s = s.m_next) {
			s.UpdateSweepRadius(this.m_sweep.localCenter);
		}

		const oldType: number /** int */ = this.m_type;
		if (this.m_invMass == 0.0 && this.m_invI == 0.0) {
			this.m_type = b2Body.e_staticType;
		} else {
			this.m_type = b2Body.e_dynamicType;
		}

		// If the body type changed, we need to refilter the broad-phase proxies.
		if (oldType != this.m_type) {
			for (s = this.m_shapeList; s; s = s.m_next) {
				s.RefilterProxy(this.m_world.m_broadPhase, this.m_xf);
			}
		}
	}

	/// Set the position of the body's origin and rotation (radians).
	/// This breaks any contacts and wakes the other bodies.
	/// @param position the new world position of the body's origin (not necessarily
	/// the center of mass).
	/// @param angle the new world rotation angle of the body in radians.
	/// @return false if the movement put a shape outside the world. In this case the
	/// body is automatically frozen.
	public SetXForm(position: b2Vec2, angle: number): boolean {

		let s: b2Shape;

		//b2Settings.b2Assert(this.m_world.m_lock == false);
		if (this.m_world.m_lock == true) {
			return true;
		}

		if (this.IsFrozen()) {
			return false;
		}

		this.m_xf.R.Set(angle);
		this.m_xf.position.SetV(position);

		//this.m_sweep.c0 = this.m_sweep.c = b2Mul(this.m_xf, this.m_sweep.localCenter);
		//b2MulMV(this.m_xf.R, this.m_sweep.localCenter);
		const tMat: b2Mat22 = this.m_xf.R;
		const tVec: b2Vec2 = this.m_sweep.localCenter;
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//return T.position + b2Mul(T.R, v);
		this.m_sweep.c.x += this.m_xf.position.x;
		this.m_sweep.c.y += this.m_xf.position.y;
		//this.m_sweep.c0 = this.m_sweep.c
		this.m_sweep.c0.SetV(this.m_sweep.c);

		this.m_sweep.a0 = this.m_sweep.a = angle;

		let freeze: boolean = false;
		for (s = this.m_shapeList; s; s = s.m_next) {
			const inRange: boolean = s.Synchronize(this.m_world.m_broadPhase, this.m_xf, this.m_xf);

			if (inRange == false) {
				freeze = true;
				break;
			}
		}

		if (freeze == true) {
			this.m_flags |= b2Body.e_frozenFlag;
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0.0;
			for (s = this.m_shapeList; s; s = s.m_next) {
				s.DestroyProxy(this.m_world.m_broadPhase);
			}

			// Failure
			return false;
		}

		// Success
		this.m_world.m_broadPhase.Commit();
		return true;

	}

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	public GetXForm(): b2XForm {
		return this.m_xf;
	}

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	public GetPosition(): b2Vec2 {
		return this.m_xf.position;
	}

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	public GetAngle(): number {
		return this.m_sweep.a;
	}

	/// Get the world position of the center of mass.
	public GetWorldCenter(): b2Vec2 {
		return this.m_sweep.c;
	}

	/// Get the local position of the center of mass.
	public GetLocalCenter(): b2Vec2 {
		return this.m_sweep.localCenter;
	}

	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	public SetLinearVelocity(v: b2Vec2): void {
		this.m_linearVelocity.SetV(v);
	}

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	public GetLinearVelocity(): b2Vec2 {
		return this.m_linearVelocity;
	}

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	public SetAngularVelocity(omega: number): void {
		this.m_angularVelocity = omega;
	}

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	public GetAngularVelocity(): number {
		return this.m_angularVelocity;
	}

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	public ApplyForce(force: b2Vec2, point: b2Vec2): void {
		if (this.IsSleeping()) {
			this.WakeUp();
		}
		//this.m_force += force;
		this.m_force.x += force.x;
		this.m_force.y += force.y;
		//this.m_torque += b2Cross(point - this.m_sweep.c, force);
		this.m_torque += ((point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x);
	}

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// This wakes up the body.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	public ApplyTorque(torque: number): void {
		if (this.IsSleeping()) {
			this.WakeUp();
		}
		this.m_torque += torque;
	}

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	public ApplyImpulse(impulse: b2Vec2, point: b2Vec2): void {
		if (this.IsSleeping()) {
			this.WakeUp();
		}
		//this.m_linearVelocity += this.m_invMass * impulse;
		this.m_linearVelocity.x += this.m_invMass * impulse.x;
		this.m_linearVelocity.y += this.m_invMass * impulse.y;
		//this.m_angularVelocity += this.m_invI * b2Cross(point - this.m_sweep.c, impulse);
		this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
	}

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	public GetMass(): number {
		return this.m_mass;
	}

	/// Get the central rotational inertia of the body.
	/// @return the rotational inertia, usually in kg-m^2.
	public GetInertia(): number {
		return this.m_I;
	}

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	public GetWorldPoint(localPoint: b2Vec2): b2Vec2 {
		//return b2Math.b2MulX(this.m_xf, localPoint);
		const A: b2Mat22 = this.m_xf.R;
		const u: b2Vec2 = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y,
								  A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		u.x += this.m_xf.position.x;
		u.y += this.m_xf.position.y;
		return u;
	}

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	public GetWorldVector(localVector: b2Vec2): b2Vec2 {
		return b2Math.b2MulMV(this.m_xf.R, localVector);
	}

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	public GetLocalPoint(worldPoint: b2Vec2): b2Vec2 {
		return b2Math.b2MulXT(this.m_xf, worldPoint);
	}

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	public GetLocalVector(worldVector: b2Vec2): b2Vec2 {
		return b2Math.b2MulTMV(this.m_xf.R, worldVector);
	}

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	public GetLinearVelocityFromWorldPoint(worldPoint: b2Vec2): b2Vec2 {
		//return          this.m_linearVelocity   + b2Cross(this.m_angularVelocity,   worldPoint   - this.m_sweep.c);
		return new b2Vec2(this.m_linearVelocity.x +         this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y),
		                  this.m_linearVelocity.x -         this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
	}

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	public GetLinearVelocityFromLocalPoint(localPoint: b2Vec2): b2Vec2 {
		//return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
		const A: b2Mat22 = this.m_xf.R;
		const worldPoint: b2Vec2 = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y,
								  A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		worldPoint.x += this.m_xf.position.x;
		worldPoint.y += this.m_xf.position.y;
		return new b2Vec2(this.m_linearVelocity.x +         this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y),
		                  this.m_linearVelocity.x -         this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
	}

	/// Is this body treated like a bullet for continuous collision detection?
	public IsBullet(): boolean {
		return (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
	}

	/// Should this body be treated like a bullet for continuous collision detection?
	public SetBullet(flag: boolean): void {
		if (flag) {
			this.m_flags |= b2Body.e_bulletFlag;
		} else {
			this.m_flags &= ~b2Body.e_bulletFlag;
		}
	}

	/// Is this body static (immovable)?
	public IsStatic(): boolean {
		return this.m_type == b2Body.e_staticType;
	}

	/// Is this body dynamic (movable)?
	public IsDynamic(): boolean {
		return this.m_type == b2Body.e_dynamicType;
	}

	/// Is this body frozen?
	public IsFrozen(): boolean {
		return (this.m_flags & b2Body.e_frozenFlag) == b2Body.e_frozenFlag;
	}

	/// Is this body sleeping (not simulating).
	public IsSleeping(): boolean {
		return (this.m_flags & b2Body.e_sleepFlag) == b2Body.e_sleepFlag;
	}

	/// You can disable sleeping on this body.
	public AllowSleeping(flag: boolean): void {
		if (flag) {
			this.m_flags |= b2Body.e_allowSleepFlag;
		} else {
			this.m_flags &= ~b2Body.e_allowSleepFlag;
			this.WakeUp();
		}
	}

	/// Wake up this body so it will begin simulating.
	public WakeUp(): void {
		this.m_flags &= ~b2Body.e_sleepFlag;
		this.m_sleepTime = 0.0;
	}

	/// Put this body to sleep so it will stop simulating.
	/// This also sets the velocity to zero.
	public PutToSleep(): void {
		this.m_flags |= b2Body.e_sleepFlag;
		this.m_sleepTime = 0.0;
		this.m_linearVelocity.SetZero();
		this.m_angularVelocity = 0.0;
		this.m_force.SetZero();
		this.m_torque = 0.0;
	}

	/// Get the list of all shapes attached to this body.
	public GetShapeList(): b2Shape {
		return this.m_shapeList;
	}

	/// Get the list of all joints attached to this body.
	public GetJointList(): b2JointEdge {
		return this.m_jointList;
	}

	/// Get the next body in the world's body list.
	public GetNext(): b2Body {
		return this.m_next;
	}

	/// Get the user data pointer that was provided in the body definition.
	public GetUserData(): any {
		return this.m_userData;
	}

	/// Set the user data. Use this to store your application specific data.
	public SetUserData(data: any): void {
		this.m_userData = data;
	}

	/// Get the parent world of this body.
	public GetWorld(): b2World {
		return this.m_world;
	}

	//--------------- Internals Below -------------------

	// Constructor
	constructor(bd: b2BodyDef, world: b2World) {
		//b2Settings.b2Assert(world.m_lock == false);

		this.m_flags = 0;

		if (bd.isBullet) {
			this.m_flags |= b2Body.e_bulletFlag;
		}
		if (bd.fixedRotation) {
			this.m_flags |= b2Body.e_fixedRotationFlag;
		}
		if (bd.allowSleep) {
			this.m_flags |= b2Body.e_allowSleepFlag;
		}
		if (bd.isSleeping) {
			this.m_flags |= b2Body.e_sleepFlag;
		}

		this.m_world = world;

		this.m_xf.position.SetV(bd.position);
		this.m_xf.R.Set(bd.angle);

		this.m_sweep.localCenter.SetV(bd.massData.center);
		this.m_sweep.t0 = 1.0;
		this.m_sweep.a0 = this.m_sweep.a = bd.angle;

		//this.m_sweep.c0 = this.m_sweep.c = b2Mul(this.m_xf, this.m_sweep.localCenter);
		//b2MulMV(this.m_xf.R, this.m_sweep.localCenter);
		const tMat: b2Mat22 = this.m_xf.R;
		const tVec: b2Vec2 = this.m_sweep.localCenter;
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//return T.position + b2Mul(T.R, v);
		this.m_sweep.c.x += this.m_xf.position.x;
		this.m_sweep.c.y += this.m_xf.position.y;
		//this.m_sweep.c0 = this.m_sweep.c
		this.m_sweep.c0.SetV(this.m_sweep.c);

		this.m_jointList = null;
		this.m_contactList = null;
		this.m_prev = null;
		this.m_next = null;

		this.m_linearDamping = bd.linearDamping;
		this.m_angularDamping = bd.angularDamping;

		this.m_force.Set(0.0, 0.0);
		this.m_torque = 0.0;

		this.m_linearVelocity.SetZero();
		this.m_angularVelocity = 0.0;

		this.m_sleepTime = 0.0;

		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;

		this.m_mass = bd.massData.mass;

		if (this.m_mass > 0.0) {
			this.m_invMass = 1.0 / this.m_mass;
		}

		if ((this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
			this.m_I = bd.massData.I;
		}

		if (this.m_I > 0.0) {
			this.m_invI = 1.0 / this.m_I;
		}

		if (this.m_invMass == 0.0 && this.m_invI == 0.0) {
			this.m_type = b2Body.e_staticType;
		} else {
			this.m_type = b2Body.e_dynamicType;
		}

		this.m_userData = bd.userData;

		this.m_shapeList = null;
		this.m_shapeCount = 0;
	}

	// Destructor
	//~b2Body();

	//
	private static s_xf1: b2XForm = new b2XForm();
	//
	public SynchronizeShapes(): boolean {

		const xf1: b2XForm = b2Body.s_xf1;
		xf1.R.Set(this.m_sweep.a0);
		//xf1.position = this.m_sweep.c0 - b2Mul(xf1.R, this.m_sweep.localCenter);
		const tMat: b2Mat22 = xf1.R;
		const tVec: b2Vec2 = this.m_sweep.localCenter;
		xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		let s: b2Shape;

		let inRange: boolean = true;
		for (s = this.m_shapeList; s; s = s.m_next) {
			inRange = s.Synchronize(this.m_world.m_broadPhase, xf1, this.m_xf);
			if (inRange == false) {
				break;
			}
		}

		if (inRange == false) {
			this.m_flags |= b2Body.e_frozenFlag;
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0.0;
			for (s = this.m_shapeList; s; s = s.m_next) {
				s.DestroyProxy(this.m_world.m_broadPhase);
			}

			// Failure
			return false;
		}

		// Success
		return true;

	}

	public SynchronizeTransform(): void {
		this.m_xf.R.Set(this.m_sweep.a);
		//this.m_xf.position = this.m_sweep.c - b2Mul(this.m_xf.R, this.m_sweep.localCenter);
		const tMat: b2Mat22 = this.m_xf.R;
		const tVec: b2Vec2 = this.m_sweep.localCenter;
		this.m_xf.position.x = this.m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		this.m_xf.position.y = this.m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
	}

	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	public IsConnected(other: b2Body): boolean {
		for (let jn: b2JointEdge = this.m_jointList; jn; jn = jn.next) {
			if (jn.other == other)
				return jn.joint.m_collideConnected == false;
		}

		return false;
	}

	public Advance(t: number): void {
		// Advance to the new safe time.
		this.m_sweep.Advance(t);
		this.m_sweep.c.SetV(this.m_sweep.c0);
		this.m_sweep.a = this.m_sweep.a0;
		this.SynchronizeTransform();
	}

	public m_flags: number /** uint */;
	public m_type: number /** int */;

	public m_xf: b2XForm = new b2XForm();		// the body origin transform

	public m_sweep: b2Sweep = new b2Sweep();	// the swept motion for CCD

	public m_linearVelocity: b2Vec2 = new b2Vec2();
	public m_angularVelocity: number;

	public m_force: b2Vec2 = new b2Vec2();
	public m_torque: number;

	public m_world: b2World;
	public m_prev: b2Body;
	public m_next: b2Body;

	public m_shapeList: b2Shape;
	public m_shapeCount: number /** int */;

	public m_jointList: b2JointEdge;
	public m_contactList: b2ContactEdge;

	public m_mass: number
	public m_invMass: number;
	public m_I: number
	public m_invI: number;

	public m_linearDamping: number;
	public m_angularDamping: number;

	public m_sleepTime: number;

	public m_userData: any;

	// this.m_flags
	//enum
	//{
	public static e_frozenFlag: number /** uint */			= 0x0002;
	public static e_islandFlag: number /** uint */			= 0x0004;
	public static e_sleepFlag: number /** uint */			= 0x0008;
	public static e_allowSleepFlag: number /** uint */		= 0x0010;
	public static e_bulletFlag: number /** uint */			= 0x0020;
	public static e_fixedRotationFlag: number /** uint */	= 0x0040;
	//};

	// this.m_type
	//enum
	//{
	public static e_staticType: number /** uint */ 	= 1;
	public static e_dynamicType: number /** uint */ 	= 2;
	public static e_maxTypes: number /** uint */ 		= 3;
	//};

}