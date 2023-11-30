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

import { b2Vec2 } from '../../Common/Math';
import { b2JointDef, b2JointEdge } from '../Joints';
import { b2TimeStep } from '../b2TimeStep';
import { b2Body } from '../b2Body';
import { b2DistanceJoint, b2MouseJoint, b2PrismaticJoint, b2RevoluteJoint, b2PulleyJoint, b2GearJoint } from '../Joints';
import { b2DistanceJointDef, b2MouseJointDef, b2PrismaticJointDef, b2RevoluteJointDef, b2PulleyJointDef, b2GearJointDef } from '../Joints';

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
export class b2Joint {
	readonly __fast__ = true;

	/// Get the type of the concrete joint.
	public GetType(): number /** int */{
		return this.m_type;
	}

	/// Get the anchor point on body1 in world coordinates.
	public GetAnchor1(): b2Vec2 {return null;}
	/// Get the anchor point on body2 in world coordinates.
	public GetAnchor2(): b2Vec2 {return null;}

	/// Get the reaction force on body2 at the joint anchor.
	public GetReactionForce(): b2Vec2 {return null;}
	/// Get the reaction torque on body2.
	public GetReactionTorque(): number {return 0.0;}

	/// Get the first body attached to this joint.
	public GetBody1(): b2Body {
		return this.m_body1;
	}

	/// Get the second body attached to this joint.
	public GetBody2(): b2Body {
		return this.m_body2;
	}

	/// Get the next joint the world joint list.
	public GetNext(): b2Joint {
		return this.m_next;
	}

	/// Get the user data pointer.
	public GetUserData(): any {
		return this.m_userData;
	}

	/// Set the user data pointer.
	public SetUserData(data: any): void {
		this.m_userData = data;
	}

	//--------------- Internals Below -------------------

	public static Create(def: b2JointDef, allocator: any): b2Joint {
		let joint: b2Joint = null;

		switch (def.type) {
			case b2Joint.e_distanceJoint:
				{
				//void* mem = allocator->Allocate(sizeof(b2DistanceJoint));
					joint = new b2DistanceJoint(def as b2DistanceJointDef);
				}
				break;

			case b2Joint.e_mouseJoint:
				{
				//void* mem = allocator->Allocate(sizeof(b2MouseJoint));
					joint = new b2MouseJoint(def as b2MouseJointDef);
				}
				break;

			case b2Joint.e_prismaticJoint:
				{
				//void* mem = allocator->Allocate(sizeof(b2PrismaticJoint));
					joint = new b2PrismaticJoint(def as b2PrismaticJointDef);
				}
				break;

			case b2Joint.e_revoluteJoint:
				{
				//void* mem = allocator->Allocate(sizeof(b2RevoluteJoint));
					joint = new b2RevoluteJoint(def as b2RevoluteJointDef);
				}
				break;

			case b2Joint.e_pulleyJoint:
				{
				//void* mem = allocator->Allocate(sizeof(b2PulleyJoint));
					joint = new b2PulleyJoint(def as b2PulleyJointDef);
				}
				break;

			case b2Joint.e_gearJoint:
				{
				//void* mem = allocator->Allocate(sizeof(b2GearJoint));
					joint = new b2GearJoint(def as b2GearJointDef);
				}
				break;

			default:
			//b2Settings.b2Assert(false);
				break;
		}

		return joint;
	}

	public static Destroy(joint: b2Joint, allocator: any): void {
		/*joint->~b2Joint();
		switch (joint.m_type)
		{
		case e_distanceJoint:
			allocator->Free(joint, sizeof(b2DistanceJoint));
			break;

		case e_mouseJoint:
			allocator->Free(joint, sizeof(b2MouseJoint));
			break;

		case e_prismaticJoint:
			allocator->Free(joint, sizeof(b2PrismaticJoint));
			break;

		case e_revoluteJoint:
			allocator->Free(joint, sizeof(b2RevoluteJoint));
			break;

		case e_pulleyJoint:
			allocator->Free(joint, sizeof(b2PulleyJoint));
			break;

		case e_gearJoint:
			allocator->Free(joint, sizeof(b2GearJoint));
			break;

		default:
			b2Assert(false);
			break;
		}*/
	}

	constructor(def: b2JointDef) {
		this.m_type = def.type;
		this.m_prev = null;
		this.m_next = null;
		this.m_body1 = def.body1;
		this.m_body2 = def.body2;
		this.m_collideConnected = def.collideConnected;
		this.m_islandFlag = false;
		this.m_userData = def.userData;
	}
	//virtual ~b2Joint() {}

	public InitVelocityConstraints(step: b2TimeStep): void {}
	public SolveVelocityConstraints(step: b2TimeStep): void {}

	// This returns true if the position errors are within tolerance.
	public InitPositionConstraints(): void {}
	public SolvePositionConstraints(): boolean {return false;}

	public m_type: number /** int */;
	public m_prev: b2Joint;
	public m_next: b2Joint;
	public m_node1: b2JointEdge = new b2JointEdge();
	public m_node2: b2JointEdge = new b2JointEdge();
	public m_body1: b2Body;
	public m_body2: b2Body;

	public m_inv_dt: number;

	public m_islandFlag: boolean;
	public m_collideConnected: boolean;

	public m_userData: any;

	// ENUMS

	// enum b2JointType
	public static readonly e_unknownJoint: number /** int */ = 0;
	public static readonly e_revoluteJoint: number /** int */ = 1;
	public static readonly e_prismaticJoint: number /** int */ = 2;
	public static readonly e_distanceJoint: number /** int */ = 3;
	public static readonly e_pulleyJoint: number /** int */ = 4;
	public static readonly e_mouseJoint: number /** int */ = 5;
	public static readonly e_gearJoint: number /** int */ = 6;

	// enum b2LimitState
	public static readonly e_inactiveLimit: number /** int */ = 0;
	public static readonly e_atLowerLimit: number /** int */ = 1;
	public static readonly e_atUpperLimit: number /** int */ = 2;
	public static readonly e_equalLimits: number /** int */ = 3;

}