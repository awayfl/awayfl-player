import { b2Vec2 } from '../../Common/Math';
import { b2Body } from '../b2Body';
import { b2Settings } from '../../Common/b2Settings';
import { b2JointEdge, b2JointDef, b2DistanceJoint, b2PulleyJoint, b2MouseJoint, b2PrismaticJoint, b2RevoluteJoint, b2GearJoint, b2LineJoint, b2WeldJoint, b2FrictionJoint } from '../Joints';
import { b2TimeStep } from '../b2TimeStep';
import { b2DistanceJointDef, b2PulleyJointDef, b2MouseJointDef, b2PrismaticJointDef, b2RevoluteJointDef, b2GearJointDef, b2LineJointDef, b2WeldJointDef, b2FrictionJointDef } from '../Joints';

/**
* The base joint class. Joints are used to constraint two bodies together in
* various fashions. Some joints also feature limits and motors.
* @see b2JointDef
*/
export class b2Joint {
	/**
	* Get the type of the concrete joint.
	*/
	public GetType(): number /** int */{
		return this.m_type;
	}

	/**
	* Get the anchor point on bodyA in world coordinates.
	*/
	public GetAnchorA(): b2Vec2 {return null;}
	/**
	* Get the anchor point on bodyB in world coordinates.
	*/
	public GetAnchorB(): b2Vec2 {return null;}

	/**
	* Get the reaction force on body2 at the joint anchor in Newtons.
	*/
	public GetReactionForce(inv_dt: number): b2Vec2 {return null;}
	/**
	* Get the reaction torque on body2 in N*m.
	*/
	public GetReactionTorque(inv_dt: number): number {return 0.0;}

	/**
	* Get the first body attached to this joint.
	*/
	public GetBodyA(): b2Body {
		return this.m_bodyA;
	}

	/**
	* Get the second body attached to this joint.
	*/
	public GetBodyB(): b2Body {
		return this.m_bodyB;
	}

	/**
	* Get the next joint the world joint list.
	*/
	public GetNext(): b2Joint {
		return this.m_next;
	}

	/**
	* Get the user data pointer.
	*/
	public GetUserData(): any {
		return this.m_userData;
	}

	/**
	* Set the user data pointer.
	*/
	public SetUserData(data: any): void {
		this.m_userData = data;
	}

	/**
	 * Short-cut function to determine if either body is inactive.
	 * @return
	 */
	public IsActive(): boolean {
		return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
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

			case b2Joint.e_lineJoint:
				{
				//void* mem = allocator->Allocate(sizeof(b2LineJoint));
					joint = new b2LineJoint(def as b2LineJointDef);
				}
				break;

			case b2Joint.e_weldJoint:
				{
				//void* mem = allocator->Allocate(sizeof(b2WeldJoint));
					joint = new b2WeldJoint(def as b2WeldJointDef);
				}
				break;

			case b2Joint.e_frictionJoint:
				{
				//void* mem = allocator->Allocate(sizeof(b2FrictionJoint));
					joint = new b2FrictionJoint(def as b2FrictionJointDef);
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
		case b2Joint.e_distanceJoint:
			allocator->Free(joint, sizeof(b2DistanceJoint));
			break;

		case b2Joint.e_mouseJoint:
			allocator->Free(joint, sizeof(b2MouseJoint));
			break;

		case b2Joint.e_prismaticJoint:
			allocator->Free(joint, sizeof(b2PrismaticJoint));
			break;

		case b2Joint.e_revoluteJoint:
			allocator->Free(joint, sizeof(b2RevoluteJoint));
			break;

		case b2Joint.e_pulleyJoint:
			allocator->Free(joint, sizeof(b2PulleyJoint));
			break;

		case b2Joint.e_gearJoint:
			allocator->Free(joint, sizeof(b2GearJoint));
			break;

		case b2Joint.e_lineJoint:
			allocator->Free(joint, sizeof(b2LineJoint));
			break;

		case b2Joint.e_weldJoint:
			allocator->Free(joint, sizeof(b2WeldJoint));
			break;

		case b2Joint.e_frictionJoint:
			allocator->Free(joint, sizeof(b2FrictionJoint));
			break;

		default:
			b2Assert(false);
			break;
		}*/
	}

	/** @private */
	constructor(def: b2JointDef) {
		b2Settings.b2Assert(def.bodyA != def.bodyB);
		this.m_type = def.type;
		this.m_prev = null;
		this.m_next = null;
		this.m_bodyA = def.bodyA;
		this.m_bodyB = def.bodyB;
		this.m_collideConnected = def.collideConnected;
		this.m_islandFlag = false;
		this.m_userData = def.userData;
	}
	//virtual ~b2Joint() {}

	public InitVelocityConstraints(step: b2TimeStep): void {}
	public SolveVelocityConstraints(step: b2TimeStep): void { }
	public FinalizeVelocityConstraints(): void {}

	// This returns true if the position errors are within tolerance.
	public SolvePositionConstraints(baumgarte: number): boolean { return false; }

	public m_type: number /** int */;
	public m_prev: b2Joint;
	public m_next: b2Joint;
	public m_edgeA: b2JointEdge = new b2JointEdge();
	public m_edgeB: b2JointEdge = new b2JointEdge();
	public m_bodyA: b2Body;
	public m_bodyB: b2Body;

	public m_islandFlag: boolean;
	public m_collideConnected: boolean;

	private m_userData: any;

	// Cache here per time step to reduce cache misses.
	public m_localCenterA: b2Vec2 = new b2Vec2();
	public m_localCenterB: b2Vec2 = new b2Vec2();
	public m_invMassA: number;
	public m_invMassB: number;
	public m_invIA: number;
	public m_invIB: number;

	// ENUMS

	// enum b2JointType
	public static readonly e_unknownJoint: number /** int */ = 0;
	public static readonly e_revoluteJoint: number /** int */ = 1;
	public static readonly e_prismaticJoint: number /** int */ = 2;
	public static readonly e_distanceJoint: number /** int */ = 3;
	public static readonly e_pulleyJoint: number /** int */ = 4;
	public static readonly e_mouseJoint: number /** int */ = 5;
	public static readonly e_gearJoint: number /** int */ = 6;
	public static readonly e_lineJoint: number /** int */ = 7;
	public static readonly e_weldJoint: number /** int */ = 8;
	public static readonly e_frictionJoint: number /** int */ = 9;

	// enum b2LimitState
	public static readonly e_inactiveLimit: number /** int */ = 0;
	public static readonly e_atLowerLimit: number /** int */ = 1;
	public static readonly e_atUpperLimit: number /** int */ = 2;
	public static readonly e_equalLimits: number /** int */ = 3;

}