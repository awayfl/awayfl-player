import { b2Body } from '../b2Body';
import { b2Manifold } from '../../Collision/b2Manifold';
import { b2Shape } from '../../Collision/Shapes/b2Shape';
import { b2ContactEdge } from './b2ContactEdge';
import { b2Fixture } from '../b2Fixture';
import { b2Settings } from '../../Common/b2Settings';
import { b2CircleShape } from '../../Collision/Shapes/b2CircleShape';
import { b2TimeOfImpact } from '../../Collision/b2TimeOfImpact';
import { b2Sweep, b2Transform } from '../../Common/Math';
import { b2TOIInput } from '../../Collision/b2TOIInput';
import { b2ManifoldPoint } from '../../Collision/b2ManifoldPoint';
import { b2ContactID } from '../../Collision/b2ContactID';
import { b2ContactListener } from '../b2ContactListener';
import { b2WorldManifold } from '../../Collision/b2WorldManifold';

/**
* The class manages contact between two shapes. A contact exists for each overlapping
* AABB in the broad-phase (except if filtered). Therefore a contact object may exist
* that has no contact points.
*/
export class b2Contact {
	readonly __fast__ = true;

	/**
	 * Get the contact manifold. Do not modify the manifold unless you understand the
	 * internals of Box2D
	 */
	public GetManifold(): b2Manifold {
		return this.m_manifold;
	}

	/**
	 * Get the world manifold
	 */
	public GetWorldManifold(worldManifold: b2WorldManifold): void {
		const bodyA: b2Body = this.m_fixtureA.GetBody();
		const bodyB: b2Body = this.m_fixtureB.GetBody();
		const shapeA: b2Shape = this.m_fixtureA.GetShape();
		const shapeB: b2Shape = this.m_fixtureB.GetShape();

		worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	}

	/**
	 * Is this contact touching.
	 */
	public IsTouching(): boolean {
		return (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag;
	}

	/**
	 * Does this contact generate TOI events for continuous simulation
	 */
	public IsContinuous(): boolean {
		return (this.m_flags & b2Contact.e_continuousFlag) == b2Contact.e_continuousFlag;
	}

	/**
	 * Change this to be a sensor or-non-sensor contact.
	 */
	public SetSensor(sensor: boolean): void {
		if (sensor) {
			this.m_flags |= b2Contact.e_sensorFlag;
		} else {
			this.m_flags &= ~b2Contact.e_sensorFlag;
		}
	}

	/**
	 * Is this contact a sensor?
	 */
	public IsSensor(): boolean {
		return (this.m_flags & b2Contact.e_sensorFlag) == b2Contact.e_sensorFlag;
	}

	/**
	 * Enable/disable this contact. This can be used inside the pre-solve
	 * contact listener. The contact is only disabled for the current
	 * time step (or sub-step in continuous collision).
	 */
	public SetEnabled(flag: boolean): void {
		if (flag) {
			this.m_flags |= b2Contact.e_enabledFlag;
		} else {
			this.m_flags &= ~b2Contact.e_enabledFlag;
		}
	}

	/**
	 * Has this contact been disabled?
	 * @return
	 */
	public IsEnabled(): boolean {
		return (this.m_flags & b2Contact.e_enabledFlag) == b2Contact.e_enabledFlag;
	}

	/**
	* Get the next contact in the world's contact list.
	*/
	public GetNext(): b2Contact {
		return this.m_next;
	}

	/**
	* Get the first fixture in this contact.
	*/
	public GetFixtureA(): b2Fixture {
		return this.m_fixtureA;
	}

	/**
	* Get the second fixture in this contact.
	*/
	public GetFixtureB(): b2Fixture {
		return this.m_fixtureB;
	}

	/**
	 * Flag this contact for filtering. Filtering will occur the next time step.
	 */
	public FlagForFiltering(): void {
		this.m_flags |= b2Contact.e_filterFlag;
	}

	//--------------- Internals Below -------------------

	// m_flags
	// enum
	// This contact should not participate in Solve
	// The contact equivalent of sensors
	public static e_sensorFlag: number /** uint */		= 0x0001;
	// Generate TOI events.
	public static e_continuousFlag: number /** uint */	= 0x0002;
	// Used when crawling contact graph when forming islands.
	public static e_islandFlag: number /** uint */		= 0x0004;
	// Used in SolveTOI to indicate the cached toi value is still valid.
	public static e_toiFlag: number /** uint */		= 0x0008;
	// Set when shapes are touching
	public static e_touchingFlag: number /** uint */	= 0x0010;
	// This contact can be disabled (by user)
	public static e_enabledFlag: number /** uint */	= 0x0020;
	// This contact needs filtering because a fixture filter was changed.
	public static e_filterFlag: number /** uint */		= 0x0040;

	constructor() {
		// Real work is done in Reset
	}

	/** @private */
	public Reset(fixtureA: b2Fixture, fixtureB: b2Fixture): void {
		this.m_flags = b2Contact.e_enabledFlag;

		if (!fixtureA || !fixtureB) {
			this.m_fixtureA = null;
			this.m_fixtureB = null;
			return;
		}

		if (fixtureA.IsSensor() || fixtureB.IsSensor()) {
			this.m_flags |= b2Contact.e_sensorFlag;
		}

		const bodyA: b2Body = fixtureA.GetBody();
		const bodyB: b2Body = fixtureB.GetBody();

		if (bodyA.GetType() != b2Body.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() != b2Body.b2_dynamicBody || bodyB.IsBullet()) {
			this.m_flags |= b2Contact.e_continuousFlag;
		}

		this.m_fixtureA = fixtureA;
		this.m_fixtureB = fixtureB;

		this.m_manifold.m_pointCount = 0;

		this.m_prev = null;
		this.m_next = null;

		this.m_nodeA.contact = null;
		this.m_nodeA.prev = null;
		this.m_nodeA.next = null;
		this.m_nodeA.other = null;

		this.m_nodeB.contact = null;
		this.m_nodeB.prev = null;
		this.m_nodeB.next = null;
		this.m_nodeB.other = null;
	}

	public Update(listener: b2ContactListener): void {
		// Swap old & new manifold
		const tManifold: b2Manifold = this.m_oldManifold;
		this.m_oldManifold = this.m_manifold;
		this.m_manifold = tManifold;

		// Re-enable this contact
		this.m_flags |= b2Contact.e_enabledFlag;

		let touching: boolean = false;
		const wasTouching: boolean = (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag;

		const bodyA: b2Body = this.m_fixtureA.m_body;
		const bodyB: b2Body = this.m_fixtureB.m_body;

		const aabbOverlap: boolean = this.m_fixtureA.m_aabb.TestOverlap(this.m_fixtureB.m_aabb);

		// Is this contat a sensor?
		if (this.m_flags  & b2Contact.e_sensorFlag) {
			if (aabbOverlap) {
				const shapeA: b2Shape = this.m_fixtureA.GetShape();
				const shapeB: b2Shape = this.m_fixtureB.GetShape();
				const xfA: b2Transform = bodyA.GetTransform();
				const xfB: b2Transform = bodyB.GetTransform();
				touching = b2Shape.TestOverlap(shapeA, xfA, shapeB, xfB);
			}

			// Sensors don't generate manifolds
			this.m_manifold.m_pointCount = 0;
		} else {
			// Slow contacts don't generate TOI events.
			if (bodyA.GetType() != b2Body.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() != b2Body.b2_dynamicBody || bodyB.IsBullet()) {
				this.m_flags |= b2Contact.e_continuousFlag;
			} else {
				this.m_flags &= ~b2Contact.e_continuousFlag;
			}

			if (aabbOverlap) {
				this.Evaluate();

				touching = this.m_manifold.m_pointCount > 0;

				// Match old contact ids to new contact ids and copy the
				// stored impulses to warm start the solver.
				for (let i: number /** int */ = 0; i < this.m_manifold.m_pointCount; ++i) {
					const mp2: b2ManifoldPoint = this.m_manifold.m_points[i];
					mp2.m_normalImpulse = 0.0;
					mp2.m_tangentImpulse = 0.0;
					const id2: b2ContactID = mp2.m_id;

					for (let j: number /** int */ = 0; j < this.m_oldManifold.m_pointCount; ++j) {
						const mp1: b2ManifoldPoint = this.m_oldManifold.m_points[j];

						if (mp1.m_id.key == id2.key) {
							mp2.m_normalImpulse = mp1.m_normalImpulse;
							mp2.m_tangentImpulse = mp1.m_tangentImpulse;
							break;
						}
					}
				}

			} else {
				this.m_manifold.m_pointCount = 0;
			}
			if (touching != wasTouching) {
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}
		}

		if (touching) {
			this.m_flags |= b2Contact.e_touchingFlag;
		} else {
			this.m_flags &= ~b2Contact.e_touchingFlag;
		}

		if (wasTouching == false && touching == true) {
			listener.BeginContact(this);
		}

		if (wasTouching == true && touching == false) {
			listener.EndContact(this);
		}

		if ((this.m_flags & b2Contact.e_sensorFlag) == 0) {
			listener.PreSolve(this, this.m_oldManifold);
		}
	}

	//virtual ~b2Contact() {}

	public Evaluate(): void {}

	private static s_input: b2TOIInput = new b2TOIInput();
	public ComputeTOI(sweepA: b2Sweep, sweepB: b2Sweep): number {
		b2Contact.s_input.proxyA.Set(this.m_fixtureA.GetShape());
		b2Contact.s_input.proxyB.Set(this.m_fixtureB.GetShape());
		b2Contact.s_input.sweepA = sweepA;
		b2Contact.s_input.sweepB = sweepB;
		b2Contact.s_input.tolerance = b2Settings.b2_linearSlop;
		return b2TimeOfImpact.TimeOfImpact(b2Contact.s_input);
	}

	public m_flags: number /** uint */;

	// World pool and list pointers.
	public m_prev: b2Contact;
	public m_next: b2Contact;

	// Nodes for connecting bodies.
	public m_nodeA: b2ContactEdge = new b2ContactEdge();
	public m_nodeB: b2ContactEdge = new b2ContactEdge();

	public m_fixtureA: b2Fixture;
	public m_fixtureB: b2Fixture;

	public m_manifold: b2Manifold = new b2Manifold();
	public m_oldManifold: b2Manifold = new b2Manifold();

	public m_toi: number;
}