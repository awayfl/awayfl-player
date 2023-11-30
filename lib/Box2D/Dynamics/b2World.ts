import { ASMethodClosure, ASClass } from '@awayfl/avm2';

import { b2Vec2, b2Math, b2Transform, b2Sweep } from '../Common/Math';
import { b2Island } from './b2Island';
import { b2Body } from './b2Body';
import { b2Contact, b2ContactEdge, b2ContactSolver } from './Contacts';
import { b2PolygonShape } from '../Collision/Shapes/b2PolygonShape';
import { b2CircleShape } from '../Collision/Shapes/b2CircleShape';
import { b2Shape } from '../Collision/Shapes/b2Shape';
import { b2Settings } from '../Common/b2Settings';
import { b2Fixture } from './b2Fixture';
import { b2JointEdge, b2PulleyJoint, b2Joint, b2JointDef } from './Joints';
import { b2TimeStep } from './b2TimeStep';
import { b2Controller } from './Controllers/b2Controller';
import { b2Color } from '../Common/b2Color';
import { b2EdgeShape } from '../Collision/Shapes/b2EdgeShape';
import { b2DebugDraw } from './b2DebugDraw';
import { b2DestructionListener } from './b2DestructionListener';
import { b2BodyDef } from './b2BodyDef';
import { b2ContactListener } from './b2ContactListener';
import { b2ContactManager } from './b2ContactManager';
import { b2RayCastInput } from '../Collision/b2RayCastInput';
import { b2RayCastOutput } from '../Collision/b2RayCastOutput';
import { IBroadPhase } from '../Collision/IBroadPhase';
import { b2AABB } from '../Collision/b2AABB';
import { b2ContactFilter } from './b2ContactFilter';
import { b2ControllerEdge } from './Controllers/b2ControllerEdge';

// unbox methods from ASClass to real class prototupe
function unBoxMethods (object: ASClass | any, prototype: any) {
	if (!object || typeof object['traits'] === 'undefined') {
		return object;
	}

	const names = Object.getOwnPropertyNames(prototype);
	const mangle = '$Bg';

	for (const name of names) {
		if (!object[name] && object[mangle + name]) {
			object[name] = object[mangle + name];
		}
	}

	return object;
}

/**
* The world class manages all physics entities, dynamic simulation,
* and asynchronous queries.
*/
export class b2World {
	__fast__: boolean = true;

	// Construct a world object.
	/**
	* @param gravity the world gravity vector.
	* @param doSleep improve performance by not simulating inactive bodies.
	*/
	constructor(gravity: b2Vec2, doSleep: boolean) {

		this.m_destructionListener = null;
		this.m_debugDraw = null;

		this.m_bodyList = null;
		this.m_contactList = null;
		this.m_jointList = null;
		this.m_controllerList = null;

		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
		this.m_controllerCount = 0;

		b2World.m_warmStarting = true;
		b2World.m_continuousPhysics = true;

		this.m_allowSleep = doSleep;
		this.m_gravity = gravity;

		this.m_inv_dt0 = 0.0;

		this.m_contactManager.m_world = this;

		const bd: b2BodyDef = new b2BodyDef();
		this.m_groundBody = this.CreateBody(bd);
	}

	/**
	* Destruct the world. All physics entities are destroyed and all heap memory is released.
	*/
	//~b2World();

	/**
	* Register a destruction listener.
	*/
	public SetDestructionListener(listener: b2DestructionListener): void {

		this.m_destructionListener = unBoxMethods(listener, b2DestructionListener.prototype);
	}

	/**
	* Register a contact filter to provide specific control over collision.
	* Otherwise the default filter is used (b2_defaultFilter).
	*/
	public SetContactFilter(filter: b2ContactFilter | ASClass | null): void {
		this.m_contactManager.m_contactFilter = unBoxMethods(filter, b2ContactFilter.prototype);
	}

	/**
	* Register a contact event listener
	*/
	public SetContactListener(listener: b2ContactListener | ASClass | null): void {
		this.m_contactManager.m_contactListener = unBoxMethods(listener, b2ContactListener.prototype);
	}

	/**
	* Register a routine for debug drawing. The debug draw functions are called
	* inside the b2World::Step method, so make sure your renderer is ready to
	* consume draw commands when you call Step().
	*/
	public SetDebugDraw(debugDraw: b2DebugDraw): void {
		this.m_debugDraw = debugDraw;
	}

	/**
	 * Use the given object as a broadphase.
	 * The old broadphase will not be cleanly emptied.
	 * @warning It is not recommended you call this except immediately after constructing the world.
	 * @warning This function is locked during callbacks.
	 */
	public SetBroadPhase(broadPhase: IBroadPhase): void {
		const oldBroadPhase: IBroadPhase = this.m_contactManager.m_broadPhase;
		this.m_contactManager.m_broadPhase = broadPhase;
		for (let b: b2Body = this.m_bodyList; b; b = b.m_next) {
			for (let f: b2Fixture = b.m_fixtureList; f; f = f.m_next) {
				f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
			}
		}
	}

	/**
	* Perform validation of internal data structures.
	*/
	public Validate(): void {
		this.m_contactManager.m_broadPhase.Validate();
	}

	/**
	* Get the number of broad-phase proxies.
	*/
	public GetProxyCount(): number /** int */
	{
		return this.m_contactManager.m_broadPhase.GetProxyCount();
	}

	/**
	* Create a rigid body given a definition. No reference to the definition
	* is retained.
	* @warning This function is locked during callbacks.
	*/
	public CreateBody(def: b2BodyDef): b2Body {

		//b2Settings.b2Assert(this.m_lock == false);
		if (this.IsLocked() == true) {
			return null;
		}

		//void* mem = this.m_blockAllocator.Allocate(sizeof(b2Body));
		const b: b2Body = new b2Body(def, this);

		// Add to world doubly linked list.
		b.m_prev = null;
		b.m_next = this.m_bodyList;
		if (this.m_bodyList) {
			this.m_bodyList.m_prev = b;
		}
		this.m_bodyList = b;
		++this.m_bodyCount;

		return b;

	}

	/**
	* Destroy a rigid body given a definition. No reference to the definition
	* is retained. This function is locked during callbacks.
	* @warning This automatically deletes all associated shapes and joints.
	* @warning This function is locked during callbacks.
	*/
	public DestroyBody(b: b2Body): void {

		//b2Settings.b2Assert(this.m_bodyCount > 0);
		//b2Settings.b2Assert(this.m_lock == false);
		if (this.IsLocked() == true) {
			return;
		}

		// Delete the attached joints.
		let jn: b2JointEdge = b.m_jointList;
		while (jn) {
			const jn0: b2JointEdge = jn;
			jn = jn.next;

			if (this.m_destructionListener) {
				this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
			}

			this.DestroyJoint(jn0.joint);
		}

		// Detach controllers attached to this body
		let coe: b2ControllerEdge = b.m_controllerList;
		while (coe) {
			const coe0: b2ControllerEdge = coe;
			coe = coe.nextController;
			coe0.controller.RemoveBody(b);
		}

		// Delete the attached contacts.
		let ce: b2ContactEdge = b.m_contactList;
		while (ce) {
			const ce0: b2ContactEdge = ce;
			ce = ce.next;
			this.m_contactManager.Destroy(ce0.contact);
		}
		b.m_contactList = null;

		// Delete the attached fixtures. This destroys broad-phase
		// proxies.
		let f: b2Fixture = b.m_fixtureList;
		while (f) {
			const f0: b2Fixture = f;
			f = f.m_next;

			if (this.m_destructionListener) {
				this.m_destructionListener.SayGoodbyeFixture(f0);
			}

			f0.DestroyProxy(this.m_contactManager.m_broadPhase);
			f0.Destroy();
			//f0->~b2Fixture();
			//this.m_blockAllocator.Free(f0, sizeof(b2Fixture));

		}
		b.m_fixtureList = null;
		b.m_fixtureCount = 0;

		// Remove world body list.
		if (b.m_prev) {
			b.m_prev.m_next = b.m_next;
		}

		if (b.m_next) {
			b.m_next.m_prev = b.m_prev;
		}

		if (b == this.m_bodyList) {
			this.m_bodyList = b.m_next;
		}

		--this.m_bodyCount;
		//b->~b2Body();
		//this.m_blockAllocator.Free(b, sizeof(b2Body));

	}

	/**
	* Create a joint to constrain bodies together. No reference to the definition
	* is retained. This may cause the connected bodies to cease colliding.
	* @warning This function is locked during callbacks.
	*/
	public CreateJoint(def: b2JointDef): b2Joint {

		//b2Settings.b2Assert(this.m_lock == false);

		const j: b2Joint = b2Joint.Create(def, null);

		// Connect to the world list.
		j.m_prev = null;
		j.m_next = this.m_jointList;
		if (this.m_jointList) {
			this.m_jointList.m_prev = j;
		}
		this.m_jointList = j;
		++this.m_jointCount;

		// Connect to the bodies' doubly linked lists.
		j.m_edgeA.joint = j;
		j.m_edgeA.other = j.m_bodyB;
		j.m_edgeA.prev = null;
		j.m_edgeA.next = j.m_bodyA.m_jointList;
		if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = j.m_edgeA;
		j.m_bodyA.m_jointList = j.m_edgeA;

		j.m_edgeB.joint = j;
		j.m_edgeB.other = j.m_bodyA;
		j.m_edgeB.prev = null;
		j.m_edgeB.next = j.m_bodyB.m_jointList;
		if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = j.m_edgeB;
		j.m_bodyB.m_jointList = j.m_edgeB;

		const bodyA: b2Body = def.bodyA;
		const bodyB: b2Body = def.bodyB;

		// If the joint prevents collisions, then flag any contacts for filtering.
		if (def.collideConnected == false) {
			let edge: b2ContactEdge = bodyB.GetContactList();
			while (edge) {
				if (edge.other == bodyA) {
					// Flag the contact for filtering at the next time step (where either
					// body is awake).
					edge.contact.FlagForFiltering();
				}

				edge = edge.next;
			}
		}

		// Note: creating a joint doesn't wake the bodies.

		return j;

	}

	/**
	* Destroy a joint. This may cause the connected bodies to begin colliding.
	* @warning This function is locked during callbacks.
	*/
	public DestroyJoint(j: b2Joint): void {

		//b2Settings.b2Assert(this.m_lock == false);

		const collideConnected: boolean = j.m_collideConnected;

		// Remove from the doubly linked list.
		if (j.m_prev) {
			j.m_prev.m_next = j.m_next;
		}

		if (j.m_next) {
			j.m_next.m_prev = j.m_prev;
		}

		if (j == this.m_jointList) {
			this.m_jointList = j.m_next;
		}

		// Disconnect from island graph.
		const bodyA: b2Body = j.m_bodyA;
		const bodyB: b2Body = j.m_bodyB;

		// Wake up connected bodies.
		bodyA.SetAwake(true);
		bodyB.SetAwake(true);

		// Remove from body 1.
		if (j.m_edgeA.prev) {
			j.m_edgeA.prev.next = j.m_edgeA.next;
		}

		if (j.m_edgeA.next) {
			j.m_edgeA.next.prev = j.m_edgeA.prev;
		}

		if (j.m_edgeA == bodyA.m_jointList) {
			bodyA.m_jointList = j.m_edgeA.next;
		}

		j.m_edgeA.prev = null;
		j.m_edgeA.next = null;

		// Remove from body 2
		if (j.m_edgeB.prev) {
			j.m_edgeB.prev.next = j.m_edgeB.next;
		}

		if (j.m_edgeB.next) {
			j.m_edgeB.next.prev = j.m_edgeB.prev;
		}

		if (j.m_edgeB == bodyB.m_jointList) {
			bodyB.m_jointList = j.m_edgeB.next;
		}

		j.m_edgeB.prev = null;
		j.m_edgeB.next = null;

		b2Joint.Destroy(j, null);

		//b2Settings.b2Assert(this.m_jointCount > 0);
		--this.m_jointCount;

		// If the joint prevents collisions, then flag any contacts for filtering.
		if (collideConnected == false) {
			let edge: b2ContactEdge = bodyB.GetContactList();
			while (edge) {
				if (edge.other == bodyA) {
					// Flag the contact for filtering at the next time step (where either
					// body is awake).
					edge.contact.FlagForFiltering();
				}

				edge = edge.next;
			}
		}

	}

	/**
	 * Add a controller to the world list
	 */
	public AddController(c: b2Controller): b2Controller {
		c.m_next = this.m_controllerList;
		c.m_prev = null;
		this.m_controllerList = c;

		c.m_world = this;

		this.m_controllerCount++;

		return c;
	}

	public RemoveController(c: b2Controller): void {
		//TODO: Remove bodies from controller
		if (c.m_prev)
			c.m_prev.m_next = c.m_next;
		if (c.m_next)
			c.m_next.m_prev = c.m_prev;
		if (this.m_controllerList == c)
			this.m_controllerList = c.m_next;

		this.m_controllerCount--;
	}

	public CreateController(controller: b2Controller): b2Controller {
		if (controller.m_world != this)
			throw new Error('Controller can only be a member of one world');

		controller.m_next = this.m_controllerList;
		controller.m_prev = null;
		if (this.m_controllerList)
			this.m_controllerList.m_prev = controller;
		this.m_controllerList = controller;
		++this.m_controllerCount;

		controller.m_world = this;

		return controller;
	}

	public DestroyController(controller: b2Controller): void {
		//b2Settings.b2Assert(this.m_controllerCount > 0);
		controller.Clear();
		if (controller.m_next)
			controller.m_next.m_prev = controller.m_prev;
		if (controller.m_prev)
			controller.m_prev.m_next = controller.m_next;
		if (controller == this.m_controllerList)
			this.m_controllerList = controller.m_next;
		--this.m_controllerCount;
	}

	/**
	* Enable/disable warm starting. For testing.
	*/
	public SetWarmStarting(flag: boolean): void { b2World.m_warmStarting = flag; }

	/**
	* Enable/disable continuous physics. For testing.
	*/
	public SetContinuousPhysics(flag: boolean): void { b2World.m_continuousPhysics = flag; }

	/**
	* Get the number of bodies.
	*/
	public GetBodyCount(): number /** int */
	{
		return this.m_bodyCount;
	}

	/**
	* Get the number of joints.
	*/
	public GetJointCount(): number /** int */
	{
		return this.m_jointCount;
	}

	/**
	* Get the number of contacts (each may have 0 or more contact points).
	*/
	public GetContactCount(): number /** int */
	{
		return this.m_contactCount;
	}

	/**
	* Change the global gravity vector.
	*/
	public SetGravity(gravity: b2Vec2): void {
		this.m_gravity = gravity;
	}

	/**
	* Get the global gravity vector.
	*/
	public GetGravity(): b2Vec2 {
		return this.m_gravity;
	}

	/**
	* The world provides a single static ground body with no collision shapes.
	* You can use this to simplify the creation of joints and static shapes.
	*/
	public GetGroundBody(): b2Body {
		return this.m_groundBody;
	}

	private static s_timestep2: b2TimeStep = new b2TimeStep();
	/**
	* Take a time step. This performs collision detection, integration,
	* and constraint solution.
	* @param timeStep the amount of time to simulate, this should not vary.
	* @param velocityIterations for the velocity constraint solver.
	* @param positionIterations for the position constraint solver.
	*/
	public Step(dt: number, velocityIterations: number /** int */, positionIterations: number /** int */): void {
		if (this.m_flags & b2World.e_newFixture) {
			this.m_contactManager.FindNewContacts();
			this.m_flags &= ~b2World.e_newFixture;
		}

		this.m_flags |= b2World.e_locked;

		const step: b2TimeStep = b2World.s_timestep2;
		step.dt = dt;
		step.velocityIterations = velocityIterations;
		step.positionIterations = positionIterations;
		if (dt > 0.0) {
			step.inv_dt = 1.0 / dt;
		} else {
			step.inv_dt = 0.0;
		}

		step.dtRatio = this.m_inv_dt0 * dt;

		step.warmStarting = b2World.m_warmStarting;

		// Update contacts.
		this.m_contactManager.Collide();

		// Integrate velocities, solve velocity constraints, and integrate positions.
		if (step.dt > 0.0) {
			this.Solve(step);
		}

		// Handle TOI events.
		if (b2World.m_continuousPhysics && step.dt > 0.0) {
			this.SolveTOI(step);
		}

		if (step.dt > 0.0) {
			this.m_inv_dt0 = step.inv_dt;
		}
		this.m_flags &= ~b2World.e_locked;
	}

	/**
	 * Call this after you are done with time steps to clear the forces. You normally
	 * call this after each call to Step, unless you are performing sub-steps.
	 */
	public ClearForces(): void {
		for (let body: b2Body = this.m_bodyList; body; body = body.m_next) {
			body.m_force.SetZero();
			body.m_torque = 0.0;
		}
	}

	private static s_xf: b2Transform = new b2Transform();
	/**
	 * Call this to draw shapes and other debug draw data.
	 */
	public DrawDebugData(): void {

		if (this.m_debugDraw == null) {
			return;
		}

		this.m_debugDraw.m_sprite.graphics.clear();

		const flags: number /** uint */ = this.m_debugDraw.GetFlags();

		let i: number /** int */;
		let b: b2Body;
		let f: b2Fixture;
		let s: b2Shape;
		let j: b2Joint;
		let bp: IBroadPhase;
		const invQ: b2Vec2 = new b2Vec2;
		const x1: b2Vec2 = new b2Vec2;
		const x2: b2Vec2 = new b2Vec2;
		let xf: b2Transform;
		const b1: b2AABB = new b2AABB();
		const b2: b2AABB = new b2AABB();
		let vs: Array<b2Vec2> = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];

		// Store color here and reuse, to reduce allocations
		const color: b2Color = new b2Color(0, 0, 0);

		if (flags & b2DebugDraw.e_shapeBit) {
			for (b = this.m_bodyList; b; b = b.m_next) {
				xf = b.m_xf;
				for (f = b.GetFixtureList(); f; f = f.m_next) {
					s = f.GetShape();
					if (b.IsActive() == false) {
						color.Set(0.5, 0.5, 0.3);
						this.DrawShape(s, xf, color);
					} else if (b.GetType() == b2Body.b2_staticBody) {
						color.Set(0.5, 0.9, 0.5);
						this.DrawShape(s, xf, color);
					} else if (b.GetType() == b2Body.b2_kinematicBody) {
						color.Set(0.5, 0.5, 0.9);
						this.DrawShape(s, xf, color);
					} else if (b.IsAwake() == false) {
						color.Set(0.6, 0.6, 0.6);
						this.DrawShape(s, xf, color);
					} else {
						color.Set(0.9, 0.7, 0.7);
						this.DrawShape(s, xf, color);
					}
				}
			}
		}

		if (flags & b2DebugDraw.e_jointBit) {
			for (j = this.m_jointList; j; j = j.m_next) {
				this.DrawJoint(j);
			}
		}

		if (flags & b2DebugDraw.e_controllerBit) {
			for (let c: b2Controller = this.m_controllerList; c; c = c.m_next) {
				c.Draw(this.m_debugDraw);
			}
		}

		if (flags & b2DebugDraw.e_pairBit) {
			color.Set(0.3, 0.9, 0.9);
			for (let contact: b2Contact = this.m_contactManager.m_contactList; contact; contact = contact.GetNext()) {
				const fixtureA: b2Fixture = contact.GetFixtureA();
				const fixtureB: b2Fixture = contact.GetFixtureB();

				const cA: b2Vec2 = fixtureA.GetAABB().GetCenter();
				const cB: b2Vec2 = fixtureB.GetAABB().GetCenter();

				this.m_debugDraw.DrawSegment(cA, cB, color);
			}
		}

		if (flags & b2DebugDraw.e_aabbBit) {
			bp = this.m_contactManager.m_broadPhase;

			vs = [new b2Vec2(),new b2Vec2(),new b2Vec2(),new b2Vec2()];

			for (b = this.m_bodyList; b; b = b.GetNext()) {
				if (b.IsActive() == false) {
					continue;
				}
				for (f = b.GetFixtureList(); f; f = f.GetNext()) {
					const aabb: b2AABB = bp.GetFatAABB(f.m_proxy);
					vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
					vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
					vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
					vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

					this.m_debugDraw.DrawPolygon(vs, 4, color);
				}
			}
		}

		if (flags & b2DebugDraw.e_centerOfMassBit) {
			for (b = this.m_bodyList; b; b = b.m_next) {
				xf = b2World.s_xf;
				xf.R = b.m_xf.R;
				xf.position = b.GetWorldCenter();
				this.m_debugDraw.DrawTransform(xf);
			}
		}
	}

	/**
	 * Query the world for all fixtures that potentially overlap the
	 * provided AABB.
	 * @param callback a user implemented callback class. It should match signature
	 * <code>function Callback(fixture:b2Fixture):boolean</code>
	 * Return true to continue to the next fixture.
	 * @param aabb the query box.
	 */
	public QueryAABB(callback: Function | ASMethodClosure, aabb: b2AABB): void {
		const broadPhase: IBroadPhase = this.m_contactManager.m_broadPhase;

		function WorldQueryWrapper(proxy: any): boolean {
			if (typeof callback === 'function') {
				return callback(broadPhase.GetUserData(proxy));
			} else {
				return callback.axApply(null, [broadPhase.GetUserData(proxy)]);
			}
		}

		broadPhase.Query(WorldQueryWrapper, aabb);
	}

	/**
	 * Query the world for all fixtures that precisely overlap the
	 * provided transformed shape.
	 * @param callback a user implemented callback class. It should match signature
	 * <code>function Callback(fixture:b2Fixture):boolean</code>
	 * Return true to continue to the next fixture.
	 * @asonly
	 */
	public QueryShape(callback: Function, shape: b2Shape, transform: b2Transform = null): void {
		if (transform == null) {
			transform = new b2Transform();
			transform.SetIdentity();
		}
		const broadPhase: IBroadPhase = this.m_contactManager.m_broadPhase;
		function WorldQueryWrapper(proxy: any): boolean {
			const fixture: b2Fixture = broadPhase.GetUserData(proxy) as b2Fixture;
			if (b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform()))
				return callback(fixture);
			return true;
		}
		const aabb: b2AABB = new b2AABB();
		shape.ComputeAABB(aabb, transform);
		broadPhase.Query(WorldQueryWrapper, aabb);
	}

	/**
	 * Query the world for all fixtures that contain a point.
	 * @param callback a user implemented callback class. It should match signature
	 * <code>function Callback(fixture:b2Fixture):boolean</code>
	 * Return true to continue to the next fixture.
	 * @asonly
	 */
	public QueryPoint(callback: Function, p: b2Vec2): void {
		const broadPhase: IBroadPhase = this.m_contactManager.m_broadPhase;
		function WorldQueryWrapper(proxy: any): boolean {
			const fixture: b2Fixture = broadPhase.GetUserData(proxy) as b2Fixture;
			if (fixture.TestPoint(p))
				return callback(fixture);
			return true;
		}
		// Make a small box.
		const aabb: b2AABB = new b2AABB();
		aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
		aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);
		broadPhase.Query(WorldQueryWrapper, aabb);
	}

	/**
	 * Ray-cast the world for all fixtures in the path of the ray. Your callback
	 * Controls whether you get the closest point, any point, or n-points
	 * The ray-cast ignores shapes that contain the starting point
	 * @param callback A callback function which must be of signature:
	 * <code>function Callback(fixture:b2Fixture,    // The fixture hit by the ray
	 * point:b2Vec2,         // The point of initial intersection
	 * normal:b2Vec2,        // The normal vector at the point of intersection
	 * fraction:number       // The fractional length along the ray of the intersection
	 * ):number
	 * </code>
	 * Callback should return the new length of the ray as a fraction of the original length.
	 * By returning 0, you immediately terminate.
	 * By returning 1, you continue wiht the original ray.
	 * By returning the current fraction, you proceed to find the closest point.
	 * @param point1 the ray starting point
	 * @param point2 the ray ending point
	 */
	public RayCast(callback: Function, point1: b2Vec2, point2: b2Vec2): void {
		const broadPhase: IBroadPhase = this.m_contactManager.m_broadPhase;
		const output: b2RayCastOutput = new b2RayCastOutput;
		function RayCastWrapper(input: b2RayCastInput, proxy: any): number {
			const userData: any = broadPhase.GetUserData(proxy);
			const fixture: b2Fixture = userData as b2Fixture;
			const hit: boolean = fixture.RayCast(output, input);
			if (hit) {
				const fraction: number = output.fraction;
				const point: b2Vec2 = new b2Vec2(
					(1.0 - fraction) * point1.x + fraction * point2.x,
					(1.0 - fraction) * point1.y + fraction * point2.y);
				return callback(fixture, point, output.normal, fraction);
			}
			return input.maxFraction;
		}
		const input: b2RayCastInput = new b2RayCastInput(point1, point2);
		broadPhase.RayCast(RayCastWrapper, input);
	}

	public RayCastOne(point1: b2Vec2, point2: b2Vec2): b2Fixture {
		let result: b2Fixture;
		function RayCastOneWrapper(fixture: b2Fixture, point: b2Vec2, normal: b2Vec2, fraction: number): number {
			result = fixture;
			return fraction;
		}
		this.RayCast(RayCastOneWrapper, point1, point2);
		return result;
	}

	public RayCastAll(point1: b2Vec2, point2: b2Vec2): Array<b2Fixture> {
		const result: Array<b2Fixture> = new Array<b2Fixture>();
		function RayCastAllWrapper(fixture: b2Fixture, point: b2Vec2, normal: b2Vec2, fraction: number): number {
			result[result.length] = fixture;
			return 1;
		}
		this.RayCast(RayCastAllWrapper, point1, point2);
		return result;
	}

	/**
	* Get the world body list. With the returned body, use b2Body::GetNext to get
	* the next body in the world list. A NULL body indicates the end of the list.
	* @return the head of the world body list.
	*/
	public GetBodyList(): b2Body {
		return this.m_bodyList;
	}

	/**
	* Get the world joint list. With the returned joint, use b2Joint::GetNext to get
	* the next joint in the world list. A NULL joint indicates the end of the list.
	* @return the head of the world joint list.
	*/
	public GetJointList(): b2Joint {
		return this.m_jointList;
	}

	/**
	 * Get the world contact list. With the returned contact, use b2Contact::GetNext to get
	 * the next contact in the world list. A NULL contact indicates the end of the list.
	 * @return the head of the world contact list.
	 * @warning contacts are
	 */
	public GetContactList(): b2Contact {
		return this.m_contactList;
	}

	/**
	 * Is the world locked (in the middle of a time step).
	 */
	public IsLocked(): boolean {
		return (this.m_flags & b2World.e_locked) > 0;
	}

	//--------------- Internals Below -------------------
	// Internal yet public to make life easier.

	// Find islands, integrate and solve constraints, solve position constraints
	private s_stack: Array<b2Body> = new Array<b2Body>();
	public Solve(step: b2TimeStep): void {
		let b: b2Body;

		// Step all controllers
		for (let controller: b2Controller = this.m_controllerList;controller;controller = controller.m_next) {
			controller.Step(step);
		}

		// Size the island for the worst case.
		const island: b2Island = this.m_island;
		island.Initialize(this.m_bodyCount, this.m_contactCount, this.m_jointCount, null, this.m_contactManager.m_contactListener, this.m_contactSolver);

		// Clear all the island flags.
		for (b = this.m_bodyList; b; b = b.m_next) {
			b.m_flags &= ~b2Body.e_islandFlag;
		}
		for (let c: b2Contact = this.m_contactList; c; c = c.m_next) {
			c.m_flags &= ~b2Contact.e_islandFlag;
		}
		for (let j: b2Joint = this.m_jointList; j; j = j.m_next) {
			j.m_islandFlag = false;
		}

		// Build and simulate all awake islands.
		const stackSize: number /** int */ = this.m_bodyCount;
		//b2Body** stack = (b2Body**)this.m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
		const stack: Array<b2Body> = this.s_stack;
		for (let seed: b2Body = this.m_bodyList; seed; seed = seed.m_next) {
			if (seed.m_flags & b2Body.e_islandFlag) {
				continue;
			}

			if (seed.IsAwake() == false || seed.IsActive() == false) {
				continue;
			}

			// The seed can be dynamic or kinematic.
			if (seed.GetType() == b2Body.b2_staticBody) {
				continue;
			}

			// Reset island and stack.
			island.Clear();
			let stackCount: number /** int */ = 0;
			stack[stackCount++] = seed;
			seed.m_flags |= b2Body.e_islandFlag;

			// Perform a depth first search (DFS) on the constraint graph.
			while (stackCount > 0) {
				// Grab the next body off the stack and add it to the island.
				b = stack[--stackCount];
				//b2Assert(b.IsActive() == true);
				island.AddBody(b);

				// Make sure the body is awake.
				if (b.IsAwake() == false) {
					b.SetAwake(true);
				}

				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b.GetType() == b2Body.b2_staticBody) {
					continue;
				}

				var other: b2Body;
				// Search all contacts connected to this body.
				for (let ce: b2ContactEdge = b.m_contactList; ce; ce = ce.next) {
					// Has this contact already been added to an island?
					if (ce.contact.m_flags & b2Contact.e_islandFlag) {
						continue;
					}

					// Is this contact solid and touching?
					if (ce.contact.IsSensor() == true ||
						ce.contact.IsEnabled() == false ||
						ce.contact.IsTouching() == false) {
						continue;
					}

					island.AddContact(ce.contact);
					ce.contact.m_flags |= b2Contact.e_islandFlag;

					//var other:b2Body = ce.other;
					other = ce.other;

					// Was the other body already added to this island?
					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}

					//b2Settings.b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}

				// Search all joints connect to this body.
				for (let jn: b2JointEdge = b.m_jointList; jn; jn = jn.next) {
					if (jn.joint.m_islandFlag == true) {
						continue;
					}

					other = jn.other;

					// Don't simulate joints connected to inactive bodies.
					if (other.IsActive() == false) {
						continue;
					}

					island.AddJoint(jn.joint);
					jn.joint.m_islandFlag = true;

					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}

					//b2Settings.b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
			}
			island.Solve(step, this.m_gravity, this.m_allowSleep);

			// Post solve cleanup.
			for (var i: number /** int */ = 0; i < island.m_bodyCount; ++i) {
				// Allow static bodies to participate in other islands.
				b = island.m_bodies[i];
				if (b.GetType() == b2Body.b2_staticBody) {
					b.m_flags &= ~b2Body.e_islandFlag;
				}
			}
		}

		//this.m_stackAllocator.Free(stack);
		for (i = 0; i < stack.length;++i) {
			if (!stack[i]) break;
			stack[i] = null;
		}

		// Synchronize fixutres, check for out of range bodies.
		for (b = this.m_bodyList; b; b = b.m_next) {
			if (b.IsAwake() == false || b.IsActive() == false) {
				continue;
			}

			if (b.GetType() == b2Body.b2_staticBody) {
				continue;
			}

			// Update fixtures (for broad-phase).
			b.SynchronizeFixtures();
		}

		// Look for new contacts.
		this.m_contactManager.FindNewContacts();

	}

	private static s_backupA: b2Sweep = new b2Sweep();
	private static s_backupB: b2Sweep = new b2Sweep();
	private static s_timestep: b2TimeStep = new b2TimeStep();
	private static s_queue: Array<b2Body> = new Array<b2Body>();
	// Find TOI contacts and solve them.
	public SolveTOI(step: b2TimeStep): void {

		let b: b2Body;
		let fA: b2Fixture;
		let fB: b2Fixture;
		let bA: b2Body;
		let bB: b2Body;
		let cEdge: b2ContactEdge;
		let j: b2Joint;

		// Reserve an island and a queue for TOI island solution.
		const island: b2Island = this.m_island;
		island.Initialize(this.m_bodyCount, b2Settings.b2_maxTOIContactsPerIsland, b2Settings.b2_maxTOIJointsPerIsland, null, this.m_contactManager.m_contactListener, this.m_contactSolver);

		//Simple one pass queue
		//Relies on the fact that we're only making one pass
		//through and each body can only be pushed/popped one.
		//To push:
		//  queue[queueStart+queueSize++] = newElement;
		//To pop:
		//  poppedElement = queue[queueStart++];
		//  --queueSize;

		const queue: Array<b2Body> = b2World.s_queue;

		for (b = this.m_bodyList; b; b = b.m_next) {
			b.m_flags &= ~b2Body.e_islandFlag;
			b.m_sweep.t0 = 0.0;
		}

		let c: b2Contact;
		for (c = this.m_contactList; c; c = c.m_next) {
			// Invalidate TOI
			c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
		}

		for (j = this.m_jointList; j; j = j.m_next) {
			j.m_islandFlag = false;
		}

		// Find TOI events and solve them.
		for (;;) {
			// Find the first TOI.
			let minContact: b2Contact = null;
			let minTOI: number = 1.0;

			for (c = this.m_contactList; c; c = c.m_next) {
				// Can this contact generate a solid TOI contact?
 				if (c.IsSensor() == true ||
					c.IsEnabled() == false ||
					c.IsContinuous() == false) {
					continue;
				}

				// TODO_ERIN keep a counter on the contact, only respond to M TOIs per contact.

				let toi: number = 1.0;
				if (c.m_flags & b2Contact.e_toiFlag) {
					// This contact has a valid cached TOI.
					toi = c.m_toi;
				} else {
					// Compute the TOI for this contact.
					fA = c.m_fixtureA;
					fB = c.m_fixtureB;
					bA = fA.m_body;
					bB = fB.m_body;

					if ((bA.GetType() != b2Body.b2_dynamicBody || bA.IsAwake() == false) &&
						(bB.GetType() != b2Body.b2_dynamicBody || bB.IsAwake() == false)) {
						continue;
					}

					// Put the sweeps onto the same time interval.
					let t0: number = bA.m_sweep.t0;

					if (bA.m_sweep.t0 < bB.m_sweep.t0) {
						t0 = bB.m_sweep.t0;
						bA.m_sweep.Advance(t0);
					} else if (bB.m_sweep.t0 < bA.m_sweep.t0) {
						t0 = bA.m_sweep.t0;
						bB.m_sweep.Advance(t0);
					}

					//b2Settings.b2Assert(t0 < 1.0f);

					// Compute the time of impact.
					toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep);
					b2Settings.b2Assert(0.0 <= toi && toi <= 1.0);

					// If the TOI is in range ...
					if (toi > 0.0 && toi < 1.0) {
						// Interpolate on the actual range.
						//toi = Math.min((1.0 - toi) * t0 + toi, 1.0);
						toi = (1.0 - toi) * t0 + toi;
						if (toi > 1) toi = 1;
					}

					c.m_toi = toi;
					c.m_flags |= b2Contact.e_toiFlag;
				}

				if (Number.MIN_VALUE < toi && toi < minTOI) {
					// This is the minimum TOI found so far.
					minContact = c;
					minTOI = toi;
				}
			}

			if (minContact == null || 1.0 - 100.0 * Number.MIN_VALUE < minTOI) {
				// No more TOI events. Done!
				break;
			}

			// Advance the bodies to the TOI.
			fA = minContact.m_fixtureA;
			fB = minContact.m_fixtureB;
			bA = fA.m_body;
			bB = fB.m_body;
			b2World.s_backupA.Set(bA.m_sweep);
			b2World.s_backupB.Set(bB.m_sweep);
			bA.Advance(minTOI);
			bB.Advance(minTOI);

			// The TOI contact likely has some new contact points.
			minContact.Update(this.m_contactManager.m_contactListener);
			minContact.m_flags &= ~b2Contact.e_toiFlag;

			// Is the contact solid?
			if (minContact.IsSensor() == true ||
				minContact.IsEnabled() == false) {
				// Restore the sweeps
				bA.m_sweep.Set(b2World.s_backupA);
				bB.m_sweep.Set(b2World.s_backupB);
				bA.SynchronizeTransform();
				bB.SynchronizeTransform();
				continue;
			}

			// Did numerical issues prevent;,ontact pointjrom being generated
			if (minContact.IsTouching() == false) {
				// Give up on this TOI
				continue;
			}

			// Build the TOI island. We need a dynamic seed.
			let seed: b2Body = bA;
			if (seed.GetType() != b2Body.b2_dynamicBody) {
				seed = bB;
			}

			// Reset island and queue.
			island.Clear();
			let queueStart: number /** int */ = 0;	//start index for queue
			let queueSize: number /** int */ = 0;	//elements in queue
			queue[queueStart + queueSize++] = seed;
			seed.m_flags |= b2Body.e_islandFlag;

			// Perform a breadth first search (BFS) on the contact graph.
			while (queueSize > 0) {
				// Grab the next body off the stack and add it to the island.
				b = queue[queueStart++];
				--queueSize;

				island.AddBody(b);

				// Make sure the body is awake.
				if (b.IsAwake() == false) {
					b.SetAwake(true);
				}

				// To keep islands as small as possible, we don't
				// propagate islands across static or kinematic bodies.
				if (b.GetType() != b2Body.b2_dynamicBody) {
					continue;
				}

				// Search all contacts connected to this body.
				for (cEdge = b.m_contactList; cEdge; cEdge = cEdge.next) {
					// Does the TOI island still have space for contacts?
					if (island.m_contactCount == island.m_contactCapacity) {
						break;
					}

					// Has this contact already been added to an island?
					if (cEdge.contact.m_flags & b2Contact.e_islandFlag) {
						continue;
					}

					// Skip sperate, sensor, or disabled contacts.
					if (cEdge.contact.IsSensor() == true ||
						cEdge.contact.IsEnabled() == false ||
						cEdge.contact.IsTouching() == false) {
						continue;
					}

					island.AddContact(cEdge.contact);
					cEdge.contact.m_flags |= b2Contact.e_islandFlag;

					// Update other body.
					var other: b2Body = cEdge.other;

					// Was the other body already added to this island?
					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}

					// Synchronize the connected body.
					if (other.GetType() != b2Body.b2_staticBody) {
						other.Advance(minTOI);
						other.SetAwake(true);
					}

					//b2Settings.b2Assert(queueStart + queueSize < queueCapacity);
					queue[queueStart + queueSize] = other;
					++queueSize;
					other.m_flags |= b2Body.e_islandFlag;
				}

				for (let jEdge: b2JointEdge = b.m_jointList; jEdge; jEdge = jEdge.next) {
					if (island.m_jointCount == island.m_jointCapacity)
						continue;

					if (jEdge.joint.m_islandFlag == true)
						continue;

					other = jEdge.other;
					if (other.IsActive() == false) {
						continue;
					}

					island.AddJoint(jEdge.joint);
					jEdge.joint.m_islandFlag = true;

					if (other.m_flags & b2Body.e_islandFlag)
						continue;

					// Synchronize the connected body.
					if (other.GetType() != b2Body.b2_staticBody) {
						other.Advance(minTOI);
						other.SetAwake(true);
					}

					//b2Settings.b2Assert(queueStart + queueSize < queueCapacity);
					queue[queueStart + queueSize] = other;
					++queueSize;
					other.m_flags |= b2Body.e_islandFlag;
				}
			}

			const subStep: b2TimeStep = b2World.s_timestep;
			subStep.warmStarting = false;
			subStep.dt = (1.0 - minTOI) * step.dt;
			subStep.inv_dt = 1.0 / subStep.dt;
			subStep.dtRatio = 0.0;
			subStep.velocityIterations = step.velocityIterations;
			subStep.positionIterations = step.positionIterations;

			island.SolveTOI(subStep);

			var i: number /** int */;
			// Post solve cleanup.
			for (i = 0; i < island.m_bodyCount; ++i) {
				// Allow bodies to participate in future TOI islands.
				b = island.m_bodies[i];
				b.m_flags &= ~b2Body.e_islandFlag;

				if (b.IsAwake() == false) {
					continue;
				}

				if (b.GetType() != b2Body.b2_dynamicBody) {
					continue;
				}

				// Update fixtures (for broad-phase).
				b.SynchronizeFixtures();

				// Invalidate all contact TOIs associated with this body. Some of these
				// may not be in the island because they were not touching.
				for (cEdge = b.m_contactList; cEdge; cEdge = cEdge.next) {
					cEdge.contact.m_flags &= ~b2Contact.e_toiFlag;
				}
			}

			for (i = 0; i < island.m_contactCount; ++i) {
				// Allow contacts to participate in future TOI islands.
				c = island.m_contacts[i];
				c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
			}

			for (i = 0; i < island.m_jointCount;++i) {
				// Allow joints to participate in future TOI islands
				j = island.m_joints[i];
				j.m_islandFlag = false;
			}

			// Commit fixture proxy movements to the broad-phase so that new contacts are created.
			// Also, some contacts can be destroyed.
			this.m_contactManager.FindNewContacts();
		}

		//this.m_stackAllocator.Free(queue);
	}

	private static s_jointColor: b2Color = new b2Color(0.5, 0.8, 0.8);
	//
	public DrawJoint(joint: b2Joint): void {

		const b1: b2Body = joint.GetBodyA();
		const b2: b2Body = joint.GetBodyB();
		const xf1: b2Transform = b1.m_xf;
		const xf2: b2Transform = b2.m_xf;
		const x1: b2Vec2 = xf1.position;
		const x2: b2Vec2 = xf2.position;
		const p1: b2Vec2 = joint.GetAnchorA();
		const p2: b2Vec2 = joint.GetAnchorB();

		//b2Color color(0.5f, 0.8f, 0.8f);
		const color: b2Color = b2World.s_jointColor;

		switch (joint.m_type) {
			case b2Joint.e_distanceJoint:
				this.m_debugDraw.DrawSegment(p1, p2, color);
				break;

			case b2Joint.e_pulleyJoint:
				{
					const pulley: b2PulleyJoint = (joint as b2PulleyJoint);
					const s1: b2Vec2 = pulley.GetGroundAnchorA();
					const s2: b2Vec2 = pulley.GetGroundAnchorB();
					this.m_debugDraw.DrawSegment(s1, p1, color);
					this.m_debugDraw.DrawSegment(s2, p2, color);
					this.m_debugDraw.DrawSegment(s1, s2, color);
				}
				break;

			case b2Joint.e_mouseJoint:
				this.m_debugDraw.DrawSegment(p1, p2, color);
				break;

			default:
				if (b1 != this.m_groundBody)
					this.m_debugDraw.DrawSegment(x1, p1, color);
				this.m_debugDraw.DrawSegment(p1, p2, color);
				if (b2 != this.m_groundBody)
					this.m_debugDraw.DrawSegment(x2, p2, color);
		}
	}

	public DrawShape(shape: b2Shape, xf: b2Transform, color: b2Color): void {

		switch (shape.m_type) {
			case b2Shape.e_circleShape:
				{
					const circle: b2CircleShape = <b2CircleShape> shape;

					const center: b2Vec2 = b2Math.MulX(xf, circle.m_p);
					const radius: number = circle.m_radius;
					const axis: b2Vec2 = xf.R.col1;

					this.m_debugDraw.DrawSolidCircle(center, radius, axis, color);
				}
				break;

			case b2Shape.e_polygonShape:
				{
					let i: number /** int */;
					const poly: b2PolygonShape = <b2PolygonShape> shape;
					const vertexCount: number /** int */ = poly.GetVertexCount();
					const localVertices: Array<b2Vec2> = poly.GetVertices();

					const vertices: Array<b2Vec2> = new Array<b2Vec2>(vertexCount);

					for (i = 0; i < vertexCount; ++i) {
						vertices[i] = b2Math.MulX(xf, localVertices[i]);
					}

					this.m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
				}
				break;

			case b2Shape.e_edgeShape:
				{
					const edge: b2EdgeShape = shape as b2EdgeShape;

					this.m_debugDraw.DrawSegment(b2Math.MulX(xf, edge.GetVertex1()), b2Math.MulX(xf, edge.GetVertex2()), color);

				}
				break;
		}
	}

	public m_flags: number /** int */;

	public m_contactManager: b2ContactManager = new b2ContactManager();

	// These two are stored purely for efficiency purposes, they don't maintain
	// any data outside of a call to Step
	private m_contactSolver: b2ContactSolver = new b2ContactSolver();
	private m_island: b2Island = new b2Island();

	public m_bodyList: b2Body;
	private m_jointList: b2Joint;

	public m_contactList: b2Contact;

	private m_bodyCount: number /** int */;
	public m_contactCount: number /** int */;
	private m_jointCount: number /** int */;
	private m_controllerList: b2Controller;
	private m_controllerCount: number /** int */;

	private m_gravity: b2Vec2;
	private m_allowSleep: boolean;

	public m_groundBody: b2Body;

	private m_destructionListener: b2DestructionListener;
	private m_debugDraw: b2DebugDraw;

	// This is used to compute the time step ratio to support a variable time step.
	private m_inv_dt0: number;

	// This is for debugging the solver.
	private static m_warmStarting: boolean;

	// This is for debugging the solver.
	private static m_continuousPhysics: boolean;

	// this.m_flags
	public static readonly e_newFixture: number /** int */ = 0x0001;
	public static readonly e_locked: number /** int */ = 0x0002;

}