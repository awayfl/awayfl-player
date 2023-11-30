import { b2EdgeShape } from '../Collision/Shapes/b2EdgeShape';
import { b2Math, b2Vec2, b2Transform, b2Mat22, b2Sweep } from '../Common/Math';
import { b2Settings } from '../Common/b2Settings';
import { b2BodyDef } from './b2BodyDef';
import { b2Fixture } from './b2Fixture';
import { b2FixtureDef } from './b2FixtureDef';
import { IBroadPhase } from '../Collision/IBroadPhase';
import { b2World } from './b2World';
import { b2Shape } from '../Collision/Shapes/b2Shape';
import { b2ContactEdge } from './Contacts';
import { b2Contact } from './Contacts';
import { b2JointEdge } from './Joints';
import { b2MassData } from '../Collision/Shapes/b2MassData';
import { b2ControllerEdge } from './Controllers/b2ControllerEdge';

/**
* A rigid body.
*/
export class b2Body {
	__fast__ = true;

	private connectEdges(s1: b2EdgeShape, s2: b2EdgeShape, angle1: number): number {
		const angle2: number = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
		const coreOffset: number = Math.tan((angle2 - angle1) * 0.5);
		let core: b2Vec2 = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
		core = b2Math.SubtractVV(core, s2.GetNormalVector());
		core = b2Math.MulFV(b2Settings.b2_toiSlop, core);
		core = b2Math.AddVV(core, s2.GetVertex1());
		const cornerDir: b2Vec2 = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
		cornerDir.Normalize();
		const convex: boolean = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0.0;
		s1.SetNextEdge(s2, core, cornerDir, convex);
		s2.SetPrevEdge(s1, core, cornerDir, convex);
		return angle2;
	}

	/**
	 * Creates a fixture and attach it to this body. Use this function if you need
	 * to set some fixture parameters, like friction. Otherwise you can create the
	 * fixture directly from a shape.
	 * If the density is non-zero, this function automatically updates the mass of the body.
	 * Contacts are not created until the next time step.
	 * @param fixtureDef the fixture definition.
	 * @warning This function is locked during callbacks.
	 */
	public CreateFixture(def: b2FixtureDef): b2Fixture {
		//b2Settings.b2Assert(this.m_world.IsLocked() == false);
		if (this.m_world.IsLocked() == true) {
			return null;
		}

		// TODO: Decide on a better place to initialize edgeShapes. (b2Shape::Create() can't
		//       return more than one shape to add to parent body... maybe it should add
		//       shapes directly to the body instead of returning them?)
		/*
		if (def.type == b2Shape.e_edgeShape) {
			var edgeDef: b2EdgeChainDef = def as b2EdgeChainDef;
			var v1: b2Vec2;
			var v2: b2Vec2;
			var i: int;

			if (edgeDef.isALoop) {
				v1 = edgeDef.vertices[edgeDef.vertexCount-1];
				i = 0;
			} else {
				v1 = edgeDef.vertices[0];
				i = 1;
			}

			var s0: b2EdgeShape = null;
			var s1: b2EdgeShape = null;
			var s2: b2EdgeShape = null;
			var angle: number = 0.0;
			for (; i < edgeDef.vertexCount; i++) {
				v2 = edgeDef.vertices[i];

				//void* mem = this.m_world->m_blockAllocator.Allocate(sizeof(b2EdgeShape));
				s2 = new b2EdgeShape(v1, v2, def);
				s2.this.m_next = m_shapeList;
				m_shapeList = s2;
				++m_shapeCount;
				s2.m_body = this;
				s2.CreateProxy(this.m_world.m_broadPhase, this.m_xf);
				s2.UpdateSweepRadius(this.m_sweep.localCenter);

				if (s1 == null) {
					s0 = s2;
					angle = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
				} else {
					angle = connectEdges(s1, s2, angle);
				}
				s1 = s2;
				v1 = v2;
			}
			if (edgeDef.isALoop) connectEdges(s1, s0, angle);
			return s0;
		}*/

		const fixture: b2Fixture = new b2Fixture();
		fixture.Create(this, this.m_xf, def);

		if (this.m_flags & b2Body.e_activeFlag) {
			const broadPhase: IBroadPhase = this.m_world.m_contactManager.m_broadPhase;
			fixture.CreateProxy(broadPhase, this.m_xf);
		}

		fixture.m_next = this.m_fixtureList;
		this.m_fixtureList = fixture;
		++this.m_fixtureCount;

		fixture.m_body = this;

		// Adjust mass properties if needed
		if (fixture.m_density > 0.0) {
			this.ResetMassData();
		}

		// Let the world know we have a new fixture. This will cause new contacts to be created
		// at the beginning of the next time step.
		this.m_world.m_flags |= b2World.e_newFixture;

		return fixture;
	}

	/**
	 * Creates a fixture from a shape and attach it to this body.
	 * This is a convenience function. Use b2FixtureDef if you need to set parameters
	 * like friction, restitution, user data, or filtering.
	 * This function automatically updates the mass of the body.
	 * @param shape the shape to be cloned.
	 * @param density the shape density (set to zero for static bodies).
	 * @warning This function is locked during callbacks.
	 */
	public CreateFixture2(shape: b2Shape, density: number = 0.0): b2Fixture {
		const def: b2FixtureDef = new b2FixtureDef();
		def.shape = shape;
		def.density = density;

		return this.CreateFixture(def);
	}

	/**
	 * Destroy a fixture. This removes the fixture from the broad-phase and
	 * destroys all contacts associated with this fixture. This will
	 * automatically adjust the mass of the body if the body is dynamic and the
	 * fixture has positive density.
	 * All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	 * @param fixture the fixture to be removed.
	 * @warning This function is locked during callbacks.
	 */
	public DestroyFixture(fixture: b2Fixture): void {
		//b2Settings.b2Assert(this.m_world.IsLocked() == false);
		if (this.m_world.IsLocked() == true) {
			return;
		}

		//b2Settings.b2Assert(this.m_fixtureCount > 0);
		//b2Fixture** node = &this.m_fixtureList;
		let node: b2Fixture = this.m_fixtureList;
		let ppF: b2Fixture = null; // Fix pointer-pointer stuff
		let found: boolean = false;
		while (node != null) {
			if (node == fixture) {
				if (ppF)
					ppF.m_next = fixture.m_next;
				else
					this.m_fixtureList = fixture.m_next;
				//node = fixture.this.m_next;
				found = true;
				break;
			}

			ppF = node;
			node = node.m_next;
		}

		// You tried to remove a shape that is not attached to this body.
		//b2Settings.b2Assert(found);

		// Destroy any contacts associated with the fixture.
		let edge: b2ContactEdge = this.m_contactList;
		while (edge) {
			const c: b2Contact = edge.contact;
			edge = edge.next;

			const fixtureA: b2Fixture = c.GetFixtureA();
			const fixtureB: b2Fixture = c.GetFixtureB();
			if (fixture == fixtureA || fixture == fixtureB) {
				// This destros the contact and removes it from
				// this body's contact list
				this.m_world.m_contactManager.Destroy(c);
			}
		}

		if (this.m_flags & b2Body.e_activeFlag) {
			const broadPhase: IBroadPhase = this.m_world.m_contactManager.m_broadPhase;
			fixture.DestroyProxy(broadPhase);
		} else {
			//b2Assert(fixture->m_proxyId == b2BroadPhase::e_nullProxy);
		}

		fixture.Destroy();
		fixture.m_body = null;
		fixture.m_next = null;

		--this.m_fixtureCount;

		// Reset the mass data.
		this.ResetMassData();
	}

	/**
	* Set the position of the body's origin and rotation (radians).
	* This breaks any contacts and wakes the other bodies.
	* @param position the new world position of the body's origin (not necessarily
	* the center of mass).
	* @param angle the new world rotation angle of the body in radians.
	*/
	public SetPositionAndAngle(position: b2Vec2, angle: number): void {

		let f: b2Fixture;

		//b2Settings.b2Assert(this.m_world.IsLocked() == false);
		if (this.m_world.IsLocked() == true) {
			return;
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

		const broadPhase: IBroadPhase = this.m_world.m_contactManager.m_broadPhase;
		for (f = this.m_fixtureList; f; f = f.m_next) {
			f.Synchronize(broadPhase, this.m_xf, this.m_xf);
		}
		this.m_world.m_contactManager.FindNewContacts();
	}

	/**
	 * Set the position of the body's origin and rotation (radians).
	 * This breaks any contacts and wakes the other bodies.
	 * Note this is less efficient than the other overload - you should use that
	 * if the angle is available.
	 * @param xf the transform of position and angle to set the bdoy to.
	 */
	public SetTransform(xf: b2Transform): void {
		this.SetPositionAndAngle(xf.position, xf.GetAngle());
	}

	/**
	* Get the body transform for the body's origin.
	* @return the world transform of the body's origin.
	*/
	public GetTransform(): b2Transform {
		return this.m_xf;
	}

	/**
	* Get the world body origin position.
	* @return the world position of the body's origin.
	*/
	public GetPosition(): b2Vec2 {
		return this.m_xf.position;
	}

	/**
	 * Setthe world body origin position.
	 * @param position the new position of the body
	 */
	public SetPosition(position: b2Vec2): void {
		this.SetPositionAndAngle(position, this.GetAngle());
	}

	/**
	* Get the angle in radians.
	* @return the current world rotation angle in radians.
	*/
	public GetAngle(): number {
		return this.m_sweep.a;
	}

	/**
	 * Set the world body angle
	 * @param angle the new angle of the body.
	 */
	public SetAngle(angle: number): void {
		this.SetPositionAndAngle(this.GetPosition(), angle);
	}

	/**
	* Get the world position of the center of mass.
	*/
	public GetWorldCenter(): b2Vec2 {
		return this.m_sweep.c;
	}

	/**
	* Get the local position of the center of mass.
	*/
	public GetLocalCenter(): b2Vec2 {
		return this.m_sweep.localCenter;
	}

	/**
	* Set the linear velocity of the center of mass.
	* @param v the new linear velocity of the center of mass.
	*/
	public SetLinearVelocity(v: b2Vec2): void {
		if (this.m_type == b2Body.b2_staticBody) {
			return;
		}
		this.m_linearVelocity.SetV(v);
	}

	/**
	* Get the linear velocity of the center of mass.
	* @return the linear velocity of the center of mass.
	*/
	public GetLinearVelocity(): b2Vec2 {
		return this.m_linearVelocity;
	}

	/**
	* Set the angular velocity.
	* @param omega the new angular velocity in radians/second.
	*/
	public SetAngularVelocity(omega: number): void {
		if (this.m_type == b2Body.b2_staticBody) {
			return;
		}
		this.m_angularVelocity = omega;
	}

	/**
	* Get the angular velocity.
	* @return the angular velocity in radians/second.
	*/
	public GetAngularVelocity(): number {
		return this.m_angularVelocity;
	}

	/**
	 * Get the definition containing the body properties.
	 * @asonly
	 */
	public GetDefinition(): b2BodyDef {
		const bd: b2BodyDef = new b2BodyDef();
		bd.type = this.GetType();
		bd.allowSleep = (this.m_flags & b2Body.e_allowSleepFlag) == b2Body.e_allowSleepFlag;
		bd.angle = this.GetAngle();
		bd.angularDamping = this.m_angularDamping;
		bd.angularVelocity = this.m_angularVelocity;
		bd.fixedRotation = (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
		bd.bullet = (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
		bd.awake = (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
		bd.linearDamping = this.m_linearDamping;
		bd.linearVelocity.SetV(this.GetLinearVelocity());
		bd.position = this.GetPosition();
		bd.userData = this.GetUserData();
		return bd;
	}

	/**
	* Apply a force at a world point. If the force is not
	* applied at the center of mass, it will generate a torque and
	* affect the angular velocity. This wakes up the body.
	* @param force the world force vector, usually in Newtons (N).
	* @param point the world position of the point of application.
	*/
	public ApplyForce(force: b2Vec2, point: b2Vec2): void {
		if (this.m_type != b2Body.b2_dynamicBody) {
			return;
		}

		if (this.IsAwake() == false) {
			this.SetAwake(true);
		}

		//this.m_force += force;
		this.m_force.x += force.x;
		this.m_force.y += force.y;
		//this.m_torque += b2Cross(point - this.m_sweep.c, force);
		this.m_torque += ((point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x);
	}

	/**
	* Apply a torque. This affects the angular velocity
	* without affecting the linear velocity of the center of mass.
	* This wakes up the body.
	* @param torque about the z-axis (out of the screen), usually in N-m.
	*/
	public ApplyTorque(torque: number): void {
		if (this.m_type != b2Body.b2_dynamicBody) {
			return;
		}

		if (this.IsAwake() == false) {
			this.SetAwake(true);
		}
		this.m_torque += torque;
	}

	/**
	* Apply an impulse at a point. This immediately modifies the velocity.
	* It also modifies the angular velocity if the point of application
	* is not at the center of mass. This wakes up the body.
	* @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	* @param point the world position of the point of application.
	*/
	public ApplyImpulse(impulse: b2Vec2, point: b2Vec2): void {
		if (this.m_type != b2Body.b2_dynamicBody) {
			return;
		}

		if (this.IsAwake() == false) {
			this.SetAwake(true);
		}
		//this.m_linearVelocity += this.m_invMass * impulse;
		this.m_linearVelocity.x += this.m_invMass * impulse.x;
		this.m_linearVelocity.y += this.m_invMass * impulse.y;
		//this.m_angularVelocity += this.m_invI * b2Cross(point - this.m_sweep.c, impulse);
		this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
	}

	/**
	 * Splits a body into two, preserving dynamic properties
	 * @param	callback Called once per fixture, return true to move this fixture to the new body
	 * <code>function Callback(fixture:b2Fixture):boolean</code>
	 * @return The newly created bodies
	 * @asonly
	 */
	public Split(callback: Function): b2Body {
		const linearVelocity: b2Vec2 = this.GetLinearVelocity().Copy();//Reset mass will alter this
		const angularVelocity: number = this.GetAngularVelocity();
		const center: b2Vec2 = this.GetWorldCenter();
		const body1: b2Body = this;
		const body2: b2Body = this.m_world.CreateBody(this.GetDefinition());

		let prev: b2Fixture;
		for (let f: b2Fixture = body1.m_fixtureList; f;) {
			if (callback(f)) {
				const next: b2Fixture = f.m_next;
				// Remove fixture
				if (prev) {
					prev.m_next = next;
				} else {
					body1.m_fixtureList = next;
				}
				body1.m_fixtureCount--;

				// Add fixture
				f.m_next = body2.m_fixtureList;
				body2.m_fixtureList = f;
				body2.m_fixtureCount++;

				f.m_body = body2;

				f = next;
			} else {
				prev = f;
				f = f.m_next;
			}
		}

		body1.ResetMassData();
		body2.ResetMassData();

		// Compute consistent velocites for new bodies based on cached velocity
		const center1: b2Vec2 = body1.GetWorldCenter();
		const center2: b2Vec2 = body2.GetWorldCenter();

		const velocity1: b2Vec2 = b2Math.AddVV(linearVelocity,
			b2Math.CrossFV(angularVelocity,
				b2Math.SubtractVV(center1, center)));

		const velocity2: b2Vec2 = b2Math.AddVV(linearVelocity,
			b2Math.CrossFV(angularVelocity,
				b2Math.SubtractVV(center2, center)));

		body1.SetLinearVelocity(velocity1);
		body2.SetLinearVelocity(velocity2);
		body1.SetAngularVelocity(angularVelocity);
		body2.SetAngularVelocity(angularVelocity);

		body1.SynchronizeFixtures();
		body2.SynchronizeFixtures();

		return body2;
	}

	/**
	 * Merges another body into this. Only fixtures, mass and velocity are effected,
	 * Other properties are ignored
	 * @asonly
	 */
	public Merge(other: b2Body): void {
		let f: b2Fixture;
		for (f = other.m_fixtureList; f;) {
			const next: b2Fixture = f.m_next;

			// Remove fixture
			other.m_fixtureCount--;

			// Add fixture
			f.m_next = this.m_fixtureList;
			this.m_fixtureList = f;
			this.m_fixtureCount++;

			f.m_body = body2;

			f = next;
		}
		body1.m_fixtureCount = 0;

		// Recalculate velocities
		var body1: b2Body = this;
		var body2: b2Body = other;

		// Compute consistent velocites for new bodies based on cached velocity
		const center1: b2Vec2 = body1.GetWorldCenter();
		const center2: b2Vec2 = body2.GetWorldCenter();

		const velocity1: b2Vec2 = body1.GetLinearVelocity().Copy();
		const velocity2: b2Vec2 = body2.GetLinearVelocity().Copy();

		const angular1: number = body1.GetAngularVelocity();
		const angular: number = body2.GetAngularVelocity();

		// TODO

		body1.ResetMassData();

		this.SynchronizeFixtures();
	}

	/**
	* Get the total mass of the body.
	* @return the mass, usually in kilograms (kg).
	*/
	public GetMass(): number {
		return this.m_mass;
	}

	/**
	* Get the central rotational inertia of the body.
	* @return the rotational inertia, usually in kg-m^2.
	*/
	public GetInertia(): number {
		return this.m_I;
	}

	/**
	 * Get the mass data of the body. The rotational inertial is relative to the center of mass.
	 */
	public GetMassData(data: b2MassData): void {
		data.mass = this.m_mass;
		data.I = this.m_I;
		data.center.SetV(this.m_sweep.localCenter);
	}

	/**
	 * Set the mass properties to override the mass properties of the fixtures
	 * Note that this changes the center of mass position.
	 * Note that creating or destroying fixtures can also alter the mass.
	 * This function has no effect if the body isn't dynamic.
	 * @warning The supplied rotational inertia should be relative to the center of mass
	 * @param	data the mass properties.
	 */
	public SetMassData(massData: b2MassData): void {
		b2Settings.b2Assert(this.m_world.IsLocked() == false);
		if (this.m_world.IsLocked() == true) {
			return;
		}

		if (this.m_type != b2Body.b2_dynamicBody) {
			return;
		}

		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;

		this.m_mass = massData.mass;

		// Compute the center of mass.
		if (this.m_mass <= 0.0) {
			this.m_mass = 1.0;
		}
		this.m_invMass = 1.0 / this.m_mass;

		if (massData.I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
			// Center the inertia about the center of mass
			this.m_I = massData.I - this.m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
			this.m_invI = 1.0 / this.m_I;
		}

		// Move center of mass
		const oldCenter: b2Vec2 = this.m_sweep.c.Copy();
		this.m_sweep.localCenter.SetV(massData.center);
		this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
		this.m_sweep.c.SetV(this.m_sweep.c0);

		// Update center of mass velocity
		//this.m_linearVelocity += b2Cross(this.m_angularVelocity, this.m_sweep.c - oldCenter);
		this.m_linearVelocity.x += this.m_angularVelocity * -(this.m_sweep.c.y - oldCenter.y);
		this.m_linearVelocity.y += this.m_angularVelocity * +(this.m_sweep.c.x - oldCenter.x);

	}

	/**
	 * This resets the mass properties to the sum of the mass properties of the fixtures.
	 * This normally does not need to be called unless you called SetMassData to override
	 * the mass and later you want to reset the mass.
	 */
	public ResetMassData(): void {
		// Compute mass data from shapes. Each shape has it's own density
		this.m_mass = 0.0;
		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;
		this.m_sweep.localCenter.SetZero();

		// Static and kinematic bodies have zero mass.
		if (this.m_type == b2Body.b2_staticBody || this.m_type == b2Body.b2_kinematicBody) {
			return;
		}
		//b2Assert(this.m_type == b2Body.b2_dynamicBody);

		// Accumulate mass over all fixtures.
		const center: b2Vec2 = b2Vec2.Make(0, 0);
		for (let f: b2Fixture = this.m_fixtureList; f; f = f.m_next) {
			if (f.m_density == 0.0) {
				continue;
			}

			const massData: b2MassData = f.GetMassData();
			this.m_mass += massData.mass;
			center.x += massData.center.x * massData.mass;
			center.y += massData.center.y * massData.mass;
			this.m_I += massData.I;
		}

		// Compute the center of mass.
		if (this.m_mass > 0.0) {
			this.m_invMass = 1.0 / this.m_mass;
			center.x *= this.m_invMass;
			center.y *= this.m_invMass;
		} else {
			// Force all dynamic bodies to have a positive mass.
			this.m_mass = 1.0;
			this.m_invMass = 1.0;
		}

		if (this.m_I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
			// Center the inertia about the center of mass
			this.m_I -= this.m_mass * (center.x * center.x + center.y * center.y);
			this.m_I *= this.m_inertiaScale;
			b2Settings.b2Assert(this.m_I > 0);
			this.m_invI = 1.0 / this.m_I;
		} else {
			this.m_I = 0.0;
			this.m_invI = 0.0;
		}

		// Move center of mass
		const oldCenter: b2Vec2 = this.m_sweep.c.Copy();
		this.m_sweep.localCenter.SetV(center);
		this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
		this.m_sweep.c.SetV(this.m_sweep.c0);

		// Update center of mass velocity
		//this.m_linearVelocity += b2Cross(this.m_angularVelocity, this.m_sweep.c - oldCenter);
		this.m_linearVelocity.x += this.m_angularVelocity * -(this.m_sweep.c.y - oldCenter.y);
		this.m_linearVelocity.y += this.m_angularVelocity * +(this.m_sweep.c.x - oldCenter.x);

	}

	/**
	 * Get the world coordinates of a point given the local coordinates.
	 * @param localPoint a point on the body measured relative the the body's origin.
	 * @return the same point expressed in world coordinates.
	 */
	public GetWorldPoint(localPoint: b2Vec2): b2Vec2 {
		//return b2Math.b2MulX(this.m_xf, localPoint);
		const A: b2Mat22 = this.m_xf.R;
		const u: b2Vec2 = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y,
								  A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		u.x += this.m_xf.position.x;
		u.y += this.m_xf.position.y;
		return u;
	}

	/**
	 * Get the world coordinates of a vector given the local coordinates.
	 * @param localVector a vector fixed in the body.
	 * @return the same vector expressed in world coordinates.
	 */
	public GetWorldVector(localVector: b2Vec2): b2Vec2 {
		return b2Math.MulMV(this.m_xf.R, localVector);
	}

	/**
	 * Gets a local point relative to the body's origin given a world point.
	 * @param a point in world coordinates.
	 * @return the corresponding local point relative to the body's origin.
	 */
	public GetLocalPoint(worldPoint: b2Vec2): b2Vec2 {
		return b2Math.MulXT(this.m_xf, worldPoint);
	}

	/**
	 * Gets a local vector given a world vector.
	 * @param a vector in world coordinates.
	 * @return the corresponding local vector.
	 */
	public GetLocalVector(worldVector: b2Vec2): b2Vec2 {
		return b2Math.MulTMV(this.m_xf.R, worldVector);
	}

	/**
	* Get the world linear velocity of a world point attached to this body.
	* @param a point in world coordinates.
	* @return the world velocity of a point.
	*/
	public GetLinearVelocityFromWorldPoint(worldPoint: b2Vec2): b2Vec2 {
		//return          this.m_linearVelocity   + b2Cross(this.m_angularVelocity,   worldPoint   - this.m_sweep.c);
		return new b2Vec2(this.m_linearVelocity.x -         this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y),
		                  this.m_linearVelocity.y +         this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
	}

	/**
	* Get the world velocity of a local point.
	* @param a point in local coordinates.
	* @return the world velocity of a point.
	*/
	public GetLinearVelocityFromLocalPoint(localPoint: b2Vec2): b2Vec2 {
		//return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
		const A: b2Mat22 = this.m_xf.R;
		const worldPoint: b2Vec2 = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y,
		                                   A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		worldPoint.x += this.m_xf.position.x;
		worldPoint.y += this.m_xf.position.y;
		return new b2Vec2(this.m_linearVelocity.x -         this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y),
		                  this.m_linearVelocity.y +         this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
	}

	/**
	* Get the linear damping of the body.
	*/
	public GetLinearDamping(): number {
		return this.m_linearDamping;
	}

	/**
	* Set the linear damping of the body.
	*/
	public SetLinearDamping(linearDamping: number): void {
		this.m_linearDamping = linearDamping;
	}

	/**
	* Get the angular damping of the body
	*/
	public GetAngularDamping(): number {
		return this.m_angularDamping;
	}

	/**
	* Set the angular damping of the body.
	*/
	public SetAngularDamping(angularDamping: number): void {
		this.m_angularDamping = angularDamping;
	}

	/**
	 * Set the type of this body. This may alter the mass and velocity
	 * @param	type - enum stored as a static member of b2Body
	 */
	public SetType(type: number /** uint */): void {
		if (this.m_type == type) {
			return;
		}

		this.m_type = type;

		this.ResetMassData();

		if (this.m_type == b2Body.b2_staticBody) {
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0.0;
		}

		this.SetAwake(true);

		this.m_force.SetZero();
		this.m_torque = 0.0;

		// Since the body type changed, we need to flag contacts for filtering.
		for (let ce: b2ContactEdge = this.m_contactList; ce; ce = ce.next) {
			ce.contact.FlagForFiltering();
		}
	}

	/**
	 * Get the type of this body.
	 * @return type enum as a uint
	 */
	public GetType(): number /** uint */
	{
		return this.m_type;
	}

	/**
	* Should this body be treated like a bullet for continuous collision detection?
	*/
	public SetBullet(flag: boolean): void {
		if (flag) {
			this.m_flags |= b2Body.e_bulletFlag;
		} else {
			this.m_flags &= ~b2Body.e_bulletFlag;
		}
	}

	/**
	* Is this body treated like a bullet for continuous collision detection?
	*/
	public IsBullet(): Boolean {
		return (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
	}

	/**
	 * Is this body allowed to sleep
	 * @param	flag
	 */
	public SetSleepingAllowed(flag: boolean): void {
		if (flag) {
			this.m_flags |= b2Body.e_allowSleepFlag;
		} else {
			this.m_flags &= ~b2Body.e_allowSleepFlag;
			this.SetAwake(true);
		}
	}

	/**
	 * Set the sleep state of the body. A sleeping body has vety low CPU cost.
	 * @param	flag - set to true to put body to sleep, false to wake it
	 */
	public SetAwake(flag: boolean): void {
		if (flag) {
			this.m_flags |= b2Body.e_awakeFlag;
			this.m_sleepTime = 0.0;
		} else {
			this.m_flags &= ~b2Body.e_awakeFlag;
			this.m_sleepTime = 0.0;
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0.0;
			this.m_force.SetZero();
			this.m_torque = 0.0;
		}
	}

	/**
	 * Get the sleeping state of this body.
	 * @return true if body is sleeping
	 */
	public IsAwake(): boolean {
		return (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
	}

	/**
	 * Set this body to have fixed rotation. This causes the mass to be reset.
	 * @param	fixed - true means no rotation
	 */
	public SetFixedRotation(fixed: boolean): void {
		if (fixed) {
			this.m_flags |= b2Body.e_fixedRotationFlag;
		} else {
			this.m_flags &= ~b2Body.e_fixedRotationFlag;
		}

		this.ResetMassData();
	}

	/**
	* Does this body have fixed rotation?
	* @return true means fixed rotation
	*/
	public IsFixedRotation(): boolean {
		return (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
	}

	/** Set the active state of the body. An inactive body is not
	* simulated and cannot be collided with or woken up.
	* If you pass a flag of true, all fixtures will be added to the
	* broad-phase.
	* If you pass a flag of false, all fixtures will be removed from
	* the broad-phase and all contacts will be destroyed.
	* Fixtures and joints are otherwise unaffected. You may continue
	* to create/destroy fixtures and joints on inactive bodies.
	* Fixtures on an inactive body are implicitly inactive and will
	* not participate in collisions, ray-casts, or queries.
	* Joints connected to an inactive body are implicitly inactive.
	* An inactive body is still owned by a b2World object and remains
	* in the body list.
	*/
	public SetActive(flag: boolean): void {
		if (flag == this.IsActive()) {
			return;
		}

		let broadPhase: IBroadPhase;
		let f: b2Fixture;
		if (flag) {
			this.m_flags |= b2Body.e_activeFlag;

			// Create all proxies.
			broadPhase = this.m_world.m_contactManager.m_broadPhase;
			for (f = this.m_fixtureList; f; f = f.m_next) {
				f.CreateProxy(broadPhase, this.m_xf);
			}
			// Contacts are created the next time step.
		} else {
			this.m_flags &= ~b2Body.e_activeFlag;

			// Destroy all proxies.
			broadPhase = this.m_world.m_contactManager.m_broadPhase;
			for (f = this.m_fixtureList; f; f = f.m_next) {
				f.DestroyProxy(broadPhase);
			}

			// Destroy the attached contacts.
			let ce: b2ContactEdge = this.m_contactList;
			while (ce) {
				const ce0: b2ContactEdge = ce;
				ce = ce.next;
				this.m_world.m_contactManager.Destroy(ce0.contact);
			}
			this.m_contactList = null;
		}
	}

	/**
	 * Get the active state of the body.
	 * @return true if active.
	 */
	public IsActive(): boolean {
		return (this.m_flags & b2Body.e_activeFlag) == b2Body.e_activeFlag;
	}

	/**
	* Is this body allowed to sleep?
	*/
	public IsSleepingAllowed(): boolean {
		return (this.m_flags & b2Body.e_allowSleepFlag) == b2Body.e_allowSleepFlag;
	}

	/**
	* Get the list of all fixtures attached to this body.
	*/
	public GetFixtureList(): b2Fixture {
		return this.m_fixtureList;
	}

	/**
	* Get the list of all joints attached to this body.
	*/
	public GetJointList(): b2JointEdge {
		return this.m_jointList;
	}

	/**
	 * Get the list of all controllers attached to this body.
	 */
	public GetControllerList(): b2ControllerEdge {
		return this.m_controllerList;
	}

	/**
	 * Get a list of all contacts attached to this body.
	 */
	public GetContactList(): b2ContactEdge {
		return this.m_contactList;
	}

	/**
	* Get the next body in the world's body list.
	*/
	public GetNext(): b2Body {
		return this.m_next;
	}

	/**
	* Get the user data pointer that was provided in the body definition.
	*/
	public GetUserData(): any {
		return this.m_userData;
	}

	/**
	* Set the user data. Use this to store your application specific data.
	*/
	public SetUserData(data: any): void {
		this.m_userData = data;
	}

	/**
	* Get the parent world of this body.
	*/
	public GetWorld(): b2World {
		return this.m_world;
	}

	//--------------- Internals Below -------------------

	// Constructor
	/**
	 * @private
	 */
	constructor(bd: b2BodyDef, world: b2World) {
		//b2Settings.b2Assert(world.IsLocked() == false);

		//b2Settings.b2Assert(bd.position.IsValid());
 		//b2Settings.b2Assert(bd.linearVelocity.IsValid());
 		//b2Settings.b2Assert(b2Math.b2IsValid(bd.angle));
 		//b2Settings.b2Assert(b2Math.b2IsValid(bd.angularVelocity));
 		//b2Settings.b2Assert(b2Math.b2IsValid(bd.inertiaScale) && bd.inertiaScale >= 0.0);
 		//b2Settings.b2Assert(b2Math.b2IsValid(bd.angularDamping) && bd.angularDamping >= 0.0);
 		//b2Settings.b2Assert(b2Math.b2IsValid(bd.linearDamping) && bd.linearDamping >= 0.0);

		this.m_flags = 0;

		if (bd.bullet) {
			this.m_flags |= b2Body.e_bulletFlag;
		}
		if (bd.fixedRotation) {
			this.m_flags |= b2Body.e_fixedRotationFlag;
		}
		if (bd.allowSleep) {
			this.m_flags |= b2Body.e_allowSleepFlag;
		}
		if (bd.awake) {
			this.m_flags |= b2Body.e_awakeFlag;
		}
		if (bd.active) {
			this.m_flags |= b2Body.e_activeFlag;
		}

		this.m_world = world;

		this.m_xf.position.SetV(bd.position);
		this.m_xf.R.Set(bd.angle);

		this.m_sweep.localCenter.SetZero();
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
		this.m_controllerList = null;
		this.m_contactList = null;
		this.m_controllerCount = 0;
		this.m_prev = null;
		this.m_next = null;

		this.m_linearVelocity.SetV(bd.linearVelocity);
		this.m_angularVelocity = bd.angularVelocity;

		this.m_linearDamping = bd.linearDamping;
		this.m_angularDamping = bd.angularDamping;

		this.m_force.Set(0.0, 0.0);
		this.m_torque = 0.0;

		this.m_sleepTime = 0.0;

		this.m_type = bd.type;

		if (this.m_type == b2Body.b2_dynamicBody) {
			this.m_mass = 1.0;
			this.m_invMass = 1.0;
		} else {
			this.m_mass = 0.0;
			this.m_invMass = 0.0;
		}

		this.m_I = 0.0;
		this.m_invI = 0.0;

		this.m_inertiaScale = bd.inertiaScale;

		this.m_userData = bd.userData;

		this.m_fixtureList = null;
		this.m_fixtureCount = 0;
	}

	// Destructor
	//~b2Body();

	//
	private static s_xf1: b2Transform = new b2Transform();
	//
	public SynchronizeFixtures(): void {

		const xf1: b2Transform = b2Body.s_xf1;
		xf1.R.Set(this.m_sweep.a0);
		//xf1.position = this.m_sweep.c0 - b2Mul(xf1.R, this.m_sweep.localCenter);
		const tMat: b2Mat22 = xf1.R;
		const tVec: b2Vec2 = this.m_sweep.localCenter;
		xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		let f: b2Fixture;
		const broadPhase: IBroadPhase = this.m_world.m_contactManager.m_broadPhase;
		for (f = this.m_fixtureList; f; f = f.m_next) {
			f.Synchronize(broadPhase, xf1, this.m_xf);
		}
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
	public ShouldCollide(other: b2Body): Boolean {
		// At least one body should be dynamic
		if (this.m_type != b2Body.b2_dynamicBody && other.m_type != b2Body.b2_dynamicBody) {
			return false;
		}
		// Does a joint prevent collision?
		for (let jn: b2JointEdge = this.m_jointList; jn; jn = jn.next) {
			if (jn.other == other)
				if (jn.joint.m_collideConnected == false) {
					return false;
				}
		}

		return true;
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

	public m_islandIndex: number /** int */;

	public m_xf: b2Transform = new b2Transform();		// the body origin transform

	public m_sweep: b2Sweep = new b2Sweep();	// the swept motion for CCD

	public m_linearVelocity: b2Vec2 = new b2Vec2();
	public m_angularVelocity: number;

	public m_force: b2Vec2 = new b2Vec2();
	public m_torque: number;

	public m_world: b2World;
	public m_prev: b2Body;
	public m_next: b2Body;

	public m_fixtureList: b2Fixture;
	public m_fixtureCount: number /** int */;

	public m_controllerList: b2ControllerEdge;
	public m_controllerCount: number /** int */;

	public m_jointList: b2JointEdge;
	public m_contactList: b2ContactEdge;

	public m_mass: number;
	public m_invMass: number;
	public m_I: number
	public m_invI: number;

	public m_inertiaScale: number;

	public m_linearDamping: number;
	public m_angularDamping: number;

	public m_sleepTime: number;

	private m_userData: any;

	// m_flags
	//enum
	//{
	public static e_islandFlag: number /** uint */			= 0x0001;
	public static e_awakeFlag: number /** uint */			= 0x0002;
	public static e_allowSleepFlag: number /** uint */		= 0x0004;
	public static e_bulletFlag: number /** uint */			= 0x0008;
	public static e_fixedRotationFlag: number /** uint */	= 0x0010;
	public static e_activeFlag: number /** uint */			= 0x0020;
	//};

	// this.m_type
	//enum
	//{
	/// The body type.
	/// static: zero mass, zero velocity, may be manually moved
	/// kinematic: zero mass, non-zero velocity set by user, moved by solver
	/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
	public static b2_staticBody: number /** uint */ = 0;
	public static b2_kinematicBody: number /** uint */ = 1;
	public static b2_dynamicBody: number /** uint */ = 2;
	//};

}