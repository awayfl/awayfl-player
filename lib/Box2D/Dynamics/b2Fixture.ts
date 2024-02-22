import { b2Shape } from '../Collision/Shapes/b2Shape';
import { b2Body } from './b2Body';
import { b2Vec2, b2Transform, b2Math } from '../Common/Math';
import { b2RayCastOutput } from '../Collision/b2RayCastOutput';
import { b2RayCastInput } from '../Collision/b2RayCastInput';
import { b2MassData } from '../Collision/Shapes/b2MassData';
import { b2AABB } from '../Collision/b2AABB';
import { b2FixtureDef } from './b2FixtureDef';
import { b2FilterData } from './b2FilterData';
import { b2ContactEdge, b2Contact } from './Contacts';
import { IBroadPhase } from '../Collision/IBroadPhase';

/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture
 * inherits its transform from its parent. Fixtures hold additional non-geometric data
 * such as friction, collision filters, etc.
 * Fixtures are created via b2Body::CreateFixture.
 * @warning you cannot reuse fixtures.
 */
export class b2Fixture {
	__fast__: boolean = true;

	/**
	 * Get the type of the child shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public GetType(): number /** int */
	{
		return this.m_shape.GetType();
	}

	/**
	 * Get the child shape. You can modify the child shape, however you should not change the
	 * number of vertices because this will crash some collision caching mechanisms.
	 */
	public GetShape(): b2Shape {
		return this.m_shape;
	}

	/**
	 * Set if this fixture is a sensor.
	 */
	public SetSensor(sensor: boolean): void {
		if (this.m_isSensor == sensor)
			return;

		this.m_isSensor = sensor;

		if (this.m_body == null)
			return;

		let edge: b2ContactEdge = this.m_body.GetContactList();
		while (edge) {
			const contact: b2Contact = edge.contact;
			const fixtureA: b2Fixture = contact.GetFixtureA();
			const fixtureB: b2Fixture = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
			edge = edge.next;
		}

	}

	/**
	 * Is this fixture a sensor (non-solid)?
	 * @return the true if the shape is a sensor.
	 */
	public IsSensor(): boolean {
		return this.m_isSensor;
	}

	/**
	 * Set the contact filtering data. This will not update contacts until the next time
	 * step when either parent body is active and awake.
	 */
	public SetFilterData(filter: b2FilterData): void {
		this.m_filter = filter.Copy();

		if (this.m_body)
			return;

		let edge: b2ContactEdge = this.m_body.GetContactList();
		while (edge) {
			const contact: b2Contact = edge.contact;
			const fixtureA: b2Fixture = contact.GetFixtureA();
			const fixtureB: b2Fixture = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.FlagForFiltering();
			edge = edge.next;
		}
	}

	/**
	 * Get the contact filtering data.
	 */
	public GetFilterData(): b2FilterData {
		return this.m_filter.Copy();
	}

	/**
	 * Get the parent body of this fixture. This is NULL if the fixture is not attached.
	 * @return the parent body.
	 */
	public GetBody(): b2Body {
		return this.m_body;
	}

	/**
	 * Get the next fixture in the parent body's fixture list.
	 * @return the next shape.
	 */
	public GetNext(): b2Fixture {
		return this.m_next;
	}

	/**
	 * Get the user data that was assigned in the fixture definition. Use this to
	 * store your application specific data.
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
	 * Test a point for containment in this fixture.
	 * @param xf the shape world transform.
	 * @param p a point in world coordinates.
	 */
	public TestPoint(p: b2Vec2): boolean {
		return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
	}

	/**
	 * Perform a ray cast against this shape.
	 * @param output the ray-cast results.
	 * @param input the ray-cast input parameters.
	 */
	public RayCast(output: b2RayCastOutput, input: b2RayCastInput): boolean {
		return this.m_shape.RayCast(output, input, this.m_body.GetTransform());
	}

	/**
	 * Get the mass data for this fixture. The mass data is based on the density and
	 * the shape. The rotational inertia is about the shape's origin. This operation may be expensive
	 * @param massData - this is a reference to a valid massData, if it is null a new b2MassData is allocated and then returned
	 * @note if the input is null then you must get the return value.
	 */
	public GetMassData(massData: b2MassData = null): b2MassData {
		if (massData == null) {
			massData = new b2MassData();
		}
		this.m_shape.ComputeMass(massData, this.m_density);
		return massData;
	}

	/**
	 * Set the density of this fixture. This will _not_ automatically adjust the mass
	 * of the body. You must call b2Body::ResetMassData to update the body's mass.
	 * @param	density
	 */
	public SetDensity(density: number): void {
		//b2Settings.b2Assert(b2Math.b2IsValid(density) && density >= 0.0);
		this.m_density = density;
	}

	/**
	 * Get the density of this fixture.
	 * @return density
	 */
	public GetDensity(): number {
		return this.m_density;
	}

	/**
	 * Get the coefficient of friction.
	 */
	public GetFriction(): number {
		return this.m_friction;
	}

	/**
	 * Set the coefficient of friction.
	 */
	public SetFriction(friction: number): void {
		this.m_friction = friction;
	}

	/**
	 * Get the coefficient of restitution.
	 */
	public GetRestitution(): number {
		return this.m_restitution;
	}

	/**
	 * Get the coefficient of restitution.
	 */
	public SetRestitution(restitution: number): void {
		this.m_restitution = restitution;
	}

	/**
	 * Get the fixture's AABB. This AABB may be enlarge and/or stale.
	 * If you need a more accurate AABB, compute it using the shape and
	 * the body transform.
	 * @return
	 */
	public GetAABB(): b2AABB {
		return this.m_aabb;
	}

	/**
	 * @private
	 */
	constructor() {
		this.m_aabb = new b2AABB();
		this.m_userData = null;
		this.m_body = null;
		this.m_next = null;
		//this.m_proxyId = b2BroadPhase.e_nullProxy;
		this.m_shape = null;
		this.m_density = 0.0;

		this.m_friction = 0.0;
		this.m_restitution = 0.0;
	}

	/**
	 * the destructor cannot access the allocator (no destructor arguments allowed by C++).
	 *  We need separation create/destroy functions from the constructor/destructor because
	 */
	public Create(body: b2Body, xf: b2Transform, def: b2FixtureDef): void {
		this.m_userData = def.userData;
		this.m_friction = def.friction;
		this.m_restitution = def.restitution;

		this.m_body = body;
		this.m_next = null;

		this.m_filter = def.filter.Copy();

		this.m_isSensor = def.isSensor;

		this.m_shape = def.shape.Copy();

		this.m_density = def.density;
	}

	/**
	 * the destructor cannot access the allocator (no destructor arguments allowed by C++).
	 *  We need separation create/destroy functions from the constructor/destructor because
	 */
	public Destroy(): void {
		// The proxy must be destroyed before calling this.
		//b2Assert(m_proxyId == b2BroadPhase::e_nullProxy);

		// Free the child shape
		this.m_shape = null;
	}

	/**
	 * This supports body activation/deactivation.
	 */
	public CreateProxy(broadPhase: IBroadPhase, xf: b2Transform): void {
		//b2Assert(m_proxyId == b2BroadPhase::e_nullProxy);

		// Create proxy in the broad-phase.
		this.m_shape.ComputeAABB(this.m_aabb, xf);
		this.m_proxy = broadPhase.CreateProxy(this.m_aabb, this);
	}

	/**
	 * This supports body activation/deactivation.
	 */
	public DestroyProxy(broadPhase: IBroadPhase): void {
		if (this.m_proxy == null) {
			return;
		}

		// Destroy proxy in the broad-phase.
		broadPhase.DestroyProxy(this.m_proxy);
		this.m_proxy = null;
	}

	public Synchronize(broadPhase: IBroadPhase, transform1: b2Transform, transform2: b2Transform): void {
		if (!this.m_proxy)
			return;

		// Compute an AABB that ocvers the swept shape (may miss some rotation effect)
		const aabb1: b2AABB = new b2AABB();
		const aabb2: b2AABB = new b2AABB();
		this.m_shape.ComputeAABB(aabb1, transform1);
		this.m_shape.ComputeAABB(aabb2, transform2);

		this.m_aabb.Combine(aabb1, aabb2);
		const displacement: b2Vec2 = b2Math.SubtractVV(transform2.position, transform1.position);
		broadPhase.MoveProxy(this.m_proxy, this.m_aabb, displacement);
	}

	private m_massData: b2MassData;

	public m_aabb: b2AABB;
	public m_density: number;
	public m_next: b2Fixture;
	public m_body: b2Body;
	public m_shape: b2Shape;

	public m_friction: number;
	public m_restitution: number;

	public m_proxy: any;
	public m_filter: b2FilterData = new b2FilterData();

	public m_isSensor: boolean;

	public m_userData: any;
}
