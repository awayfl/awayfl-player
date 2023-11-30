// Delegate of b2World.

import { b2Fixture } from './b2Fixture';
import { b2Body } from './b2Body';
import { b2World } from './b2World';
import { IBroadPhase } from '../Collision/IBroadPhase';
import { b2ContactFilter } from './b2ContactFilter';
import { b2ContactListener } from './b2ContactListener';
import { b2Contact, b2ContactEdge, b2ContactFactory } from './Contacts';
import { b2ContactPoint } from '../Collision/b2ContactPoint';
import { b2DynamicTreeBroadPhase } from '../Collision/b2DynamicTreeBroadPhase';

/**
* @private
*/
export class b2ContactManager {
	__fast__: boolean = true;

	constructor() {
		this.m_world = null;
		this.m_contactCount = 0;
		this.m_contactFilter = b2ContactFilter.b2_defaultFilter;
		this.m_contactListener = b2ContactListener.b2_defaultListener;
		this.m_contactFactory = new b2ContactFactory(this.m_allocator);
		this.m_broadPhase = new b2DynamicTreeBroadPhase();

		this.AddPairDelegate = (proxyUserDataA: any, proxyUserDataB: any) => this.AddPair(proxyUserDataA, proxyUserDataB);
	}

	public AddPairDelegate: (proxyUserDataA: any, proxyUserDataB: any) => void;
	// This is a callback from the broadphase when two AABB proxies begin
	// to overlap. We create a b2Contact to manage the narrow phase.
	public AddPair(proxyUserDataA: any, proxyUserDataB: any): void {
		let fixtureA: b2Fixture = proxyUserDataA as b2Fixture;
		let fixtureB: b2Fixture = proxyUserDataB as b2Fixture;

		let bodyA: b2Body = fixtureA.GetBody();
		let bodyB: b2Body = fixtureB.GetBody();

		// Are the fixtures on the same body?
		if (bodyA == bodyB)
			return;

		// Does a contact already exist?
		let edge: b2ContactEdge = bodyB.GetContactList();
		while (edge) {
			if (edge.other == bodyA) {
				const fA: b2Fixture = edge.contact.GetFixtureA();
				const fB: b2Fixture = edge.contact.GetFixtureB();
				if (fA == fixtureA && fB == fixtureB)
					return;
				if (fA == fixtureB && fB == fixtureA)
					return;
			}
			edge = edge.next;
		}

		//Does a joint override collision? Is at least one body dynamic?
		if (bodyB.ShouldCollide(bodyA) == false) {
			return;
		}

		// Check user filtering
		if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
			return;
		}

		// Call the factory.
		const c: b2Contact = this.m_contactFactory.Create(fixtureA, fixtureB);

		// Contact creation may swap shapes.
		fixtureA = c.GetFixtureA();
		fixtureB = c.GetFixtureB();
		bodyA = fixtureA.m_body;
		bodyB = fixtureB.m_body;

		// Insert into the world.
		c.m_prev = null;
		c.m_next = this.m_world.m_contactList;
		if (this.m_world.m_contactList != null) {
			this.m_world.m_contactList.m_prev = c;
		}
		this.m_world.m_contactList = c;

		// Connect to island graph.

		// Connect to body A
		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;

		c.m_nodeA.prev = null;
		c.m_nodeA.next = bodyA.m_contactList;
		if (bodyA.m_contactList != null) {
			bodyA.m_contactList.prev = c.m_nodeA;
		}
		bodyA.m_contactList = c.m_nodeA;

		// Connect to body 2
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;

		c.m_nodeB.prev = null;
		c.m_nodeB.next = bodyB.m_contactList;
		if (bodyB.m_contactList != null) {
			bodyB.m_contactList.prev = c.m_nodeB;
		}
		bodyB.m_contactList = c.m_nodeB;

		++this.m_world.m_contactCount;
		return;

	}

	public FindNewContacts(): void {
		this.m_broadPhase.UpdatePairs(this.AddPairDelegate);
	}

	private static readonly s_evalCP: b2ContactPoint = new b2ContactPoint();
	public Destroy(c: b2Contact): void {

		const fixtureA: b2Fixture = c.GetFixtureA();
		const fixtureB: b2Fixture = c.GetFixtureB();
		const bodyA: b2Body = fixtureA.GetBody();
		const bodyB: b2Body = fixtureB.GetBody();

		if (c.IsTouching()) {
			this.m_contactListener.EndContact(c);
		}

		// Remove from the world.
		if (c.m_prev) {
			c.m_prev.m_next = c.m_next;
		}

		if (c.m_next) {
			c.m_next.m_prev = c.m_prev;
		}

		if (c == this.m_world.m_contactList) {
			this.m_world.m_contactList = c.m_next;
		}

		// Remove from body A
		if (c.m_nodeA.prev) {
			c.m_nodeA.prev.next = c.m_nodeA.next;
		}

		if (c.m_nodeA.next) {
			c.m_nodeA.next.prev = c.m_nodeA.prev;
		}

		if (c.m_nodeA == bodyA.m_contactList) {
			bodyA.m_contactList = c.m_nodeA.next;
		}

		// Remove from body 2
		if (c.m_nodeB.prev) {
			c.m_nodeB.prev.next = c.m_nodeB.next;
		}

		if (c.m_nodeB.next) {
			c.m_nodeB.next.prev = c.m_nodeB.prev;
		}

		if (c.m_nodeB == bodyB.m_contactList) {
			bodyB.m_contactList = c.m_nodeB.next;
		}

		// Call the factory.
		this.m_contactFactory.Destroy(c);
		--this.m_contactCount;
	}

	// This is the top level collision call for the time step. Here
	// all the narrow phase collision is processed for the world
	// contact list.
	public Collide(): void {
		// Update awake contacts.
		let c: b2Contact = this.m_world.m_contactList;
		while (c) {
			const fixtureA: b2Fixture = c.GetFixtureA();
			const fixtureB: b2Fixture = c.GetFixtureB();
			const bodyA: b2Body = fixtureA.GetBody();
			const bodyB: b2Body = fixtureB.GetBody();
			if (bodyA.IsAwake() == false && bodyB.IsAwake() == false) {
				c = c.GetNext();
				continue;
			}

			// Is this contact flagged for filtering?
			if (c.m_flags & b2Contact.e_filterFlag) {
				// Should these bodies collide?
				if (bodyB.ShouldCollide(bodyA) == false) {
					var cNuke: b2Contact = c;
					c = cNuke.GetNext();
					this.Destroy(cNuke);
					continue;
				}

				// Check user filtering.
				if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
					cNuke = c;
					c = cNuke.GetNext();
					this.Destroy(cNuke);
					continue;
				}

				// Clear the filtering flag
				c.m_flags &= ~b2Contact.e_filterFlag;
			}

			const proxyA: any = fixtureA.m_proxy;
			const proxyB: any = fixtureB.m_proxy;

			const overlap: boolean = this.m_broadPhase.TestOverlap(proxyA, proxyB);

			// Here we destroy contacts that cease to overlap in the broadphase
			if (overlap == false) {
				cNuke = c;
				c = cNuke.GetNext();
				this.Destroy(cNuke);
				continue;
			}

			c.Update(this.m_contactListener);
			c = c.GetNext();
		}
	}

	public m_world: b2World;
	public m_broadPhase: IBroadPhase;

	public m_contactList: b2Contact;
	public m_contactCount: number /** int */;
	public m_contactFilter: b2ContactFilter;
	public m_contactListener: b2ContactListener;
	public m_contactFactory: b2ContactFactory;
	public m_allocator: any;
}