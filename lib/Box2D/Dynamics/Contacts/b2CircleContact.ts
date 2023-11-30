import { b2Contact } from '../Contacts';
import { b2Fixture } from '../b2Fixture';
import { b2Body } from '../b2Body';
import { b2CircleShape } from '../../Collision/Shapes/b2CircleShape';
import { b2Collision } from '../../Collision/b2Collision';

/**
* @private
*/
export class b2CircleContact extends b2Contact {
	public static Create(allocator: any): b2Contact {
		return new b2CircleContact();
	}

	public static Destroy(contact: b2Contact, allocator: any): void {
		//
	}

	public Reset(fixtureA: b2Fixture, fixtureB: b2Fixture): void {
		super.Reset(fixtureA, fixtureB);
		//b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_circleShape);
		//b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_circleShape);
	}
	//~b2CircleContact() {}

	public Evaluate(): void {
		const bA: b2Body = this.m_fixtureA.GetBody();
		const bB: b2Body = this.m_fixtureB.GetBody();

		b2Collision.CollideCircles(this.m_manifold,
			this.m_fixtureA.GetShape() as b2CircleShape, bA.m_xf,
			this.m_fixtureB.GetShape() as b2CircleShape, bB.m_xf);
	}
}