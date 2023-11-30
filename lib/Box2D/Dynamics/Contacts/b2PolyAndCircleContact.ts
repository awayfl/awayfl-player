import { b2Fixture } from '../b2Fixture';
import { b2Settings } from '../../Common/b2Settings';
import { b2Shape } from '../../Collision/Shapes/b2Shape';
import { b2Contact } from '../Contacts';
import { b2Body } from '../b2Body';
import { b2Collision } from '../../Collision/b2Collision';
import { b2PolygonShape } from '../../Collision/Shapes/b2PolygonShape';
import { b2CircleShape } from '../../Collision/Shapes/b2CircleShape';

/**
* @private
*/
export class b2PolyAndCircleContact extends b2Contact {

	public static Create(allocator: any): b2Contact {
		return new b2PolyAndCircleContact();
	}

	public static Destroy(contact: b2Contact, allocator: any): void {
	}

	public Reset(fixtureA: b2Fixture, fixtureB: b2Fixture): void {
		super.Reset(fixtureA, fixtureB);
		b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
		b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_circleShape);
	}
	//~b2PolyAndCircleContact() {}

	public Evaluate(): void {
		const bA: b2Body = this.m_fixtureA.m_body;
		const bB: b2Body = this.m_fixtureB.m_body;

		b2Collision.CollidePolygonAndCircle(this.m_manifold,
			this.m_fixtureA.GetShape() as b2PolygonShape, bA.m_xf,
			this.m_fixtureB.GetShape() as b2CircleShape, bB.m_xf);
	}
}