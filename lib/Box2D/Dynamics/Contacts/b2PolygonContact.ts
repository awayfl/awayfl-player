import { b2Contact } from '../Contacts';
import { b2Fixture } from '../b2Fixture';
import { b2Body } from '../b2Body';
import { b2Collision } from '../../Collision/b2Collision';
import { b2PolygonShape } from '../../Collision/Shapes/b2PolygonShape';

/**
* @private
*/
export class b2PolygonContact extends b2Contact {
	public static Create(allocator: any): b2Contact {
		//void* mem = allocator->Allocate(sizeof(b2PolyContact));
		return new b2PolygonContact();
	}

	public static Destroy(contact: b2Contact, allocator: any): void {
		//((b2PolyContact*)contact)->~b2PolyContact();
		//allocator->Free(contact, sizeof(b2PolyContact));
	}

	public Reset(fixtureA: b2Fixture, fixtureB: b2Fixture): void {
		super.Reset(fixtureA, fixtureB);
		//b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_polygonShape);
		//b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_polygonShape);
	}
	//~b2PolyContact() {}

	public Evaluate(): void {
		const bA: b2Body = this.m_fixtureA.GetBody();
		const bB: b2Body = this.m_fixtureB.GetBody();

		b2Collision.CollidePolygons(this.m_manifold,
			this.m_fixtureA.GetShape() as b2PolygonShape, bA.m_xf,
			this.m_fixtureB.GetShape() as b2PolygonShape, bB.m_xf);
	}
}