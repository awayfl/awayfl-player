import { b2Shape } from '../../Collision/Shapes/b2Shape';
import { b2Fixture } from '../b2Fixture';
import { b2ContactRegister, b2Contact, b2CircleContact, b2PolyAndCircleContact, b2PolygonContact, b2EdgeAndCircleContact, b2PolyAndEdgeContact } from '../Contacts';

/**
 * This class manages creation and destruction of b2Contact objects.
 * @private
 */
export class b2ContactFactory {
	constructor(allocator: any) {
		this.m_allocator = allocator;
		this.InitializeRegisters();
	}

	public AddType(createFcn: Function, destroyFcn: Function, type1: number /** int */, type2: number /** int */): void {
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);

		this.m_registers[type1][type2].createFcn = createFcn;
		this.m_registers[type1][type2].destroyFcn = destroyFcn;
		this.m_registers[type1][type2].primary = true;

		if (type1 != type2) {
			this.m_registers[type2][type1].createFcn = createFcn;
			this.m_registers[type2][type1].destroyFcn = destroyFcn;
			this.m_registers[type2][type1].primary = false;
		}
	}

	public InitializeRegisters(): void {
		this.m_registers = new Array<Array<b2ContactRegister> >(b2Shape.e_shapeTypeCount);
		for (let i: number /** int */ = 0; i < b2Shape.e_shapeTypeCount; i++) {
			this.m_registers[i] = new Array<b2ContactRegister>(b2Shape.e_shapeTypeCount);
			for (let j: number /** int */ = 0; j < b2Shape.e_shapeTypeCount; j++) {
				this.m_registers[i][j] = new b2ContactRegister();
			}
		}

		this.AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
		this.AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
		this.AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);

		this.AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape);
		this.AddType(b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape);
	}

	public Create(fixtureA: b2Fixture, fixtureB: b2Fixture): b2Contact {
		const type1: number /** int */ = fixtureA.GetType();
		const type2: number /** int */ = fixtureB.GetType();

		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);

		const reg: b2ContactRegister = this.m_registers[type1][type2];

		let c: b2Contact;

		if (reg.pool) {
			// Pop a contact off the pool
			c = reg.pool;
			reg.pool = c.m_next;
			reg.poolCount--;
			c.Reset(fixtureA, fixtureB);
			return c;
		}

		const createFcn: Function = reg.createFcn;
		if (createFcn != null) {
			if (reg.primary) {
				c = createFcn(this.m_allocator);
				c.Reset(fixtureA, fixtureB);
				return c;
			} else {
				c = createFcn(this.m_allocator);
				c.Reset(fixtureB, fixtureA);
				return c;
			}
		} else {
			return null;
		}
	}

	public Destroy(contact: b2Contact): void {
		if (contact.m_manifold.m_pointCount > 0) {
			contact.m_fixtureA.m_body.SetAwake(true);
			contact.m_fixtureB.m_body.SetAwake(true);
		}

		const type1: number /** int */ = contact.m_fixtureA.GetType();
		const type2: number /** int */ = contact.m_fixtureB.GetType();

		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);

		const reg: b2ContactRegister = this.m_registers[type1][type2];

		if (true) {
			reg.poolCount++;
			contact.m_next = reg.pool;
			reg.pool = contact;
		}

		const destroyFcn: Function = reg.destroyFcn;
		destroyFcn(contact, this.m_allocator);
	}

	private m_registers: Array<Array<b2ContactRegister> >;
	private m_allocator: any;
}