/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

import { b2Shape } from '../../Collision/Shapes/b2Shape';
import { b2ContactRegister } from './b2ContactRegister';
import { b2Manifold } from '../../Collision/b2Manifold';
import { b2Math } from '../../Common/Math';
import { b2ContactEdge } from './b2ContactEdge';
import { b2ContactListener } from '../b2ContactListener';
import { b2Body } from '../b2Body';

//typedef b2Contact* b2ContactCreateFcn(b2Shape* shape1, b2Shape* shape2, b2BlockAllocator* allocator);
//typedef void b2ContactDestroyFcn(b2Contact* contact, b2BlockAllocator* allocator);

export class b2Contact {
	readonly __fast__ = true;

	public GetManifolds(): b2Manifold[] {return null;}

	/// Get the number of manifolds. This is 0 or 1 between convex shapes.
	/// This may be greater than 1 for convex-vs-concave shapes. Each
	/// manifold holds up to two contact points with a shared contact normal.
	public GetManifoldCount(): number /** int */
	{
		return this.m_manifoldCount;
	}

	/// Is this contact solid?
	/// @return true if this contact should generate a response.
	public IsSolid(): boolean {
		return (this.m_flags & b2Contact.e_nonSolidFlag) == 0;
	}

	/// Get the next contact in the world's contact list.
	public GetNext(): b2Contact {
		return this.m_next;
	}

	/// Get the first shape in this contact.
	public GetShape1(): b2Shape {
		return this.m_shape1;
	}

	/// Get the second shape in this contact.
	public GetShape2(): b2Shape {
		return this.m_shape2;
	}

	//--------------- Internals Below -------------------

	// m_flags
	// enum
	public static e_nonSolidFlag: number /** uint */	= 0x0001;
	public static e_slowFlag: number /** uint */		= 0x0002;
	public static e_islandFlag: number /** uint */		= 0x0004;
	public static e_toiFlag: number /** uint */		= 0x0008;

	public static AddType(createFcn: Function, destroyFcn: Function, type1: number /** int */, type2: number /** int */): void {
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);

		b2Contact.s_registers[type1][type2].createFcn = createFcn;
		b2Contact.s_registers[type1][type2].destroyFcn = destroyFcn;
		b2Contact.s_registers[type1][type2].primary = true;

		if (type1 != type2) {
			b2Contact.s_registers[type2][type1].createFcn = createFcn;
			b2Contact.s_registers[type2][type1].destroyFcn = destroyFcn;
			b2Contact.s_registers[type2][type1].primary = false;
		}
	}

	constructor(s1: b2Shape = null, s2: b2Shape = null) {
		this.m_flags = 0;

		if (!s1 || !s2) {
			this.m_shape1 = null;
			this.m_shape2 = null;
			return;
		}

		if (s1.IsSensor() || s2.IsSensor()) {
			this.m_flags |= b2Contact.e_nonSolidFlag;
		}

		this.m_shape1 = s1;
		this.m_shape2 = s2;

		this.m_manifoldCount = 0;

		this.m_friction = Math.sqrt(this.m_shape1.m_friction * this.m_shape2.m_friction);
		this.m_restitution = b2Math.b2Max(this.m_shape1.m_restitution, this.m_shape2.m_restitution);

		this.m_prev = null;
		this.m_next = null;

		this.m_node1.contact = null;
		this.m_node1.prev = null;
		this.m_node1.next = null;
		this.m_node1.other = null;

		this.m_node2.contact = null;
		this.m_node2.prev = null;
		this.m_node2.next = null;
		this.m_node2.other = null;
	}

	public Update(listener: b2ContactListener): void {
		const oldCount: number /** int */ = this.m_manifoldCount;

		this.Evaluate(listener);

		const newCount: number /** int */ = this.m_manifoldCount;

		const body1: b2Body = this.m_shape1.m_body;
		const body2: b2Body = this.m_shape2.m_body;

		if (newCount == 0 && oldCount > 0) {
			body1.WakeUp();
			body2.WakeUp();
		}

		// Slow contacts don't generate TOI events.
		if (body1.IsStatic() || body1.IsBullet() || body2.IsStatic() || body2.IsBullet()) {
			this.m_flags &= ~b2Contact.e_slowFlag;
		} else {
			this.m_flags |= b2Contact.e_slowFlag;
		}
	}

	//virtual ~b2Contact() {}

	public Evaluate(listener: b2ContactListener): void {}
	public static s_registers: b2ContactRegister[][]; //[][]
	public static s_initialized: boolean = false;

	public m_flags: number /** uint */;

	// World pool and list pointers.
	public m_prev: b2Contact;
	public m_next: b2Contact;

	// Nodes for connecting bodies.
	public m_node1: b2ContactEdge = new b2ContactEdge();
	public m_node2: b2ContactEdge = new b2ContactEdge();

	public m_shape1: b2Shape;
	public m_shape2: b2Shape;

	public m_manifoldCount: number /** int */;

	// Combined friction
	public m_friction: number;
	public m_restitution: number;

	public m_toi: number;

}