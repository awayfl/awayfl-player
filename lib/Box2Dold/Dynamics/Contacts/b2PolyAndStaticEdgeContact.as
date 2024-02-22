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

package Box2D.Dynamics.Contacts{


import Box2D.Collision.Shapes.*;
import Box2D.Collision.*;
import Box2D.Dynamics.*;
import Box2D.Common.*;
import Box2D.Common.Math.*;


public class b2PolyAndStaticEdgeContact extends b2Contact{
	
	static public function Create(shape1:b2Shape, shape2:b2Shape, allocator:*):b2Contact{
		return new b2PolyAndStaticEdgeContact(shape1, shape2);
	}
	static public function Destroy(contact:b2Contact, allocator:*): void{
	}

	public function b2PolyAndStaticEdgeContact(shape1:b2Shape, shape2:b2Shape){
		super(shape1, shape2);
		
		m_manifold = m_manifolds[0];
		
		//b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_polygonShape);
		//b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_staticEdgeShape);
		m_manifold.pointCount = 0;
		var point:b2ManifoldPoint = m_manifold.points[0];
		point.normalImpulse = 0.0;
		point.tangentImpulse = 0.0;
	}
	//~b2PolyAndStaticEdgeContact() {}

	//
	static public const s_evalCP:b2ContactPoint = new b2ContactPoint();
	public override function Evaluate(listener:b2ContactListener): void{
		var i:int;
		var v1:b2Vec2;
		var v2:b2Vec2;
		var mp0:b2ManifoldPoint;
		
		var b1:b2Body = m_shape1.m_body;
		var b2:b2Body = m_shape2.m_body;
		
		//b2Manifold m0;
		//memcpy(&m0, &m_manifold, sizeof(b2Manifold));
		// TODO: make sure this is completely necessary
		m0.Set(m_manifold);
		
		b2CollidePolygonAndStaticEdge(m_manifold, m_shape1 as b2PolygonShape, b1.m_xf, m_shape2 as b2StaticEdgeShape, b2.m_xf);
		
		var persisted:Array = [false, false];
		
		var cp:b2ContactPoint = s_evalCP;
		cp.shape1 = m_shape1;
		cp.shape2 = m_shape2;
		cp.friction = m_friction;
		cp.restitution = m_restitution;
		
		// Match contact ids to facilitate warm starting.
		if (m_manifold.pointCount > 0)
		{
			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (i = 0; i < m_manifold.pointCount; ++i)
			{
				var mp:b2ManifoldPoint = m_manifold.points[ i ];
				mp.normalImpulse = 0.0;
				mp.tangentImpulse = 0.0;
				var found:Boolean = false;
				var idKey:uint = mp.id._key;
	
				for (var j:int = 0; j < m0.pointCount; ++j)
				{
					if (persisted[j] == true)
					{
						continue;
					}
	
					mp0 = m0.points[ j ];
	
					if (mp0.id._key == idKey)
					{
						persisted[j] = true;
						mp.normalImpulse = mp0.normalImpulse;
						mp.tangentImpulse = mp0.tangentImpulse;
	
						// A persistent point.
						found = true;
	
						// Report persistent point.
						if (listener != null)
						{
							cp.position = b1.GetWorldPoint(mp.localPoint1);
							v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
							v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
							cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
							cp.normal.SetV(m_manifold.normal);
							cp.separation = mp.separation;
							cp.id.key = idKey;
							listener.Persist(cp);
						}
						break;
					}
				}
	
				// Report added point.
				if (found == false && listener != null)
				{
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.normal.SetV(m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.key = idKey;
					listener.Add(cp);
				}
			}
	
			m_manifoldCount = 1;
		}
		else
		{
			m_manifoldCount = 0;
		}
		
		if (listener == null)
		{
			return;
		}
		
		// Report removed points.
		for (i = 0; i < m0.pointCount; ++i)
		{
			if (persisted[i])
			{
				continue;
			}
			
			mp0 = m0.points[ i ];
			cp.position = b1.GetWorldPoint(mp0.localPoint1);
			v1 = b1.GetLinearVelocityFromLocalPoint(mp0.localPoint1);
			v2 = b2.GetLinearVelocityFromLocalPoint(mp0.localPoint2);
			cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
			cp.normal.SetV(m0.normal);
			cp.separation = mp0.separation;
			cp.id.key = mp0.id._key;
			listener.Remove(cp);
		}
	}
	
	public override function GetManifolds():Array
	{
		return m_manifolds;
	}

	public var m_manifolds:Array = [new b2Manifold()];
	public var m_manifold:b2Manifold;
	public var m0:b2Manifold = new b2Manifold();
	
	
	private static const k_slop:Number = 100.0 * Number.MIN_VALUE;
	
	
	static public function b2CollidePolygonAndStaticEdge(
		manifold:b2Manifold, 
		polygon:b2PolygonShape, xf1:b2XForm,
		edge:b2StaticEdgeShape, xf2:b2XForm) : void
	{
		manifold.pointCount = 0;
		var tPoint:b2ManifoldPoint;
		var tVec:b2Vec2;
		var tVec2:b2Vec2;
		var tMat:b2Mat22;
		var dX:Number;
		var dY:Number;
		
		// Convert the edge's two vertices and normal vector to the poly's space:
		tMat = xf1.R;
		
		dX = edge.m_v1.x - xf1.position.x;
		dY = edge.m_v1.y - xf1.position.y;
		var v1x:Number = (dX * tMat.col1.x + dY * tMat.col1.y);
		var v1y:Number = (dX * tMat.col2.x + dY * tMat.col2.y);
		
		dX = edge.m_v2.x - xf1.position.x;
		dY = edge.m_v2.y - xf1.position.y;
		var v2x:Number = (dX * tMat.col1.x + dY * tMat.col1.y);
		var v2y:Number = (dX * tMat.col2.x + dY * tMat.col2.y);
		
		dX = edge.m_normal.x;
		dY = edge.m_normal.y;
		var enx:Number = (dX * tMat.col1.x + dY * tMat.col1.y);
		var eny:Number = (dX * tMat.col2.x + dY * tMat.col2.y);
		
		var separation1: Number;
		var separation2: Number;
		var separationMax: Number = -Infinity;
		var separationV1: Boolean;
		var separationIndex: int;
		
		var vertexCount:int = polygon.m_vertexCount;
		var vertices:Array = polygon.m_vertices;
		var normals:Array = polygon.m_normals;
		
		var prevEN: Number;
		var nextEN: Number;
		var enterStartIndex: int = -1;
		var enterEndIndex: int = -1;
		var exitStartIndex: int = -1;
		var exitEndIndex: int = -1;
		var enterEN: Number;
		var exitEN: Number;
		var deepestEN: Number = Infinity;
		var deepestIndex: int;

		tVec = vertices[vertexCount-1];
		dX = tVec.x-v1x;
		dY = tVec.y-v1y;
		prevEN = dX * enx + dY * eny;
		
		for (var i:int = 0; i < vertexCount; i++)
		{
			tVec = vertices[i];
			tVec2 = normals[i];
			
			dX = v1x - tVec.x;
			dY = v1y - tVec.y;
			separation1 = dX * tVec2.x + dY * tVec2.y;
			
			dX = v2x - tVec.x;
			dY = v2y - tVec.y;
			separation2 = dX * tVec2.x + dY * tVec2.y;
			
			if (separation2 < separation1) {
				if (separation2 > separationMax) {
					separationMax = separation2;
					separationV1 = false;
					separationIndex = i;
				}
			} else {
				if (separation1 > separationMax) {
					separationMax = separation1;
					separationV1 = true;
					separationIndex = i;
				}
			}
			
			nextEN = -(dX * enx + dY * eny);
			if (nextEN >= 0 && prevEN < 0) {
				exitStartIndex = (i == 0) ? vertexCount-1 : i-1;
				exitEndIndex = i;
				exitEN = prevEN;
			} else if (nextEN < 0 && prevEN >= 0) {
				enterStartIndex = (i == 0) ? vertexCount-1 : i-1;
				enterEndIndex = i;
				enterEN = nextEN;
			}
			if (nextEN < deepestEN) {
				deepestEN = nextEN;
				deepestIndex = i;
			}
			prevEN = nextEN;
		}
		
		if (enterStartIndex == -1) {
			// poly is entirely below or entirely above edge, return with no contact:
			return;
		}
		if (separationMax > 0) {
			// poly is laterally disjoint with edge, return with no contact:
			return;
		}
		
		if (separationMax > deepestEN + b2Settings.b2_linearSlop) {
			tVec2 = normals[separationIndex];
			var ecx:Number;
			var ecy:Number;
			if (separationV1) {
				dX = edge.m_cornerDir1.x;
				dY = edge.m_cornerDir1.y;
				ecx = (dX * tMat.col1.x + dY * tMat.col1.y);
				ecy = (dX * tMat.col2.x + dY * tMat.col2.y);
			} else {
				dX = edge.m_cornerDir2.x;
				dY = edge.m_cornerDir2.y;
				ecx = -(dX * tMat.col1.x + dY * tMat.col1.y);
				ecy = -(dX * tMat.col2.x + dY * tMat.col2.y);
			}
			if (tVec2.x * ecx + tVec2.y * ecy >= 0) {
				// -normal angle is closer to adjacent edge than this edge, ignore it:
				return;
			}
			
			manifold.pointCount = 1;
			
			manifold.normal.Set(tMat.col1.x * tVec2.x + tMat.col2.x * tVec2.y,
			                    tMat.col1.y * tVec2.x + tMat.col2.y * tVec2.y);
			
			tPoint = manifold.points[0];
			tPoint.id.features.incidentEdge = separationIndex;
			tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
			tPoint.id.features.referenceEdge = 0;
			tPoint.id.features.flip = 0;
			
			if (separationV1) {
				tPoint.localPoint1.Set(v1x, v1y);
				tPoint.localPoint2.SetV(edge.m_v1);
			} else {
				tPoint.localPoint1.Set(v2x, v2y);
				tPoint.localPoint2.SetV(edge.m_v2);
			}
			
			tPoint.separation = separationMax;
		} else {
			manifold.normal.Set(-edge.m_normal.x, -edge.m_normal.y);
			tVec = vertices[enterEndIndex];
			
			// Check whether we only need one contact point.
			if (enterEndIndex == exitStartIndex) {
				manifold.pointCount = 1;
				tPoint = manifold.points[0];
				tPoint.id.features.incidentEdge = enterEndIndex;
				tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
				tPoint.id.features.referenceEdge = 0;
				tPoint.id.features.flip = 0;
				tPoint.localPoint1.SetV(tVec);
				tPoint.localPoint2.Set(xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y),
									   xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y));
				tPoint.separation = enterEN;
				return;
			}
			
			manifold.pointCount = 2;
			dX = edge.m_direction.x;
			dY = edge.m_direction.y;
			var edx:Number = (dX * tMat.col1.x + dY * tMat.col1.y);
			var edy:Number = (dX * tMat.col2.x + dY * tMat.col2.y);
			var ed1: Number = edx * (tVec.x - v1x) + edy * (tVec.y - v1y);
			var ed2: Number;
			exitEndIndex = (enterEndIndex == vertexCount - 1) ? 0 : enterEndIndex + 1;
			if (exitEndIndex != exitStartIndex) {
				exitStartIndex = exitEndIndex;
				tVec2 = vertices[exitStartIndex];
				dX = tVec2.x - v1x;
				dY = tVec2.y - v1y;
				exitEN = enx * dX + eny * dY;
				ed2 = edx * dX + edy * dY;
			} else {
				tVec2 = vertices[exitStartIndex];
				ed2 = edx * (tVec2.x - v1x) + edy * (tVec2.y - v1y);
			}
			
			
			tPoint = manifold.points[0];
			tPoint.id.features.incidentEdge = enterEndIndex;
			tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
			tPoint.id.features.referenceEdge = 0;
			tPoint.id.features.flip = 0;
			
			
			if (ed1 > edge.m_length) {
				tPoint.localPoint1.Set(v2x, v2y);
				tPoint.localPoint2.SetV(edge.m_v2);
				dX = (edge.m_length - ed2) / (ed1 - ed2);
				if (dX > 100 * Number.MIN_VALUE && dX <1) {
					tPoint.separation = exitEN * (1-dX) + enterEN * dX;
				} else {
					tPoint.separation = enterEN;
				}
			} else {
				tPoint.localPoint1.SetV(tVec);
				tPoint.localPoint2.Set(xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y),
									   xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y));
				tPoint.separation = enterEN;
			}
			
			tPoint = manifold.points[1];
			tPoint.id.features.incidentEdge = exitStartIndex;
			tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
			tPoint.id.features.referenceEdge = 0;
			tPoint.id.features.flip = 0;
			
			if (ed2 < 0) {
				tPoint.localPoint1.Set(v1x, v1y);
				tPoint.localPoint2.SetV(edge.m_v1);
				dX = (-ed1) / (ed2 - ed1);
				if (dX > 100 * Number.MIN_VALUE && dX <1) {
					tPoint.separation = enterEN * (1-dX) + exitEN * dX;
				} else {
					tPoint.separation = exitEN;
				}
			} else {
				tPoint.localPoint1.SetV(tVec2);
				tPoint.localPoint2.Set(xf1.position.x + (tMat.col1.x * tVec2.x + tMat.col2.x * tVec2.y),
				                       xf1.position.y + (tMat.col1.y * tVec2.x + tMat.col2.y * tVec2.y));
				tPoint.separation = exitEN;
			}
			return;
		}
	}
	
}

}

