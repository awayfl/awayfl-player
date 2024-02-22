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


import Box2D.Collision.Shapes.*
import Box2D.Collision.*
import Box2D.Dynamics.*
import Box2D.Common.*
import Box2D.Common.Math.*

public class b2ConcaveArcAndCircleContact extends b2PolyAndCircleContact
{
	static public function Create(shape1:b2Shape, shape2:b2Shape, allocator:*):b2Contact{
		return new b2ConcaveArcAndCircleContact(shape1, shape2);
	}
	static public function Destroy(contact:b2Contact, allocator:*) : void{
		//
	}

	public function b2ConcaveArcAndCircleContact(shape1:b2Shape, shape2:b2Shape){
		super(shape1, shape2);
	}
	//~b2CircleContact() {}
	
	//
	static private var s_evalCP:b2ContactPoint = new b2ContactPoint();
	
	//Edited version of b2CollidePolyAndCircle	
	static public function b2CollideConcaveArcAndCircle(
		manifolds:Array, 
		polygon:b2ConcaveArcShape, xf1:b2XForm,		//Note the type defying name
		circle:b2CircleShape, xf2:b2XForm) : Number
	{
		var conservative:Boolean = false;
		var b2_nullFeature:uint=b2Collision.b2_nullFeature;
		var manifold:b2Manifold = manifolds[0];
		var manifoldCount:Number = 0;
		
		manifold.pointCount = 0;
		var tPoint:b2ManifoldPoint;
		
		var dX:Number;
		var dY:Number;
		
		var tVec:b2Vec2;
		var tMat:b2Mat22;
		
		var positionX:Number;
		var positionY:Number;
		
		// Compute circle position in the frame of the polygon.
		//b2Vec2 c = b2Mul(xf2, circle->m_localPosition);
		tMat = xf2.R;
		tVec = circle.m_localPosition;
		var cX:Number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var cY:Number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		//b2Vec2 cLocal = b2MulT(xf1, c);
		dX = cX - xf1.position.x;
		dY = cY - xf1.position.y;
		tMat = xf1.R;
		var cLocalX:Number = (dX * tMat.col1.x + dY * tMat.col1.y)
		var cLocalY:Number = (dX * tMat.col2.x + dY * tMat.col2.y)
		
		var dist:Number;
		
		// Find the min separating edge.
		var normalIndex:int = 0;
		var separation:Number = -Number.MAX_VALUE;
		var radius:Number = circle.m_radius;
		for (var i:int = 1; i < polygon.m_vertexCount; ++i)
		{
			//float32 s = b2Dot(polygon->m_normals[i], cLocal - polygon->m_vertices[i]);
			dX = cLocalX-polygon.m_vertices[i].x;
			dY = cLocalY-polygon.m_vertices[i].y;
			var s:Number = polygon.m_normals[i].x * dX + polygon.m_normals[i].y * dY;
			
			if (s > radius)
			{
				// Early out.
				manifoldCount = 0;
				return manifoldCount;
			}
			
			if (s > separation)
			{
				separation = s;
				normalIndex = i;
			}
		}
		
		//Calculate the arc separation
		if( true || normalIndex==1 || normalIndex==(polygon.m_vertexCount-1))
		{
			
			s = polygon.m_normals[0].x * (cLocalX-polygon.m_vertices[0].x) + polygon.m_normals[0].y * (cLocalY-polygon.m_vertices[0].y);
			
			var c2X:Number = cLocalX-polygon.m_arcCenter.x;
			var c2Y:Number = cLocalY-polygon.m_arcCenter.y;
			var c2:Number = Math.sqrt(c2X*c2X+c2Y*c2Y);
			c2X /= c2;
			c2Y /= c2;
			
			s = Math.max(s,polygon.m_radius - c2);
			if (s > radius)
			{
				// Early out.
				manifoldCount = 0;
				return manifoldCount;
			}
			
			if (s > separation)
			{
				separation = s;
				normalIndex = 0;
			}				
		}
		
		if((normalIndex==0) && (radius >= polygon.m_radius))
		{
			//In this case, the sphere can have two contact points, m_vertices[0] and m_vertices[1]
			
			//Flash only: Defer creating second manifold until it is actually needed
			if(manifolds.length<2){
				manifolds[1]=new b2Manifold();
				manifolds[1].pointCount = 0;
				manifolds[1].points[0].normalImpulse = 0.0;
				manifolds[1].points[0].tangentImpulse = 0.0;
			}

			manifoldCount=0;
			for(i=0;i<2;i++){
				dX = polygon.m_vertices[i].x-cLocalX;
				dY = polygon.m_vertices[i].y-cLocalY;
				var d2:Number=dX*dX+dY*dY;
				if(d2<radius*radius){
					var d:Number = Math.sqrt(d2);
					
					manifolds[manifoldCount].pointCount=1;
					tPoint = manifolds[manifoldCount].points[0];
					tPoint.id.features.incidentEdge = b2_nullFeature;
					tPoint.id.features.incidentVertex = i;
					tPoint.id.features.referenceEdge = b2_nullFeature;
					tPoint.id.features.flip = 0;
					tPoint.separation = d - radius;
					
					tPoint.normalImpulse = 0;
					tPoint.tangentImpulse = 0;
					
					dX=-dX/d;
					dY=-dY/d;
					manifolds[manifoldCount].normal.x = tMat.col1.x * dX + tMat.col2.x * dY;
					manifolds[manifoldCount].normal.y = tMat.col1.y * dX + tMat.col2.y * dY;
					//This might be outside the poly
					//Perhaps it would be better to set the position to the vertex location
					//b2Vec2 position = c - radius * manifold->normal;
					positionX = cX - radius * manifolds[manifoldCount].normal.x;
					positionY = cY - radius * manifolds[manifoldCount].normal.y;
					//manifold->points[0].localPoint1 = b2MulT(xf1, position);
					dX = positionX - xf1.position.x;
					dY = positionY - xf1.position.y;
					tMat = xf1.R;
					tPoint.localPoint1.x = (dX*tMat.col1.x + dY*tMat.col1.y);
					tPoint.localPoint1.y = (dX*tMat.col2.x + dY*tMat.col2.y);
					//manifold->points[0].localPoint2 = b2MulT(xf2, position);
					dX = positionX - xf2.position.x;
					dY = positionY - xf2.position.y;
					tMat = xf2.R;
					tPoint.localPoint2.x = (dX*tMat.col1.x + dY*tMat.col1.y);
					tPoint.localPoint2.y = (dX*tMat.col2.x + dY*tMat.col2.y);
					
					manifoldCount++;
				}
			}

			
			return manifoldCount;
			
		}
		
		
		
		
		// If the center is inside the polygon ...
		if (separation < Number.MIN_VALUE)
		{
			manifold.pointCount = 1;
			manifoldCount = 1;
			if(normalIndex == 0){
				//manifold->normal = b2Mul(xf1.R, -c2^);
				tMat = xf1.R;
				manifold.normal.x = -(tMat.col1.x * c2X + tMat.col2.x * c2Y);
				manifold.normal.y = -(tMat.col1.y * c2X + tMat.col2.y * c2Y);		
			}else{
				//manifold->normal = b2Mul(xf1.R, polygon->m_normals[normalIndex]);
				tVec = polygon.m_normals[normalIndex];
				tMat = xf1.R;
				manifold.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				manifold.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			}
			
			tPoint = manifold.points[0];
			tPoint.id.features.incidentEdge = normalIndex;
			tPoint.id.features.incidentVertex = b2_nullFeature;
			tPoint.id.features.referenceEdge = 0;
			tPoint.id.features.flip = 0;
			//b2Vec2 position = c - radius * manifold->normal;
			positionX = cX - radius * manifold.normal.x;
			positionY = cY - radius * manifold.normal.y;
			//manifold->points[0].localPoint1 = b2MulT(xf1, position);
			dX = positionX - xf1.position.x;
			dY = positionY - xf1.position.y;
			tMat = xf1.R;
			tPoint.localPoint1.x = (dX*tMat.col1.x + dY*tMat.col1.y);
			tPoint.localPoint1.y = (dX*tMat.col2.x + dY*tMat.col2.y);
			//manifold->points[0].localPoint2 = b2MulT(xf2, position);
			dX = positionX - xf2.position.x;
			dY = positionY - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (dX*tMat.col1.x + dY*tMat.col1.y);
			tPoint.localPoint2.y = (dX*tMat.col2.x + dY*tMat.col2.y);
			
			tPoint.separation = separation - radius;
			return manifoldCount;
		}
		
		// Project the circle center onto the edge segment.
		var vertIndex1:int = normalIndex;
		var vertIndex2:int = vertIndex1 + 1 < polygon.m_vertexCount ? vertIndex1 + 1 : 0;
		//var e:b2Vec2 = b2Math.SubtractVV(polygon.m_vertices[vertIndex2] , polygon.m_vertices[vertIndex1]);
		var eX:Number = polygon.m_vertices[vertIndex2].x - polygon.m_vertices[vertIndex1].x;
		var eY:Number = polygon.m_vertices[vertIndex2].y - polygon.m_vertices[vertIndex1].y;
		//var length:Number = e.Normalize();
		var length:Number = Math.sqrt(eX*eX + eY*eY);
		eX /= length;
		eY /= length;
		
		// If the edge length is zero ...
		if (length < Number.MIN_VALUE)
		{
			//d = b2Math.SubtractVV(cLocal , polygon.m_vertices[vertIndex1]);
			dX = cLocalX - polygon.m_vertices[vertIndex1].x;
			dY = cLocalY - polygon.m_vertices[vertIndex1].y;
			//dist = d.Normalize();
			dist = Math.sqrt(dX*dX + dY*dY);
			dX /= dist;
			dY /= dist;
			if (dist > radius)
			{
				manifoldCount = 0;
				return manifoldCount;
			}
			
			manifold.pointCount = 1;
			manifoldCount = 1;
			//manifold->normal = b2Mul(xf1.R, d);
			tMat = xf1.R
			manifold.normal.x = (tMat.col1.x * dX + tMat.col2.x * dY);
			manifold.normal.y = (tMat.col1.y * dX + tMat.col2.y * dY);
			tPoint = manifold.points[0];
			tPoint.id.features.incidentEdge = b2_nullFeature;
			tPoint.id.features.incidentVertex = vertIndex1;
			tPoint.id.features.referenceEdge = b2_nullFeature;
			tPoint.id.features.flip = 0;
			//b2Vec2 position = c - radius * manifold->normal;
			positionX = cX - radius * manifold.normal.x;
			positionY = cY - radius * manifold.normal.y;
			//manifold->points[0].localPoint1 = b2MulT(xf1, position);
			dX = positionX - xf1.position.x;
			dY = positionY - xf1.position.y;
			tMat = xf1.R;
			tPoint.localPoint1.x = (dX*tMat.col1.x + dY*tMat.col1.y);
			tPoint.localPoint1.y = (dX*tMat.col2.x + dY*tMat.col2.y);
			//manifold->points[0].localPoint2 = b2MulT(xf2, position);
			dX = positionX - xf2.position.x;
			dY = positionY - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (dX*tMat.col1.x + dY*tMat.col1.y);
			tPoint.localPoint2.y = (dX*tMat.col2.x + dY*tMat.col2.y);
			tPoint.separation = dist - radius;
			return manifoldCount;
		}
		
		// Project the center onto the edge.
		//float32 u = b2Dot(cLocal - polygon->m_vertices[vertIndex1], e);
		dX = cLocalX - polygon.m_vertices[vertIndex1].x;
		dY = cLocalY - polygon.m_vertices[vertIndex1].y;
		
		
		tPoint = manifold.points[0];
		tPoint.id.features.incidentEdge = 0;
		tPoint.id.features.incidentVertex = 0;
		tPoint.id.features.referenceEdge = 0;
		tPoint.id.features.flip = 0;
		
		var pX:Number, pY:Number;
		
		if(normalIndex==0){
			var norm:Number=eX*c2X+eY*c2Y;
			//This may seem like it will always eval false, but this is not so for very large arcs (where norm ~=1)
			if (c2X*polygon.m_normals[0].x+c2Y*polygon.m_normals[0].y>0)
			{
				if (norm <0 ){
					pX = polygon.m_vertices[vertIndex1].x;
					pY = polygon.m_vertices[vertIndex1].y;
					tPoint.id.features.incidentVertex = vertIndex1;
					tPoint.id.features.incidentEdge = b2_nullFeature;
				}
				else
				{
					pX = polygon.m_vertices[vertIndex2].x;
					pY = polygon.m_vertices[vertIndex2].y;
					tPoint.id.features.incidentVertex = vertIndex2;
					tPoint.id.features.incidentEdge = b2_nullFeature;
				}
			}else{
				if (norm <= -polygon.m_norm){
					pX = polygon.m_vertices[vertIndex1].x;
					pY = polygon.m_vertices[vertIndex1].y;
					tPoint.id.features.incidentVertex = vertIndex1;
					tPoint.id.features.incidentEdge = b2_nullFeature;
				}
				else if (norm >= polygon.m_norm)
				{
					pX = polygon.m_vertices[vertIndex2].x;
					pY = polygon.m_vertices[vertIndex2].y;
					tPoint.id.features.incidentVertex = vertIndex2;
					tPoint.id.features.incidentEdge = b2_nullFeature;
				}
				else
				{
					//p = b2Math.AddVV(poly.m_vertices[vertIndex1] , b2Math.MulFV(u, e));
					pX = polygon.m_arcCenter.x + c2X * polygon.m_radius;
					pY = polygon.m_arcCenter.y + c2Y * polygon.m_radius;
					tPoint.id.features.incidentEdge = vertIndex1;
				}
			}
		}else{
			var u:Number = dX*eX + dY*eY;
			if (u <= 0.0)
			{
				pX = polygon.m_vertices[vertIndex1].x;
				pY = polygon.m_vertices[vertIndex1].y;
				tPoint.id.features.incidentVertex = vertIndex1;
				tPoint.id.features.incidentEdge = b2_nullFeature;
			}
			else if (u >= length)
			{
				pX = polygon.m_vertices[vertIndex2].x;
				pY = polygon.m_vertices[vertIndex2].y;
				tPoint.id.features.incidentVertex = vertIndex2;
				tPoint.id.features.incidentEdge = b2_nullFeature;
			}
			else
			{
				//p = polygon->m_vertices[vertIndex1] + u * e;
				pX = eX * u + polygon.m_vertices[vertIndex1].x;
				pY = eY * u + polygon.m_vertices[vertIndex1].y;
				tPoint.id.features.incidentEdge = vertIndex1;
			}
		}
		
		//d = b2Math.SubtractVV(xLocal , p);
		dX = cLocalX - pX;
		dY = cLocalY - pY;
		//dist = d.Normalize();
		dist = Math.sqrt(dX*dX + dY*dY);
		dX /= dist;
		dY /= dist;
		if (dist > radius)
		{
			manifoldCount = 0;
			return manifoldCount;
		}
		
		manifold.pointCount = 1;
		manifoldCount = 1;
		//manifold->normal = b2Mul(xf1.R, d);
		tMat = xf1.R;
		manifold.normal.x = tMat.col1.x * dX + tMat.col2.x * dY;
		manifold.normal.y = tMat.col1.y * dX + tMat.col2.y * dY;
		//b2Vec2 position = c - radius * manifold->normal;
		positionX = cX - radius * manifold.normal.x;
		positionY = cY - radius * manifold.normal.y;
		//manifold->points[0].localPoint1 = b2MulT(xf1, position);
		dX = positionX - xf1.position.x;
		dY = positionY - xf1.position.y;
		tMat = xf1.R;
		tPoint.localPoint1.x = (dX*tMat.col1.x + dY*tMat.col1.y);
		tPoint.localPoint1.y = (dX*tMat.col2.x + dY*tMat.col2.y);
		//manifold->points[0].localPoint2 = b2MulT(xf2, position);
		dX = positionX - xf2.position.x;
		dY = positionY - xf2.position.y;
		tMat = xf2.R;
		tPoint.localPoint2.x = (dX*tMat.col1.x + dY*tMat.col1.y);
		tPoint.localPoint2.y = (dX*tMat.col2.x + dY*tMat.col2.y);
		tPoint.separation = dist - radius;
		
		return manifoldCount;
		
	}
	
	public override function Evaluate(listener:b2ContactListener) : void{
		
		var i:int;
		var v1:b2Vec2;
		var v2:b2Vec2;
		var mp0:b2ManifoldPoint;
		
		var b1:b2Body = m_shape1.m_body;
		var b2:b2Body = m_shape2.m_body;
		
		//b2Manifold m0;
		//memcpy(&m0, &m_manifold, sizeof(b2Manifold));
		// TODO: make sure this is completely necessary
		m0.Set(m_manifolds[0]);
		//m1.Set(m_manifolds[1]);
		
		m_manifoldCount = b2CollideConcaveArcAndCircle(m_manifolds, m_shape1 as b2ConcaveArcShape, b1.m_xf, m_shape2 as b2CircleShape, b2.m_xf);
		
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

};

}