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

import { b2CircleShape } from "../../Collision/Shapes/b2CircleShape";
import { b2Shape } from "../../Collision/Shapes/b2Shape";
import { b2StaticEdgeShape } from "../../Collision/Shapes/b2StaticEdgeShape";
import { b2ContactPoint } from "../../Collision/b2ContactPoint";
import { b2Manifold } from "../../Collision/b2Manifold";
import { b2ManifoldPoint } from "../../Collision/b2ManifoldPoint";
import { b2Mat22, b2Vec2, b2XForm } from "../../Common/Math";
import { b2Body } from "../b2Body";
import { b2ContactListener } from "../b2ContactListener";
import { b2Contact } from "./b2Contact";

export class b2StaticEdgeAndCircleContact extends b2Contact
{
	public static Create(shape1:b2Shape, shape2:b2Shape, allocator:any):b2Contact{
		return new b2StaticEdgeAndCircleContact(shape1, shape2);
	}
	public static Destroy(contact:b2Contact, allocator:any) : void{
	}

	constructor(shape1:b2Shape, shape2:b2Shape){
		super(shape1, shape2);
		
		this.m_manifold = this.m_manifolds[0];
		
		//b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_staticEdgeShape);
		//b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_circleShape);
		this.m_manifold.pointCount = 0;
		var point:b2ManifoldPoint = this.m_manifold.points[0];
		point.normalImpulse = 0.0;
		point.tangentImpulse = 0.0;
	}
	//~b2StaticEdgeAndCircleContact() {}
	
	private static s_evalCP:b2ContactPoint = new b2ContactPoint();
	public Evaluate(listener:b2ContactListener) : void{
		var v1:b2Vec2;
		var v2:b2Vec2;
		var mp0:b2ManifoldPoint;
		
		var b1:b2Body = this.m_shape1.m_body;
		var b2:b2Body = this.m_shape2.m_body;
		
		//b2Manifold m0;
		//memcpy(&m0, &m_manifold, sizeof(b2Manifold));
		// TODO: make sure this is completely necessary
		this.m0.Set(this.m_manifold);
		
		b2StaticEdgeAndCircleContact.b2CollideStaticEdgeAndCircle(this.m_manifold, this.m_shape1 as b2StaticEdgeShape, b1.m_xf, this.m_shape2 as b2CircleShape, b2.m_xf);
		
		var cp:b2ContactPoint = b2StaticEdgeAndCircleContact.s_evalCP;
		cp.shape1 = this.m_shape1;
		cp.shape2 = this.m_shape2;
		cp.friction = this.m_friction;
		cp.restitution = this.m_restitution;
		
		if (this.m_manifold.pointCount > 0)
		{
			this.m_manifoldCount = 1;
			var mp:b2ManifoldPoint = this.m_manifold.points[ 0 ];
			
			if (this.m0.pointCount == 0)
			{
				mp.normalImpulse = 0.0;
				mp.tangentImpulse = 0.0;
	
				if (listener)
				{
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.normal.SetV(this.m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.key = mp.id._key;
					listener.Add(cp);
				}
			} else
			{
				mp0 = this.m0.points[ 0 ];
				mp.normalImpulse = mp0.normalImpulse;
				mp.tangentImpulse = mp0.tangentImpulse;
				
				if (listener)
				{
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.normal.SetV(this.m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.key = mp.id._key;
					listener.Persist(cp);
				}
			}
		}
		else
		{
			this.m_manifoldCount = 0;
			if (this.m0.pointCount > 0 && listener)
			{
				mp0 = this.m0.points[ 0 ];
				cp.position = b1.GetWorldPoint(mp0.localPoint1);
				v1 = b1.GetLinearVelocityFromLocalPoint(mp0.localPoint1);
				v2 = b2.GetLinearVelocityFromLocalPoint(mp0.localPoint2);
				cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
				cp.normal.SetV(this.m0.normal);
				cp.separation = mp0.separation;
				cp.id.key = mp0.id._key;
				listener.Remove(cp);
			}
		}
	}
	
	public GetManifolds():b2Manifold[]
	{
		return this.m_manifolds;
	}

	private m_manifolds:b2Manifold[] = [new b2Manifold()];
	public m_manifold:b2Manifold;
	private m0:b2Manifold = new b2Manifold();







	public static b2CollideStaticEdgeAndCircle(
		manifold:b2Manifold, 
		edge:b2StaticEdgeShape, xf1:b2XForm, 
		circle:b2CircleShape, xf2:b2XForm) : void
	{
		manifold.pointCount = 0;
		
		var dX:number;
		var dY:number;
		var separation:number;
		var tPoint:b2ManifoldPoint;
		
		var tMat:b2Mat22 = xf2.R; 
		var tVec:b2Vec2 = circle.m_localPosition;
		var circleX:number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var circleY:number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		var dirDist:number = (circleX - edge.m_v1.x) * edge.m_direction.x + (circleY - edge.m_v1.y) * edge.m_direction.y;
		if (dirDist <= 0) {
			dX = circleX - edge.m_v1.x;
			dY = circleY - edge.m_v1.y;
			if (dX * edge.m_cornerDir1.x + dY * edge.m_cornerDir1.y < 0) {
				return;
			}
		} else if (dirDist >= edge.m_length) {
			dX = circleX - edge.m_v2.x;
			dY = circleY - edge.m_v2.y;
			if (dX * edge.m_cornerDir2.x + dY * edge.m_cornerDir2.y > 0) {
				return;
			}
		} else {
			tVec = edge.m_normal;
			separation = (circleX - edge.m_v1.x) * tVec.x + 
			             (circleY - edge.m_v1.y) * tVec.y;
			if (separation > circle.m_radius || separation < -circle.m_radius) {
				return;
			}
			separation -= circle.m_radius;
			manifold.normal.x = tVec.x;
			manifold.normal.y = tVec.y;
			manifold.pointCount = 1;
			tPoint = manifold.points[0];
			tPoint.id.key = 0;
			tPoint.separation = separation;
			circleX -= circle.m_radius * tVec.x;
			circleY -= circle.m_radius * tVec.y;
			tPoint.localPoint1.x = circleX;
			tPoint.localPoint1.y = circleY;
			dX = circleX - xf2.position.x;
			dY = circleY - xf2.position.y;
			tPoint.localPoint2.x = (dX * tMat.col1.x + dY * tMat.col1.y );
			tPoint.localPoint2.y = (dX * tMat.col2.x + dY * tMat.col2.y );
			return;
		}
		
		var distSqr:number = dX * dX + dY * dY;
		if (distSqr > circle.m_radius * circle.m_radius)
		{
			return;
		}
		
		if (distSqr < Number.MIN_VALUE)
		{
			separation = -circle.m_radius;
			manifold.normal.SetV(edge.m_normal);
		}
		else
		{
			var dist:number = Math.sqrt(distSqr);
			separation = dist - circle.m_radius;
			var a:number = 1.0 / dist;
			manifold.normal.x = a * dX;
			manifold.normal.y = a * dY;
		}
		
		manifold.pointCount = 1;
		tPoint = manifold.points[0];
		tPoint.id.key = 0;
		tPoint.separation = separation;
		
		circleX -= circle.m_radius * manifold.normal.x;
		circleY -= circle.m_radius * manifold.normal.y;
		
		tPoint.localPoint1.x = circleX;
		tPoint.localPoint1.y = circleY;
		dX = circleX - xf2.position.x;
		dY = circleY - xf2.position.y;
		tPoint.localPoint2.x = (dX * tMat.col1.x + dY * tMat.col1.y );
		tPoint.localPoint2.y = (dX * tMat.col2.x + dY * tMat.col2.y );
	}
}