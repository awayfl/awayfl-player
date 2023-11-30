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


public class b2PolyAndConcaveArcContact extends b2PolygonContact
{
	static public function Create(shape1:b2Shape, shape2:b2Shape, allocator:*):b2Contact{
		//void* mem = allocator->Allocate(sizeof(b2PolyContact));
		return new b2PolyAndConcaveArcContact(shape1, shape2);
	}
	static public function Destroy(contact:b2Contact, allocator:*): void{
		//((b2PolyContact*)contact)->~b2PolyContact();
		//allocator->Free(contact, sizeof(b2PolyContact));
	}

	public function b2PolyAndConcaveArcContact(shape1:b2Shape, shape2:b2Shape): void{
		super(shape1, shape2);
	}
	//~b2PolyContact() {}

	//Flash only: Allow variable sized manifold field
	private function enlargeManifolds(l:Number):void{
		while(m_arcManifolds.length<l){
			var tMani:b2Manifold = new b2Manifold();
			tMani.pointCount=0;
			tMani.points[0].normalImpulse=0;
			tMani.points[0].tangentImpulse=0;
			m_arcManifolds.push(tMani);
		}
	}
	
	public override function Evaluate(listener:b2ContactListener): void{
		
		var b1:b2Body = m_shape1.m_body;
		var b2:b2Body = m_shape2.m_body;
		
		var b2_nullFeature:uint=b2Collision.b2_nullFeature;
		//var cornerMatchTol=b2Settings.b2_linearSlop;
		var i:int,j:int;
		var tMani:b2Manifold;
		
		//Store previous impulses
		//Flash only: Using dictionary for contact matching
		//TODO: Avoid creating or destroying objects here, as in super()
		var impulses:Array=[];
		var cp:b2ManifoldPoint,cp0:b2ManifoldPoint;
		var singleKey:Number;
		if(!m_arcColl)
		{
			for(i=0; i<m_manifoldCount; i++)
			{
				for(j=0; j<m_manifolds[i].pointCount; j++)
				{
					cp0 = m_manifolds[i].points[j];
					impulses[cp0.id.key] = cp = new b2ManifoldPoint();
					cp.normalImpulse = cp0.normalImpulse;
					cp.tangentImpulse = cp0.tangentImpulse;
				}
			}
			
			if(m_manifoldCount==1 && m_manifolds[0].pointCount==1)
			{
				singleKey = m_manifolds[0].points[0].id.key;
			}else{
				singleKey = -1;
			}
		}else{
			for(i=0; i<m_manifoldCount; i++)
			{
				for(j=0; j<m_arcManifolds[i].pointCount; j++)
				{
					cp0 = m_arcManifolds[i].points[j];
					impulses[cp0.id.key] = cp = new b2ManifoldPoint();
					cp.normalImpulse = cp0.normalImpulse;
					cp.tangentImpulse = cp0.tangentImpulse;
				}
			}
			
			if(m_manifoldCount==1 && m_arcManifolds[0].pointCount==1)
			{
				singleKey = m_arcManifolds[0].points[0].id.key;
			}else{
				singleKey = -1;
			}			
		}
		
		//Do ordinary poly collision
		super.Evaluate(listener);
		
		var polyManiCount:Number = m_manifoldCount;
		//The poly bounds the arc anyway
		if(m_manifoldCount==0)
			return;
		
		cp = m_manifolds[0].points[0];
		var features:Features = cp.id.features;
		var edge:Number = features.flip?features.referenceEdge:features.incidentEdge;
		var polySep:Number = -Number.MAX_VALUE;
		if(m_manifoldCount==1 && edge!=0 && cp.separation<0)
			polySep = cp.separation;
		
		//Do arc vs poly collision
		{
			//We have hit the arc edge, so we now test against the arc
			
			var poly:b2PolygonShape = m_shape1 as b2PolygonShape;
			var arc:b2ConcaveArcShape = m_shape2 as b2ConcaveArcShape;
			
			var arcSep:Number = Number.MAX_VALUE;
			
			// Compute circle position in the frame of the polygon.
			//TODO: Inline the calculations
			
			//From local in arc to local in poly v= poly.R^T*(arc.R * v+arc.pos-poly.pos)
			var localCenter:b2Vec2 = b2Math.b2MulXT(b1.m_xf, b2Math.b2MulX(b2.m_xf,arc.m_arcCenter))
			var local0:b2Vec2 = b2Math.b2MulX(b2.m_xf,arc.m_vertices[0])
			var world0:b2Vec2 = local0.Copy();
			local0 = b2Math.b2MulXT(b1.m_xf,local0);
			var local1:b2Vec2 = b2Math.b2MulX(b2.m_xf,arc.m_vertices[1])
			var world1:b2Vec2 = local1.Copy();
			local1 = b2Math.b2MulXT(b1.m_xf,local1);	
			var localNorm:b2Vec2 = b2Math.b2MulTMV(b1.m_xf.R,b2Math.b2MulMV(b2.m_xf.R,arc.m_normals[0].Copy()));
			
			var worlds:Array = [world0,world1];
			var locals:Array = [local0,local1];
			
			var strictMode:Boolean = false;
			
			m_manifoldCount = 0;
			//Used for when we are investigating near ends of the arc, but find a vx collision that is not actually on the end. Stores the vx
			var autoApprove:Array = [-1,-1];
			//If we have found a collision near the ends, we can use that information later to restrict collisions not on the end. Stores the edge
			var arcLimitations:Array = [-1,-1];
			
			//Find a collision between the ends of the arc and the poly
			//We do this by treating each end as if it were a wedge extending to infinity
			//Where one side of the wedge is colinear with the flat side of the end
			//And the other side is the tangent to the arc.
			//We then do a simplified version of PolyContact
			if(edge==0)
			for(i=0;i<2;i++)
			{
				var maxS:Number = -Number.MAX_VALUE;
				var maxEdge:Number = -1;
				var local:b2Vec2 = locals[i];
				var s:Number;
				var arcNormal1:b2Vec2,arcNormal2:b2Vec2;
				if(i==0)
				{
					arcNormal1 = b2Math.b2MulTMV(b1.m_xf.R,b2Math.b2MulMV(b2.m_xf.R,arc.m_normals[arc.m_vertexCount-1]));
					arcNormal2 = b2Math.SubtractVV(localCenter,local);
					arcNormal2.Normalize();
				}else{
					arcNormal2 = b2Math.b2MulTMV(b1.m_xf.R,b2Math.b2MulMV(b2.m_xf.R,arc.m_normals[1]));
					arcNormal1 = b2Math.SubtractVV(localCenter,local);
					arcNormal1.Normalize();
				}
				
				//Check for wedge stabbing the poly
				//FindMaxSeparation
				for(j=0;j<poly.m_vertexCount;j++)
				{
					//Check that the edge is angled correctly
					var polyVx:b2Vec2 = poly.m_vertices[j]
					var polyNormal:b2Vec2 = poly.m_normals[j];
					
					if(polyNormal.x*arcNormal1.y-polyNormal.y*arcNormal1.x<0)
						continue;
					if(polyNormal.x*arcNormal2.y-polyNormal.y*arcNormal2.x>0)
						continue;
					
					var v2x:Number = local.x-polyVx.x;
					var v2y:Number = local.y-polyVx.y;
						
					//Get separation
					s = v2x*polyNormal.x+v2y*polyNormal.y;
					if(s>0)
					{
						maxEdge = -1;
						break; 
					}
					if(s>maxS)
					{
						maxS = s;
						maxEdge = j;
					}
				}
				
				if(maxEdge!=-1)
				{
					polyNormal = poly.m_normals[maxEdge];
					polyVx = poly.m_vertices[maxEdge];
					var polyVx2:b2Vec2 = poly.m_vertices[maxEdge+1<poly.m_vertexCount?maxEdge+1:0];
					
					//FindIncidentEdge
					var dot1:Number = polyNormal.x*arcNormal1.x+polyNormal.y*arcNormal1.y;
					var dot2:Number = polyNormal.x*arcNormal2.x+polyNormal.y*arcNormal2.y;
					var tangent:b2Vec2;
					if(dot1<dot2)
					{
						tangent = b2Math.b2CrossVF(arcNormal1,1);
					}else{
						tangent = b2Math.b2CrossVF(arcNormal2,-1);
					}
					//Clip
					var t1:Number = (local.x-polyVx.x)*polyNormal.x+(local.y-polyVx.y)*polyNormal.y;
					var t2:Number = tangent.x*polyNormal.x+tangent.y*polyNormal.y;
					var t3:Number = -t1/t2;
					v2x = polyVx2.x-polyVx.x;
					v2y = polyVx2.y-polyVx.y;
					var t4:Number = (local.x+t3*tangent.x-polyVx.x)*v2x+(local.y+t3*tangent.y-polyVx.y)*v2y;
					if(t4<0 || t4>v2x*v2x+v2y*v2y){
						maxEdge=-1;
					}
				}
				
				//Technically, we should check for the poly stabbing the wedge case as well
				//But this has problems about treating the wedge as infinite
				//Instead we do a cheaper test. It's not so much wrong as just different from PolyContact
				if(maxEdge!=-1)
				{
					arcLimitations[i] = maxEdge;
					if(i==0)
					{
						s = (polyVx2.x-local.x)*arcNormal2.x+(polyVx2.y-local.y)*arcNormal2.y;
						if(maxS<s&&s<0)
						{
							autoApprove[i] = maxEdge+1<poly.m_vertexCount?maxEdge+1:0;
							maxEdge = -1;
						}
					}else{
						s = (polyVx.x-local.x)*arcNormal1.x+(polyVx.y-local.y)*arcNormal1.y;
						if(maxS<s&&s<0)
						{
							autoApprove[i] = maxEdge;
							maxEdge = -1;
						}
					}
				}
				
				
				if(maxEdge!=-1){
					//Generate a collision
					m_manifoldCount++;
					enlargeManifolds(m_manifoldCount);
					tMani = m_arcManifolds[m_manifoldCount-1];
					tMani.pointCount = 1;
					var tPoint:b2ManifoldPoint = tMani.points[0];
					tPoint.localPoint1.SetV(locals[i]);
					tPoint.localPoint2.SetV(arc.m_vertices[i])
					tPoint.id.features.incidentEdge = b2_nullFeature;
					tPoint.id.features.incidentVertex = i;
					tPoint.id.features.referenceEdge = maxEdge;
					tPoint.id.features.flip = 0;
					tPoint.separation = maxS;
					tPoint.normalImpulse = 0;
					tPoint.tangentImpulse = 0;
					tMani.normal = b2Math.b2MulMV(b1.m_xf.R, poly.m_normals[maxEdge]);
					
					arcSep = Math.min(arcSep,tPoint.separation);
					
				}
			}
			
			//Find collision. between poly and the main part of arc
			//Note due to concavity, we can have a seperate manifold for each point
			//On the other hand, we will have no two point manifolds, which makes life easier
			
			var bool1:Boolean;
			var tVec:b2Vec2;
			var topVx:b2Vec2
			var d2:Number,d:Number,s1:Number,s2:Number;
			var l:Number;
			
			var direction:Number;
			var currVx:Number,startVx:Number;
			
			if(arcLimitations[1]==-1)
			{
				if(arcLimitations[0]==-1)
				{
					direction = 1;
					startVx = 0;
				}else{
					direction = poly.m_vertexCount-1;//I.e. -1 mod poly.m_vertexCount
					startVx = arcLimitations[0]+1==poly.m_vertexCount?0:arcLimitations[0]+1;
				}
			}else{
				direction = 1;
				startVx = arcLimitations[1];
			}
			
			//We restrict our matches to one contiguous block
			var foundBlock: Boolean =false;
			
			//direction = 1;
			//startVx = 0;
			
			currVx=startVx;
			do
			{
				//Innocent until proven guilty
				bool1=true;
				//Test vs arcLimitations
				tVec = poly.m_vertices[currVx];
				if(arcLimitations[0]!=-1)
				{
					l = arcLimitations[0];
					topVx = poly.m_vertices[l+1==poly.m_vertexCount?0:l+1];
					bool1 = bool1 && ( (tVec.x-topVx.x)*poly.m_normals[l].y-(tVec.y-topVx.y)*poly.m_normals[l].x>0 )
				}
				if(arcLimitations[1]!=-1)
				{
					l = arcLimitations[1];
					topVx = poly.m_vertices[l];
					bool1 = bool1 && ( (tVec.x-topVx.x)*poly.m_normals[l].y-(tVec.y-topVx.y)*poly.m_normals[l].x<0 )
				}
				
				if(foundBlock&&!bool1)
				{
					//Make this loop the last, just incase it's autoapprove
					startVx = (currVx+direction) % poly.m_vertexCount;
				}
				if(bool1) foundBlock=true;
				
				//Test vs __^__
				tVec = new b2Vec2(poly.m_vertices[currVx].x-localCenter.x,poly.m_vertices[currVx].y-localCenter.y);
				d2=tVec.x*tVec.x+tVec.y*tVec.y;
				d = Math.sqrt(d2);
				s1 = arc.m_radius-d;
				s2 = (poly.m_vertices[currVx].x-local0.x)*localNorm.x+(poly.m_vertices[currVx].y-local0.y)*localNorm.y;
				s = Math.max(s1,s2);
				arcSep = Math.min(arcSep, s);
				
				bool1 = bool1 && (s<0)
				
				
				
				if(bool1||currVx==autoApprove[0]||currVx==autoApprove[1])
				{
					
					m_manifoldCount++;
					enlargeManifolds(m_manifoldCount);
					tMani=m_arcManifolds[m_manifoldCount-1];
					tMani.pointCount=1;
					tPoint = tMani.points[0];
					tPoint.id.features.incidentEdge = b2_nullFeature;
					tPoint.id.features.incidentVertex = currVx;
					tPoint.id.features.referenceEdge = 0;
					tPoint.id.features.flip = 0;
					
					tPoint.normalImpulse = 0;
					tPoint.tangentImpulse = 0;
					
					tVec.x /= d;
					tVec.y /= d;
					
					var tMat:b2Mat22 = b1.m_xf.R;
					tMani.normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
					tMani.normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;		
					
					tVec=poly.m_vertices[currVx];
					//tVec=new b2Vec2(localCenter.x,localCenter.y);//Use this instead?
					tPoint.localPoint1.SetV(tVec);
					tPoint.localPoint2.SetV(b2Math.b2MulXT(b2.m_xf,b2Math.b2MulX(b1.m_xf,tVec)))
					tPoint.separation = arc.m_radius-d;
					
				}
				currVx = (currVx+direction) % poly.m_vertexCount;
			}while(currVx!=startVx)
		}
		var arcManiCount:int=m_manifoldCount;
		if(polySep>arcSep)
		{
			m_arcColl = false;
			m_manifoldCount = polyManiCount;
			//Match contact points
			for(i=0;i<m_manifoldCount;i++)
			{
				for(j=0;j<m_manifolds[i].pointCount;j++)
				{
					cp=m_manifolds[i].points[j];
					cp0=impulses[cp.id.key]
					if(cp0)
					{
						cp.normalImpulse=cp0.normalImpulse;
						cp.tangentImpulse=cp0.tangentImpulse;
					}
				}
			}
			
			if(singleKey!=-1&&m_manifoldCount==1&&m_manifolds[0].pointCount==1){
				cp=m_manifolds[0].points[0];
				cp0=impulses[singleKey]
				if(cp0)
				{
					cp.normalImpulse=cp0.normalImpulse;
					cp.tangentImpulse=cp0.tangentImpulse;
				}				
			}			
		}else{
			m_arcColl = true;
			m_manifoldCount = arcManiCount;
			//Match contact points
			for(i=0;i<m_manifoldCount;i++)
			{
				for(j=0;j<m_arcManifolds[i].pointCount;j++)
				{
					cp=m_arcManifolds[i].points[j];
					cp0=impulses[cp.id.key]
					if(cp0)
					{
						cp.normalImpulse=cp0.normalImpulse;
						cp.tangentImpulse=cp0.tangentImpulse;
					}
				}
			}
			
			if(singleKey!=-1&&m_manifoldCount==1&&m_arcManifolds[0].pointCount==1){
				cp=m_arcManifolds[0].points[0];
				cp0=impulses[singleKey]
				if(cp0)
				{
					cp.normalImpulse=cp0.normalImpulse;
					cp.tangentImpulse=cp0.tangentImpulse;
				}				
			}			
		}
	}
	
	public override function GetManifolds():Array
	{
		if(m_arcColl)
				return m_arcManifolds;
		else	return m_manifolds;
	}
	
	//Manifolds to return if collision with arc
	private var m_arcManifolds:Array=[];
	//Is collision with arc part, or poly part?
	private var m_arcColl:Boolean;
};

}
