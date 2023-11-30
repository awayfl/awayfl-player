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

package Box2D.Collision.Shapes{



import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Collision.Shapes.*;
import Box2D.Dynamics.*;
import Box2D.Collision.*;



public class b2StaticEdgeChain
{
	public function b2StaticEdgeChain(def:b2ShapeDef, world: b2World){
		//b2Settings.b2Assert(def.type == e_staticEdgeShape);
		var edgeDef:b2StaticEdgeChainDef = def as b2StaticEdgeChainDef;
		
		isALoop = edgeDef.isALoop;
		
		var i: int;
		var v1: b2Vec2;
		var v2: b2Vec2;
		
		if (edgeDef.isALoop) {
			i = 0;
			v1 = edgeDef.vertices[edgeDef.vertexCount-1];
		} else {
			i = 1;
			v1 = edgeDef.vertices[0];
		}
		v1 = v1.Copy();
		
		for (; i < edgeDef.vertexCount; i++) {
			v2 = edgeDef.vertices[i];
			v2 = v2.Copy();
			
			//void* mem = world.m_blockAllocator->Allocate(sizeof(b2CircleShape));
			var s: b2StaticEdgeShape = new b2StaticEdgeShape(v1, v2, edgeDef);
			edges.push(s);
			s.m_chain = this;
			
			s.m_next = world.m_groundBody.m_shapeList;
			world.m_groundBody.m_shapeList = s;
			++world.m_groundBody.m_shapeCount;
			
			s.m_body = world.m_groundBody;
			
			// Add the shape to the world's broad-phase.
			s.CreateProxy(world.m_broadPhase, world.m_groundBody.m_xf);
			
			// Compute the sweep radius for CCD.
			s.UpdateSweepRadius(world.m_groundBody.m_sweep.localCenter);
			
			v1 = v2;
		}
		
		var s2: b2StaticEdgeShape;
		var angle: Number;
		var angle2: Number;
		if (edgeDef.isALoop) {
			s = edges[edgeDef.vertexCount-1];
			i = 0
		} else {
			s = edges[0];
			i = 1;
		}
		angle = Math.atan2(s.m_direction.y, s.m_direction.x);
		for (; i < edges.length; i++) {
			s2 = edges[i];
			angle2 = Math.atan2(s2.m_direction.y, s2.m_direction.x);
			var core: b2Vec2 = s2.m_direction.Copy();
			core.Multiply(Math.tan((angle2 - angle) * 0.5));
			core.Subtract(s2.m_normal);
			core.Multiply(b2Settings.b2_toiSlop);
			core.Add(s2.m_v1);
			var cornerDir: b2Vec2 = s.m_direction.Copy();
			cornerDir.Add(s2.m_direction);
			cornerDir.Normalize();
			s.SetNextEdge(s2, core, cornerDir);
			s2.SetPrevEdge(s, core, cornerDir);
			s = s2;
			angle = angle2;
		}
	}
	
	public var edges: Array = new Array();
	public var isALoop:Boolean;
};

}