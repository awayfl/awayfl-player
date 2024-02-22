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



public class b2StaticEdgeShape extends b2Shape
{
	/// @see b2Shape::TestSegment
	public override function TestSegment(	transform:b2XForm,
						lambda:Array, // float pointer
						normal:b2Vec2, // pointer
						segment:b2Segment,
						maxLambda:Number) :Boolean
	{
		// This is mostly copied from b2Segment.TestSegment(), since 
		// b2StaticEdgeShapes are much like b2Segments.
		var k_slop:Number = 100.0 * Number.MIN_VALUE;
		var rX:Number = segment.p2.x - segment.p1.x;
		var rY:Number = segment.p2.y - segment.p1.y;
		var nX:Number = m_v2.y - m_v1.y;
		var nY:Number = m_v1.x - m_v2.x;
		var denom:Number = -(rX*nX + rY*nY);
		if (denom > k_slop) {
			var bX:Number = segment.p1.x - m_v1.x;
			var bY:Number = segment.p1.y - m_v1.y;
			var a:Number = (bX*nX + bY*nY);
			if (0.0 <= a && a <= maxLambda * denom) {
				var mu2:Number = -rX * bY + rY * bX;
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
					a /= denom;
					var nLen:Number = Math.sqrt(nX*nX + nY*nY);
					nX /= nLen;
					nY /= nLen;
					lambda[0] = a;
					normal.Set(nX, nY);
					return true;
				}
			}
		}
		return false;
	}

	/// @see b2Shape::ComputeAABB
	public override function ComputeAABB(aabb:b2AABB, transform:b2XForm) : void{
		if (m_v1.x < m_v2.x) {
			aabb.lowerBound.x = m_v1.x;
			aabb.upperBound.x = m_v2.x;
		} else {
			aabb.lowerBound.x = m_v2.x;
			aabb.upperBound.x = m_v1.x;
		}
		if (m_v1.y < m_v2.y) {
			aabb.lowerBound.y = m_v1.y;
			aabb.upperBound.y = m_v2.y;
		} else {
			aabb.lowerBound.y = m_v2.y;
			aabb.upperBound.y = m_v1.y;
		}
	}

	/// @see b2Shape::ComputeSweptAABB
	public override function ComputeSweptAABB(	aabb:b2AABB,
							transform1:b2XForm,
							transform2:b2XForm) : void
	{
		return ComputeAABB(aabb, transform1);
	}

	public function GetVertex1() : b2Vec2{
		return m_v1;
	}

	public function GetVertex2() : b2Vec2{
		return m_v2;
	}
	
	public function GetLength() : Number{
		return m_length;
	}
	
	public function GetNormalVector() : b2Vec2{
		return m_normal;
	}
	
	public function GetDirectionVector() : b2Vec2{
		return m_direction;
	}
	
	public function GetParentChain() : b2StaticEdgeChain{
		return m_chain;
	}

	public function GetFirstVertex(xf:b2XForm) : b2Vec2{
		return m_coreV1;
	}

	public function Support(xf:b2XForm, dX:Number, dY:Number) : b2Vec2{
		return (m_coreV1.x*dX + m_coreV1.y*dY) > (m_coreV2.x*dX + m_coreV2.y*dY) ? m_coreV1 : m_coreV2;
	}
	
	//--------------- Internals Below -------------------

	public function b2StaticEdgeShape(v1: b2Vec2, v2: b2Vec2, edgeDef:b2StaticEdgeChainDef){
		super(edgeDef);
		
		m_type = e_staticEdgeShape;
		m_v1 = v1;
		m_v2 = v2;
		
		m_direction = m_v2.Copy();
		m_direction.Subtract(m_v1);
		m_length = m_direction.Normalize();
		m_normal = new b2Vec2(m_direction.y, -m_direction.x);
		
		m_coreV1 = m_normal.Copy();
		m_coreV1.Subtract(m_direction);
		m_coreV1.Multiply(-b2Settings.b2_toiSlop);
		m_coreV1.Add(m_v1);
		m_coreV2 = m_normal.Copy();
		m_coreV2.Add(m_direction);
		m_coreV2.Multiply(-b2Settings.b2_toiSlop);
		m_coreV2.Add(m_v2);
		
		m_cornerDir1 = m_normal;
		m_cornerDir2 = m_normal.Copy();
		m_cornerDir2.Multiply(-1);
	}
	
	public function SetPrevEdge(edge: b2StaticEdgeShape, core: b2Vec2, cornerDir: b2Vec2): void {
		m_prevEdge = edge;
		m_coreV1 = core;
		m_cornerDir1 = cornerDir;
	}
	public function SetNextEdge(edge: b2StaticEdgeShape, core: b2Vec2, cornerDir: b2Vec2): void {
		m_nextEdge = edge;
		m_coreV2 = core;
		m_cornerDir2 = cornerDir;
	}

	// Update the sweep radius (maximum radius) as measured from
	// a local center point.
	public override function UpdateSweepRadius(center:b2Vec2) : void{
		var dX:Number = m_v1.x - center.x;
		var dY:Number = m_v1.y - center.y;
		var d1:Number = dX*dX + dY*dY;
		dX = m_v2.x - center.x;
		dY = m_v2.y - center.y;
		var d2:Number = dX*dX + dY*dY;
		
		dX = Math.sqrt(d1 > d2 ? d1 : d2); // length
		m_sweepRadius = dX;
	}

	// Local positions of vertices in parent body
	public var m_v1:b2Vec2;
	public var m_v2:b2Vec2;
	
	// "Core" vertices with TOI slop for b2Distance functions:
	public var m_coreV1:b2Vec2;
	public var m_coreV2:b2Vec2;
	
	// Linear distance from v1 to v2:
	public var m_length: Number;
	
	// Perpendicular unit vector point, pointing from the solid side to the empty side: 
	public var m_normal:b2Vec2;
	
	// Parallel unit vector, pointing from v1 to v2:
	public var m_direction:b2Vec2;
	
	// Unit vector halfway between m_direction and m_prevEdge.m_direction:
	public var m_cornerDir1:b2Vec2;
	
	// Unit vector halfway between m_direction and m_nextEdge.m_direction:
	public var m_cornerDir2:b2Vec2;
	
	// Reference to parent chain and adjacent siblings:
	public var m_chain:b2StaticEdgeChain;
	public var m_nextEdge: b2StaticEdgeShape;
	public var m_prevEdge: b2StaticEdgeShape;
};

}