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

import { b2Vec2, b2XForm } from "../../Common/Math";
import { b2Settings } from "../../Common/b2Settings";
import { b2AABB } from "../b2AABB";
import { b2Segment } from "../b2Segment";
import { b2Shape } from "./b2Shape";
import { b2StaticEdgeChain } from "./b2StaticEdgeChain";
import { b2StaticEdgeChainDef } from "./b2StaticEdgeChainDef";

export class b2StaticEdgeShape extends b2Shape
{
	/// @see b2Shape::TestSegment
	public TestSegment(	transform:b2XForm,
						lambda:number[], // float pointer
						normal:b2Vec2, // pointer
						segment:b2Segment,
						maxLambda:number) :boolean
	{
		// This is mostly copied from b2Segment.TestSegment(), since 
		// b2StaticEdgeShapes are much like b2Segments.
		var k_slop:number = 100.0 * Number.MIN_VALUE;
		var rX:number = segment.p2.x - segment.p1.x;
		var rY:number = segment.p2.y - segment.p1.y;
		var nX:number = this.m_v2.y - this.m_v1.y;
		var nY:number = this.m_v1.x - this.m_v2.x;
		var denom:number = -(rX*nX + rY*nY);
		if (denom > k_slop) {
			var bX:number = segment.p1.x - this.m_v1.x;
			var bY:number = segment.p1.y - this.m_v1.y;
			var a:number = (bX*nX + bY*nY);
			if (0.0 <= a && a <= maxLambda * denom) {
				var mu2:number = -rX * bY + rY * bX;
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
					a /= denom;
					var nLen:number = Math.sqrt(nX*nX + nY*nY);
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
	public ComputeAABB(aabb:b2AABB, transform:b2XForm) : void{
		if (this.m_v1.x < this.m_v2.x) {
			aabb.lowerBound.x = this.m_v1.x;
			aabb.upperBound.x = this.m_v2.x;
		} else {
			aabb.lowerBound.x = this.m_v2.x;
			aabb.upperBound.x = this.m_v1.x;
		}
		if (this.m_v1.y < this.m_v2.y) {
			aabb.lowerBound.y = this.m_v1.y;
			aabb.upperBound.y = this.m_v2.y;
		} else {
			aabb.lowerBound.y = this.m_v2.y;
			aabb.upperBound.y = this.m_v1.y;
		}
	}

	/// @see b2Shape::ComputeSweptAABB
	public ComputeSweptAABB(	aabb:b2AABB,
							transform1:b2XForm,
							transform2:b2XForm) : void
	{
		return this.ComputeAABB(aabb, transform1);
	}

	public GetVertex1() : b2Vec2{
		return this.m_v1;
	}

	public GetVertex2() : b2Vec2{
		return this.m_v2;
	}
	
	public GetLength() :number{
		return this.m_length;
	}
	
	public GetNormalVector() : b2Vec2{
		return this.m_normal;
	}
	
	public GetDirectionVector() : b2Vec2{
		return this.m_direction;
	}
	
	public GetParentChain() : b2StaticEdgeChain{
		return this.m_chain;
	}

	public GetFirstVertex(xf:b2XForm) : b2Vec2{
		return this.m_coreV1;
	}

	public Support(xf:b2XForm, dX:number, dY:number) : b2Vec2{
		return (this.m_coreV1.x*dX + this.m_coreV1.y*dY) > (this.m_coreV2.x*dX + this.m_coreV2.y*dY) ? this.m_coreV1 : this.m_coreV2;
	}
	
	//--------------- Internals Below -------------------

	constructor(v1: b2Vec2, v2: b2Vec2, edgeDef:b2StaticEdgeChainDef){
		super(edgeDef);
		
		this.m_type = b2Shape.e_staticEdgeShape;
		this.m_v1 = v1;
		this.m_v2 = v2;
		
		this.m_direction = this.m_v2.Copy();
		this.m_direction.Subtract(this.m_v1);
		this.m_length = this.m_direction.Normalize();
		this.m_normal = new b2Vec2(this.m_direction.y, -this.m_direction.x);
		
		this.m_coreV1 = this.m_normal.Copy();
		this.m_coreV1.Subtract(this.m_direction);
		this.m_coreV1.Multiply(-b2Settings.b2_toiSlop);
		this.m_coreV1.Add(this.m_v1);
		this.m_coreV2 = this.m_normal.Copy();
		this.m_coreV2.Add(this.m_direction);
		this.m_coreV2.Multiply(-b2Settings.b2_toiSlop);
		this.m_coreV2.Add(this.m_v2);
		
		this.m_cornerDir1 = this.m_normal;
		this.m_cornerDir2 = this.m_normal.Copy();
		this.m_cornerDir2.Multiply(-1);
	}
	
	public SetPrevEdge(edge: b2StaticEdgeShape, core: b2Vec2, cornerDir: b2Vec2): void {
		this.m_prevEdge = edge;
		this.m_coreV1 = core;
		this.m_cornerDir1 = cornerDir;
	}
	public SetNextEdge(edge: b2StaticEdgeShape, core: b2Vec2, cornerDir: b2Vec2): void {
		this.m_nextEdge = edge;
		this.m_coreV2 = core;
		this.m_cornerDir2 = cornerDir;
	}

	// Update the sweep radius (maximum radius) as measured from
	// a local center point.
	public UpdateSweepRadius(center:b2Vec2) : void{
		var dX:number = this.m_v1.x - center.x;
		var dY:number = this.m_v1.y - center.y;
		var d1:number = dX*dX + dY*dY;
		dX = this.m_v2.x - center.x;
		dY = this.m_v2.y - center.y;
		var d2:number = dX*dX + dY*dY;
		
		dX = Math.sqrt(d1 > d2 ? d1 : d2); // length
		this.m_sweepRadius = dX;
	}

	// Local positions of vertices in parent body
	public m_v1:b2Vec2;
	public m_v2:b2Vec2;
	
	// "Core" vertices with TOI slop for b2Distance functions:
	public m_coreV1:b2Vec2;
	public m_coreV2:b2Vec2;
	
	// Linear distance from v1 to v2:
	public m_length: number;
	
	// Perpendicular unit vector point, pointing from the solid side to the empty side: 
	public m_normal:b2Vec2;
	
	// Parallel unit vector, pointing from v1 to v2:
	public m_direction:b2Vec2;
	
	// Unit vector halfway between m_direction and m_prevEdge.m_direction:
	public m_cornerDir1:b2Vec2;
	
	// Unit vector halfway between m_direction and m_nextEdge.m_direction:
	public m_cornerDir2:b2Vec2;
	
	// Reference to parent chain and adjacent siblings:
	public m_chain:b2StaticEdgeChain;
	public m_nextEdge: b2StaticEdgeShape;
	public m_prevEdge: b2StaticEdgeShape;
}