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

import { b2Mat22, b2Vec2, b2XForm } from "../../Common/Math";
import { b2Settings } from "../../Common/b2Settings";
import { b2Segment } from "../b2Segment";
import { b2ConcaveArcDef } from "./b2ConcaveArcDef";
import { b2MassData } from "./b2MassData";
import { b2PolygonShape } from "./b2PolygonShape";
import { b2Shape } from "./b2Shape";
import { b2ShapeDef } from "./b2ShapeDef";


///A polygon with a circle subtracted from it.
///It works exactly the same as b2PolyShape, except the edge from vertex 0 to vertex 1 is a concave arc,
///with radius given by the radius property. The diameter should be longer than the distance between the
///vertices, and the rest of the polygonal shape should be large enough to enclose the resulting curve (this is not checked).
export class b2ConcaveArcShape extends b2PolygonShape
{
	/// @see b2Shape::TestPoint
	public TestPoint(xf:b2XForm, p:b2Vec2) : boolean{
		
		//b2Vec2 pLocal = b2MulT(xf.R, p - xf.position);
		var tMat:b2Mat22 = xf.R;
		var tX:number = p.x - xf.position.x;
		var tY:number = p.y - xf.position.y;
		var pLocalX:number = (tX*tMat.col1.x + tY*tMat.col1.y);
		var pLocalY:number = (tX*tMat.col2.x + tY*tMat.col2.y);
		
		for (var i:number /** uint */ = 0; i < this.m_vertexCount; ++i)
		{
			//float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
			var tVec:b2Vec2 = this.m_vertices[i];
			tX = pLocalX - tVec.x;
			tY = pLocalY - tVec.y;
			var dot:number = (tVec.x * tX + tVec.y * tY);
			if (dot > 0.0)
			{
				return false;
			}
		}

		tX = pLocalX - this.m_arcCenter.x;
		tY = pLocalY - this.m_arcCenter.y;
		
		return (tX*tX+tY*tY)>this.m_radius2;
	}

	/// @see b2Shape::TestSegment
	public TestSegment( xf:b2XForm,
		lambda:number[], // float ptr
		normal:b2Vec2, // ptr
		segment:b2Segment,
		maxLambda:number) : boolean
	{
		b2Settings.b2Assert(false);
		return false;
	}

	/// @see b2Shape::ComputeMass
	public ComputeMass(massData:b2MassData) : void{
		/*var polyMass:b2MassData = new b2MassData();
		var triMass:b2MassData = new b2MassData();
		var segMass:b2MassData = new b2MassData();
		
		super.ComputeMass(polyMass);
		
		if(m_dot > Number.MIN_VALUE)
			TriangleMass(triMass, m_vertices[0], m_arcCenter, m_vertices[1], m_density);
		
		SegmentMass(segMass, m_arcCenter, m_radius, m_normals[0].Negative(), m_norm*m_radius*2, -m_density);
		
		//massData=polyMass+triMass+segMass
		massData.Set(b2MassData.Add(polyMass,triMass,segMass));*/
		
		super.ComputeMass(massData);
	}

	//--------------- Internals Below -------------------
	
	constructor(def:b2ShapeDef){
		super(def);
		
		var arcDef:b2ConcaveArcDef = def as b2ConcaveArcDef;
		
		
		this.m_radius = arcDef.radius;
		this.m_radius2 = this.m_radius*this.m_radius;
		
		var p1:b2Vec2 = this.m_vertices[0];
		var p2:b2Vec2 = this.m_vertices[1];
		
		//Find the point at m_radius from p1 and p2;
		
		var dx:number = p1.x-p2.x;
		var dy:number = p1.y-p2.y;
		var d2:number = dx*dx+dy*dy;
		var d:number=Math.sqrt(d2);
		if(d2/4 > this.m_radius2){
			//Increase radius to fit the edge it is replacing
			this.m_radius2 = d2/4;
			this.m_radius = d/2;
		}
		var dot:number = Math.sqrt(this.m_radius2-d2*.25);//The perp distance from p1p2 to m_arcCenter
		this.m_arcCenter = new b2Vec2( (p1.x+p2.x)/2+dot*this.m_normals[0].x, (p1.y+p2.y)/2+dot*this.m_normals[0].y );
		this.m_dot=dot/this.m_radius;
		this.m_norm=d/2/this.m_radius;
		
		//Adjust core vertices so that the core curved edge is of radius m_radius+b2Settings.b2_toiSlop, from core vertex 0 to core vertex 1, and with the same center
		var coreRadius2: number = (this.m_radius+b2Settings.b2_toiSlop)*(this.m_radius+b2Settings.b2_toiSlop);
		var nx:number,ny:number;
		//Vertex 0
		nx=this.m_normals[this.m_vertexCount-1].x;
		ny=this.m_normals[this.m_vertexCount-1].y;
		dx=this.m_coreVertices[0].x-this.m_arcCenter.x;
		dy=this.m_coreVertices[0].y-this.m_arcCenter.y;
		d=dx*nx+dy*ny;
		d2=Math.sqrt(coreRadius2-d*d);
		this.m_coreVertices[0].x=this.m_arcCenter.x+d*nx+d2*ny;
		this.m_coreVertices[0].y=this.m_arcCenter.y+d*ny-d2*nx;
		//Vertex 1
		nx=this.m_normals[1].x;
		ny=this.m_normals[1].y;
		dx=this.m_coreVertices[1].x-this.m_arcCenter.x;
		dy=this.m_coreVertices[1].y-this.m_arcCenter.y;
		d=dx*nx+dy*ny;
		d2=Math.sqrt(coreRadius2-d*d);
		this.m_coreVertices[1].x=this.m_arcCenter.x+d*nx-d2*ny;
		this.m_coreVertices[1].y=this.m_arcCenter.y+d*ny+d2*nx;
		
		
		this.m_type = b2Shape.e_concaveArcShape;
	}
	
	//Calculate the mass of a segment of a circle
	//arcCenter, radius defines the circle
	//norm defines the reverse direction of the segment
	//d is the distance between the two vertices on the perimeter
	//See the code for the relationship between d and theta, the angle of the segment.
	public static SegmentMass(massData:b2MassData, arcCenter:b2Vec2, radius:number, norm:b2Vec2, d:number, density:number): void
	{
		//var theta:number = Math.acos(dot / radius) * 2;
		var theta:number = Math.asin(d / radius / 2) * 2;
		massData.mass = 0.5 * radius * radius * theta * density;
		var v:number = 2/3 * d / theta;
		if(theta < Number.MIN_VALUE) v=2/3*radius;
		massData.center = new b2Vec2(arcCenter.x - norm.x*v, arcCenter.y - norm.y*v);
		massData.I = 0.5 * massData.mass * radius * radius - massData.mass * v * v;
	}
	
	public static TriangleMass(massData:b2MassData, p1:b2Vec2, p2:b2Vec2, p3:b2Vec2, density:number): void
	{
		var k_inv3:number = 1.0 / 3.0;
		
		//b2Vec2 e1 = p2 - p1;
		var e1X:number = p2.x - p1.x;
		var e1Y:number = p2.y - p1.y;
		//b2Vec2 e2 = p3 - p1;
		var e2X:number = p3.x - p1.x;
		var e2Y:number = p3.y - p1.y;
		
		//float32 D = b2Cross(e1, e2);
		var D: number = e1X * e2Y - e1Y * e2X;
		
		//float32 triangleArea = 0.5f * D;
		var triangleArea:number = 0.5 * D;
		//area += triangleArea;
		massData.mass = triangleArea * density;
		
		// Area weighted centroid
		//center += triangleArea * k_inv3 * (p1 + p2 + p3);
		//centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
		//centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
		massData.center.x = k_inv3 * (p1.x + p2.x + p3.x);
		massData.center.y = k_inv3 * (p1.y + p2.y + p3.y);
		
		//float32 px = p1.x, py = p1.y;
		var px:number = p1.x;
		var py:number = p1.y;
		//float32 ex1 = e1.x, ey1 = e1.y;
		var ex1:number = e1X;
		var ey1:number = e1Y;
		//float32 ex2 = e2.x, ey2 = e2.y;
		var ex2:number = e2X;
		var ey2:number = e2Y;
		
		//float32 intx2 = k_inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
		var intx2:number = k_inv3 * (0.25 * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5*px*px;
		//float32 inty2 = k_inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;
		var inty2:number = k_inv3 * (0.25 * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5*py*py;
		
		//I += D * (intx2 + inty2);
		massData.I = D * (intx2 + inty2) * density;
	}
	
	public m_arcCenter:b2Vec2;
	public m_norm:number;//=sin(theta/2) for theta the angle of the arc
	public m_dot:number;//=cos(theta/2) for theta the angle of the arc
	public m_radius:number;
	public m_radius2:number;//=m_radius*m_radius. Flash Only: In C++ we should calculate this each time

}