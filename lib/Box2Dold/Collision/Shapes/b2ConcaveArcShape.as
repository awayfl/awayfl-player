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

///A polygon with a circle subtracted from it.
///It works exactly the same as b2PolyShape, except the edge from vertex 0 to vertex 1 is a concave arc,
///with radius given by the radius property. The diameter should be longer than the distance between the
///vertices, and the rest of the polygonal shape should be large enough to enclose the resulting curve (this is not checked).
public class b2ConcaveArcShape extends b2PolygonShape
{
	/// @see b2Shape::TestPoint
	public override function TestPoint(xf:b2XForm, p:b2Vec2) : Boolean{
		
		//b2Vec2 pLocal = b2MulT(xf.R, p - xf.position);
		var tMat:b2Mat22 = xf.R;
		var tX:Number = p.x - xf.position.x;
		var tY:Number = p.y - xf.position.y;
		var pLocalX:Number = (tX*tMat.col1.x + tY*tMat.col1.y);
		var pLocalY:Number = (tX*tMat.col2.x + tY*tMat.col2.y);
		
		for (var i:int = 0; i < m_vertexCount; ++i)
		{
			//float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
			var tVec:b2Vec2 = m_vertices[i];
			tX = pLocalX - tVec.x;
			tY = pLocalY - tVec.y;
			var dot:Number = (tVec.x * tX + tVec.y * tY);
			if (dot > 0.0)
			{
				return false;
			}
		}

		tX = pLocalX - m_arcCenter.x;
		tY = pLocalY - m_arcCenter.y;
		
		return (tX*tX+tY*tY)>m_radius2;
	}

	/// @see b2Shape::TestSegment
	public override function TestSegment( xf:b2XForm,
		lambda:Array, // float ptr
		normal:b2Vec2, // ptr
		segment:b2Segment,
		maxLambda:Number) : Boolean
	{
		b2Settings.b2Assert(false);
		return false;
	}

	/// @see b2Shape::ComputeMass
	public override function ComputeMass(massData:b2MassData) : void{
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
	
	public function b2ConcaveArcShape(def:b2ShapeDef){
		super(def);
		
		var arcDef:b2ConcaveArcDef = def as b2ConcaveArcDef;
		
		
		m_radius = arcDef.radius;
		m_radius2 = m_radius*m_radius;
		
		var p1:b2Vec2 = m_vertices[0];
		var p2:b2Vec2 = m_vertices[1];
		
		//Find the point at m_radius from p1 and p2;
		
		var dx:Number = p1.x-p2.x;
		var dy:Number = p1.y-p2.y;
		var d2:Number = dx*dx+dy*dy;
		var d:Number=Math.sqrt(d2);
		if(d2/4 > m_radius2){
			//Increase radius to fit the edge it is replacing
			m_radius2 = d2/4;
			m_radius = d/2;
		}
		var dot:Number = Math.sqrt(m_radius2-d2*.25);//The perp distance from p1p2 to m_arcCenter
		m_arcCenter = new b2Vec2( (p1.x+p2.x)/2+dot*m_normals[0].x, (p1.y+p2.y)/2+dot*m_normals[0].y );
		m_dot=dot/m_radius;
		m_norm=d/2/m_radius;
		
		//Adjust core vertices so that the core curved edge is of radius m_radius+b2Settings.b2_toiSlop, from core vertex 0 to core vertex 1, and with the same center
		var coreRadius2: Number = (m_radius+b2Settings.b2_toiSlop)*(m_radius+b2Settings.b2_toiSlop);
		var nx:Number,ny:Number;
		//Vertex 0
		nx=m_normals[m_vertexCount-1].x;
		ny=m_normals[m_vertexCount-1].y;
		dx=m_coreVertices[0].x-m_arcCenter.x;
		dy=m_coreVertices[0].y-m_arcCenter.y;
		d=dx*nx+dy*ny;
		d2=Math.sqrt(coreRadius2-d*d);
		m_coreVertices[0].x=m_arcCenter.x+d*nx+d2*ny;
		m_coreVertices[0].y=m_arcCenter.y+d*ny-d2*nx;
		//Vertex 1
		nx=m_normals[1].x;
		ny=m_normals[1].y;
		dx=m_coreVertices[1].x-m_arcCenter.x;
		dy=m_coreVertices[1].y-m_arcCenter.y;
		d=dx*nx+dy*ny;
		d2=Math.sqrt(coreRadius2-d*d);
		m_coreVertices[1].x=m_arcCenter.x+d*nx-d2*ny;
		m_coreVertices[1].y=m_arcCenter.y+d*ny+d2*nx;
		
		
		m_type = b2Shape.e_concaveArcShape;
	}
	
	//Calculate the mass of a segment of a circle
	//arcCenter, radius defines the circle
	//norm defines the reverse direction of the segment
	//d is the distance between the two vertices on the perimeter
	//See the code for the relationship between d and theta, the angle of the segment.
	static public function SegmentMass(massData:b2MassData, arcCenter:b2Vec2, radius:Number, norm:b2Vec2, d:Number, density:Number): void
	{
		//var theta:Number = Math.acos(dot / radius) * 2;
		var theta:Number = Math.asin(d / radius / 2) * 2;
		massData.mass = 0.5 * radius * radius * theta * density;
		var v:Number = 2/3 * d / theta;
		if(theta < Number.MIN_VALUE) v=2/3*radius;
		massData.center = new b2Vec2(arcCenter.x - norm.x*v, arcCenter.y - norm.y*v);
		massData.I = 0.5 * massData.mass * radius * radius - massData.mass * v * v;
	}
	
	static public function TriangleMass(massData:b2MassData, p1:b2Vec2, p2:b2Vec2, p3:b2Vec2, density:Number): void
	{
		var k_inv3:Number = 1.0 / 3.0;
		
		//b2Vec2 e1 = p2 - p1;
		var e1X:Number = p2.x - p1.x;
		var e1Y:Number = p2.y - p1.y;
		//b2Vec2 e2 = p3 - p1;
		var e2X:Number = p3.x - p1.x;
		var e2Y:Number = p3.y - p1.y;
		
		//float32 D = b2Cross(e1, e2);
		var D: Number = e1X * e2Y - e1Y * e2X;
		
		//float32 triangleArea = 0.5f * D;
		var triangleArea:Number = 0.5 * D;
		//area += triangleArea;
		massData.mass = triangleArea * density;
		
		// Area weighted centroid
		//center += triangleArea * k_inv3 * (p1 + p2 + p3);
		//centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
		//centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
		massData.center.x = k_inv3 * (p1.x + p2.x + p3.x);
		massData.center.y = k_inv3 * (p1.y + p2.y + p3.y);
		
		//float32 px = p1.x, py = p1.y;
		var px:Number = p1.x;
		var py:Number = p1.y;
		//float32 ex1 = e1.x, ey1 = e1.y;
		var ex1:Number = e1X;
		var ey1:Number = e1Y;
		//float32 ex2 = e2.x, ey2 = e2.y;
		var ex2:Number = e2X;
		var ey2:Number = e2Y;
		
		//float32 intx2 = k_inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
		var intx2:Number = k_inv3 * (0.25 * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5*px*px;
		//float32 inty2 = k_inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;
		var inty2:Number = k_inv3 * (0.25 * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5*py*py;
		
		//I += D * (intx2 + inty2);
		massData.I = D * (intx2 + inty2) * density;
	}
	
	public var m_arcCenter:b2Vec2;
	public var m_norm:Number;//=sin(theta/2) for theta the angle of the arc
	public var m_dot:Number;//=cos(theta/2) for theta the angle of the arc
	public var m_radius:Number;
	public var m_radius2:Number;//=m_radius*m_radius. Flash Only: In C++ we should calculate this each time

};

}