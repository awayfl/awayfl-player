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

///A circle intersected with a half plane.
///The circle is as it would be for a b2CircleShape, i.e. centered at localPosition and of radius radius.
///The half plane is represented by norm, giving the outward normal, and offset, which is the offset of
///the halfplane's edge from the center of the circle. A zero offset means a semi-circle, with a positive
///offset being larger than that and a negative one smaller.
public class b2ConvexArcShape extends b2Shape
{
	/// @see b2Shape::TestPoint
	public override function TestPoint(xf:b2XForm, p:b2Vec2) : Boolean{
		b2Settings.b2Assert(false);
		return false;
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

	/// @see b2Shape::ComputeAABB
	public override function ComputeAABB(aabb:b2AABB, xf:b2XForm) : void{
		super.ComputeAABB(aabb, xf);
	}

	/// @see b2Shape::ComputeSweptAABB
	public override function ComputeSweptAABB(	aabb:b2AABB,
		transform1:b2XForm,
		transform2:b2XForm) : void
	{
		super.ComputeSweptAABB(aabb, transform1, transform2);
	}

	/// @see b2Shape::ComputeMass
	public override function ComputeMass(massData:b2MassData) : void{
	}

	/// Get the oriented bounding box relative to the parent body.
	public function GetOBB() : b2OBB{
		return m_obb;
	}


	//--------------- Internals Below -------------------
	
	public function b2ConvexArcShape(def:b2ShapeDef){
		super(def);
		
	}

	public override function UpdateSweepRadius(center:b2Vec2) : void{
	}

	
	public function Support(xf:b2XForm, dX:Number, dY:Number) : b2Vec2{
		b2Settings.b2Assert(false);
		return null;
	}

	// Local position of the circle center in parent body frame.
	public var m_localPosition:b2Vec2 = new b2Vec2();

	// Local position oriented bounding box. The OBB center is relative to
	// shape position.
	public var m_obb:b2OBB = new b2OBB();
	//Only has 2 vertices
	public var m_vertices:Array = [new b2Vec2(), new b2Vec2()];//Like b2PolyShape, these are relative to m_localPosition
	public var m_radius:Number;
	public var m_norm:b2Vec2 = new b2Vec2();
	public var m_offset:Number;
	public var m_d:Number;//Length of straight section
	public var m_dot:Number;
	
	
};

}