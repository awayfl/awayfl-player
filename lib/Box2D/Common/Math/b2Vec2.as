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

package Box2D.Common.Math{

	
import Box2D.Common.*;


/**
* A 2D column vector.
*/

public class b2Vec2
{
	public function b2Vec2(x_:number=0, y_:number=0) : void {x=x_; y=y_;};

	public function SetZero() : void { x = 0.0; y = 0.0; }
	public function Set(x_:number=0, y_:number=0) : void {x=x_; y=y_;};
	public function SetV(v:b2Vec2) : void {x=v.x; y=v.y;};

	public function GetNegative():b2Vec2 { return new b2Vec2(-x, -y); }
	public function NegativeSelf():void { x = -x; y = -y; }
	
	static public function Make(x_:number, y_:number):b2Vec2
	{
		return new b2Vec2(x_, y_);
	}
	
	public function Copy():b2Vec2{
		return new b2Vec2(x,y);
	}
	
	public function Add(v:b2Vec2) : void
	{
		x += v.x; y += v.y;
	}
	
	public function Subtract(v:b2Vec2) : void
	{
		x -= v.x; y -= v.y;
	}

	public function Multiply(a:number) : void
	{
		x *= a; y *= a;
	}
	
	public function MulM(A:b2Mat22) : void
	{
		var tX:number = x;
		x = A.col1.x * tX + A.col2.x * y;
		y = A.col1.y * tX + A.col2.y * y;
	}
	
	public function MulTM(A:b2Mat22) : void
	{
		var tX:number = b2Math.Dot(this, A.col1);
		y = b2Math.Dot(this, A.col2);
		x = tX;
	}
	
	public function CrossVF(s:number) : void
	{
		var tX:number = x;
		x = s * y;
		y = -s * tX;
	}
	
	public function CrossFV(s:number) : void
	{
		var tX:number = x;
		x = -s * y;
		y = s * tX;
	}
	
	public function MinV(b:b2Vec2) : void
	{
		x = x < b.x ? x : b.x;
		y = y < b.y ? y : b.y;
	}
	
	public function MaxV(b:b2Vec2) : void
	{
		x = x > b.x ? x : b.x;
		y = y > b.y ? y : b.y;
	}
	
	public function Abs() : void
	{
		if (x < 0) x = -x;
		if (y < 0) y = -y;
	}

	public function Length():number
	{
		return Math.sqrt(x * x + y * y);
	}
	
	public function LengthSquared():number
	{
		return (x * x + y * y);
	}

	public function Normalize():number
	{
		var length:number = Math.sqrt(x * x + y * y);
		if (length < Number.MIN_VALUE)
		{
			return 0.0;
		}
		var invLength:number = 1.0 / length;
		x *= invLength;
		y *= invLength;
		
		return length;
	}

	public function IsValid():Boolean
	{
		return b2Math.IsValid(x) && b2Math.IsValid(y);
	}

	public var x:number;
	public var y:number;
};

}