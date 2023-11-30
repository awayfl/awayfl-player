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

import { b2Math, b2Vec2 } from '../../Common/Math';

/// This holds the mass data computed for a shape.
export class b2MassData {
	/// The mass of the shape, usually in kilograms.
	public mass: number = 0.0;
	/// The position of the shape's centroid relative to the shape's origin.
	public center: b2Vec2 = new b2Vec2(0,0);
	/// The rotational inertia of the shape.
	public I: number = 0.0;

	public static Add(...masses: b2MassData[]):b2MassData
	{
		var finalMass:b2MassData=new b2MassData();
		for (let massData of masses){
			finalMass.mass += massData.mass;
		}
		//b2Settings.b2Assert(finalMass.mass>=0);
		if(Math.abs(finalMass.mass)>Number.MIN_VALUE){
			for (let massData of masses){
				finalMass.center.x += massData.mass*massData.center.x;
				finalMass.center.y += massData.mass*massData.center.y;
			}
			finalMass.center.x /= finalMass.mass;
			finalMass.center.y /= finalMass.mass;
			for (let massData of masses){
				finalMass.I += massData.I;
				var r:b2Vec2 = b2Math.SubtractVV(massData.center, finalMass.center);
				finalMass.I += massData.mass * b2Math.b2Dot(r, r);
			}
		}
		return finalMass;
	}
	
	public Set(massData:b2MassData):void
	{
		this.mass = massData.mass;
		this.center.x = massData.center.x;
		this.center.y = massData.center.y;
		this.I = massData.I;
	}
}