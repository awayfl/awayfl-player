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

import { b2Vec2 } from '../../Common/Math';

export class b2Jacobian {
	public linear1: b2Vec2 = new b2Vec2();
	public angular1: number;
	public linear2: b2Vec2 = new b2Vec2();
	public angular2: number;

	public SetZero(): void {
		this.linear1.SetZero(); this.angular1 = 0.0;
		this.linear2.SetZero(); this.angular2 = 0.0;
	}

	public Set(x1: b2Vec2, a1: number, x2: b2Vec2, a2: number): void {
		this.linear1.SetV(x1); this.angular1 = a1;
		this.linear2.SetV(x2); this.angular2 = a2;
	}

	public Compute(x1: b2Vec2, a1: number, x2: b2Vec2, a2: number): number {

		//return b2Math.b2Dot(linear1, x1) + angular1 * a1 + b2Math.b2Dot(linear2, x2) + angular2 * a2;
		return (this.linear1.x * x1.x + this.linear1.y * x1.y) + this.angular1 * a1 + (this.linear2.x * x2.x + this.linear2.y * x2.y) + this.angular2 * a2;
	}
}