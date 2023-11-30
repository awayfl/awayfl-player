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

import { b2Math } from './Math';

/// A 2D column vector.

export class b2Color {

	constructor(rr: number, gg: number, bb: number) {
		this._r = (255 * b2Math.b2Clamp(rr, 0.0, 1.0)) >>> 0;
		this._g = (255 * b2Math.b2Clamp(gg, 0.0, 1.0)) >>> 0;
		this._b = (255 * b2Math.b2Clamp(bb, 0.0, 1.0)) >>> 0;
	}

	public Set(rr: number, gg: number, bb: number): void {
		this._r = (255 * b2Math.b2Clamp(rr, 0.0, 1.0)) >>> 0;
		this._g = (255 * b2Math.b2Clamp(gg, 0.0, 1.0)) >>> 0;
		this._b = (255 * b2Math.b2Clamp(bb, 0.0, 1.0)) >>> 0;
	}

	// R
	public set r(rr: number) {
		this._r = (255 * b2Math.b2Clamp(rr, 0.0, 1.0)) >>> 0;
	}

	// G
	public set g(gg: number) {
		this._g = (255 * b2Math.b2Clamp(gg, 0.0, 1.0)) >>> 0;
	}

	// B
	public set b(bb: number) {
		this._b = (255 * b2Math.b2Clamp(bb, 0.0, 1.0)) >>> 0;
	}

	// Color
	public get color(): number /** uint */{
		return (this._r) | (this._g << 8) | (this._b << 16);
	}

	private _r: number /** uint */ = 0;
	private _g: number /** uint */ = 0;
	private _b: number /** uint */ = 0;

}