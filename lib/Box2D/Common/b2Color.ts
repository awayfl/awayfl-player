/**
* Color for debug drawing. Each value has the range [0,1].
*/

import { b2Math } from './Math';

export class b2Color {

	constructor(rr: number, gg: number, bb: number) {
		this._r = 255 * b2Math.Clamp(rr, 0.0, 1.0) >>> 0;
		this._g = 255 * b2Math.Clamp(gg, 0.0, 1.0) >>> 0;
		this._b = 255 * b2Math.Clamp(bb, 0.0, 1.0) >>> 0;
	}

	public Set(rr: number, gg: number, bb: number): void {
		this._r = 255 * b2Math.Clamp(rr, 0.0, 1.0) >>> 0;
		this._g = 255 * b2Math.Clamp(gg, 0.0, 1.0) >>> 0;
		this._b = 255 * b2Math.Clamp(bb, 0.0, 1.0) >>> 0;
	}

	// R
	public set r(rr: number) {
		this._r = 255 * b2Math.Clamp(rr, 0.0, 1.0) >>> 0;
	}

	// G
	public set g(gg: number) {
		this._g = 255 * b2Math.Clamp(gg, 0.0, 1.0) >>> 0;
	}

	// B
	public set b(bb: number) {
		this._b = 255 * b2Math.Clamp(bb, 0.0, 1.0) >>> 0;
	}

	// Color
	public get color(): number /** uint */{
		return (this._r << 16) | (this._g << 8) | (this._b);
	}

	private _r: number /** uint */ = 0;
	private _g: number /** uint */ = 0;
	private _b: number /** uint */ = 0;

}