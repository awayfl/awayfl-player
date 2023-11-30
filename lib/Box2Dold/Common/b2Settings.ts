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

import { b2Vec2 } from './Math';

export class b2Settings {

	public static readonly USHRT_MAX: number /** int */ = 0x0000ffff;

	public static readonly b2_pi: number = Math.PI;

	// Collision
	public static readonly b2_maxManifoldPoints: number /** int */ = 2;
	public static readonly b2_maxPolygonVertices: number /** int */ = 8;
	public static readonly b2_maxProxies: number /** int */ = 512;				// this must be a power of two
	public static readonly b2_maxPairs: number /** int */ = 8 * b2Settings.b2_maxProxies;	// this must be a power of two

	// Dynamics

	/// A small length used as a collision and constraint tolerance. Usually it is
	/// chosen to be numerically significant, but visually insignificant.
	public static readonly b2_linearSlop: number = 0.005;	// 0.5 cm

	/// A small angle used as a collision and constraint tolerance. Usually it is
	/// chosen to be numerically significant, but visually insignificant.
	public static readonly b2_angularSlop: number = 2.0 / 180.0 * b2Settings.b2_pi;			// 2 degrees

	/// Continuous collision detection (CCD) works with core, shrunken shapes. This is the
	/// amount by which shapes are automatically shrunk to work with CCD. This must be
	/// larger than b2_linearSlop.
	public static readonly b2_toiSlop: number = 8.0 * b2Settings.b2_linearSlop;

	/// Maximum number of contacts to be handled to solve a TOI island.
	public static readonly b2_maxTOIContactsPerIsland: number /** int */ = 32;

	/// A velocity threshold for elastic collisions. Any collision with a relative linear
	/// velocity below this threshold will be treated as inelastic.
	public static readonly b2_velocityThreshold: number = 1.0;		// 1 m/s

	/// The maximum linear position correction used when solving constraints. This helps to
	/// prevent overshoot.
	public static readonly b2_maxLinearCorrection: number = 0.2;	// 20 cm

	/// The maximum angular position correction used when solving constraints. This helps to
	/// prevent overshoot.
	public static readonly b2_maxAngularCorrection: number = 8.0 / 180.0 * b2Settings.b2_pi;			// 8 degrees

	/// The maximum linear velocity of a body. This limit is very large and is used
	/// to prevent numerical problems. You shouldn't need to adjust this.
	public static readonly b2_maxLinearVelocity: number = 200.0;
	public static readonly b2_maxLinearVelocitySquared: number = b2Settings.b2_maxLinearVelocity * b2Settings.b2_maxLinearVelocity;

	/// The maximum angular velocity of a body. This limit is very large and is used
	/// to prevent numerical problems. You shouldn't need to adjust this.
	public static readonly b2_maxAngularVelocity: number = 250.0;
	public static readonly b2_maxAngularVelocitySquared: number = b2Settings.b2_maxAngularVelocity * b2Settings.b2_maxAngularVelocity;

	/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
	/// that overlap is removed in one time step. However using values close to 1 often lead
	/// to overshoot.
	public static readonly b2_contactBaumgarte: number = 0.2;

	// Sleep

	/// The time that a body must be still before it will go to sleep.
	public static readonly b2_timeToSleep: number = 0.5;					// half a second
	/// A body cannot sleep if its linear velocity is above this tolerance.
	public static readonly b2_linearSleepTolerance: number = 0.01;			// 1 cm/s
	/// A body cannot sleep if its angular velocity is above this tolerance.
	public static readonly b2_angularSleepTolerance: number = 2.0 / 180.0;	// 2 degrees/s

	// assert
	public static b2Assert(a: boolean): void {
		if (!a) {
			let nullVec: b2Vec2;
			nullVec.x++;
		}
	}
}