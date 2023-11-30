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

import { b2Settings } from '../Common/b2Settings';

// The pair manager is used by the broad-phase to quickly add/remove/find pairs
// of overlapping proxies. It is based closely on code provided by Pierre Terdiman.
// http://www.codercorner.com/IncrementalSAP.txt

export class b2Pair {

	public SetBuffered(): void	{ this.status |= b2Pair.e_pairBuffered; }
	public ClearBuffered(): void	{ this.status &= ~b2Pair.e_pairBuffered; }
	public IsBuffered(): boolean	{ return (this.status & b2Pair.e_pairBuffered) == b2Pair.e_pairBuffered; }

	public SetRemoved(): void		{ this.status |= b2Pair.e_pairRemoved; }
	public ClearRemoved(): void	{ this.status &= ~b2Pair.e_pairRemoved; }
	public IsRemoved(): boolean		{ return (this.status & b2Pair.e_pairRemoved) == b2Pair.e_pairRemoved; }

	public SetFinal(): void		{ this.status |= b2Pair.e_pairFinal; }
	public IsFinal(): boolean		{ return (this.status & b2Pair.e_pairFinal) == b2Pair.e_pairFinal; }

	public userData: any = null;
	public proxyId1: number /** uint */;
	public proxyId2: number /** uint */;
	public next: number /** uint */;
	public status: number /** uint */;

	// STATIC
	public static b2_nullPair: number /** uint */ = b2Settings.USHRT_MAX;
	public static b2_nullProxy: number /** uint */ = b2Settings.USHRT_MAX;
	public static b2_tableCapacity: number /** int */ = b2Settings.b2_maxPairs;	// must be a power of two
	public static b2_tableMask: number /** int */ = b2Pair.b2_tableCapacity - 1;

	// enum
	public static e_pairBuffered: number /** uint */ = 0x0001;
	public static e_pairRemoved: number /** uint */ = 0x0002;
	public static e_pairFinal: number /** uint */ = 0x0004;

}