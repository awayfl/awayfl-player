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

import { b2ContactID } from './b2ContactID';

// We use contact ids to facilitate warm starting.
export class Features {
	///< The edge that defines the outward contact normal.
	public set referenceEdge(value: number /** int */) {
		this._referenceEdge = value;
		this._m_id._key = (this._m_id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
	}

	public get referenceEdge(): number /** int */{
		return this._referenceEdge;
	}

	public _referenceEdge: number /** int */;

	///< The edge most anti-parallel to the reference edge.
	public set incidentEdge(value: number /** int */) {
		this._incidentEdge = value;
		this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
	}

	public get incidentEdge(): number /** int */{
		return this._incidentEdge;
	}

	public _incidentEdge: number /** int */;

	///< The vertex (0 or 1) on the incident edge that was clipped.
	public set incidentVertex(value: number /** int */) {
		this._incidentVertex = value;
		this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
	}

	public get incidentVertex(): number /** int */{
		return this._incidentVertex;
	}

	public _incidentVertex: number /** int */;

	///< A value of 1 indicates that the reference edge is on shape2.
	public set flip(value: number /** int */) {
		this._flip = value;
		this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
	}

	public get flip(): number /** int */{
		return this._flip;
	}

	public _flip: number /** int */;

	public _m_id: b2ContactID;
}