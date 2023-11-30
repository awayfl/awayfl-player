import { b2ContactID } from './b2ContactID';

/**
* We use contact ids to facilitate warm starting.
*/
export class Features {
	readonly __fast__ = true;
	/**
    * The edge that defines the outward contact normal.
    */
	public get referenceEdge(): number /** int */{
		return this._referenceEdge;
	}

	public set referenceEdge(value: number /** int */) {
		this._referenceEdge = value;
		this._m_id._key = (this._m_id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
	}

	public _referenceEdge: number /** int */;

	/**
    * The edge most anti-parallel to the reference edge.
    */
	public get incidentEdge(): number /** int */{
		return this._incidentEdge;
	}

	public set incidentEdge(value: number /** int */) {
		this._incidentEdge = value;
		this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
	}

	public _incidentEdge: number /** int */;

	/**
    * The vertex (0 or 1) on the incident edge that was clipped.
    */
	public get incidentVertex(): number /** int */{
		return this._incidentVertex;
	}

	public set incidentVertex(value: number /** int */) {
		this._incidentVertex = value;
		this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
	}

	public _incidentVertex: number /** int */;

	/**
    * A value of 1 indicates that the reference edge is on shape2.
    */
	public get flip(): number /** int */{
		return this._flip;
	}

	public set flip(value: number /** int */) {
		this._flip = value;
		this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
	}

	public _flip: number /** int */;

	public _m_id: b2ContactID;
}