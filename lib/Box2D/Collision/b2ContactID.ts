import { Features } from './Features';

/**
* We use contact ids to facilitate warm starting.
*/
export class b2ContactID {
	readonly __fast__ = true;

	constructor() {
		this.features._m_id = this;

	}

	public Set(id: b2ContactID): void {
		this.key = id._key;
	}

	public Copy(): b2ContactID {
		const id: b2ContactID = new b2ContactID();
		id.key = this.key;
		return id;
	}

	public get key(): number /** uint */ {
		return this._key;
	}

	public set key(value: number /** uint */) {
		this._key = value;
		this.features._referenceEdge = this._key & 0x000000ff;
		this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
		this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
		this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
	}

	public features: Features = new Features();
	/** Used to quickly compare contact ids. */
	public _key: number /** uint */;
}