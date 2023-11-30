import { b2Vec2 } from '../Common/Math';
import { b2ContactID } from './b2ContactID';

/**
* @private
*/
export class ClipVertex {
	public Set(other: ClipVertex): void {
		this.v.SetV(other.v);
		this.id.Set(other.id);
	}

	public v: b2Vec2 = new b2Vec2();
	public id: b2ContactID = new b2ContactID();
}