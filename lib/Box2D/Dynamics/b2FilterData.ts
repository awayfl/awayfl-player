/**
* This holds contact filtering data.
*/
export class b2FilterData {
	__fast__ = true;

	public Copy(): b2FilterData {
		const copy: b2FilterData = new b2FilterData();
		copy.categoryBits = this.categoryBits;
		copy.maskBits = this.maskBits;
		copy.groupIndex = this.groupIndex;
		return copy;
	}

	/**
    * The collision category bits. Normally you would just set one bit.
    */
	public categoryBits: number /** uint */ = 0x0001;

	/**
    * The collision mask bits. This states the categories that this
    * shape would accept for collision.
    */
	public maskBits: number /** uint */ = 0xFFFF;

	/**
    * Collision groups allow a certain group of objects to never collide (negative)
    * or always collide (positive). Zero means no collision group. Non-zero group
    * filtering always wins against the mask bits.
    */
	public groupIndex: number /** int */ = 0;
}