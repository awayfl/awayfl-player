import { b2AABB } from './b2AABB';

/**
 * A node in the dynamic tree. The client does not interact with this directly.
 * @private
 */
export class b2DynamicTreeNode {
	public IsLeaf(): boolean {
		return this.child1 == null;
	}

	public userData: any;
	public aabb: b2AABB = new b2AABB();
	public parent: b2DynamicTreeNode;
	public child1: b2DynamicTreeNode;
	public child2: b2DynamicTreeNode;
}