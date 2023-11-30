import { b2AABB } from './b2AABB';
import { b2DynamicTreeNode } from './b2DynamicTreeNode';
import { b2Vec2, b2Math } from '../Common/Math';
import { b2Settings } from '../Common/b2Settings';
import { b2RayCastInput } from './b2RayCastInput';

/**
 * A dynamic tree arranges data in a binary tree to accelerate
 * queries such as volume queries and ray casts. Leafs are proxies
 * with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
 * so that the proxy AABB is bigger than the client object. This allows the client
 * object to move by small amounts without triggering a tree update.
 *
 * Nodes are pooled.
 */
export class b2DynamicTree {
	/**
	 * Constructing the tree initializes the node pool.
	 */
	constructor() {
		this.m_root = null;

		// TODO: Maybe allocate some free nodes?
		this.m_freeList = null;
		this.m_path = 0;

		this.m_insertionCount = 0;
	}
	/*
	public Dump(node:b2DynamicTreeNode=null, depth:number=0):void
	{
		if (!node)
		{
			node = m_root;
		}
		if (!node) return;
		for (var i:number = 0; i < depth; i++) s += " ";
		if (node.userData)
		{
			var ud:* = (node.userData as b2Fixture).GetBody().GetUserData();
			trace(s + ud);
		}else {
			trace(s + "-");
		}
		if (node.child1)
			Dump(node.child1, depth + 1);
		if (node.child2)
			Dump(node.child2, depth + 1);
	}
	*/

	/**
	 * Create a proxy. Provide a tight fitting AABB and a userData.
	 */
	public CreateProxy(aabb: b2AABB, userData: any): b2DynamicTreeNode {
		const node: b2DynamicTreeNode = this.AllocateNode();

		// Fatten the aabb.
		const extendX: number = b2Settings.b2_aabbExtension;
		const extendY: number = b2Settings.b2_aabbExtension;
		node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
		node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
		node.aabb.upperBound.x = aabb.upperBound.x + extendX;
		node.aabb.upperBound.y = aabb.upperBound.y + extendY;

		node.userData = userData;

		this.InsertLeaf(node);
		return node;
	}

	/**
	 * Destroy a proxy. This asserts if the id is invalid.
	 */
	public DestroyProxy(proxy: b2DynamicTreeNode): void {
		//b2Settings.b2Assert(proxy.IsLeaf());
		this.RemoveLeaf(proxy);
		this.FreeNode(proxy);
	}

	/**
	 * Move a proxy with a swept AABB. If the proxy has moved outside of its fattened AABB,
	 * then the proxy is removed from the tree and re-inserted. Otherwise
	 * the function returns immediately.
	 */
	public MoveProxy(proxy: b2DynamicTreeNode, aabb: b2AABB, displacement: b2Vec2): boolean {
		b2Settings.b2Assert(proxy.IsLeaf());

		if (proxy.aabb.Contains(aabb)) {
			return false;
		}

		this.RemoveLeaf(proxy);

		// Extend AABB
		const extendX: number = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.x > 0 ? displacement.x : -displacement.x);
		const extendY: number = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.y > 0 ? displacement.y : -displacement.y);
		proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
		proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
		proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
		proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;

		this.InsertLeaf(proxy);
		return true;
	}

	/**
	 * Perform some iterations to re-balance the tree.
	 */
	public Rebalance(iterations: number /** int */): void {
		if (this.m_root == null)
			return;

		for (let i: number /** int */ = 0; i < iterations; i++) {
			let node: b2DynamicTreeNode = this.m_root;
			let bit: number /** uint */ = 0;
			while (node.IsLeaf() == false) {
				node = (this.m_path >> bit) & 1 ? node.child2 : node.child1;
				bit = (bit + 1) & 31; // 0-31 bits in a uint
			}
			++this.m_path;

			this.RemoveLeaf(node);
			this.InsertLeaf(node);
		}
	}

	public GetFatAABB(proxy: b2DynamicTreeNode): b2AABB {
		return proxy.aabb;
	}

	/**
	 * Get user data from a proxy. Returns null if the proxy is invalid.
	 */
	public GetUserData(proxy: b2DynamicTreeNode): any {
		return proxy.userData;
	}

	/**
	 * Query an AABB for overlapping proxies. The callback
	 * is called for each proxy that overlaps the supplied AABB.
	 * The callback should match function signature
	 * <code>fuction callback(proxy:b2DynamicTreeNode):boolean</code>
	 * and should return false to trigger premature termination.
	 */
	public Query(callback: Function, aabb: b2AABB): void {
		if (this.m_root == null)
			return;

		const stack: Array<b2DynamicTreeNode> = new Array<b2DynamicTreeNode>();

		let count: number /** int */ = 0;
		stack[count++] = this.m_root;

		while (count > 0) {
			const node: b2DynamicTreeNode = stack[--count];

			if (node.aabb.TestOverlap(aabb)) {
				if (node.IsLeaf()) {
					const proceed: boolean = callback(node);
					if (!proceed)
						return;
				} else {
					// No stack limit, so no assert
					stack[count++] = node.child1;
					stack[count++] = node.child2;
				}
			}
		}
	}

	/**
	 * Ray-cast against the proxies in the tree. This relies on the callback
	 * to perform a exact ray-cast in the case were the proxy contains a shape.
	 * The callback also performs the any collision filtering. This has performance
	 * roughly equal to k * log(n), where k is the number of collisions and n is the
	 * number of proxies in the tree.
	 * @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	 * @param callback a callback class that is called for each proxy that is hit by the ray.
	 * It should be of signature:
	 * <code>function callback(input:b2RayCastInput, proxy:*):void</code>
	 */
	public RayCast(callback: Function, input: b2RayCastInput): void {
		if (this.m_root == null)
			return;

		const p1: b2Vec2 = input.p1;
		const p2: b2Vec2 = input.p2;
		const r: b2Vec2 = b2Math.SubtractVV(p1, p2);
		//b2Settings.b2Assert(r.LengthSquared() > 0.0);
		r.Normalize();

		// v is perpendicular to the segment
		const v: b2Vec2 = b2Math.CrossFV(1.0, r);
		const abs_v: b2Vec2 = b2Math.AbsV(v);

		let maxFraction: number = input.maxFraction;

		// Build a bounding box for the segment
		const segmentAABB: b2AABB = new b2AABB();
		let tX: number;
		let tY: number;
		{
			tX = p1.x + maxFraction * (p2.x - p1.x);
			tY = p1.y + maxFraction * (p2.y - p1.y);
			segmentAABB.lowerBound.x = Math.min(p1.x, tX);
			segmentAABB.lowerBound.y = Math.min(p1.y, tY);
			segmentAABB.upperBound.x = Math.max(p1.x, tX);
			segmentAABB.upperBound.y = Math.max(p1.y, tY);
		}

		const stack: Array<b2DynamicTreeNode> = new Array<b2DynamicTreeNode>();

		let count: number /** int */ = 0;
		stack[count++] = this.m_root;

		while (count > 0) {
			const node: b2DynamicTreeNode = stack[--count];

			if (node.aabb.TestOverlap(segmentAABB) == false) {
				continue;
			}

			// Separating axis for segment (Gino, p80)
			// |dot(v, p1 - c)| > dot(|v|,h)

			const c: b2Vec2 = node.aabb.GetCenter();
			const h: b2Vec2 = node.aabb.GetExtents();
			const separation: number = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y))
									- abs_v.x * h.x - abs_v.y * h.y;
			if (separation > 0.0)
				continue;

			if (node.IsLeaf()) {
				const subInput: b2RayCastInput = new b2RayCastInput();
				subInput.p1 = input.p1;
				subInput.p2 = input.p2;
				subInput.maxFraction = input.maxFraction;

				maxFraction = callback(subInput, node);

				if (maxFraction == 0.0)
					return;

				//Update the segment bounding box
				{
					tX = p1.x + maxFraction * (p2.x - p1.x);
					tY = p1.y + maxFraction * (p2.y - p1.y);
					segmentAABB.lowerBound.x = Math.min(p1.x, tX);
					segmentAABB.lowerBound.y = Math.min(p1.y, tY);
					segmentAABB.upperBound.x = Math.max(p1.x, tX);
					segmentAABB.upperBound.y = Math.max(p1.y, tY);
				}
			} else {
				// No stack limit, so no assert
				stack[count++] = node.child1;
				stack[count++] = node.child2;
			}
		}
	}

	private AllocateNode(): b2DynamicTreeNode {
		// Peel a node off the free list
		if (this.m_freeList) {
			const node: b2DynamicTreeNode = this.m_freeList;
			this.m_freeList = node.parent;
			node.parent = null;
			node.child1 = null;
			node.child2 = null;
			return node;
		}

		// Ignore length pool expansion and relocation found in the C++
		// As we are using heap allocation
		return new b2DynamicTreeNode();
	}

	private FreeNode(node: b2DynamicTreeNode): void {
		node.parent = this.m_freeList;
		this.m_freeList = node;
	}

	private InsertLeaf(leaf: b2DynamicTreeNode): void {
		++this.m_insertionCount;

		if (this.m_root == null) {
			this.m_root = leaf;
			this.m_root.parent = null;
			return;
		}

		const center: b2Vec2 = leaf.aabb.GetCenter();
		let sibling: b2DynamicTreeNode = this.m_root;
		if (sibling.IsLeaf() == false) {
			do {
				const child1: b2DynamicTreeNode = sibling.child1;
				const child2: b2DynamicTreeNode = sibling.child2;

				//b2Vec2 delta1 = b2Abs(m_nodes[child1].aabb.GetCenter() - center);
				//b2Vec2 delta2 = b2Abs(m_nodes[child2].aabb.GetCenter() - center);
				//float32 norm1 = delta1.x + delta1.y;
				//float32 norm2 = delta2.x + delta2.y;

				const norm1: number = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x)
									+ Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
				const norm2: number = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x)
									+ Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);

				if (norm1 < norm2) {
					sibling = child1;
				} else {
					sibling = child2;
				}
			}
			while (sibling.IsLeaf() == false);
		}

		// Create a parent for the siblings
		let node1: b2DynamicTreeNode = sibling.parent;
		let node2: b2DynamicTreeNode = this.AllocateNode();
		node2.parent = node1;
		node2.userData = null;
		node2.aabb.Combine(leaf.aabb, sibling.aabb);
		if (node1) {
			if (sibling.parent.child1 == sibling) {
				node1.child1 = node2;
			} else {
				node1.child2 = node2;
			}

			node2.child1 = sibling;
			node2.child2 = leaf;
			sibling.parent = node2;
			leaf.parent = node2;
			do {
				if (node1.aabb.Contains(node2.aabb))
					break;

				node1.aabb.Combine(node1.child1.aabb, node1.child2.aabb);
				node2 = node1;
				node1 = node1.parent;
			}
			while (node1);
		} else {
			node2.child1 = sibling;
			node2.child2 = leaf;
			sibling.parent = node2;
			leaf.parent = node2;
			this.m_root = node2;
		}

	}

	private RemoveLeaf(leaf: b2DynamicTreeNode): void {
		if (leaf == this.m_root) {
			this.m_root = null;
			return;
		}

		const node2: b2DynamicTreeNode = leaf.parent;
		let node1: b2DynamicTreeNode = node2.parent;
		let sibling: b2DynamicTreeNode;
		if (node2.child1 == leaf) {
			sibling = node2.child2;
		} else {
			sibling = node2.child1;
		}

		if (node1) {
			// Destroy node2 and connect node1 to sibling
			if (node1.child1 == node2) {
				node1.child1 = sibling;
			} else {
				node1.child2 = sibling;
			}
			sibling.parent = node1;
			this.FreeNode(node2);

			// Adjust the ancestor bounds
			while (node1) {
				const oldAABB: b2AABB = node1.aabb;
				node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb);

				if (oldAABB.Contains(node1.aabb))
					break;

				node1 = node1.parent;
			}
		} else {
			this.m_root = sibling;
			sibling.parent = null;
			this.FreeNode(node2);
		}
	}

	private m_root: b2DynamicTreeNode;
	private m_freeList: b2DynamicTreeNode;

	/** This is used for incrementally traverse the tree for rebalancing */
	private m_path: number /** uint */;

	private m_insertionCount: number /** int */;
}