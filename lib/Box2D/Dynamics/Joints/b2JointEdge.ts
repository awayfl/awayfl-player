/**
* A joint edge is used to connect bodies and joints together
* in a joint graph where each body is a node and each joint
* is an edge. A joint edge belongs to a doubly linked list
* maintained in each attached body. Each joint has two joint
* nodes, one for each attached body.
*/

import { b2Body } from '../b2Body';
import { b2Joint } from '../Joints';

export class b2JointEdge {

	/** Provides quick access to the other body attached. */
	public other: b2Body;
	/** The joint */
	public joint: b2Joint;
	/** The previous joint edge in the body's joint list */
	public prev: b2JointEdge;
	/** The next joint edge in the body's joint list */
	public next: b2JointEdge;
}