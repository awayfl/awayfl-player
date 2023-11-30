import { b2Body } from '../b2Body';
import { b2Contact } from '../Contacts';

/**
* A contact edge is used to connect bodies and contacts together
* in a contact graph where each body is a node and each contact
* is an edge. A contact edge belongs to a doubly linked list
* maintained in each attached body. Each contact has two contact
* nodes, one for each attached body.
*/
export class b2ContactEdge {
	public other: b2Body;
	public contact: b2Contact;
	public prev: b2ContactEdge;
	public next: b2ContactEdge;
}