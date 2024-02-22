import { b2Body } from '../b2Body';
import { b2Controller } from './b2Controller';

export class b2ControllerEdge {
	/** provides quick access to other end of this edge */
	public controller: b2Controller;
	/** the body */
	public body: b2Body;
	/** the previous controller edge in the controllers's body list */
	public prevBody: b2ControllerEdge;
	/** the next controller edge in the controllers's body list */
	public nextBody: b2ControllerEdge;
	/** the previous controller edge in the body's controller list */
	public prevController: b2ControllerEdge;
	/** the next controller edge in the body's controller list */
	public nextController: b2ControllerEdge;
}