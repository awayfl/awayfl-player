import { b2TimeStep } from '../b2TimeStep';
import { b2Body } from '../b2Body';
import { b2ControllerEdge } from './b2ControllerEdge';
import { b2World } from '../b2World';
import { b2DebugDraw } from '../b2DebugDraw';

/**
 * Base class for controllers. Controllers are a convience for encapsulating common
 * per-step functionality.
 */
export class b2Controller {
	public Step(step: b2TimeStep): void {}

	public Draw(debugDraw: b2DebugDraw): void { }

	public AddBody(body: b2Body): void {
		const edge: b2ControllerEdge = new b2ControllerEdge();
		edge.controller = this;
		edge.body = body;
		//
		edge.nextBody = this.m_bodyList;
		edge.prevBody = null;
		this.m_bodyList = edge;
		if (edge.nextBody)
			edge.nextBody.prevBody = edge;
		this.m_bodyCount++;
		//
		edge.nextController = body.m_controllerList;
		edge.prevController = null;
		body.m_controllerList = edge;
		if (edge.nextController)
			edge.nextController.prevController = edge;
		body.m_controllerCount++;
	}

	public RemoveBody(body: b2Body): void {
		let edge: b2ControllerEdge = body.m_controllerList;
		while (edge && edge.controller != this)
			edge = edge.nextController;

		//Attempted to remove a body that was not attached?
		//b2Settings.b2Assert(bEdge != null);

		if (edge.prevBody)
			edge.prevBody.nextBody = edge.nextBody;
		if (edge.nextBody)
			edge.nextBody.prevBody = edge.prevBody;
		if (edge.nextController)
			edge.nextController.prevController = edge.prevController;
		if (edge.prevController)
			edge.prevController.nextController = edge.nextController;
		if (this.m_bodyList == edge)
			this.m_bodyList = edge.nextBody;
		if (body.m_controllerList == edge)
			body.m_controllerList = edge.nextController;
		body.m_controllerCount--;
		this.m_bodyCount--;
		//b2Settings.b2Assert(body.m_controllerCount >= 0);
		//b2Settings.b2Assert(m_bodyCount >= 0);
	}

	public Clear(): void {
		while (this.m_bodyList)
			this.RemoveBody(this.m_bodyList.body);
	}

	public GetNext(): b2Controller {return this.m_next;}
	public GetWorld(): b2World { return this.m_world; }

	public GetBodyList(): b2ControllerEdge {
		return this.m_bodyList;
	}

	public m_next: b2Controller;
	public m_prev: b2Controller;

	protected m_bodyList: b2ControllerEdge;
	protected m_bodyCount: number /** int */;

	public m_world: b2World;
}