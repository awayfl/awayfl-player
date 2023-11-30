import { b2Contact } from './Contacts';
import { b2Manifold } from '../Collision/b2Manifold';
import { b2ContactImpulse } from './b2ContactImpulse';

/**
 * Implement this class to get contact information. You can use these results for
 * things like sounds and game logic. You can also get contact results by
 * traversing the contact lists after the time step. However, you might miss
 * some contacts because continuous physics leads to sub-stepping.
 * Additionally you may receive multiple callbacks for the same contact in a
 * single time step.
 * You should strive to make your callbacks efficient because there may be
 * many callbacks per time step.
 * @warning You cannot create/destroy Box2D entities inside these callbacks.
 */
export class b2ContactListener {
	__fast__: boolean = true;

	/**
	 * Called when two fixtures begin to touch.
	 */
	public BeginContact(contact: b2Contact): void { }

	/**
	 * Called when two fixtures cease to touch.
	 */
	public EndContact(contact: b2Contact): void { }

	/**
	 * This is called after a contact is updated. This allows you to inspect a
	 * contact before it goes to the solver. If you are careful, you can modify the
	 * contact manifold (e.g. disable contact).
	 * A copy of the old manifold is provided so that you can detect changes.
	 * Note: this is called only for awake bodies.
	 * Note: this is called even when the number of contact points is zero.
	 * Note: this is not called for sensors.
	 * Note: if you set the number of contact points to zero, you will not
	 * get an EndContact callback. However, you may get a BeginContact callback
	 * the next step.
	 */
	public PreSolve(contact: b2Contact, oldManifold: b2Manifold): void {}

	/**
	 * This lets you inspect a contact after the solver is finished. This is useful
	 * for inspecting impulses.
	 * Note: the contact manifold does not include time of impact impulses, which can be
	 * arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
	 * in a separate data structure.
	 * Note: this is only called for contacts that are touching, solid, and awake.
	 */
	public PostSolve(contact: b2Contact, impulse: b2ContactImpulse): void { }

	public static b2_defaultListener: b2ContactListener = new b2ContactListener();
}