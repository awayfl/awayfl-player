import { b2Fixture } from './b2Fixture';
import { b2FilterData } from './b2FilterData';

/**
* Implement this class to provide collision filtering. In other words, you can implement
* this class if you want finer control over contact creation.
*/
export class b2ContactFilter {
	__fast__: boolean = true;

	/**
	* Return true if contact calculations should be performed between these two fixtures.
	* @warning for performance reasons this is only called when the AABBs begin to overlap.
	*/
	public ShouldCollide(fixtureA: b2Fixture, fixtureB: b2Fixture): Boolean {
		const filter1: b2FilterData = fixtureA.GetFilterData();
		const filter2: b2FilterData = fixtureB.GetFilterData();

		if (filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0) {
			return filter1.groupIndex > 0;
		}

		const collide: boolean = (filter1.maskBits & filter2.categoryBits) != 0 && (filter1.categoryBits & filter2.maskBits) != 0;
		return collide;
	}

	/**
	* Return true if the given fixture should be considered for ray intersection.
	* By default, userData is cast as a b2Fixture and collision is resolved according to ShouldCollide
	* @see ShouldCollide()
	* @see b2World#Raycast
	* @param userData	arbitrary data passed from Raycast or RaycastOne
	* @param fixture		the fixture that we are testing for filtering
	* @return a Boolean, with a value of false indicating that this fixture should be ignored.
	*/
	public RayCollide(userData: any, fixture: b2Fixture): Boolean {
		if (!userData)
			return true;
		return this.ShouldCollide(userData as b2Fixture, fixture);
	}

	public static b2_defaultFilter: b2ContactFilter = new b2ContactFilter();

}
