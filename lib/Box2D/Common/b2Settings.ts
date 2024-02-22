/**
* This class controls Box2D global settings
*/
export class b2Settings {

	/**
    * The current version of Box2D
    */
	public static readonly VERSION: String = '2.1alpha';

	public static readonly USHRT_MAX: number /** int */ = 0x0000ffff;

	public static readonly b2_pi: number = Math.PI;

	// Collision
	/**
     *   Number of manifold points in a b2Manifold. This should NEVER change.
     */
	public static readonly b2_maxManifoldPoints: number /** int */ = 2;

	/*
     * The growable broadphase doesn't have upper limits,
	 * so there is no b2_maxProxies or b2_maxPairs settings.
     */
	//public static readonly b2_maxProxies:number /** int */ = 0;
	//public static readonly b2_maxPairs:number /** int */ = 8 * b2_maxProxies;

	/**
	 * This is used to fatten AABBs in the dynamic tree. This allows proxies
	 * to move by a small amount without triggering a tree adjustment.
	 * This is in meters.
	 */
	public static readonly b2_aabbExtension: number = 0.1;

 	/**
 	 * This is used to fatten AABBs in the dynamic tree. This is used to predict
 	 * the future position based on the current displacement.
	 * This is a dimensionless multiplier.
	 */
	public static readonly b2_aabbMultiplier: number = 2.0;

	// Dynamics

	/**
	* A small length used as a collision and readonlyraint tolerance. Usually it is
	* chosen to be numerically significant, but visually insignificant.
	*/
	public static readonly b2_linearSlop: number = 0.005;	// 0.5 cm

	/**
	 * The radius of the polygon/edge shape skin. This should not be modified. Making
	 * this smaller means polygons will have and insufficient for continuous collision.
	 * Making it larger may create artifacts for vertex collision.
	 */
	public static readonly b2_polygonRadius: number = 2.0 * b2Settings.b2_linearSlop;

	/**
	* A small angle used as a collision and readonlyraint tolerance. Usually it is
	* chosen to be numerically significant, but visually insignificant.
	*/
	public static readonly b2_angularSlop: number = 2.0 / 180.0 * b2Settings.b2_pi;			// 2 degrees

	/**
	* Continuous collision detection (CCD) works with core, shrunken shapes. This is the
	* amount by which shapes are automatically shrunk to work with CCD. This must be
	* larger than b2_linearSlop.
    * @see b2_linearSlop
	*/
	public static readonly b2_toiSlop: number = 8.0 * b2Settings.b2_linearSlop;

	/**
	* Maximum number of contacts to be handled to solve a TOI island.
	*/
	public static readonly b2_maxTOIContactsPerIsland: number /** int */ = 32;

	/**
	* Maximum number of joints to be handled to solve a TOI island.
	*/
	public static readonly b2_maxTOIJointsPerIsland: number /** int */ = 32;

	/**
	* A velocity threshold for elastic collisions. Any collision with a relative linear
	* velocity below this threshold will be treated as inelastic.
	*/
	public static readonly b2_velocityThreshold: number = 1.0;		// 1 m/s

	/**
	* The maximum linear position correction used when solving readonlyraints. This helps to
	* prevent overshoot.
	*/
	public static readonly b2_maxLinearCorrection: number = 0.2;	// 20 cm

	/**
	* The maximum angular position correction used when solving readonlyraints. This helps to
	* prevent overshoot.
	*/
	public static readonly b2_maxAngularCorrection: number = 8.0 / 180.0 * b2Settings.b2_pi;			// 8 degrees

	/**
	* The maximum linear velocity of a body. This limit is very large and is used
	* to prevent numerical problems. You shouldn't need to adjust this.
	*/
	public static readonly b2_maxTranslation: number = 2.0;
	public static readonly b2_maxTranslationSquared: number = b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation;

	/**
	* The maximum angular velocity of a body. This limit is very large and is used
	* to prevent numerical problems. You shouldn't need to adjust this.
	*/
	public static readonly b2_maxRotation: number = 0.5 * b2Settings.b2_pi;
	public static readonly b2_maxRotationSquared: number = b2Settings.b2_maxRotation * b2Settings.b2_maxRotation;

	/**
	* This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
	* that overlap is removed in one time step. However using values close to 1 often lead
	* to overshoot.
	*/
	public static readonly b2_contactBaumgarte: number = 0.2;

	/**
	 * Friction mixing law. Feel free to customize this.
	 */
	public static b2MixFriction(friction1: number, friction2: number): number {
		return Math.sqrt(friction1 * friction2);
	}

	/**
	 * Restitution mixing law. Feel free to customize this.
	 */
	public static b2MixRestitution(restitution1: number, restitution2: number): number {
		return restitution1 > restitution2 ? restitution1 : restitution2;
	}

	// Sleep

	/**
	* The time that a body must be still before it will go to sleep.
	*/
	public static readonly b2_timeToSleep: number = 0.5;					// half a second
	/**
	* A body cannot sleep if its linear velocity is above this tolerance.
	*/
	public static readonly b2_linearSleepTolerance: number = 0.01;			// 1 cm/s
	/**
	* A body cannot sleep if its angular velocity is above this tolerance.
	*/
	public static readonly b2_angularSleepTolerance: number = 2.0 / 180.0 * b2Settings.b2_pi;	// 2 degrees/s

	// assert
	/**
    * b2Assert is used internally to handle assertions. By default, calls are commented out to save performance,
    * so they serve more as documentation than anything else.
    */
	public static b2Assert(a: boolean): void {
		if (!a) {
			//var nullVec:b2Vec2;
			//nullVec.x++;
			throw 'Assertion Failed';
		}
	}
}
