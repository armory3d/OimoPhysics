package oimo.dynamics.callback;
import oimo.collision.narrowphase.ConvexOverlapHit;
import oimo.dynamics.rigidbody.Shape;

/**
 * A callback interface for convex overlap queries in a world.
 */
@:interface
class ConvexOverlapCallback {
	/**
	 * Default constructor.
	 */
	public function new() {
	}

	/**
	 * This is called for every shape that overlaps the query shape.
	 *
	 * `hit` is reused by the world and is only valid during this call.
	 */
	public function process(shape:Shape, hit:ConvexOverlapHit):Void {
	}
}
