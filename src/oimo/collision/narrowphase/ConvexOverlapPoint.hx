package oimo.collision.narrowphase;
import oimo.common.Vec3;

/**
 * A contact point produced by a convex overlap query.
 */
@:build(oimo.m.B.bu())
class ConvexOverlapPoint {
	/**
	 * The closest point on the query geometry.
	 */
	public var position1:Vec3;

	/**
	 * The closest point on the target geometry.
	 */
	public var position2:Vec3;

	/**
	 * The amount of overlap. A value of `0` means the geometries are touching.
	 */
	public var depth:Float;

	@:dox(hide)
	public function new() {
		position1 = new Vec3();
		position2 = new Vec3();
		depth = 0;
	}
}
