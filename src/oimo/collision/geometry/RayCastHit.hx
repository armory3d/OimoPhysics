package oimo.collision.geometry;
import oimo.common.Vec3;

/**
 * A single ray cast hit data.
 */
class RayCastHit {
	/**
	 * The position the ray hit at.
	 */
	public var position:Vec3;

	/**
	 * The normal vector of the surface the ray hit.
	 */
	public var normal:Vec3;

	/**
	 * The ratio of the position the ray hit from the start point to the end point.
	 */
	public var fraction:Float;

	/**
	 * The index of the static mesh triangle a convex cast hit, or `-1` when the hit
	 * geometry is not a static mesh. Ray casts leave it at `-1`.
	 */
	public var triangleIndex:Int;

	/**
	 * Default constructor.
	 */
	public function new() {
		position = new Vec3();
		normal = new Vec3();
		fraction = 0;
		triangleIndex = -1;
	}
}
