package oimo.collision.narrowphase;
import haxe.ds.Vector;
import oimo.common.Setting;
import oimo.common.Vec3;

/**
 * The result of a convex overlap query.
 */
@:build(oimo.m.B.bu())
class ConvexOverlapHit {
	/**
	 * The contact normal, pointing from the target geometry towards the query
	 * geometry. This is the direction in which the query shape is pushed out.
	 */
	public var normal:Vec3;

	/**
	 * The number of valid points in `points`.
	 */
	public var numPoints:Int;

	/**
	 * The contact points. Only the first `numPoints` entries are valid.
	 */
	public var points:Vector<ConvexOverlapPoint>;

	@:dox(hide)
	public function new() {
		normal = new Vec3();
		numPoints = 0;
		points = new Vector<ConvexOverlapPoint>(Setting.maxManifoldPoints);
		for (i in 0...Setting.maxManifoldPoints) {
			points[i] = new ConvexOverlapPoint();
		}
	}

	/**
	 * Copies the detector result into this hit.
	 */
	public function copyFrom(result:DetectorResult):Void {
		normal.copyFrom(result.normal);

		var count:Int = result.numPoints;
		if (count > points.length) count = points.length;
		numPoints = count;

		for (i in 0...count) {
			var source:DetectorResultPoint = result.points[i];
			var target:ConvexOverlapPoint = points[i];
			target.position1.copyFrom(source.position1);
			target.position2.copyFrom(source.position2);
			target.depth = source.depth;
		}
	}

	/**
	 * Returns the maximum penetration depth of the valid points, or `0` when
	 * there are no points.
	 */
	public function getMaxDepth():Float {
		var max:Float = 0;
		for (i in 0...numPoints) {
			if (points[i].depth > max) max = points[i].depth;
		}
		return max;
	}

	/**
	 * Clears the result.
	 */
	public inline function clear():Void {
		normal.zero();
		numPoints = 0;
		for (p in points) {
			p.position1.zero();
			p.position2.zero();
			p.depth = 0;
		}
	}
}
