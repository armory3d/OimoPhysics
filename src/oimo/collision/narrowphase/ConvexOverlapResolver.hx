package oimo.collision.narrowphase;
import oimo.collision.geometry.ConvexGeometry;
import oimo.collision.geometry.Geometry;
import oimo.collision.geometry.GeometryType;
import oimo.collision.narrowphase.detector.ConvexStaticMeshDetector;
import oimo.collision.narrowphase.detector.Detector;
import oimo.common.Transform;

/**
 * Resolves convex overlap queries against the supported collision geometries.
 */
@:build(oimo.m.B.bu())
class ConvexOverlapResolver {
	var convex:ConvexGeometry;
	var transform:Transform;
	var collisionMatrix:CollisionMatrix;
	var staticMeshDetector:ConvexStaticMeshDetector;
	var result:DetectorResult;
	var convexHit:ConvexOverlapHit;
	var meshHits:Bool;

	public function new() {
		convex = null;
		transform = new Transform();
		collisionMatrix = new CollisionMatrix();
		staticMeshDetector = new ConvexStaticMeshDetector(false, true);
		result = new DetectorResult();
		convexHit = new ConvexOverlapHit();
		meshHits = false;
	}

	/**
	 * Prepares the query shape and its world transform.
	 */
	public function prepare(convex:ConvexGeometry, transform:Transform):Void {
		if (convex == null || transform == null) {
			this.convex = null;
			return;
		}
		this.convex = convex;
		this.transform.copyFrom(transform);
	}

	/**
	 * Tests `target` against the prepared query shape and returns the number of
	 * hits: at most one for a convex target, one per overlapping triangle for a
	 * static mesh. Read them with `getHit`; they stay valid until the next call.
	 */
	public function resolve(target:Geometry, targetTransform:Transform):Int {
		meshHits = false;
		if (convex == null || target == null || targetTransform == null) return 0;

		if (target._type == GeometryType._STATIC_MESH) {
			// A dedicated query-mode detector in convex-first order. This avoids
			// the shared contact detector's hull margin and persistence, and
			// guarantees the normal points from the mesh to the query shape.
			meshHits = true;
			return staticMeshDetector.detectAll(convex, cast target, transform, targetTransform);
		}

		var detector:Detector = collisionMatrix.getDetector(convex._type, target._type);
		if (detector == null) return 0;

		detector.detect(result, convex, target, transform, targetTransform, null);
		if (result.numPoints == 0) return 0;

		convexHit.copyFrom(result);
		return 1;
	}

	public inline function getHit(index:Int):ConvexOverlapHit {
		return meshHits ? staticMeshDetector.getContact(index) : convexHit;
	}
}
