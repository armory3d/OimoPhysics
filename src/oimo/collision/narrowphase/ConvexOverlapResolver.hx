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

	public function new() {
		convex = null;
		transform = new Transform();
		collisionMatrix = new CollisionMatrix();
		staticMeshDetector = new ConvexStaticMeshDetector(false, true);
		result = new DetectorResult();
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
	 * Tests `target` against the prepared query shape and writes the overlap
	 * manifold to `hit`.
	 */
	public function resolve(target:Geometry, targetTransform:Transform, hit:ConvexOverlapHit):Bool {
		if (convex == null || target == null || targetTransform == null || hit == null) return false;

		if (target._type == GeometryType._STATIC_MESH) {
			// Use a dedicated query-mode detector in convex-first order. This
			// avoids the shared contact detector's hull margin and persistence,
			// and guarantees the normal points from the mesh to the query shape.
			staticMeshDetector.detect(result, convex, target, transform, targetTransform, null);
		} else {
			var detector:Detector = collisionMatrix.getDetector(convex._type, target._type);
			if (detector == null) return false;

			detector.detect(result, convex, target, transform, targetTransform, null);
		}

		if (result.numPoints == 0) return false;

		hit.copyFrom(result);
		return true;
	}
}
