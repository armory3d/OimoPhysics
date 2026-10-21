package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.collision.narrowphase.detector.gjkepa.*;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * ConvexGeometry vs StaticMeshGeometry collision detector.
 *
 * This detector reports the deepest triangle contact for the query shape. It is
 * not a full mesh manifold, so a shape deeply inside a closed mesh can exit
 * through any of the overlapping triangles.
 */
@:build(oimo.m.B.bu())
class ConvexStaticMeshDetector extends Detector {
	static inline var QUERY_DEPTH_EPSILON:Float = 1e-6;

	var queryMode:Bool;

	// Scratch data. Static-mesh detection is frequent enough that allocating
	// these per triangle is measurable in both contacts and queries.
	var triangleIndices:Array<Int>;
	var worldAabb:Aabb;
	var localAabb:Aabb;
	var triangleVertices:Array<Vec3>;
	var triangleGeom:ConvexHullGeometry;
	var edge1:Vec3;
	var edge2:Vec3;
	var toConvex:Vec3;
	var meshPoint:Vec3;
	var bestNormalWorld:Vec3;
	var bestContactPointConvex:Vec3;
	var bestContactPointMesh:Vec3;

	/**
	 * `queryMode` enables strict positive-overlap semantics for convex overlap
	 * queries. Contact detection keeps the existing touching behavior.
	 */
	public function new(swapped:Bool = false, queryMode:Bool = false) {
		super(swapped);
		this.queryMode = queryMode;

		triangleIndices = [];
		worldAabb = new Aabb();
		localAabb = new Aabb();
		triangleVertices = [new Vec3(), new Vec3(), new Vec3()];
		triangleGeom = new ConvexHullGeometry(triangleVertices);
		triangleGeom._gjkMargin = 0;
		edge1 = new Vec3();
		edge2 = new Vec3();
		toConvex = new Vec3();
		meshPoint = new Vec3();
		bestNormalWorld = new Vec3();
		bestContactPointConvex = new Vec3();
		bestContactPointMesh = new Vec3();
	}

	override function detectImpl(result:DetectorResult, geom1:Geometry, geom2:Geometry, tf1:Transform, tf2:Transform, cachedData:CachedDetectorData):Void {
		var convex:ConvexGeometry = cast geom1;
		var staticMesh:StaticMeshGeometry = cast geom2;

		var indices:Array<Int> = triangleIndices;
		var worldAabb:Aabb = this.worldAabb;
		var localAabb:Aabb = this.localAabb;
		var triVertices:Array<Vec3> = triangleVertices;
		var triGeom:ConvexHullGeometry = triangleGeom;
		var edge1:Vec3 = this.edge1;
		var edge2:Vec3 = this.edge2;
		var toConvex:Vec3 = this.toConvex;
		var meshPoint:Vec3 = this.meshPoint;
		var bestNormal:Vec3 = bestNormalWorld;
		var bestPointConvex:Vec3 = bestContactPointConvex;
		var bestPointMesh:Vec3 = bestContactPointMesh;
		var isQuery:Bool = queryMode;

		result.incremental = false;

		convex._computeAabb(worldAabb, tf1);
		staticMesh.worldAabbToLocal(worldAabb, tf2, localAabb);

		staticMesh.queryTrianglesInto(localAabb, indices);
		if (indices.length == 0) {
			return;
		}

		var gjkEpa:GjkEpa = GjkEpa.getInstance();

		var bestPenetrationDepth:Float = MathUtil.NEGATIVE_INFINITY;
		var bestContactFound:Bool = false;

		var margin1:Float = convex._gjkMargin;
		if (isQuery && convex._type == GeometryType._CONVEX_HULL) {
			// Hull support points are already on the hull surface, unlike the
			// shrunken cores used by primitives. Adding the margin would inflate
			// the hull and manufacture hits for queries that only touch it.
			margin1 = 0;
		}
		var margin2:Float = triGeom._gjkMargin;
		var marginSum:Float = margin1 + margin2;

		for (triangleIndex in indices) {
			staticMesh.getTriangleVertices(triangleIndex, triVertices[0], triVertices[1], triVertices[2]);

			var v1:Vec3 = triVertices[0];
			var v2:Vec3 = triVertices[1];
			var v3:Vec3 = triVertices[2];

			edge1.copyFrom(v2).subEq(v1);
			edge2.copyFrom(v3).subEq(v1);
			if (edge1.crossEq(edge2).lengthSq() < 1e-12) {
				// Skip degenerate triangles.
				continue;
			}

			var status:Int = gjkEpa.computeClosestPoints(convex, triGeom, tf1, tf2, null);
			var distanceFallback:Bool = false;
			if (status != GjkEpaResultState.SUCCEEDED) {
				if (!isQuery) continue;

				// EPA can fail for a shape whose core lies exactly on a triangle
				// (for example, a sphere centered on a floor). The non-EPA path
				// still gives a conservative core distance, which is enough to
				// decide overlap; the witness points are approximate in this case.
				status = gjkEpa.computeDistance(convex, triGeom, tf1, tf2, null);
				if (status != GjkEpaResultState.SUCCEEDED) continue;
				distanceFallback = true;
			}

			var distance:Float = gjkEpa.distance;
			if (isQuery) {
				// A query reports strict overlap, not exact touching.
				if (distance >= marginSum) continue;
			} else if (distance > marginSum) {
				continue;
			}

			var penetrationDepth:Float = marginSum - distance;
			if (isQuery && !(penetrationDepth > QUERY_DEPTH_EPSILON)) {
				// Treat numerically tiny penetrations as touching. Exact touching
				// is not part of the query contract.
				continue;
			}

			if (penetrationDepth > bestPenetrationDepth) {
				if (distanceFallback) {
					// Build a stable world-space triangle normal and orient it
					// from the mesh toward the query center.
					var v1Local:IVec3;
					var v1World:IVec3;
					var normalLocal:IVec3;
					var normalWorld:IVec3;
					M.vec3_fromVec3(v1Local, v1);
					M.vec3_mulMat3(v1World, v1Local, tf2._rotation);
					M.vec3_add(v1World, v1World, tf2._position);
					M.vec3_toVec3(meshPoint, v1World);
					M.vec3_toVec3(toConvex, tf1._position);
					toConvex.subEq(meshPoint);

					M.vec3_fromVec3(normalLocal, edge1);
					M.vec3_mulMat3(normalWorld, normalLocal, tf2._rotation);
					M.vec3_toVec3(bestNormal, normalWorld);
					if (bestNormal.dot(toConvex) < 0) {
						bestNormal.negateEq();
					}

					if (bestNormal.lengthSq() < 1e-12) {
						continue;
					}
					bestNormal.normalize();

					bestPenetrationDepth = penetrationDepth;
					bestContactFound = true;
					M.vec3_toVec3(bestPointConvex, tf1._position);
					bestPointConvex.addScaledEq(bestNormal, -margin1);
					M.vec3_toVec3(bestPointMesh, tf1._position);
					continue;
				}

				bestNormal.copyFrom(gjkEpa.closestPoint1).subEq(gjkEpa.closestPoint2);
				if (distance < 0) {
					bestNormal.negateEq();
				}

				if (bestNormal.lengthSq() < 1e-12) {
					// GJK can produce coincident witness points for degenerate
					// configurations. Use the geometric triangle normal, transformed
					// to world space and oriented from the mesh toward the query.
					var normalLocal:IVec3;
					var normalWorld:IVec3;
					M.vec3_fromVec3(normalLocal, edge1);
					M.vec3_mulMat3(normalWorld, normalLocal, tf2._rotation);
					M.vec3_toVec3(bestNormal, normalWorld);
					M.vec3_toVec3(toConvex, tf1._position);
					toConvex.subEq(gjkEpa.closestPoint2);
					if (bestNormal.dot(toConvex) < 0) {
						bestNormal.negateEq();
					}
				}

				if (bestNormal.lengthSq() < 1e-12) {
					continue;
				}
				bestNormal.normalize();

				bestPenetrationDepth = penetrationDepth;
				bestContactFound = true;
				bestPointConvex.copyFrom(gjkEpa.closestPoint1).addScaledEq(bestNormal, -margin1);
				bestPointMesh.copyFrom(gjkEpa.closestPoint2).addScaledEq(bestNormal, margin2);
			}
		}

		if (bestContactFound) {
			var normalInWorld:IVec3;
			var contactPointConvex:IVec3;
			var contactPointMesh:IVec3;
			M.vec3_fromVec3(normalInWorld, bestNormal);
			M.vec3_fromVec3(contactPointConvex, bestPointConvex);
			M.vec3_fromVec3(contactPointMesh, bestPointMesh);

			M.call(setNormal(result, normalInWorld));
			M.call(addPoint(result, contactPointConvex, contactPointMesh, bestPenetrationDepth, 0));
		}
	}
}
