package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.collision.narrowphase.detector.gjkepa.*;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * ConvexGeometry vs StaticMeshGeometry collision detector.
 */
@:build(oimo.m.B.bu())
class ConvexStaticMeshDetector extends Detector {

	public function new(swapped:Bool = false) {
		super(swapped);
	}

	override function detectImpl(result:DetectorResult, geom1:Geometry, geom2:Geometry, tf1:Transform, tf2:Transform, cachedData:CachedDetectorData):Void {
		var convex:ConvexGeometry = cast geom1;
		var staticMesh:StaticMeshGeometry = cast geom2;

		result.incremental = false;

		var worldAABB:Aabb = new Aabb();
		convex._computeAabb(worldAABB, tf1);

		var localAABB:Aabb = new Aabb();
		_transformAABBToLocal(worldAABB, tf2, localAABB);

		var triangleIndices:Array<Int> = staticMesh.queryTriangles(localAABB);
		if (triangleIndices.length == 0) {
			return;
		}

		var gjkEpa:GjkEpa = GjkEpa.getInstance();
		var bestPenetrationDepth:Float = -Math.POSITIVE_INFINITY;
		var bestContactFound:Bool = false;
		var bestNormalWorld:Vec3 = new Vec3();
		var bestContactPointConvex:Vec3 = new Vec3();
		var bestContactPointMesh:Vec3 = new Vec3();

		// Reuse triangle geometry to avoid allocations
		var triangleVertices:Array<Vec3> = [new Vec3(), new Vec3(), new Vec3()];
		var triangleGeom:ConvexHullGeometry = null;
		for (triangleIndex in triangleIndices) {
			staticMesh.getTriangleVertices(triangleIndex, triangleVertices[0], triangleVertices[1], triangleVertices[2]);

			var v1 = triangleVertices[0];
			var v2 = triangleVertices[1];
			var v3 = triangleVertices[2];

			// Skip degenerate triangles
			var edge1 = new Vec3().copyFrom(v2).subEq(v1);
			var edge2 = new Vec3().copyFrom(v3).subEq(v1);
			var cross = edge1.cross(edge2);
			if (cross.lengthSq() < 1e-12) {
				continue;
			}

			if (triangleGeom == null) {
				triangleGeom = new ConvexHullGeometry(triangleVertices);
			} else {
				triangleGeom._vertices[0].copyFrom(v1);
				triangleGeom._vertices[1].copyFrom(v2);
				triangleGeom._vertices[2].copyFrom(v3);
				triangleGeom._updateMass();
			}
			triangleGeom._gjkMargin = 0;

			var status:Int = gjkEpa.computeClosestPoints(convex, triangleGeom, tf1, tf2, null);

			if (status != GjkEpaResultState.SUCCEEDED) {
				continue;
			}

			var margin1:Float = convex._gjkMargin;
			var margin2:Float = triangleGeom._gjkMargin;

			if (gjkEpa.distance > margin1 + margin2) {
				continue;
			}

			var penetrationDepth:Float = margin1 + margin2 - gjkEpa.distance;

			if (penetrationDepth > bestPenetrationDepth) {
				bestPenetrationDepth = penetrationDepth;
				bestContactFound = true;

				bestNormalWorld.copyFrom(gjkEpa.closestPoint1).subEq(gjkEpa.closestPoint2);
				if (gjkEpa.distance < 0) {
					bestNormalWorld.negateEq();
				}
				bestNormalWorld.normalize();

				bestContactPointConvex.copyFrom(gjkEpa.closestPoint1).addScaledEq(bestNormalWorld, -margin1);
				bestContactPointMesh.copyFrom(gjkEpa.closestPoint2).addScaledEq(bestNormalWorld, margin2);
			}
		}

		if (bestContactFound) {
			var normalInWorld:IVec3;
			M.vec3_fromVec3(normalInWorld, bestNormalWorld);

			setNormal(result, M.vec3_get(normalInWorld, 0), M.vec3_get(normalInWorld, 1), M.vec3_get(normalInWorld, 2));

			var contactPointConvex:IVec3;
			var contactPointMesh:IVec3;
			M.vec3_fromVec3(contactPointConvex, bestContactPointConvex);
			M.vec3_fromVec3(contactPointMesh, bestContactPointMesh);

			addPoint(result,
				M.vec3_get(contactPointConvex, 0), M.vec3_get(contactPointConvex, 1), M.vec3_get(contactPointConvex, 2),
				M.vec3_get(contactPointMesh, 0), M.vec3_get(contactPointMesh, 1), M.vec3_get(contactPointMesh, 2),
				bestPenetrationDepth, 0);
		}
	}

	// Helper method to transform AABB from world space to mesh local space
	function _transformAABBToLocal(worldAABB:Aabb, meshTransform:Transform, localAABB:Aabb):Void {
		var minCorner:IVec3;
		var maxCorner:IVec3;
		var transformedCorner:IVec3;

		M.vec3_assign(minCorner, worldAABB._min);
		M.vec3_sub(minCorner, minCorner, meshTransform._position);
		M.vec3_mulMat3Transposed(minCorner, minCorner, meshTransform._rotation);
		M.vec3_assign(localAABB._min, minCorner);
		M.vec3_assign(localAABB._max, minCorner);

		for (i in 0...8) {
			M.vec3_set(transformedCorner,
				(i & 1) != 0 ? M.vec3_get(worldAABB._max, 0) : M.vec3_get(worldAABB._min, 0),
				(i & 2) != 0 ? M.vec3_get(worldAABB._max, 1) : M.vec3_get(worldAABB._min, 1),
				(i & 4) != 0 ? M.vec3_get(worldAABB._max, 2) : M.vec3_get(worldAABB._min, 2)
			);

			M.vec3_sub(transformedCorner, transformedCorner, meshTransform._position);
			M.vec3_mulMat3Transposed(transformedCorner, transformedCorner, meshTransform._rotation);

			M.vec3_min(localAABB._min, localAABB._min, transformedCorner);
			M.vec3_max(localAABB._max, localAABB._max, transformedCorner);
		}
	}
}