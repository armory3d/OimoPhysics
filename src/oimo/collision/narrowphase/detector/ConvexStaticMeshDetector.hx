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
 * through any of the overlapping triangles. Spheres use a closest-point-on-
 * triangle fast path; other convex shapes use GJK/EPA per triangle.
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
	var sphereCenterLocal:Vec3;
	var sphereClosestLocal:Vec3;
	var sphereNormalLocal:Vec3;
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
		sphereCenterLocal = new Vec3();
		sphereClosestLocal = new Vec3();
		sphereNormalLocal = new Vec3();
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

		var margin1:Float = convex._gjkMargin;
		if (isQuery && convex._type == GeometryType._CONVEX_HULL) {
			// Hull support points are already on the hull surface, unlike the
			// shrunken cores used by primitives. Adding the margin would inflate
			// the hull and manufacture hits for queries that only touch it.
			margin1 = 0;
		}
		var margin2:Float = triGeom._gjkMargin;
		var marginSum:Float = margin1 + margin2;

		if (convex._type == GeometryType._SPHERE) {
			detectSphere(result, staticMesh, tf1, tf2, margin1, margin2, isQuery);
			return;
		}

		convex._computeAabb(worldAabb, tf1);
		staticMesh.worldAabbToLocal(worldAabb, tf2, localAabb);

		staticMesh.queryTrianglesInto(localAabb, indices);
		if (indices.length == 0) {
			return;
		}

		var bestPenetrationDepth:Float = MathUtil.NEGATIVE_INFINITY;
		var bestContactFound:Bool = false;

		var gjkEpa:GjkEpa = GjkEpa.getInstance();

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
			setBestContact(result, bestPenetrationDepth);
		}
	}

	function setBestContact(result:DetectorResult, depth:Float):Void {
		var normalInWorld:IVec3;
		var contactPointConvex:IVec3;
		var contactPointMesh:IVec3;
		M.vec3_fromVec3(normalInWorld, bestNormalWorld);
		M.vec3_fromVec3(contactPointConvex, bestContactPointConvex);
		M.vec3_fromVec3(contactPointMesh, bestContactPointMesh);

		M.call(setNormal(result, normalInWorld));
		M.call(addPoint(result, contactPointConvex, contactPointMesh, depth, 0));
	}

	function detectSphere(
		result:DetectorResult,
		staticMesh:StaticMeshGeometry,
		tf1:Transform,
		tf2:Transform,
		margin1:Float,
		margin2:Float,
		isQuery:Bool
	):Void {
		var meshVertices = staticMesh.getVertices();
		var meshIndices = staticMesh.getIndices();
		var centerLocal:Vec3 = sphereCenterLocal;
		var closestLocal:Vec3 = sphereClosestLocal;
		var normalLocal:Vec3 = sphereNormalLocal;
		var bestNormal:Vec3 = bestNormalWorld;
		var bestPointConvex:Vec3 = bestContactPointConvex;
		var bestPointMesh:Vec3 = bestContactPointMesh;

		// Work in mesh-local space: the sphere core is its center.
		var centerRel:IVec3;
		var centerLocalI:IVec3;
		M.vec3_sub(centerRel, tf1._position, tf2._position);
		M.vec3_mulMat3Transposed(centerLocalI, centerRel, tf2._rotation);
		M.vec3_toVec3(centerLocal, centerLocalI);

		var px:Float = centerLocal.x;
		var py:Float = centerLocal.y;
		var pz:Float = centerLocal.z;
		var indices:Array<Int> = triangleIndices;
		// Use the effective sphere radius (the GJK margin), not the geometry
		// AABB, so the candidate set matches the exact distance test below.
		staticMesh.queryTrianglesSphereInto(px, py, pz, margin1, indices);
		if (indices.length == 0) return;

		var marginSum:Float = margin1 + margin2;
		var marginSq:Float = marginSum * marginSum;

		var bestDepth:Float = MathUtil.NEGATIVE_INFINITY;
		var found:Bool = false;

		for (triangleIndex in indices) {
			var vertexIndex:Int = triangleIndex * 3;
			var v1:Vec3 = meshVertices[meshIndices[vertexIndex]];
			var v2:Vec3 = meshVertices[meshIndices[vertexIndex + 1]];
			var v3:Vec3 = meshVertices[meshIndices[vertexIndex + 2]];

			var abx:Float = v2.x - v1.x;
			var aby:Float = v2.y - v1.y;
			var abz:Float = v2.z - v1.z;
			var acx:Float = v3.x - v1.x;
			var acy:Float = v3.y - v1.y;
			var acz:Float = v3.z - v1.z;
			var nx:Float = aby * acz - abz * acy;
			var ny:Float = abz * acx - abx * acz;
			var nz:Float = abx * acy - aby * acx;
			var normalLenSq:Float = nx * nx + ny * ny + nz * nz;
			if (normalLenSq < 1e-12) {
				continue;
			}

			var distanceSq:Float = closestPointOnTriangle(
				px, py, pz,
				v1.x, v1.y, v1.z,
				v2.x, v2.y, v2.z,
				v3.x, v3.y, v3.z,
				closestLocal
			);

			// Match the existing detector's exact-touch behavior: contact mode
			// requires positive penetration, while query mode reports a hit only
			// above QUERY_DEPTH_EPSILON.
			if (distanceSq >= marginSq) continue;

			var depth:Float;
			if (distanceSq <= 0) {
				// A sphere centered on a triangle is the one case where the
				// closest-point normal is undefined. Query mode uses the triangle
				// normal; contact mode keeps the previous no-contact behavior.
				if (!isQuery) continue;
				depth = marginSum;
				if (!(depth > QUERY_DEPTH_EPSILON)) continue;
				var invNormalLen:Float = 1.0 / MathUtil.sqrt(normalLenSq);
				normalLocal.init(nx * invNormalLen, ny * invNormalLen, nz * invNormalLen);
			} else {
				var distance:Float = MathUtil.sqrt(distanceSq);
				depth = marginSum - distance;
				if (isQuery && !(depth > QUERY_DEPTH_EPSILON)) continue;
				var invDistance:Float = 1.0 / distance;
				normalLocal.init(
					(px - closestLocal.x) * invDistance,
					(py - closestLocal.y) * invDistance,
					(pz - closestLocal.z) * invDistance
				);
			}

			if (depth > bestDepth) {
				var normalWorld:IVec3;
				var normalLocalI:IVec3;
				M.vec3_fromVec3(normalLocalI, normalLocal);
				M.vec3_mulMat3(normalWorld, normalLocalI, tf2._rotation);
				M.vec3_toVec3(bestNormal, normalWorld);
				bestNormal.normalize();

				var closestWorld:IVec3;
				var closestLocalI:IVec3;
				M.vec3_fromVec3(closestLocalI, closestLocal);
				M.vec3_mulMat3(closestWorld, closestLocalI, tf2._rotation);
				M.vec3_add(closestWorld, closestWorld, tf2._position);
				M.vec3_toVec3(bestPointMesh, closestWorld);

				M.vec3_toVec3(bestPointConvex, tf1._position);
				bestPointConvex.addScaledEq(bestNormal, -margin1);

				bestDepth = depth;
				found = true;

				// The maximum possible query depth is marginSum (distance 0),
				// so no later triangle can improve the result.
				if (isQuery && bestDepth >= marginSum) break;
			}
		}

		if (found) {
			setBestContact(result, bestDepth);
		}
	}

	function closestPointOnTriangle(
		px:Float, py:Float, pz:Float,
		ax:Float, ay:Float, az:Float,
		bx:Float, by:Float, bz:Float,
		cx:Float, cy:Float, cz:Float,
		out:Vec3
	):Float {
		var abx:Float = bx - ax;
		var aby:Float = by - ay;
		var abz:Float = bz - az;
		var acx:Float = cx - ax;
		var acy:Float = cy - ay;
		var acz:Float = cz - az;
		var apx:Float = px - ax;
		var apy:Float = py - ay;
		var apz:Float = pz - az;

		var d1:Float = abx * apx + aby * apy + abz * apz;
		var d2:Float = acx * apx + acy * apy + acz * apz;

		var qx:Float;
		var qy:Float;
		var qz:Float;

		if (d1 <= 0 && d2 <= 0) {
			qx = ax; qy = ay; qz = az;
		} else {
			var bpx:Float = px - bx;
			var bpy:Float = py - by;
			var bpz:Float = pz - bz;
			var d3:Float = abx * bpx + aby * bpy + abz * bpz;
			var d4:Float = acx * bpx + acy * bpy + acz * bpz;

			if (d3 >= 0 && d4 <= d3) {
				qx = bx; qy = by; qz = bz;
			} else {
				var vc:Float = d1 * d4 - d3 * d2;
				if (vc <= 0 && d1 >= 0 && d3 <= 0) {
					var v:Float = d1 / (d1 - d3);
					qx = ax + abx * v; qy = ay + aby * v; qz = az + abz * v;
				} else {
					var cpx:Float = px - cx;
					var cpy:Float = py - cy;
					var cpz:Float = pz - cz;
					var d5:Float = abx * cpx + aby * cpy + abz * cpz;
					var d6:Float = acx * cpx + acy * cpy + acz * cpz;

					if (d6 >= 0 && d5 <= d6) {
						qx = cx; qy = cy; qz = cz;
					} else {
						var vb:Float = d5 * d2 - d1 * d6;
						if (vb <= 0 && d2 >= 0 && d6 <= 0) {
							var w:Float = d2 / (d2 - d6);
							qx = ax + acx * w; qy = ay + acy * w; qz = az + acz * w;
						} else {
							var va:Float = d3 * d6 - d5 * d4;
							if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
								var w:Float = (d4 - d3) / ((d4 - d3) + (d5 - d6));
								qx = bx + (cx - bx) * w;
								qy = by + (cy - by) * w;
								qz = bz + (cz - bz) * w;
							} else {
								var denom:Float = va + vb + vc;
								if (denom == 0) {
									qx = ax; qy = ay; qz = az;
								} else {
									var invDenom:Float = 1.0 / denom;
									var v:Float = vb * invDenom;
									var w:Float = vc * invDenom;
									qx = ax + abx * v + acx * w;
									qy = ay + aby * v + acy * w;
									qz = az + abz * v + acz * w;
								}
							}
						}
					}
				}
			}
		}

		out.init(qx, qy, qz);
		var dx:Float = px - qx;
		var dy:Float = py - qy;
		var dz:Float = pz - qz;
		return dx * dx + dy * dy + dz * dz;
	}
}
