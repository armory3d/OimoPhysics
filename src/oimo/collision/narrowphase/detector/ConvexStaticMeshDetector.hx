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
 * Contacts (and `detect`) report the deepest triangle contact; `detectAll`
 * reports one contact per overlapping triangle, for queries. Neither is a full
 * mesh manifold, so a shape deeply inside a closed mesh can exit
 * through any of the overlapping triangles. Spheres use a closest-point-on-
 * triangle fast path. Capsules use a segment-tight candidate query before the
 * usual per-triangle GJK/EPA test; other convex shapes use GJK/EPA per triangle.
 *
 * The split between candidate selection and exact per-triangle testing is the
 * same one Bullet Physics uses for convex-vs-concave shapes (cull triangles, then test
 * each one).
 */
@:build(oimo.m.B.bu())
class ConvexStaticMeshDetector extends Detector {
	static inline var QUERY_DEPTH_EPSILON:Float = 1e-6;

	var queryMode:Bool;

	// Where triangle contacts go: the deepest is kept, or with `collectAll`
	// (`detectAll`) every one is appended to `contacts`.
	var collectAll:Bool;
	var contacts:Array<ConvexOverlapHit>;
	var numContacts:Int;
	var collectResult:DetectorResult;
	var currentTriangle:Int;
	var bestDepth:Float;
	var bestFound:Bool;

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
	var capsuleCenterLocal:Vec3;
	var capsuleAxisLocal:Vec3;
	var capsuleP0:Vec3;
	var capsuleP1:Vec3;
	var capsuleUnitY:Vec3;
	var bestNormalWorld:Vec3;
	var bestContactPointConvex:Vec3;
	var bestContactPointMesh:Vec3;
	var candidateNormal:Vec3;
	var candidatePointConvex:Vec3;
	var candidatePointMesh:Vec3;

	/**
	 * `queryMode` enables strict positive-overlap semantics for convex overlap
	 * queries. Contact detection keeps the existing touching behavior.
	 */
	public function new(swapped:Bool = false, queryMode:Bool = false) {
		super(swapped);
		this.queryMode = queryMode;

		collectAll = false;
		contacts = [new ConvexOverlapHit()];
		numContacts = 0;
		collectResult = new DetectorResult();
		currentTriangle = -1;
		bestDepth = 0;
		bestFound = false;

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
		capsuleCenterLocal = new Vec3();
		capsuleAxisLocal = new Vec3();
		capsuleP0 = new Vec3();
		capsuleP1 = new Vec3();
		capsuleUnitY = new Vec3(0, 1, 0);
		bestNormalWorld = new Vec3();
		bestContactPointConvex = new Vec3();
		bestContactPointMesh = new Vec3();
		candidateNormal = new Vec3();
		candidatePointConvex = new Vec3();
		candidatePointMesh = new Vec3();
	}

	/**
	 * Query mode only: finds one contact per triangle of `staticMesh` that
	 * `convex` overlaps, with the normal pointing from the triangle towards the
	 * convex. Returns the number of contacts; read them with `getContact`. They
	 * stay valid until the next call.
	 */
	public function detectAll(convex:ConvexGeometry, staticMesh:StaticMeshGeometry, convexTransform:Transform, meshTransform:Transform):Int {
		numContacts = 0;
		if (!queryMode || swapped) return 0;
		collectAll = true;
		collectResult.clear();
		try {
			detectImpl(collectResult, convex, staticMesh, convexTransform, meshTransform, null);
		} catch (e:Dynamic) {
			collectAll = false;
			throw e;
		}
		collectAll = false;
		return numContacts;
	}

	public inline function getContact(index:Int):ConvexOverlapHit {
		return contacts[index];
	}

	// Whether a contact of this depth would be kept; lets callers skip building it.
	inline function wants(depth:Float):Bool {
		return collectAll || !bestFound || depth > bestDepth;
	}

	// Takes the candidate contact of `currentTriangle`.
	function report(depth:Float):Void {
		if (collectAll) {
			if (numContacts == contacts.length) contacts.push(new ConvexOverlapHit());
			contacts[numContacts++].setSingle(candidateNormal, candidatePointConvex, candidatePointMesh, depth, currentTriangle);
			return;
		}
		bestNormalWorld.copyFrom(candidateNormal);
		bestContactPointConvex.copyFrom(candidatePointConvex);
		bestContactPointMesh.copyFrom(candidatePointMesh);
		bestDepth = depth;
		bestFound = true;
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
		var isQuery:Bool = queryMode;

		result.incremental = false;
		bestFound = false;

		// Hulls keep their GJK margin in queries too, so queries see them exactly
		// as contacts do (a hull is inflated by its margin, like in Bullet).
		var margin1:Float = convex._gjkMargin;
		var margin2:Float = triGeom._gjkMargin;
		var marginSum:Float = margin1 + margin2;

		if (convex._type == GeometryType._SPHERE) {
			detectSphere(result, staticMesh, tf1, tf2, margin1, margin2, isQuery);
			return;
		}

		if (convex._type == GeometryType._CAPSULE) {
			var capsule:CapsuleGeometry = cast convex;
			if (capsule._halfHeight <= 0) {
				// A zero-height capsule is a sphere.
				detectSphere(result, staticMesh, tf1, tf2, margin1, margin2, isQuery);
				return;
			}

			// Use a segment-tight candidate query instead of the capsule's
			// expanded AABB. For a capsule resting near a flat mesh this prunes
			// the box corners and most of the long axis, which dominates the
			// cost of large-mesh queries.
			var centerLocal:Vec3 = capsuleCenterLocal;
			var axisLocal:Vec3 = capsuleAxisLocal;
			var p0:Vec3 = capsuleP0;
			var p1:Vec3 = capsuleP1;
			var centerRel:IVec3;
			var centerLocalI:IVec3;
			M.vec3_sub(centerRel, tf1._position, tf2._position);
			M.vec3_mulMat3Transposed(centerLocalI, centerRel, tf2._rotation);
			M.vec3_toVec3(centerLocal, centerLocalI);

			var unitY:IVec3;
			var axisWorld:IVec3;
			var axisLocalI:IVec3;
			M.vec3_fromVec3(unitY, capsuleUnitY);
			M.vec3_mulMat3(axisWorld, unitY, tf1._rotation);
			M.vec3_mulMat3Transposed(axisLocalI, axisWorld, tf2._rotation);
			M.vec3_toVec3(axisLocal, axisLocalI);

			p0.copyFrom(centerLocal).addScaledEq(axisLocal, -capsule._halfHeight);
			p1.copyFrom(centerLocal).addScaledEq(axisLocal, capsule._halfHeight);
			staticMesh.queryTrianglesSegmentInto(
				p0.x, p0.y, p0.z,
				p1.x, p1.y, p1.z,
				marginSum, indices
			);
		} else {
			convex._computeAabb(worldAabb, tf1);
			staticMesh.worldAabbToLocal(worldAabb, tf2, localAabb);

			staticMesh.queryTrianglesInto(localAabb, indices);
		}

		if (indices.length == 0) {
			return;
		}

		var candNormal:Vec3 = candidateNormal;
		var candPointConvex:Vec3 = candidatePointConvex;
		var candPointMesh:Vec3 = candidatePointMesh;

		var gjkEpa:GjkEpa = GjkEpa.getInstance();

		for (triangleIndex in indices) {
			currentTriangle = triangleIndex;
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

			if (wants(penetrationDepth)) {
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
					M.vec3_toVec3(candNormal, normalWorld);
					if (candNormal.dot(toConvex) < 0) {
						candNormal.negateEq();
					}

					if (candNormal.lengthSq() < 1e-12) {
						continue;
					}
					candNormal.normalize();

					M.vec3_toVec3(candPointConvex, tf1._position);
					candPointConvex.addScaledEq(candNormal, -margin1);
					M.vec3_toVec3(candPointMesh, tf1._position);
					report(penetrationDepth);
					continue;
				}

				candNormal.copyFrom(gjkEpa.closestPoint1).subEq(gjkEpa.closestPoint2);
				if (distance < 0) {
					candNormal.negateEq();
				}

				if (candNormal.lengthSq() < 1e-12) {
					// GJK can produce coincident witness points for degenerate
					// configurations. Use the geometric triangle normal, transformed
					// to world space and oriented from the mesh toward the query.
					var normalLocal:IVec3;
					var normalWorld:IVec3;
					M.vec3_fromVec3(normalLocal, edge1);
					M.vec3_mulMat3(normalWorld, normalLocal, tf2._rotation);
					M.vec3_toVec3(candNormal, normalWorld);
					M.vec3_toVec3(toConvex, tf1._position);
					toConvex.subEq(gjkEpa.closestPoint2);
					if (candNormal.dot(toConvex) < 0) {
						candNormal.negateEq();
					}
				}

				if (candNormal.lengthSq() < 1e-12) {
					continue;
				}
				candNormal.normalize();

				candPointConvex.copyFrom(gjkEpa.closestPoint1).addScaledEq(candNormal, -margin1);
				candPointMesh.copyFrom(gjkEpa.closestPoint2).addScaledEq(candNormal, margin2);
				report(penetrationDepth);
			}
		}

		if (bestFound) setBestContact(result);
	}

	function setBestContact(result:DetectorResult):Void {
		var normalInWorld:IVec3;
		var contactPointConvex:IVec3;
		var contactPointMesh:IVec3;
		M.vec3_fromVec3(normalInWorld, bestNormalWorld);
		M.vec3_fromVec3(contactPointConvex, bestContactPointConvex);
		M.vec3_fromVec3(contactPointMesh, bestContactPointMesh);

		M.call(setNormal(result, normalInWorld));
		M.call(addPoint(result, contactPointConvex, contactPointMesh, bestDepth, 0));
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

		for (triangleIndex in indices) {
			currentTriangle = triangleIndex;
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

			if (wants(depth)) {
				var normalWorld:IVec3;
				var normalLocalI:IVec3;
				M.vec3_fromVec3(normalLocalI, normalLocal);
				M.vec3_mulMat3(normalWorld, normalLocalI, tf2._rotation);
				M.vec3_toVec3(candidateNormal, normalWorld);
				candidateNormal.normalize();

				var closestWorld:IVec3;
				var closestLocalI:IVec3;
				M.vec3_fromVec3(closestLocalI, closestLocal);
				M.vec3_mulMat3(closestWorld, closestLocalI, tf2._rotation);
				M.vec3_add(closestWorld, closestWorld, tf2._position);
				M.vec3_toVec3(candidatePointMesh, closestWorld);

				M.vec3_toVec3(candidatePointConvex, tf1._position);
				candidatePointConvex.addScaledEq(candidateNormal, -margin1);

				report(depth);

				// The maximum possible query depth is marginSum (distance 0),
				// so no later triangle can improve the deepest contact.
				if (!collectAll && isQuery && bestDepth >= marginSum) break;
			}
		}

		if (bestFound) setBestContact(result);
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
