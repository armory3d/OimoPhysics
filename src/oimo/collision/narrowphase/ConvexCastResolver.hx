package oimo.collision.narrowphase;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.detector.gjkepa.*;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * Resolves convex casts against the supported collision geometries.
 */
@:build(oimo.m.B.bu())
class ConvexCastResolver {
	var convex:ConvexGeometry;
	var begin:Transform;
	var translation:Vec3;
	var zero:Vec3;

	var triangleHit:RayCastHit;
	var triangleVertices:Array<Vec3>;
	var triangleGeom:ConvexHullGeometry;
	var triangles:Array<Int>;
	var sweptAabb:Aabb;
	var localAabb:Aabb;
	var edge1:Vec3;
	var edge2:Vec3;
	var sweptAabbReady:Bool;

	public function new() {
		begin = new Transform();
		translation = new Vec3();
		zero = new Vec3();

		triangleHit = new RayCastHit();
		triangleVertices = [new Vec3(), new Vec3(), new Vec3()];
		triangleGeom = new ConvexHullGeometry(triangleVertices);
		triangleGeom._gjkMargin = 0;
		triangles = [];
		sweptAabb = new Aabb();
		localAabb = new Aabb();
		edge1 = new Vec3();
		edge2 = new Vec3();
		sweptAabbReady = false;

		convex = null;
	}

	public function prepare(convex:ConvexGeometry, begin:Transform, translation:Vec3):Void {
		this.convex = convex;
		if (begin == null || translation == null) {
			this.convex = null;
			sweptAabbReady = false;
			return;
		}
		this.begin.copyFrom(begin);
		this.translation.copyFrom(translation);
		sweptAabbReady = false;
	}

	public function resolve(target:Geometry, targetTransform:Transform, hit:RayCastHit):Bool {
		if (convex == null || target == null || targetTransform == null || hit == null) return false;

		var type:Int = target._type;

		if (type == GeometryType._STATIC_MESH) {
			return castStaticMesh(cast target, targetTransform, hit);
		}

		if (type < GeometryType._CONVEX_MIN || type > GeometryType._CONVEX_MAX) return false;

		return GjkEpa.getInstance().convexCast(convex, cast target, begin, targetTransform, translation, zero, hit);
	}

	function castStaticMesh(staticMesh:StaticMeshGeometry, meshTransform:Transform, hit:RayCastHit):Bool {
		if (convex == null || staticMesh == null || meshTransform == null || hit == null) return false;

		// A hull's support points are not shrunk by its GJK margin, unlike the
		// primitives. Casting with the margin therefore reports a hit before the
		// hull actually touches the mesh. Suppress it only for this mesh cast.
		var restoreHullMargin:Bool = convex._type == GeometryType._CONVEX_HULL && convex._gjkMargin != 0;
		var hullMargin:Float = 0;
		if (restoreHullMargin) {
			hullMargin = convex._gjkMargin;
			convex._gjkMargin = 0;
		}

		if (!sweptAabbReady) {
			convex._computeAabb(sweptAabb, begin);

			var minx:Float = M.vec3_get(sweptAabb._min, 0);
			var miny:Float = M.vec3_get(sweptAabb._min, 1);
			var minz:Float = M.vec3_get(sweptAabb._min, 2);
			var maxx:Float = M.vec3_get(sweptAabb._max, 0);
			var maxy:Float = M.vec3_get(sweptAabb._max, 1);
			var maxz:Float = M.vec3_get(sweptAabb._max, 2);

			if (translation.x < 0) minx += translation.x; else maxx += translation.x;
			if (translation.y < 0) miny += translation.y; else maxy += translation.y;
			if (translation.z < 0) minz += translation.z; else maxz += translation.z;

			M.vec3_set(sweptAabb._min, minx, miny, minz);
			M.vec3_set(sweptAabb._max, maxx, maxy, maxz);
			sweptAabbReady = true;
		}

		staticMesh.worldAabbToLocal(sweptAabb, meshTransform, localAabb);

		staticMesh.queryTrianglesInto(localAabb, triangles);
		if (triangles.length == 0) {
			if (restoreHullMargin) convex._gjkMargin = hullMargin;
			return false;
		}

		var gjkEpa:GjkEpa = GjkEpa.getInstance();
		var bestFraction:Float = MathUtil.POSITIVE_INFINITY;
		var found:Bool = false;

		for (i in 0...triangles.length) {
			staticMesh.getTriangleVertices(triangles[i], triangleVertices[0], triangleVertices[1], triangleVertices[2]);

			var v1:Vec3 = triangleVertices[0];
			var v2:Vec3 = triangleVertices[1];
			var v3:Vec3 = triangleVertices[2];

			edge1.copyFrom(v2).subEq(v1);
			edge2.copyFrom(v3).subEq(v1);
			if (edge1.crossEq(edge2).lengthSq() < 1e-12) continue;

			if (gjkEpa.convexCast(convex, triangleGeom, begin, meshTransform, translation, zero, triangleHit)) {
				if (triangleHit.fraction < bestFraction) {
					bestFraction = triangleHit.fraction;
					hit.position.copyFrom(triangleHit.position);
					hit.normal.copyFrom(triangleHit.normal);
					found = true;

					// No other triangle can be hit earlier.
					if (bestFraction <= 0) break;
				}
			}
		}

		if (restoreHullMargin) convex._gjkMargin = hullMargin;

		if (!found) return false;
		hit.fraction = bestFraction;
		return true;
	}
}
