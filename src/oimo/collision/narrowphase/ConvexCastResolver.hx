package oimo.collision.narrowphase;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.detector.gjkepa.*;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * Resolves convex casts against the supported collision geometries.
 *
 * A static mesh reports one hit per triangle the cast reaches, like Bullet
 * Physics' per-triangle results for convex-vs-concave casts, so a callback can
 * filter single triangles instead of the whole mesh.
 */
@:build(oimo.m.B.bu())
class ConvexCastResolver {
	var zero:Vec3;
	var hits:Array<RayCastHit>;

	var triangleVertices:Array<Vec3>;
	var triangleGeom:ConvexHullGeometry;
	var triangles:Array<Int>;
	var sweptAabb:Aabb;
	var localAabb:Aabb;
	var edge1:Vec3;
	var edge2:Vec3;

	public function new() {
		zero = new Vec3();
		hits = [new RayCastHit()];

		triangleVertices = [new Vec3(), new Vec3(), new Vec3()];
		triangleGeom = new ConvexHullGeometry(triangleVertices);
		triangleGeom._gjkMargin = 0;
		triangles = [];
		sweptAabb = new Aabb();
		localAabb = new Aabb();
		edge1 = new Vec3();
		edge2 = new Vec3();
	}

	/**
	 * Casts `convex` from `begin` by `translation` against `target` and returns
	 * the number of hits: at most one for a convex target, one per reached
	 * triangle for a static mesh. Read them with `getHit`; they stay valid until
	 * the next call.
	 */
	public function resolve(convex:ConvexGeometry, begin:Transform, translation:Vec3, target:Geometry, targetTransform:Transform):Int {
		var type:Int = target._type;

		if (type == GeometryType._STATIC_MESH) {
			return castStaticMesh(convex, begin, translation, cast target, targetTransform);
		}

		if (type < GeometryType._CONVEX_MIN || type > GeometryType._CONVEX_MAX) return 0;

		var hit:RayCastHit = hits[0];
		if (!GjkEpa.getInstance().convexCast(convex, cast target, begin, targetTransform, translation, zero, hit)) return 0;
		hit.triangleIndex = -1;
		return 1;
	}

	public inline function getHit(index:Int):RayCastHit {
		return hits[index];
	}

	function castStaticMesh(convex:ConvexGeometry, begin:Transform, translation:Vec3, staticMesh:StaticMeshGeometry, meshTransform:Transform):Int {
		// Candidate triangles: those in the box swept by the convex
		convex._computeAabb(sweptAabb, begin);
		var tl:IVec3;
		var zeroI:IVec3;
		var minTl:IVec3;
		var maxTl:IVec3;
		M.vec3_fromVec3(tl, translation);
		M.vec3_zero(zeroI);
		M.vec3_min(minTl, tl, zeroI);
		M.vec3_max(maxTl, tl, zeroI);
		M.vec3_add(sweptAabb._min, sweptAabb._min, minTl);
		M.vec3_add(sweptAabb._max, sweptAabb._max, maxTl);

		staticMesh.worldAabbToLocal(sweptAabb, meshTransform, localAabb);
		staticMesh.queryTrianglesInto(localAabb, triangles);

		var gjkEpa:GjkEpa = GjkEpa.getInstance();
		var numHits:Int = 0;

		for (i in 0...triangles.length) {
			var triangle:Int = triangles[i];
			staticMesh.getTriangleVertices(triangle, triangleVertices[0], triangleVertices[1], triangleVertices[2]);

			edge1.copyFrom(triangleVertices[1]).subEq(triangleVertices[0]);
			edge2.copyFrom(triangleVertices[2]).subEq(triangleVertices[0]);
			if (edge1.crossEq(edge2).lengthSq() < 1e-12) continue; // degenerate

			if (numHits == hits.length) hits.push(new RayCastHit());
			var hit:RayCastHit = hits[numHits];
			if (gjkEpa.convexCast(convex, triangleGeom, begin, meshTransform, translation, zero, hit)) {
				hit.triangleIndex = triangle;
				numHits++;
			}
		}
		return numHits;
	}
}
