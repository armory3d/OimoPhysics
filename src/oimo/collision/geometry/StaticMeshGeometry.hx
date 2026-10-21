package oimo.collision.geometry;
import haxe.ds.Vector;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * Static mesh collision geometry with BVH spatial queries.
 *
 * The input vertices and indices are copied. The mesh is treated as immutable:
 * modifying the arrays returned by `getVertices`/`getIndices` after construction
 * invalidates the BVH and the cached triangle data.
 */
@:build(oimo.m.B.bu())
class StaticMeshGeometry extends Geometry {
	public var _vertices:Vector<Vec3>;
	public var _indices:Vector<Int>;
	public var _normals:Vector<Vec3>;
	public var _numVertices:Int;
	public var _numTriangles:Int;
	var _bvh:StaticMeshBvh;

	public function new(vertices:Array<Vec3>, indices:Array<Int>, computeNormals:Bool = true) {
		super(GeometryType._STATIC_MESH);

		if (vertices == null) throw M.error("StaticMeshGeometry: vertices must not be null");
		if (indices == null) throw M.error("StaticMeshGeometry: indices must not be null");
		if (indices.length % 3 != 0) {
			throw M.error("StaticMeshGeometry: indices length must be a multiple of 3");
		}

		_numVertices = vertices.length;
		_numTriangles = Std.int(indices.length / 3);

		_vertices = new Vector<Vec3>(_numVertices);
		for (i in 0..._numVertices) {
			var vertex = vertices[i];
			if (vertex == null) {
				throw M.error("StaticMeshGeometry: vertex " + i + " must not be null");
			}
			if (!Math.isFinite(vertex.x) || !Math.isFinite(vertex.y) || !Math.isFinite(vertex.z)) {
				throw M.error("StaticMeshGeometry: vertex " + i + " has a non-finite coordinate");
			}
			_vertices[i] = new Vec3().copyFrom(vertex);
		}

		_indices = new Vector<Int>(indices.length);
		for (i in 0...indices.length) {
			var index:Int = indices[i];
			if (index < 0 || index >= _numVertices) {
				throw M.error("StaticMeshGeometry: index " + index + " is out of range");
			}
			_indices[i] = index;
		}

		_normals = new Vector<Vec3>(_numTriangles);
		if (computeNormals) _computeTriangleNormals();
		else for (i in 0..._numTriangles) _normals[i] = new Vec3(0,1,0);

		_bvh = new StaticMeshBvh(_vertices, _indices, _numTriangles);
		_updateMass();
	}

	/**
	 * Returns the internal vertex array. Do not modify it after construction;
	 * the static BVH is built from these positions.
	 */
	public inline function getVertices():Vector<Vec3> return _vertices;

	/**
	 * Returns the internal index array. Do not modify it after construction;
	 * the static BVH is built from these indices.
	 */
	public inline function getIndices():Vector<Int> return _indices;

	public inline function getNumTriangles():Int return _numTriangles;

	public function getTriangleVertices(triangleIndex:Int, v1:Vec3, v2:Vec3, v3:Vec3):Void {
		var idx = triangleIndex * 3;
		var i1 = _indices[idx];
		var i2 = _indices[idx + 1];
		var i3 = _indices[idx + 2];
		v1.copyFrom(_vertices[i1]);
		v2.copyFrom(_vertices[i2]);
		v3.copyFrom(_vertices[i3]);
	}

	public function getTriangleNormal(triangleIndex:Int, out:Vec3):Void {
		out.copyFrom(_normals[triangleIndex]);
	}

	public function queryTriangles(aabb:Aabb):Array<Int> {
		var results:Array<Int> = [];
		queryTrianglesInto(aabb, results);
		return results;
	}

	public function queryTrianglesInto(aabb:Aabb, results:Array<Int>):Void {
		_bvh.queryTrianglesInto(aabb, results);
	}

	/**
	 * Collects triangles whose AABBs are within `radius` of the local-space
	 * point (`cx`, `cy`, `cz`). The result is conservative; callers must run
	 * the exact triangle distance test.
	 */
	public function queryTrianglesSphereInto(
		cx:Float, cy:Float, cz:Float, radius:Float, results:Array<Int>
	):Void {
		_bvh.queryTrianglesSphereInto(cx, cy, cz, radius, results);
	}

	/**
	 * Transforms the world-space AABB `worldAabb` into this mesh's local space and
	 * stores the result in `localAabb`.
	 */
	public function worldAabbToLocal(worldAabb:Aabb, meshTransform:Transform, localAabb:Aabb):Void {
		var corner:IVec3;
		var local:IVec3;

		for (i in 0...8) {
			M.vec3_set(corner,
				(i & 1) != 0 ? M.vec3_get(worldAabb._max, 0) : M.vec3_get(worldAabb._min, 0),
				(i & 2) != 0 ? M.vec3_get(worldAabb._max, 1) : M.vec3_get(worldAabb._min, 1),
				(i & 4) != 0 ? M.vec3_get(worldAabb._max, 2) : M.vec3_get(worldAabb._min, 2)
			);

			M.vec3_sub(local, corner, meshTransform._position);
			M.vec3_mulMat3Transposed(local, local, meshTransform._rotation);

			if (i == 0) {
				M.vec3_assign(localAabb._min, local);
				M.vec3_assign(localAabb._max, local);
			} else {
				M.vec3_min(localAabb._min, localAabb._min, local);
				M.vec3_max(localAabb._max, localAabb._max, local);
			}
		}
	}

	override public function _rayCastLocal(begin:IVec3, end:IVec3, hit:RayCastHit):Bool {
		return _bvh.rayCast(
			M.vec3_get(begin, 0), M.vec3_get(begin, 1), M.vec3_get(begin, 2),
			M.vec3_get(end, 0), M.vec3_get(end, 1), M.vec3_get(end, 2),
			hit
		);
	}

	override public function _updateMass():Void {
		if (_numVertices == 0) {
			_volume = 0;
			M.mat3_diagonal(_inertiaCoeff, 0, 0, 0);
			return;
		}

		var minx = MathUtil.POSITIVE_INFINITY; var miny = MathUtil.POSITIVE_INFINITY; var minz = MathUtil.POSITIVE_INFINITY;
		var maxx = MathUtil.NEGATIVE_INFINITY; var maxy = MathUtil.NEGATIVE_INFINITY; var maxz = MathUtil.NEGATIVE_INFINITY;
		for (i in 0..._numVertices) {
			var v = _vertices[i];
			if (v.x < minx) minx = v.x; if (v.x > maxx) maxx = v.x;
			if (v.y < miny) miny = v.y; if (v.y > maxy) maxy = v.y;
			if (v.z < minz) minz = v.z; if (v.z > maxz) maxz = v.z;
		}
		var sizex = maxx - minx; var sizey = maxy - miny; var sizez = maxz - minz;
		_volume = sizex * sizey * sizez;
		M.mat3_diagonal(_inertiaCoeff,
			1 / 12 * (sizey * sizey + sizez * sizez),
			1 / 12 * (sizez * sizez + sizex * sizex),
			1 / 12 * (sizex * sizex + sizey * sizey)
		);
	}

	override public function _computeAabb(aabb:Aabb, tf:Transform):Void {
		if (_numVertices == 0) {
			M.vec3_assign(aabb._min, tf._position);
			M.vec3_assign(aabb._max, tf._position);
			return;
		}

		var minx = MathUtil.POSITIVE_INFINITY; var miny = MathUtil.POSITIVE_INFINITY; var minz = MathUtil.POSITIVE_INFINITY;
		var maxx = MathUtil.NEGATIVE_INFINITY; var maxy = MathUtil.NEGATIVE_INFINITY; var maxz = MathUtil.NEGATIVE_INFINITY;
		for (i in 0..._numVertices) {
			var localV:IVec3; M.vec3_fromVec3(localV, _vertices[i]);
			var worldV:IVec3; M.vec3_mulMat3(worldV, localV, tf._rotation); M.vec3_add(worldV, worldV, tf._position);
			var x = M.vec3_get(worldV, 0); var y = M.vec3_get(worldV, 1); var z = M.vec3_get(worldV, 2);
			if (x < minx) minx = x; if (x > maxx) maxx = x;
			if (y < miny) miny = y; if (y > maxy) maxy = y;
			if (z < minz) minz = z; if (z > maxz) maxz = z;
		}
		M.vec3_set(aabb._min, minx, miny, minz);
		M.vec3_set(aabb._max, maxx, maxy, maxz);
	}

	function _computeTriangleNormals():Void {
		for (i in 0..._numTriangles) {
			var idx = i * 3;
			var v1 = _vertices[_indices[idx]];
			var v2 = _vertices[_indices[idx + 1]];
			var v3 = _vertices[_indices[idx + 2]];

			var e1x = v2.x - v1.x;
			var e1y = v2.y - v1.y;
			var e1z = v2.z - v1.z;
			var e2x = v3.x - v1.x;
			var e2y = v3.y - v1.y;
			var e2z = v3.z - v1.z;

			var nx = e1y * e2z - e1z * e2y;
			var ny = e1z * e2x - e1x * e2z;
			var nz = e1x * e2y - e1y * e2x;
			var len = MathUtil.sqrt(nx * nx + ny * ny + nz * nz);

			_normals[i] = new Vec3();
			if (len > 1e-20) {
				var invLen = 1.0 / len;
				_normals[i].init(nx * invLen, ny * invLen, nz * invLen);
			}
		}
	}
}
