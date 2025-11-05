package oimo.collision.geometry;
import haxe.ds.Vector;
import oimo.collision.broadphase.bvh.BvhNode;
import oimo.collision.broadphase.bvh.BvhProxy;
import oimo.collision.broadphase.bvh.BvhTree;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * Static mesh collision geometry with BVH spatial queries.
 */
@:build(oimo.m.B.bu())
class StaticMeshGeometry extends Geometry {
	public var _vertices:Vector<Vec3>;
	public var _indices:Vector<Int>;
	public var _normals:Vector<Vec3>;
	public var _numVertices:Int;
	public var _numTriangles:Int;
	var _triangleAABBs:Vector<Aabb>;
	var _triangleBVH:BvhTree;
	var _triangleProxies:Vector<BvhProxy>;
	var _nextProxyId:Int;

	public function new(vertices:Array<Vec3>, indices:Array<Int>, computeNormals:Bool = true) {
		super(GeometryType._STATIC_MESH);
		_numVertices = vertices.length;
		_numTriangles = Std.int(indices.length / 3);

		_vertices = new Vector<Vec3>(_numVertices);
		for (i in 0..._numVertices) _vertices[i] = new Vec3().copyFrom(vertices[i]);

		_indices = new Vector<Int>(indices.length);
		for (i in 0...indices.length) _indices[i] = indices[i];

		_normals = new Vector<Vec3>(_numTriangles);
		if (computeNormals) _computeTriangleNormals();
		else for (i in 0..._numTriangles) _normals[i] = new Vec3(0,1,0);

		_buildTriangleAABBs();
		_buildTriangleBVH();
		_updateMass();
	}

	public inline function getVertices():Vector<Vec3> return _vertices;
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
		_queryBVHRecursive(_triangleBVH._root, aabb, results);
		return results;
	}

	function _queryBVHRecursive(node:BvhNode, queryAABB:Aabb, results:Array<Int>):Void {
		if (node == null) return;

		var nodeAABB = new Aabb();
		M.vec3_assign(nodeAABB._min, node._aabbMin);
		M.vec3_assign(nodeAABB._max, node._aabbMax);

		if (!nodeAABB.overlap(queryAABB)) return;

		if (node._height == 0) {
			var triangleIndex:Int = cast node._proxy.userData;
			results.push(triangleIndex);
		} else {
			_queryBVHRecursive(node._children[0], queryAABB, results);
			_queryBVHRecursive(node._children[1], queryAABB, results);
		}
	}

	override public function _rayCastLocal(begin:IVec3, end:IVec3, hit:RayCastHit):Bool {
		var rayDir:IVec3;
		M.vec3_sub(rayDir, end, begin);
		var rayLengthSq:Float = M.vec3_dot(rayDir, rayDir);
		if (rayLengthSq < 1e-12) return false;

		var rayLength:Float = MathUtil.sqrt(rayLengthSq);
		M.vec3_scale(rayDir, rayDir, 1.0 / rayLength);

		var rayAABB = new Aabb();
		var beginX = M.vec3_get(begin, 0); var beginY = M.vec3_get(begin, 1); var beginZ = M.vec3_get(begin, 2);
		var endX = M.vec3_get(end, 0); var endY = M.vec3_get(end, 1); var endZ = M.vec3_get(end, 2);
		M.vec3_set(rayAABB._min,
			beginX < endX ? beginX : endX,
			beginY < endY ? beginY : endY,
			beginZ < endZ ? beginZ : endZ
		);
		M.vec3_set(rayAABB._max,
			beginX > endX ? beginX : endX,
			beginY > endY ? beginY : endY,
			beginZ > endZ ? beginZ : endZ
		);

		var triangles:Array<Int> = queryTriangles(rayAABB);
		var closestT:Float = MathUtil.POSITIVE_INFINITY;
		var closestTriangle:Int = -1;

		for (i in 0...triangles.length) {
			var triangleIdx = triangles[i];
			var idx:Int = triangleIdx * 3;
			var i1:Int = _indices[idx];
			var i2:Int = _indices[idx + 1];
			var i3:Int = _indices[idx + 2];

			var v1:IVec3; M.vec3_fromVec3(v1, _vertices[i1]);
			var v2:IVec3; M.vec3_fromVec3(v2, _vertices[i2]);
			var v3:IVec3; M.vec3_fromVec3(v3, _vertices[i3]);

			var edge1:IVec3; M.vec3_sub(edge1, v2, v1);
			var edge2:IVec3; M.vec3_sub(edge2, v3, v1);
			var h:IVec3; M.vec3_cross(h, rayDir, edge2);
			var a:Float = M.vec3_dot(edge1, h);

			if (a > -1e-6 && a < 1e-6) continue;

			var f:Float = 1.0 / a;
			var s:IVec3; M.vec3_sub(s, begin, v1);
			var u:Float = f * M.vec3_dot(s, h);

			if (u < 0.0 || u > 1.0) continue;

			var q:IVec3; M.vec3_cross(q, s, edge1);
			var v:Float = f * M.vec3_dot(rayDir, q);

			if (v < 0.0 || u + v > 1.0) continue;

			var t:Float = f * M.vec3_dot(edge2, q);

			if (t > 1e-6 && t <= rayLength && t < closestT) {
				closestT = t;
				closestTriangle = triangleIdx;
			}
		}

		if (closestTriangle >= 0) {
			var hitPos:IVec3;
			M.vec3_addRhsScaled(hitPos, begin, rayDir, closestT);

			var hitNormal:IVec3;
			M.vec3_fromVec3(hitNormal, _normals[closestTriangle]);

			M.vec3_toVec3(hit.position, hitPos);
			M.vec3_toVec3(hit.normal, hitNormal);
			hit.fraction = closestT / rayLength;
			return true;
		}

		return false;
	}

	override public function _updateMass():Void {
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
			var i1 = _indices[idx]; var i2 = _indices[idx + 1]; var i3 = _indices[idx + 2];
			var v1 = _vertices[i1]; var v2 = _vertices[i2]; var v3 = _vertices[i3];
			var edge1 = new Vec3().copyFrom(v2).subEq(v1);
			var edge2 = new Vec3().copyFrom(v3).subEq(v1);
			var normal = new Vec3().copyFrom(edge1).crossEq(edge2).normalize();
			_normals[i] = normal;
		}
	}

	function _buildTriangleAABBs():Void {
		_triangleAABBs = new Vector<Aabb>(_numTriangles);
		for (i in 0..._numTriangles) {
			var v1 = new Vec3(); var v2 = new Vec3(); var v3 = new Vec3();
			getTriangleVertices(i, v1, v2, v3);
			var aabb = new Aabb();
			var minx = Math.min(v1.x, Math.min(v2.x, v3.x)); var miny = Math.min(v1.y, Math.min(v2.y, v3.y)); var minz = Math.min(v1.z, Math.min(v2.z, v3.z));
			var maxx = Math.max(v1.x, Math.max(v2.x, v3.x)); var maxy = Math.max(v1.y, Math.max(v2.y, v3.y)); var maxz = Math.max(v1.z, Math.max(v2.z, v3.z));
			M.vec3_set(aabb._min, minx, miny, minz); M.vec3_set(aabb._max, maxx, maxy, maxz);
			_triangleAABBs[i] = aabb;
		}
	}

	function _buildTriangleBVH():Void {
		_triangleBVH = new BvhTree();
		_triangleProxies = new Vector<BvhProxy>(_numTriangles);
		_nextProxyId = 0;

		for (i in 0..._numTriangles) {
			var proxy = new BvhProxy(i, _nextProxyId++);
			proxy._setAabb(_triangleAABBs[i]);
			_triangleBVH._insertProxy(proxy);
			_triangleProxies[i] = proxy;
		}
	}
}
