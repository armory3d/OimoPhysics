package oimo.collision.geometry;
import haxe.ds.Vector;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.m.M;

/**
 * Static bounding volume hierarchy for `StaticMeshGeometry`.
 *
 * The tree is built once, top-down, from the triangle AABBs. It is intentionally
 * separate from the dynamic broad-phase BVH: static mesh triangles never move,
 * so the child/leaf links, proxies and incremental insertion used there are
 * unnecessary overhead.
 *
 * Node data is stored in flat arrays:
 *
 * - `_bounds` contains six floats per node: min x/y/z and max x/y/z.
 * - `_links` contains four ints per node: left child, right child, leaf start
 *   and leaf triangle count. A node is a leaf when its count is greater than
 *   zero; internal nodes have a count of zero.
 */
class StaticMeshBvh {
	static inline var LEAF_SIZE:Int = 4;
	static inline var EPSILON:Float = 1e-12;
	static inline var NORMAL_EPSILON:Float = 1e-20;

	var _vertices:Vector<Vec3>;
	var _indices:Vector<Int>;
	var _numTriangles:Int;

	var _bounds:Array<Float>;
	var _links:Array<Int>;
	var _order:Vector<Int>;
	var _root:Int;
	var _nodeCount:Int;

	// Six floats per triangle: min x/y/z and max x/y/z. Kept after construction
	// because AABB queries must test individual triangles, not just leaves.
	var _triangleBounds:Vector<Float>;

	// Build-only centroid data, released after construction.
	var _centroidX:Vector<Float>;
	var _centroidY:Vector<Float>;
	var _centroidZ:Vector<Float>;

	var _queryStack:Vector<Int>;
	var _rayStack:Vector<Int>;

	public function new(vertices:Vector<Vec3>, indices:Vector<Int>, numTriangles:Int) {
		_vertices = vertices;
		_indices = indices;
		_numTriangles = numTriangles;

		_bounds = [];
		_links = [];
		_order = new Vector<Int>(numTriangles);
		_root = -1;
		_nodeCount = 0;

		_build();

		// A depth-first traversal keeps at most one pending sibling per level.
		// Allocating one slot per node is deliberately conservative.
		_queryStack = new Vector<Int>(_nodeCount + 1);
		_rayStack = new Vector<Int>(_nodeCount + 1);
	}

	/**
	 * Returns the root node index, or `-1` for an empty mesh. Intended for tests
	 * and diagnostics.
	 */
	public inline function getRoot():Int return _root;

	/**
	 * Returns the number of nodes in the tree. Intended for tests and diagnostics.
	 */
	public inline function getNodeCount():Int return _nodeCount;

	/**
	 * Appends the indices of all triangles whose AABBs strictly overlap `aabb`.
	 */
	public function queryTrianglesInto(aabb:Aabb, results:Array<Int>):Void {
		results.resize(0);
		if (_root < 0 || aabb == null) return;

		var qminX:Float = M.vec3_get(aabb._min, 0);
		var qminY:Float = M.vec3_get(aabb._min, 1);
		var qminZ:Float = M.vec3_get(aabb._min, 2);
		var qmaxX:Float = M.vec3_get(aabb._max, 0);
		var qmaxY:Float = M.vec3_get(aabb._max, 1);
		var qmaxZ:Float = M.vec3_get(aabb._max, 2);

		var stack:Vector<Int> = _queryStack;
		var stackSize:Int = 0;
		stack[stackSize] = _root;
		stackSize++;

		while (stackSize > 0) {
			stackSize--;
			var node:Int = stack[stackSize];
			var b:Int = node * 6;

			// Strict overlap, matching the previous M.aabb_overlap behavior.
			if (!(
				_bounds[b] < qmaxX && _bounds[b + 3] > qminX &&
				_bounds[b + 1] < qmaxY && _bounds[b + 4] > qminY &&
				_bounds[b + 2] < qmaxZ && _bounds[b + 5] > qminZ
			)) {
				continue;
			}

			var l:Int = node * 4;
			var count:Int = _links[l + 3];
			if (count > 0) {
				var start:Int = _links[l + 2];
				for (i in 0...count) {
					var triangle:Int = _order[start + i];
					var tb:Int = triangle * 6;
					if (_triangleBounds[tb] < qmaxX && _triangleBounds[tb + 3] > qminX &&
						_triangleBounds[tb + 1] < qmaxY && _triangleBounds[tb + 4] > qminY &&
						_triangleBounds[tb + 2] < qmaxZ && _triangleBounds[tb + 5] > qminZ) {
						results.push(triangle);
					}
				}
			} else {
				stack[stackSize] = _links[l];
				stackSize++;
				stack[stackSize] = _links[l + 1];
				stackSize++;
			}
		}
	}

	/**
	 * Collects triangles whose AABBs are within `radius` of the point
	 * (`cx`, `cy`, `cz`). This is a tighter candidate set than the bounding
	 * cube used by `queryTrianglesInto`, and is intended for sphere queries.
	 * The result is still conservative: the caller must perform the exact
	 * triangle distance test.
	 */
	public function queryTrianglesSphereInto(
		cx:Float, cy:Float, cz:Float, radius:Float, results:Array<Int>
	):Void {
		if (results == null) return;
		results.resize(0);
		if (_root < 0 || !(radius >= 0)) return;

		var radiusSq:Float = radius * radius;
		var stack:Vector<Int> = _queryStack;
		var stackSize:Int = 0;
		stack[stackSize] = _root;
		stackSize++;

		while (stackSize > 0) {
			stackSize--;
			var node:Int = stack[stackSize];
			var b:Int = node * 6;

			var dx:Float = cx < _bounds[b] ? _bounds[b] - cx : (cx > _bounds[b + 3] ? cx - _bounds[b + 3] : 0);
			var dy:Float = cy < _bounds[b + 1] ? _bounds[b + 1] - cy : (cy > _bounds[b + 4] ? cy - _bounds[b + 4] : 0);
			var dz:Float = cz < _bounds[b + 2] ? _bounds[b + 2] - cz : (cz > _bounds[b + 5] ? cz - _bounds[b + 5] : 0);
			if (dx * dx + dy * dy + dz * dz > radiusSq) continue;

			var l:Int = node * 4;
			var count:Int = _links[l + 3];
			if (count > 0) {
				var start:Int = _links[l + 2];
				for (i in 0...count) {
					var triangle:Int = _order[start + i];
					var tb:Int = triangle * 6;
					var tdx:Float = cx < _triangleBounds[tb] ? _triangleBounds[tb] - cx : (cx > _triangleBounds[tb + 3] ? cx - _triangleBounds[tb + 3] : 0);
					var tdy:Float = cy < _triangleBounds[tb + 1] ? _triangleBounds[tb + 1] - cy : (cy > _triangleBounds[tb + 4] ? cy - _triangleBounds[tb + 4] : 0);
					var tdz:Float = cz < _triangleBounds[tb + 2] ? _triangleBounds[tb + 2] - cz : (cz > _triangleBounds[tb + 5] ? cz - _triangleBounds[tb + 5] : 0);
					if (tdx * tdx + tdy * tdy + tdz * tdz <= radiusSq) {
						results.push(triangle);
					}
				}
			} else {
				stack[stackSize] = _links[l];
				stackSize++;
				stack[stackSize] = _links[l + 1];
				stackSize++;
			}
		}
	}

	/**
	 * Collects triangles whose AABBs are within `radius` of the segment
	 * (`p0`..`p1`). The segment AABB is used as a conservative lower bound:
	 * it is tighter than expanding the segment AABB by `radius` on every axis,
	 * especially for a capsule resting near a flat mesh. The caller must still
	 * perform the exact segment/triangle distance test.
	 */
	public function queryTrianglesSegmentInto(
		p0x:Float, p0y:Float, p0z:Float,
		p1x:Float, p1y:Float, p1z:Float,
		radius:Float, results:Array<Int>
	):Void {
		if (results == null) return;
		results.resize(0);
		if (_root < 0 || !(radius >= 0)) return;

		var segMinX:Float = p0x < p1x ? p0x : p1x;
		var segMinY:Float = p0y < p1y ? p0y : p1y;
		var segMinZ:Float = p0z < p1z ? p0z : p1z;
		var segMaxX:Float = p0x > p1x ? p0x : p1x;
		var segMaxY:Float = p0y > p1y ? p0y : p1y;
		var segMaxZ:Float = p0z > p1z ? p0z : p1z;
		var radiusSq:Float = radius * radius;

		var stack:Vector<Int> = _queryStack;
		var stackSize:Int = 0;
		stack[stackSize] = _root;
		stackSize++;

		while (stackSize > 0) {
			stackSize--;
			var node:Int = stack[stackSize];
			var b:Int = node * 6;

			var dx:Float = _intervalDistance(segMinX, segMaxX, _bounds[b], _bounds[b + 3]);
			var dy:Float = _intervalDistance(segMinY, segMaxY, _bounds[b + 1], _bounds[b + 4]);
			var dz:Float = _intervalDistance(segMinZ, segMaxZ, _bounds[b + 2], _bounds[b + 5]);
			if (dx * dx + dy * dy + dz * dz > radiusSq) continue;

			var l:Int = node * 4;
			var count:Int = _links[l + 3];
			if (count > 0) {
				var start:Int = _links[l + 2];
				for (i in 0...count) {
					var triangle:Int = _order[start + i];
					var tb:Int = triangle * 6;
					var tdx:Float = _intervalDistance(segMinX, segMaxX, _triangleBounds[tb], _triangleBounds[tb + 3]);
					var tdy:Float = _intervalDistance(segMinY, segMaxY, _triangleBounds[tb + 1], _triangleBounds[tb + 4]);
					var tdz:Float = _intervalDistance(segMinZ, segMaxZ, _triangleBounds[tb + 2], _triangleBounds[tb + 5]);
					if (tdx * tdx + tdy * tdy + tdz * tdz <= radiusSq) {
						results.push(triangle);
					}
				}
			} else {
				stack[stackSize] = _links[l];
				stackSize++;
				stack[stackSize] = _links[l + 1];
				stackSize++;
			}
		}
	}

	inline function _intervalDistance(aMin:Float, aMax:Float, bMin:Float, bMax:Float):Float {
		return aMax < bMin ? bMin - aMax : (bMax < aMin ? aMin - bMax : 0);
	}

	/**
	 * Raycasts against the mesh in local space. `begin`/`end` are local
	 * coordinates. Returns `true` and fills `hit` on a hit.
	 *
	 * AABB traversal uses inclusive bounds so axis-aligned rays whose AABB has
	 * zero thickness are not rejected before the triangle test.
	 */
	public function rayCast(
		beginX:Float, beginY:Float, beginZ:Float,
		endX:Float, endY:Float, endZ:Float,
		hit:RayCastHit
	):Bool {
		if (_root < 0 || hit == null) return false;

		var dirX:Float = endX - beginX;
		var dirY:Float = endY - beginY;
		var dirZ:Float = endZ - beginZ;
		var lengthSq:Float = dirX * dirX + dirY * dirY + dirZ * dirZ;
		if (lengthSq < EPSILON) return false;

		var rayLength:Float = MathUtil.sqrt(lengthSq);
		var invLength:Float = 1.0 / rayLength;
		dirX *= invLength;
		dirY *= invLength;
		dirZ *= invLength;

		var closestT:Float = rayLength;
		var closestTriangle:Int = -1;

		var stack:Vector<Int> = _rayStack;
		var stackSize:Int = 0;
		stack[stackSize] = _root;
		stackSize++;

		while (stackSize > 0) {
			stackSize--;
			var node:Int = stack[stackSize];
			if (!_rayAabb(node, beginX, beginY, beginZ, dirX, dirY, dirZ, closestT)) {
				continue;
			}

			var l:Int = node * 4;
			var count:Int = _links[l + 3];
			if (count > 0) {
				var start:Int = _links[l + 2];
				for (i in 0...count) {
					var triangle:Int = _order[start + i];
					var t:Float = _rayTriangle(
						triangle,
						beginX, beginY, beginZ,
						dirX, dirY, dirZ
					);
					if (t > 1e-6 && t <= closestT) {
						closestT = t;
						closestTriangle = triangle;
					}
				}
			} else {
				stack[stackSize] = _links[l];
				stackSize++;
				stack[stackSize] = _links[l + 1];
				stackSize++;
			}
		}

		if (closestTriangle < 0) return false;

		hit.position.init(
			beginX + dirX * closestT,
			beginY + dirY * closestT,
			beginZ + dirZ * closestT
		);

		var idx:Int = closestTriangle * 3;
		var v1:Vec3 = _vertices[_indices[idx]];
		var v2:Vec3 = _vertices[_indices[idx + 1]];
		var v3:Vec3 = _vertices[_indices[idx + 2]];

		var e1x:Float = v2.x - v1.x;
		var e1y:Float = v2.y - v1.y;
		var e1z:Float = v2.z - v1.z;
		var e2x:Float = v3.x - v1.x;
		var e2y:Float = v3.y - v1.y;
		var e2z:Float = v3.z - v1.z;

		var nx:Float = e1y * e2z - e1z * e2y;
		var ny:Float = e1z * e2x - e1x * e2z;
		var nz:Float = e1x * e2y - e1y * e2x;
		var normalLength:Float = MathUtil.sqrt(nx * nx + ny * ny + nz * nz);
		if (normalLength > NORMAL_EPSILON) {
			var invNormalLength:Float = 1.0 / normalLength;
			hit.normal.init(nx * invNormalLength, ny * invNormalLength, nz * invNormalLength);
		} else {
			hit.normal.init(0, 0, 0);
		}

		hit.fraction = closestT / rayLength;
		return true;
	}

	function _build():Void {
		if (_numTriangles == 0) return;

		_triangleBounds = new Vector<Float>(_numTriangles * 6);
		_centroidX = new Vector<Float>(_numTriangles);
		_centroidY = new Vector<Float>(_numTriangles);
		_centroidZ = new Vector<Float>(_numTriangles);

		for (i in 0..._numTriangles) {
			_order[i] = i;

			var idx:Int = i * 3;
			var v1:Vec3 = _vertices[_indices[idx]];
			var v2:Vec3 = _vertices[_indices[idx + 1]];
			var v3:Vec3 = _vertices[_indices[idx + 2]];

			var minx:Float = v1.x < v2.x ? v1.x : v2.x;
			if (v3.x < minx) minx = v3.x;
			var miny:Float = v1.y < v2.y ? v1.y : v2.y;
			if (v3.y < miny) miny = v3.y;
			var minz:Float = v1.z < v2.z ? v1.z : v2.z;
			if (v3.z < minz) minz = v3.z;

			var maxx:Float = v1.x > v2.x ? v1.x : v2.x;
			if (v3.x > maxx) maxx = v3.x;
			var maxy:Float = v1.y > v2.y ? v1.y : v2.y;
			if (v3.y > maxy) maxy = v3.y;
			var maxz:Float = v1.z > v2.z ? v1.z : v2.z;
			if (v3.z > maxz) maxz = v3.z;

			var b:Int = i * 6;
			_triangleBounds[b] = minx;
			_triangleBounds[b + 1] = miny;
			_triangleBounds[b + 2] = minz;
			_triangleBounds[b + 3] = maxx;
			_triangleBounds[b + 4] = maxy;
			_triangleBounds[b + 5] = maxz;

			_centroidX[i] = (minx + maxx) * 0.5;
			_centroidY[i] = (miny + maxy) * 0.5;
			_centroidZ[i] = (minz + maxz) * 0.5;
		}

		_root = _buildNode(0, _numTriangles);

		// Triangle bounds stay available for exact per-triangle AABB queries.
		_centroidX = null;
		_centroidY = null;
		_centroidZ = null;
	}

	function _buildNode(start:Int, end:Int):Int {
		var count:Int = end - start;

		if (count <= LEAF_SIZE) {
			var node:Int = _allocNode();
			var minx:Float = MathUtil.POSITIVE_INFINITY;
			var miny:Float = MathUtil.POSITIVE_INFINITY;
			var minz:Float = MathUtil.POSITIVE_INFINITY;
			var maxx:Float = MathUtil.NEGATIVE_INFINITY;
			var maxy:Float = MathUtil.NEGATIVE_INFINITY;
			var maxz:Float = MathUtil.NEGATIVE_INFINITY;

			for (i in start...end) {
				var triangle:Int = _order[i];
				var tb:Int = triangle * 6;
				if (_triangleBounds[tb] < minx) minx = _triangleBounds[tb];
				if (_triangleBounds[tb + 1] < miny) miny = _triangleBounds[tb + 1];
				if (_triangleBounds[tb + 2] < minz) minz = _triangleBounds[tb + 2];
				if (_triangleBounds[tb + 3] > maxx) maxx = _triangleBounds[tb + 3];
				if (_triangleBounds[tb + 4] > maxy) maxy = _triangleBounds[tb + 4];
				if (_triangleBounds[tb + 5] > maxz) maxz = _triangleBounds[tb + 5];
			}

			_setBounds(node, minx, miny, minz, maxx, maxy, maxz);
			_setLeaf(node, start, count);
			return node;
		}

		// Split along the axis with the largest centroid extent. A median split
		// keeps the tree balanced regardless of triangle order.
		var minx:Float = MathUtil.POSITIVE_INFINITY;
		var miny:Float = MathUtil.POSITIVE_INFINITY;
		var minz:Float = MathUtil.POSITIVE_INFINITY;
		var maxx:Float = MathUtil.NEGATIVE_INFINITY;
		var maxy:Float = MathUtil.NEGATIVE_INFINITY;
		var maxz:Float = MathUtil.NEGATIVE_INFINITY;

		for (i in start...end) {
			var triangle:Int = _order[i];
			if (_centroidX[triangle] < minx) minx = _centroidX[triangle];
			if (_centroidY[triangle] < miny) miny = _centroidY[triangle];
			if (_centroidZ[triangle] < minz) minz = _centroidZ[triangle];
			if (_centroidX[triangle] > maxx) maxx = _centroidX[triangle];
			if (_centroidY[triangle] > maxy) maxy = _centroidY[triangle];
			if (_centroidZ[triangle] > maxz) maxz = _centroidZ[triangle];
		}

		var extentX:Float = maxx - minx;
		var extentY:Float = maxy - miny;
		var extentZ:Float = maxz - minz;
		var axis:Int = 0;
		var extent:Float = extentX;
		if (extentY > extent) {
			axis = 1;
			extent = extentY;
		}
		if (extentZ > extent) {
			axis = 2;
			extent = extentZ;
		}

		var mid:Int = (start + end) >> 1;
		if (extent > EPSILON) {
			_nthElement(start, end, mid, axis);
		}

		var left:Int = _buildNode(start, mid);
		var right:Int = _buildNode(mid, end);

		var node:Int = _allocNode();
		_setInternal(node, left, right);

		var lb:Int = left * 6;
		var rb:Int = right * 6;
		var nminx:Float = _bounds[lb] < _bounds[rb] ? _bounds[lb] : _bounds[rb];
		var nminy:Float = _bounds[lb + 1] < _bounds[rb + 1] ? _bounds[lb + 1] : _bounds[rb + 1];
		var nminz:Float = _bounds[lb + 2] < _bounds[rb + 2] ? _bounds[lb + 2] : _bounds[rb + 2];
		var nmaxx:Float = _bounds[lb + 3] > _bounds[rb + 3] ? _bounds[lb + 3] : _bounds[rb + 3];
		var nmaxy:Float = _bounds[lb + 4] > _bounds[rb + 4] ? _bounds[lb + 4] : _bounds[rb + 4];
		var nmaxz:Float = _bounds[lb + 5] > _bounds[rb + 5] ? _bounds[lb + 5] : _bounds[rb + 5];
		_setBounds(node, nminx, nminy, nminz, nmaxx, nmaxy, nmaxz);

		return node;
	}

	inline function _centroid(triangle:Int, axis:Int):Float {
		if (axis == 0) return _centroidX[triangle];
		if (axis == 1) return _centroidY[triangle];
		return _centroidZ[triangle];
	}

	/**
	 * Partially sorts `_order[start...end)` so that the element at `k` is in its
	 * sorted position. This is a standard quickselect and is only used during
	 * construction.
	 */
	function _nthElement(start:Int, end:Int, k:Int, axis:Int):Void {
		var i:Int = start;
		var j:Int = end - 1;

		while (i < j) {
			var pivot:Float = _centroid(_order[(i + j) >> 1], axis);
			var l:Int = i;
			var r:Int = j;

			while (l <= r) {
				while (_centroid(_order[l], axis) < pivot) l++;
				while (_centroid(_order[r], axis) > pivot) r--;

				if (l <= r) {
					var tmp:Int = _order[l];
					_order[l] = _order[r];
					_order[r] = tmp;
					l++;
					r--;
				}
			}

			if (k <= r) {
				j = r;
			} else if (k >= l) {
				i = l;
			} else {
				return;
			}
		}
	}

	inline function _allocNode():Int {
		var node:Int = _nodeCount;
		_nodeCount++;

		_bounds.push(0.0);
		_bounds.push(0.0);
		_bounds.push(0.0);
		_bounds.push(0.0);
		_bounds.push(0.0);
		_bounds.push(0.0);

		_links.push(0);
		_links.push(0);
		_links.push(0);
		_links.push(0);

		return node;
	}

	inline function _setBounds(node:Int, minx:Float, miny:Float, minz:Float, maxx:Float, maxy:Float, maxz:Float):Void {
		var b:Int = node * 6;
		_bounds[b] = minx;
		_bounds[b + 1] = miny;
		_bounds[b + 2] = minz;
		_bounds[b + 3] = maxx;
		_bounds[b + 4] = maxy;
		_bounds[b + 5] = maxz;
	}

	inline function _setLeaf(node:Int, start:Int, count:Int):Void {
		var l:Int = node * 4;
		_links[l] = -1;
		_links[l + 1] = -1;
		_links[l + 2] = start;
		_links[l + 3] = count;
	}

	inline function _setInternal(node:Int, left:Int, right:Int):Void {
		var l:Int = node * 4;
		_links[l] = left;
		_links[l + 1] = right;
		_links[l + 2] = 0;
		_links[l + 3] = 0;
	}

	function _rayAabb(
		node:Int,
		ox:Float, oy:Float, oz:Float,
		dx:Float, dy:Float, dz:Float,
		maxT:Float
	):Bool {
		var b:Int = node * 6;
		var tmin:Float = 0.0;
		var tmax:Float = maxT;

		var min:Float = _bounds[b];
		var max:Float = _bounds[b + 3];
		if (Math.abs(dx) < EPSILON) {
			if (ox < min || ox > max) return false;
		} else {
			var inv:Float = 1.0 / dx;
			var t1:Float = (min - ox) * inv;
			var t2:Float = (max - ox) * inv;
			if (t1 > t2) {
				var tmp:Float = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			if (tmin > tmax) return false;
		}

		min = _bounds[b + 1];
		max = _bounds[b + 4];
		if (Math.abs(dy) < EPSILON) {
			if (oy < min || oy > max) return false;
		} else {
			var inv:Float = 1.0 / dy;
			var t1:Float = (min - oy) * inv;
			var t2:Float = (max - oy) * inv;
			if (t1 > t2) {
				var tmp:Float = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			if (tmin > tmax) return false;
		}

		min = _bounds[b + 2];
		max = _bounds[b + 5];
		if (Math.abs(dz) < EPSILON) {
			if (oz < min || oz > max) return false;
		} else {
			var inv:Float = 1.0 / dz;
			var t1:Float = (min - oz) * inv;
			var t2:Float = (max - oz) * inv;
			if (t1 > t2) {
				var tmp:Float = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			if (tmin > tmax) return false;
		}

		return true;
	}

	function _rayTriangle(
		triangle:Int,
		ox:Float, oy:Float, oz:Float,
		dx:Float, dy:Float, dz:Float
	):Float {
		var idx:Int = triangle * 3;
		var v1:Vec3 = _vertices[_indices[idx]];
		var v2:Vec3 = _vertices[_indices[idx + 1]];
		var v3:Vec3 = _vertices[_indices[idx + 2]];

		var e1x:Float = v2.x - v1.x;
		var e1y:Float = v2.y - v1.y;
		var e1z:Float = v2.z - v1.z;
		var e2x:Float = v3.x - v1.x;
		var e2y:Float = v3.y - v1.y;
		var e2z:Float = v3.z - v1.z;

		var hx:Float = dy * e2z - dz * e2y;
		var hy:Float = dz * e2x - dx * e2z;
		var hz:Float = dx * e2y - dy * e2x;
		var a:Float = e1x * hx + e1y * hy + e1z * hz;
		if (a > -1e-6 && a < 1e-6) return MathUtil.POSITIVE_INFINITY;

		var f:Float = 1.0 / a;
		var sx:Float = ox - v1.x;
		var sy:Float = oy - v1.y;
		var sz:Float = oz - v1.z;
		var u:Float = f * (sx * hx + sy * hy + sz * hz);
		if (u < 0.0 || u > 1.0) return MathUtil.POSITIVE_INFINITY;

		var qx:Float = sy * e1z - sz * e1y;
		var qy:Float = sz * e1x - sx * e1z;
		var qz:Float = sx * e1y - sy * e1x;
		var v:Float = f * (dx * qx + dy * qy + dz * qz);
		if (v < 0.0 || u + v > 1.0) return MathUtil.POSITIVE_INFINITY;

		var t:Float = f * (e2x * qx + e2y * qy + e2z * qz);
		if (t <= 1e-6) return MathUtil.POSITIVE_INFINITY;
		return t;
	}
}
