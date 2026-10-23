package oimo.collision.broadphase;
import oimo.collision.geometry.Aabb;
import oimo.collision.geometry.ConvexGeometry;
import oimo.collision.geometry.RayCastHit;
import oimo.collision.narrowphase.detector.gjkepa.GjkEpa;
import oimo.collision.narrowphase.detector.gjkepa.GjkEpaResultState;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * The abstract class of a broad-phase collision detection algorithm.
 */
@:build(oimo.m.B.bu())
class BroadPhase {
	public var _type:Int;
	public var _numProxies:Int;
	public var _proxyList:Proxy;
	public var _proxyListLast:Proxy;

	public var _proxyPairList:ProxyPair;
	public var _incremental:Bool;

	public var _testCount:Int;

	var _proxyPairPool:ProxyPair;
	var _idCount:Int;

	var _sweepAabb:Array<Aabb>;
	var _sweepTransform:Transform;

	var zero:Vec3;
	var rayCastHit:RayCastHit;

	@:dox(hide)
	public function new(type:Int) {
		_type = type;

		_numProxies = 0;
		_proxyList = null;
		_proxyListLast = null;

		_proxyPairList = null;
		_incremental = false;

		_testCount = 0;

		_proxyPairPool = null;
		_idCount = 0;

		_sweepAabb = [new Aabb(), new Aabb()];
		_sweepTransform = new Transform();

		zero = new Vec3();
		rayCastHit = new RayCastHit();
	}

	// --- private ---

	extern inline function pickAndPushProxyPair(p1:Proxy, p2:Proxy):Void {
		var pp:ProxyPair = M.singleList_pick(_proxyPairPool, _next, new ProxyPair());
		M.singleList_addFirst(_proxyPairList, _next, pp);
		pp._p1 = p1;
		pp._p2 = p2;
	}

	extern inline function poolProxyPairs():Void {
		var p:ProxyPair = _proxyPairList;
		if (p != null) {
			do {
				p._p1 = null;
				p._p2 = null;
				p = p._next;
			} while (p != null);
			_proxyPairList._next = _proxyPairPool;
			_proxyPairPool = _proxyPairList;
			_proxyPairList = null;
		}
	}

	extern inline function addProxy(p:Proxy):Void {
		_numProxies++;
		M.list_push(_proxyList, _proxyListLast, _prev, _next, p);
	}

	extern inline function removeProxy(p:Proxy):Void {
		_numProxies--;
		M.list_remove(_proxyList, _proxyListLast, _prev, _next, p);
	}

	extern inline function aabbSegmentTest(aabbMin:IVec3, aabbMax:IVec3, begin:IVec3, end:IVec3):Bool {
		inline function abs(x:Float):Float {
			return x < 0 ? -x : x;
		}
		inline function min(x:Float, y:Float):Float {
			return x < y ? x : y;
		}
		inline function max(x:Float, y:Float):Float {
			return x > y ? x : y;
		}

		var x1:Float = M.vec3_get(begin, 0);
		var y1:Float = M.vec3_get(begin, 1);
		var z1:Float = M.vec3_get(begin, 2);
		var x2:Float = M.vec3_get(end, 0);
		var y2:Float = M.vec3_get(end, 1);
		var z2:Float = M.vec3_get(end, 2);
		var sminx:Float = min(x1, x2);
		var sminy:Float = min(y1, y2);
		var sminz:Float = min(z1, z2);
		var smaxx:Float = max(x1, x2);
		var smaxy:Float = max(y1, y2);
		var smaxz:Float = max(z1, z2);
		var pminx:Float = M.vec3_get(aabbMin, 0);
		var pminy:Float = M.vec3_get(aabbMin, 1);
		var pminz:Float = M.vec3_get(aabbMin, 2);
		var pmaxx:Float = M.vec3_get(aabbMax, 0);
		var pmaxy:Float = M.vec3_get(aabbMax, 1);
		var pmaxz:Float = M.vec3_get(aabbMax, 2);

		if (
			// axis1: (1, 0, 0)
			// axis2: (0, 1, 0)
			// axis3: (0, 0, 1)
			pminx > smaxx || pmaxx < sminx ||
			pminy > smaxy || pmaxy < sminy ||
			pminz > smaxz || pmaxz < sminz
		) {
			return false;
		}

		var dx:Float = x2 - x1;
		var dy:Float = y2 - y1;
		var dz:Float = z2 - z1;
		var adx:Float = abs(dx);
		var ady:Float = abs(dy);
		var adz:Float = abs(dz);
		var pextx:Float = (pmaxx - pminx) * 0.5;
		var pexty:Float = (pmaxy - pminy) * 0.5;
		var pextz:Float = (pmaxz - pminz) * 0.5;
		var pcntx:Float = (pmaxx + pminx) * 0.5;
		var pcnty:Float = (pmaxy + pminy) * 0.5;
		var pcntz:Float = (pmaxz + pminz) * 0.5;
		var cpx:Float = x1 - pcntx;
		var cpy:Float = y1 - pcnty;
		var cpz:Float = z1 - pcntz;

		if (
			// axis4: (dx, dy, dz) x (1, 0, 0) = (0, dz, -dy)
			// axis5: (dx, dy, dz) x (0, 1, 0) = (-dz, 0, dx)
			// axis6: (dx, dy, dz) x (0, 0, 1) = (dy, -dx, 0)
			abs(cpy * dz - cpz * dy) - (pexty * adz + pextz * ady) > 0 ||
			abs(cpz * dx - cpx * dz) - (pextz * adx + pextx * adz) > 0 ||
			abs(cpx * dy - cpy * dx) - (pextx * ady + pexty * adx) > 0
		) {
			return false;
		}

		return true;
	}

	/**
	 * Conservative swept-AABB cull: the query box is the union of the convex's AABBs
	 * at the begin and end transforms. Same idea as Bullet Physics convex-vs-concave
	 * sweep, implemented here with OimoPhysics' own AABB helpers. Assumes constant
	 * rotation during the sweep, like Bullet; a rotating sweep can escape the union.
	 * Callers still run the exact narrow-phase test on every candidate.
	**/
	extern inline function aabbConvexSweepTest(aabbMin:IVec3, aabbMax:IVec3, convex:ConvexGeometry, begin:Transform, translation:Vec3):Bool {
		var beginAabb:Aabb = _sweepAabb[0];
		var endAabb:Aabb = _sweepAabb[1];

		var tl:IVec3;
		M.vec3_fromVec3(tl, translation);

		convex._computeAabb(beginAabb, begin);

		var endTransform:Transform = _sweepTransform;
		M.mat3_assign(endTransform._rotation, begin._rotation);
		M.vec3_add(endTransform._position, begin._position, tl);
		convex._computeAabb(endAabb, endTransform);

		// Union of the begin and end AABBs: the conservative swept box.
		var bmin:IVec3;
		var bmax:IVec3;
		M.vec3_min(bmin, beginAabb._min, endAabb._min);
		M.vec3_max(bmax, beginAabb._max, endAabb._max);

		// Overlap test; separated on any axis means "no possible hit".
		if (M.vec3_get(aabbMax, 0) < M.vec3_get(bmin, 0)) return false;
		if (M.vec3_get(aabbMin, 0) > M.vec3_get(bmax, 0)) return false;
		if (M.vec3_get(aabbMax, 1) < M.vec3_get(bmin, 1)) return false;
		if (M.vec3_get(aabbMin, 1) > M.vec3_get(bmax, 1)) return false;
		if (M.vec3_get(aabbMax, 2) < M.vec3_get(bmin, 2)) return false;
		if (M.vec3_get(aabbMin, 2) > M.vec3_get(bmax, 2)) return false;
		return true;
	}

	// --- public ---

	/**
	 * Returns a new proxy connected with the user data `userData` containing the axis-aligned
	 * bounding box `aabb`, and adds the proxy into the collision space.
	 */
	public function createProxy(userData:Any, aabb:Aabb):Proxy {
		return null;
	}

	/**
	 * Removes the proxy `proxy` from the collision space.
	 */
	public function destroyProxy(proxy:Proxy):Void {
	}

	/**
	 * Moves the proxy `proxy` to the axis-aligned bounding box `aabb`. `displacement` is the
	 * difference between current and previous center of the AABB. This is used for predicting
	 * movement of the proxy.
	 */
	public function moveProxy(proxy:Proxy, aabb:Aabb, displacement:Vec3):Void {
	}

	/**
	 * Returns whether the pair of `proxy1` and `proxy2` is overlapping. As proxies can be larger
	 * than the containing AABBs, two proxies may overlap even though their inner AABBs are separate.
	 */
	public inline function isOverlapping(proxy1:Proxy, proxy2:Proxy):Bool {
		return M.aabb_overlap(proxy1._aabbMin, proxy1._aabbMax, proxy2._aabbMin, proxy2._aabbMax);
	}

	/**
	 * Collects overlapping pairs of the proxies and put them into a linked list. The linked list
	 * can be get through `BroadPhase.getProxyPairList` method.
	 *
	 * Note that in order to collect pairs, the broad-phase algorithm requires to be informed of
	 * movements of proxies through `BroadPhase.moveProxy` method.
	 */
	public function collectPairs():Void {
	}

	/**
	 * Returns the linked list of collected pairs of proxies.
	 */
	public inline function getProxyPairList():ProxyPair {
		return _proxyPairList;
	}

	/**
	 * Returns whether to collect only pairs created in the last step. If this returns
	 * true, the pairs that are not collected might still be overlapping. Otherwise, such
	 * pairs are guaranteed to be separated.
	 */
	public inline function isIncremental():Bool {
		return _incremental;
	}

	/**
	 * Returns the number of broad-phase AABB tests.
	 */
	public inline function getTestCount():Int {
		return _testCount;
	}

	/**
	 * Performs a ray casting. `callback.process` is called for all proxies the line segment
	 * from `begin` to `end` intersects.
	 */
	public function rayCast(begin:Vec3, end:Vec3, callback:BroadPhaseProxyCallback):Void {
	}

	/**
	 * Performs a convex casting. `callback.process` is called for all shapes the convex geometry
	 * `convex` hits. The convex geometry translates by `translation` starting from the beginning
	 * transform `begin`.
	 */
	public function convexCast(convex:ConvexGeometry, begin:Transform, translation:Vec3, callback:BroadPhaseProxyCallback):Void {
	}

	/**
	 * Performs an AABB query. `callback.process` is called for all proxies that their AABB
	 * and `aabb` intersect.
	 */
	public function aabbTest(aabb:Aabb, callback:BroadPhaseProxyCallback):Void {
	}
}

@:build(oimo.m.B.bu())
private class ConvexSweepGeometry extends ConvexGeometry {
	var c:ConvexGeometry;
	var localTranslation:Vec3;

	public function new() {
		super(-1);
	}

	public function init(c:ConvexGeometry, transform:Transform, translation:Vec3):Void {
		this.c = c;

		var tr:IVec3;
		M.vec3_fromVec3(tr, translation);

		var localTr:IVec3;
		M.vec3_mulMat3Transposed(localTr, tr, transform._rotation);

		localTranslation = new Vec3();
		M.vec3_toVec3(localTranslation, localTr);

		_gjkMargin = c._gjkMargin;
	}

	override public function computeLocalSupportingVertex(dir:Vec3, out:Vec3):Void {
		c.computeLocalSupportingVertex(dir, out);
		if (dir.dot(localTranslation) > 0) {
			out.addEq(localTranslation);
		}
	}
}

private class AabbGeometry extends ConvexGeometry {
	public var min:Vec3;
	public var max:Vec3;

	public function new() {
		super(-1);
		min = new Vec3();
		max = new Vec3();
	}

	override public function computeLocalSupportingVertex(dir:Vec3, out:Vec3):Void {
		out.x = dir.x > 0 ? max.x : min.x;
		out.y = dir.y > 0 ? max.y : min.y;
		out.z = dir.z > 0 ? max.z : min.z;
	}
}
