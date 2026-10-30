package oimo.dynamics;
import oimo.collision.broadphase.*;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.ConvexOverlapResolver;
import oimo.common.Transform;
import oimo.dynamics.callback.ConvexOverlapCallback;
import oimo.dynamics.rigidbody.Shape;
import oimo.m.M;

/**
 * Convex overlap wrapper (broadphase -> world), used by `World.convexOverlap`.
 */
@:dox(hide)
@:build(oimo.m.B.bu())
class ConvexOverlapWrapper extends BroadPhaseProxyCallback {
	public var aabb:Aabb;

	var callback:ConvexOverlapCallback;
	var resolver:ConvexOverlapResolver;

	public function new() {
		super();
		aabb = new Aabb();
		callback = null;
		resolver = new ConvexOverlapResolver();
	}

	public function prepare(convex:ConvexGeometry, transform:Transform, callback:ConvexOverlapCallback):Void {
		this.callback = callback;
		resolver.prepare(convex, transform);
		convex._computeAabb(aabb, transform);
	}

	override public function process(proxy:Proxy):Void {
		var shape:Shape = cast proxy.userData;

		// The broadphase uses fat AABBs, so check the exact query AABB first.
		if (!M.aabb_overlap(shape._aabb._min, shape._aabb._max, aabb._min, aabb._max)) return;

		// A static mesh reports one hit per overlapping triangle.
		var numHits:Int = resolver.resolve(shape._geom, shape._transform);
		for (i in 0...numHits) {
			callback.process(shape, resolver.getHit(i));
		}
	}
}
