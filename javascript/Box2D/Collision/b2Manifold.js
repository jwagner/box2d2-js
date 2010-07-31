var b2Manifold = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Manifold.prototype.__constructor = function () {
		this.points = new Array(b2Settings.b2_maxManifoldPoints);
		for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++){
			this.points[i] = new b2ManifoldPoint();
		}
		this.normal = new b2Vec2();
	}
b2Manifold.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2Manifold.prototype.points =  null;
b2Manifold.prototype.normal =  null;
b2Manifold.prototype.pointCount =  0;
// methods
b2Manifold.prototype.Reset = function () {
		for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++){
			(this.points[i]).Reset();
		}
		this.normal.SetZero();
		this.pointCount = 0;
	}
b2Manifold.prototype.Set = function (m) {
		this.pointCount = m.pointCount;
		for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++){
			(this.points[i]).Set(m.points[i]);
		}
		this.normal.SetV(m.normal);
	}