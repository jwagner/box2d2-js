var b2ContactConstraint = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactConstraint.prototype.__constructor = function () {
		this.points = new Array(b2Settings.b2_maxManifoldPoints);
		for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++){
			this.points[i] = new b2ContactConstraintPoint();
		}
		
		
	}
b2ContactConstraint.prototype.__varz = function(){
this.normal = new b2Vec2();
}
// static attributes
// static methods
// attributes
b2ContactConstraint.prototype.points =  null;
b2ContactConstraint.prototype.normal = new b2Vec2();
b2ContactConstraint.prototype.manifold =  null;
b2ContactConstraint.prototype.body1 =  null;
b2ContactConstraint.prototype.body2 =  null;
b2ContactConstraint.prototype.friction =  null;
b2ContactConstraint.prototype.restitution =  null;
b2ContactConstraint.prototype.pointCount =  0;
// methods