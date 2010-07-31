var b2ManifoldPoint = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ManifoldPoint.prototype.__constructor = function(){}
b2ManifoldPoint.prototype.__varz = function(){
this.localPoint1 =  new b2Vec2();
this.localPoint2 =  new b2Vec2();
this.id =  new b2ContactID();
}
// static attributes
// static methods
// attributes
b2ManifoldPoint.prototype.localPoint1 =  new b2Vec2();
b2ManifoldPoint.prototype.localPoint2 =  new b2Vec2();
b2ManifoldPoint.prototype.separation =  null;
b2ManifoldPoint.prototype.normalImpulse =  null;
b2ManifoldPoint.prototype.tangentImpulse =  null;
b2ManifoldPoint.prototype.id =  new b2ContactID();
// methods
b2ManifoldPoint.prototype.Reset = function () {
		this.localPoint1.SetZero();
		this.localPoint2.SetZero();
		this.separation = 0.0;
		this.normalImpulse = 0.0;
		this.tangentImpulse = 0.0;
		this.id.key = 0;
	}
b2ManifoldPoint.prototype.Set = function (m) {
		this.localPoint1.SetV(m.localPoint1);
		this.localPoint2.SetV(m.localPoint2);
		this.separation = m.separation;
		this.normalImpulse = m.normalImpulse;
		this.tangentImpulse = m.tangentImpulse;
		this.id.key = m.id.key;
	}