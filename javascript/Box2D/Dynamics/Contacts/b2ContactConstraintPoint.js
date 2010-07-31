var b2ContactConstraintPoint = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactConstraintPoint.prototype.__constructor = function(){}
b2ContactConstraintPoint.prototype.__varz = function(){
this.localAnchor1 = new b2Vec2();
this.localAnchor2 = new b2Vec2();
this.r1 = new b2Vec2();
this.r2 = new b2Vec2();
}
// static attributes
// static methods
// attributes
b2ContactConstraintPoint.prototype.localAnchor1 = new b2Vec2();
b2ContactConstraintPoint.prototype.localAnchor2 = new b2Vec2();
b2ContactConstraintPoint.prototype.r1 = new b2Vec2();
b2ContactConstraintPoint.prototype.r2 = new b2Vec2();
b2ContactConstraintPoint.prototype.normalImpulse =  null;
b2ContactConstraintPoint.prototype.tangentImpulse =  null;
b2ContactConstraintPoint.prototype.positionImpulse =  null;
b2ContactConstraintPoint.prototype.normalMass =  null;
b2ContactConstraintPoint.prototype.tangentMass =  null;
b2ContactConstraintPoint.prototype.equalizedMass =  null;
b2ContactConstraintPoint.prototype.separation =  null;
b2ContactConstraintPoint.prototype.velocityBias =  null;
// methods