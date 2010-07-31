var b2ContactPoint = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactPoint.prototype.__constructor = function(){}
b2ContactPoint.prototype.__varz = function(){
this.position =  new b2Vec2();
this.velocity =  new b2Vec2();
this.normal =  new b2Vec2();
this.id =  new b2ContactID();
}
// static attributes
// static methods
// attributes
b2ContactPoint.prototype.shape1 =  null;
b2ContactPoint.prototype.shape2 =  null;
b2ContactPoint.prototype.position =  new b2Vec2();
b2ContactPoint.prototype.velocity =  new b2Vec2();
b2ContactPoint.prototype.normal =  new b2Vec2();
b2ContactPoint.prototype.separation =  null;
b2ContactPoint.prototype.friction =  null;
b2ContactPoint.prototype.restitution =  null;
b2ContactPoint.prototype.id =  new b2ContactID();
// methods