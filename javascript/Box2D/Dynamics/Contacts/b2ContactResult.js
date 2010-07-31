var b2ContactResult = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactResult.prototype.__constructor = function(){}
b2ContactResult.prototype.__varz = function(){
this.position =  new b2Vec2();
this.normal =  new b2Vec2();
this.id =  new b2ContactID();
}
// static attributes
// static methods
// attributes
b2ContactResult.prototype.shape1 =  null;
b2ContactResult.prototype.shape2 =  null;
b2ContactResult.prototype.position =  new b2Vec2();
b2ContactResult.prototype.normal =  new b2Vec2();
b2ContactResult.prototype.normalImpulse =  null;
b2ContactResult.prototype.tangentImpulse =  null;
b2ContactResult.prototype.id =  new b2ContactID();
// methods