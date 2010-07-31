var b2ShapeDef = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ShapeDef.prototype.__constructor = function(){}
b2ShapeDef.prototype.__varz = function(){
this.type =  b2Shape.e_unknownShape;
this.filter =  new b2FilterData();
}
// static attributes
// static methods
// attributes
b2ShapeDef.prototype.type =  b2Shape.e_unknownShape;
b2ShapeDef.prototype.userData =  null;
b2ShapeDef.prototype.friction =  0.2;
b2ShapeDef.prototype.restitution =  0.0;
b2ShapeDef.prototype.density =  0.0;
b2ShapeDef.prototype.isSensor =  false;
b2ShapeDef.prototype.filter =  new b2FilterData();
// methods