var b2CircleDef = function() {
b2ShapeDef.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2CircleDef.prototype, b2ShapeDef.prototype)
b2CircleDef.prototype._super = function(){ b2ShapeDef.prototype.__constructor.apply(this, arguments) }
b2CircleDef.prototype.__constructor = function () {
		this.type = b2Shape.e_circleShape;
		this.radius = 1.0;
	}
b2CircleDef.prototype.__varz = function(){
this.localPosition =  new b2Vec2(0.0, 0.0);
}
// static attributes
// static methods
// attributes
b2CircleDef.prototype.localPosition =  new b2Vec2(0.0, 0.0);
b2CircleDef.prototype.radius =  null;
// methods