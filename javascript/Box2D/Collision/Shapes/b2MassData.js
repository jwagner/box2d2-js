var b2MassData = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2MassData.prototype.__constructor = function(){}
b2MassData.prototype.__varz = function(){
this.center =  new b2Vec2(0,0);
}
// static attributes
// static methods
// attributes
b2MassData.prototype.mass =  0.0;
b2MassData.prototype.center =  new b2Vec2(0,0);
b2MassData.prototype.I =  0.0;
// methods