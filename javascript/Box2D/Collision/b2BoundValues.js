var b2BoundValues = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2BoundValues.prototype.__constructor = function(){}
b2BoundValues.prototype.__varz = function(){
this.lowerValues =  [0,0];
this.upperValues =  [0,0];
}
// static attributes
// static methods
// attributes
b2BoundValues.prototype.lowerValues =  [0,0];
b2BoundValues.prototype.upperValues =  [0,0];
// methods