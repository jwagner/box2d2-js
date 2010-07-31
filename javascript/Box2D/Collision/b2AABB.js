var b2AABB = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2AABB.prototype.__constructor = function(){}
b2AABB.prototype.__varz = function(){
this.lowerBound =  new b2Vec2();
this.upperBound =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2AABB.prototype.lowerBound =  new b2Vec2();
b2AABB.prototype.upperBound =  new b2Vec2();
// methods
b2AABB.prototype.IsValid = function () {
		
		var dX = this.upperBound.x - this.lowerBound.x;
		var dY = this.upperBound.y - this.lowerBound.y;
		var valid = dX >= 0.0 && dY >= 0.0;
		valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
		return valid;
	}