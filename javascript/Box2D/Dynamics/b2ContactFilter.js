var b2ContactFilter = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactFilter.prototype.__constructor = function(){}
b2ContactFilter.prototype.__varz = function(){
}
// static attributes
b2ContactFilter.b2_defaultFilter =  new b2ContactFilter();
// static methods
// attributes
// methods
b2ContactFilter.prototype.ShouldCollide = function (shape1, shape2) {
		var filter1 = shape1.GetFilterData();
		var filter2 = shape2.GetFilterData();
		
		if (filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0)
		{
			return filter1.groupIndex > 0;
		}
		
		var collide = (filter1.maskBits & filter2.categoryBits) != 0 && (filter1.categoryBits & filter2.maskBits) != 0;
		return collide;
	}