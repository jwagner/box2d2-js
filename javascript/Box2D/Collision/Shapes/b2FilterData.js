var b2FilterData = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2FilterData.prototype.__constructor = function(){}
b2FilterData.prototype.__varz = function(){
this.categoryBits =  0x0001;
this.maskBits =  0xFFFF;
}
// static attributes
// static methods
// attributes
b2FilterData.prototype.categoryBits =  0x0001;
b2FilterData.prototype.maskBits =  0xFFFF;
b2FilterData.prototype.groupIndex =  0;
// methods
b2FilterData.prototype.Copy = function () {
		var copy = new b2FilterData();
		copy.categoryBits = this.categoryBits;
		copy.maskBits = this.maskBits;
		copy.groupIndex = this.groupIndex;
		return copy;
	}