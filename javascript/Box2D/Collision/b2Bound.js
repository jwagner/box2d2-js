var b2Bound = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Bound.prototype.__constructor = function(){}
b2Bound.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2Bound.prototype.value =  0;
b2Bound.prototype.proxyId =  0;
b2Bound.prototype.stabbingCount =  0;
// methods
b2Bound.prototype.IsLower = function () { return (this.value & 1) == 0; }
b2Bound.prototype.IsUpper = function () { return (this.value & 1) == 1; }
b2Bound.prototype.Swap = function (b) {
		var tempValue = this.value;
		var tempProxyId = this.proxyId;
		var tempStabbingCount = this.stabbingCount;
		
		this.value = b.value;
		this.proxyId = b.proxyId;
		this.stabbingCount = b.stabbingCount;
		
		b.value = tempValue;
		b.proxyId = tempProxyId;
		b.stabbingCount = tempStabbingCount;
	}