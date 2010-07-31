var b2Proxy = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Proxy.prototype.__constructor = function(){}
b2Proxy.prototype.__varz = function(){
this.lowerBounds =  [parseInt(0), parseInt(0)];
this.upperBounds =  [parseInt(0), parseInt(0)];
}
// static attributes
// static methods
// attributes
b2Proxy.prototype.lowerBounds =  [parseInt(0), parseInt(0)];
b2Proxy.prototype.upperBounds =  [parseInt(0), parseInt(0)];
b2Proxy.prototype.overlapCount =  0;
b2Proxy.prototype.timeStamp =  0;
b2Proxy.prototype.userData =  null;
// methods
b2Proxy.prototype.GetNext = function () { return this.lowerBounds[0]; }
b2Proxy.prototype.SetNext = function (next) { this.lowerBounds[0] = next % 65535; }
b2Proxy.prototype.IsValid = function () { return this.overlapCount != b2BroadPhase.b2_invalid; }