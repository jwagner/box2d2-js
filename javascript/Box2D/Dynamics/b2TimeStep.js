var b2TimeStep = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2TimeStep.prototype.__constructor = function(){}
b2TimeStep.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2TimeStep.prototype.dt =  null;
b2TimeStep.prototype.inv_dt =  null;
b2TimeStep.prototype.dtRatio =  null;
b2TimeStep.prototype.maxIterations =  0;
b2TimeStep.prototype.warmStarting =  null;
b2TimeStep.prototype.positionCorrection =  null;
// methods