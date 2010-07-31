var b2Settings = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Settings.prototype.__constructor = function(){}
b2Settings.prototype.__varz = function(){
}
// static attributes
b2Settings.USHRT_MAX =  0x0000ffff;
b2Settings.b2_pi =  Math.PI;
b2Settings.b2_maxManifoldPoints =  2;
b2Settings.b2_maxPolygonVertices =  8;
b2Settings.b2_maxProxies =  512;
b2Settings.b2_maxPairs =  8 * b2Settings.b2_maxProxies;
b2Settings.b2_linearSlop =  0.005;
b2Settings.b2_angularSlop =  2.0 / 180.0 * b2Settings.b2_pi;
b2Settings.b2_toiSlop =  8.0 * b2Settings.b2_linearSlop;
b2Settings.b2_maxTOIContactsPerIsland =  32;
b2Settings.b2_velocityThreshold =  1.0;
b2Settings.b2_maxLinearCorrection =  0.2;
b2Settings.b2_maxAngularCorrection =  8.0 / 180.0 * b2Settings.b2_pi;
b2Settings.b2_maxLinearVelocity =  200.0;
b2Settings.b2_maxLinearVelocitySquared =  b2Settings.b2_maxLinearVelocity * b2Settings.b2_maxLinearVelocity;
b2Settings.b2_maxAngularVelocity =  250.0;
b2Settings.b2_maxAngularVelocitySquared =  b2Settings.b2_maxAngularVelocity * b2Settings.b2_maxAngularVelocity;
b2Settings.b2_contactBaumgarte =  0.2;
b2Settings.b2_timeToSleep =  0.5;
b2Settings.b2_linearSleepTolerance =  0.01;
b2Settings.b2_angularSleepTolerance =  2.0 / 180.0;
// static methods
b2Settings.b2Assert = function (a) {
		if (!a){
			var nullVec;
			nullVec.x++;
		}
	}
// attributes
// methods
