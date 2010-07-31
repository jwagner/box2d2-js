var b2Distance = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Distance.prototype.__constructor = function(){}
b2Distance.prototype.__varz = function(){
}
// static attributes
b2Distance.g_GJK_Iterations =  0;
b2Distance.s_p1s =  [new b2Vec2(), new b2Vec2(), new b2Vec2()];
b2Distance.s_p2s =  [new b2Vec2(), new b2Vec2(), new b2Vec2()];
b2Distance.s_points =  [new b2Vec2(), new b2Vec2(), new b2Vec2()];
b2Distance.gPoint =  new b2Point();
// static methods
b2Distance.ProcessTwo = function (x1, x2, p1s, p2s, points) {
	var points_0 = points[0];
	var points_1 = points[1];
	var p1s_0 = p1s[0];
	var p1s_1 = p1s[1];
	var p2s_0 = p2s[0];
	var p2s_1 = p2s[1];
	
	
	
	var rX = -points_1.x;
	var rY = -points_1.y;
	
	var dX = points_0.x - points_1.x;
	var dY = points_0.y - points_1.y;
	
	var length = Math.sqrt(dX*dX + dY*dY);
	dX /= length;
	dY /= length;
	
	
	var lambda = rX * dX + rY * dY;
	if (lambda <= 0.0 || length < Number.MIN_VALUE)
	{
		
		
		x1.SetV(p1s_1);
		
		x2.SetV(p2s_1);
		
		p1s_0.SetV(p1s_1);
		
		p2s_0.SetV(p2s_1);
		points_0.SetV(points_1);
		return 1;
	}

	
	lambda /= length;
	
	x1.x = p1s_1.x + lambda * (p1s_0.x - p1s_1.x);
	x1.y = p1s_1.y + lambda * (p1s_0.y - p1s_1.y);
	
	x2.x = p2s_1.x + lambda * (p2s_0.x - p2s_1.x);
	x2.y = p2s_1.y + lambda * (p2s_0.y - p2s_1.y);
	return 2;
}
b2Distance.ProcessThree = function (x1, x2, p1s, p2s, points) {
	var points_0 = points[0];
	var points_1 = points[1];
	var points_2 = points[2];
	var p1s_0 = p1s[0];
	var p1s_1 = p1s[1];
	var p1s_2 = p1s[2];
	var p2s_0 = p2s[0];
	var p2s_1 = p2s[1];
	var p2s_2 = p2s[2];
	
	
	var aX = points_0.x;
	var aY = points_0.y;
	
	var bX = points_1.x;
	var bY = points_1.y;
	
	var cX = points_2.x;
	var cY = points_2.y;

	
	var abX = bX - aX;
	var abY = bY - aY;
	
	var acX = cX - aX;
	var acY = cY - aY;
	
	var bcX = cX - bX;
	var bcY = cY - bY;

	
	var sn = -(aX * abX + aY * abY);
	var sd = (bX * abX + bY * abY);
	
	var tn = -(aX * acX + aY * acY);
	var td = (cX * acX + cY * acY);
	
	var un = -(bX * bcX + bY * bcY);
	var ud = (cX * bcX + cY * bcY);

	
	if (td <= 0.0 && ud <= 0.0)
	{
		
		
		x1.SetV(p1s_2);
		
		x2.SetV(p2s_2);
		
		p1s_0.SetV(p1s_2);
		
		p2s_0.SetV(p2s_2);
		points_0.SetV(points_2);
		return 1;
	}

	
	
	

	
	var n = abX * acY - abY * acX;

	
	
	var vc = n * (aX * bY - aY * bX); 
	
	var lambda;
	
	
	
	var va = n * (bX * cY - bY * cX); 
	if (va <= 0.0 && un >= 0.0 && ud >= 0.0 && (un+ud) > 0.0)
	{
		
		
		
		lambda = un / (un + ud);
		
		x1.x = p1s_1.x + lambda * (p1s_2.x - p1s_1.x);
		x1.y = p1s_1.y + lambda * (p1s_2.y - p1s_1.y);
		
		x2.x = p2s_1.x + lambda * (p2s_2.x - p2s_1.x);
		x2.y = p2s_1.y + lambda * (p2s_2.y - p2s_1.y);
		
		p1s_0.SetV(p1s_2);
		
		p2s_0.SetV(p2s_2);
		
		points_0.SetV(points_2);
		return 2;
	}

	
	
	var vb = n * (cX * aY - cY * aX);
	if (vb <= 0.0 && tn >= 0.0 && td >= 0.0 && (tn+td) > 0.0)
	{
		
		
		
		lambda = tn / (tn + td);
		
		x1.x = p1s_0.x + lambda * (p1s_2.x - p1s_0.x);
		x1.y = p1s_0.y + lambda * (p1s_2.y - p1s_0.y);
		
		x2.x = p2s_0.x + lambda * (p2s_2.x - p2s_0.x);
		x2.y = p2s_0.y + lambda * (p2s_2.y - p2s_0.y);
		
		p1s_1.SetV(p1s_2);
		
		p2s_1.SetV(p2s_2);
		
		points_1.SetV(points_2);
		return 2;
	}

	
	
	var denom = va + vb + vc;
	
	denom = 1.0 / denom;
	
	var u = va * denom;
	
	var v = vb * denom;
	
	var w = 1.0 - u - v;
	
	x1.x = u * p1s_0.x + v * p1s_1.x + w * p1s_2.x;
	x1.y = u * p1s_0.y + v * p1s_1.y + w * p1s_2.y;
	
	x2.x = u * p2s_0.x + v * p2s_1.x + w * p2s_2.x;
	x2.y = u * p2s_0.y + v * p2s_1.y + w * p2s_2.y;
	return 3;
}
b2Distance.InPoints = function (w, points, pointCount) {
	var k_tolerance = 100.0 * Number.MIN_VALUE;
	for (var i = 0; i < pointCount; ++i)
	{
		var points_i = points[i];
		
		var dX = Math.abs(w.x - points_i.x);
		var dY = Math.abs(w.y - points_i.y);
		
		var mX = Math.max(Math.abs(w.x), Math.abs(points_i.x));
		var mY = Math.max(Math.abs(w.y), Math.abs(points_i.y));
		
		if (dX < k_tolerance * (mX + 1.0) &&
			dY < k_tolerance * (mY + 1.0)){
			return true;
		}
	}

	return false;
}
b2Distance.DistanceGeneric = function (x1, x2, 
										shape1, xf1, 
										shape2, xf2) {
	var tVec;
	
	
	var p1s = b2Distance.s_p1s;
	var p2s = b2Distance.s_p2s;
	
	var points = b2Distance.s_points;
	
	var pointCount = 0;

	
	x1.SetV(shape1.GetFirstVertex(xf1));
	
	x2.SetV(shape2.GetFirstVertex(xf2));

	var vSqr = 0.0;
	var maxIterations = 20;
	for (var iter = 0; iter < maxIterations; ++iter)
	{
		
		var vX = x2.x - x1.x;
		var vY = x2.y - x1.y;
		
		var w1 = shape1.Support(xf1, vX, vY);
		
		var w2 = shape2.Support(xf2, -vX, -vY);
		
		vSqr = (vX*vX + vY*vY);
		
		var wX = w2.x - w1.x;
		var wY = w2.y - w1.y;
		
		var vw = (vX*wX + vY*wY);
		
		if (vSqr - vw <= 0.01 * vSqr) 
		{
			if (pointCount == 0)
			{
				
				x1.SetV(w1);
				
				x2.SetV(w2);
			}
			b2Distance.g_GJK_Iterations = iter;
			return Math.sqrt(vSqr);
		}
		
		switch (pointCount)
		{
		case 0:
			
			tVec = p1s[0];
			tVec.SetV(w1);
			
			tVec = p2s[0];
			tVec.SetV(w2);
			
			tVec = points[0];
			tVec.x = wX;
			tVec.y = wY;
			
			x1.SetV(p1s[0]);
			
			x2.SetV(p2s[0]);
			++pointCount;
			break;
			
		case 1:
			
			tVec = p1s[1];
			tVec.SetV(w1);
			
			tVec = p2s[1];
			tVec.SetV(w2);
			
			tVec = points[1];
			tVec.x = wX;
			tVec.y = wY;
			pointCount = b2Distance.ProcessTwo(x1, x2, p1s, p2s, points);
			break;
			
		case 2:
			
			tVec = p1s[2];
			tVec.SetV(w1);
			
			tVec = p2s[2];
			tVec.SetV(w2);
			
			tVec = points[2];
			tVec.x = wX;
			tVec.y = wY;
			pointCount = b2Distance.ProcessThree(x1, x2, p1s, p2s, points);
			break;
		}
		
		
		if (pointCount == 3)
		{
			b2Distance.g_GJK_Iterations = iter;
			return 0.0;
		}
		
		
		var maxSqr = -Number.MAX_VALUE;
		for (var i = 0; i < pointCount; ++i)
		{
			
			tVec = points[i];
			maxSqr = b2Math.b2Max(maxSqr, (tVec.x*tVec.x + tVec.y*tVec.y));
		}
		
		if (pointCount == 3 || vSqr <= 100.0 * Number.MIN_VALUE * maxSqr)
		{
			b2Distance.g_GJK_Iterations = iter;
			
			vX = x2.x - x1.x;
			vY = x2.y - x1.y;
			
			vSqr = (vX*vX + vY*vY);
			return Math.sqrt(vSqr);
		}
	}

	b2Distance.g_GJK_Iterations = maxIterations;
	return Math.sqrt(vSqr);
}
b2Distance.DistanceCC = function (
	x1, x2,
	circle1, xf1,
	circle2, xf2) {
	var tMat;
	var tVec;
	
	tMat = xf1.R;
	tVec = circle1.m_localPosition;
	var p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
	var p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
	
	tMat = xf2.R;
	tVec = circle2.m_localPosition;
	var p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
	var p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

	
	var dX = p2X - p1X;
	var dY = p2Y - p1Y;
	var dSqr = (dX*dX + dY*dY);
	var r1 = circle1.m_radius - b2Settings.b2_toiSlop;
	var r2 = circle2.m_radius - b2Settings.b2_toiSlop;
	var r = r1 + r2;
	if (dSqr > r * r)
	{
		
		var dLen = Math.sqrt(dSqr);
		dX /= dLen;
		dY /= dLen;
		var distance = dLen - r;
		
		x1.x = p1X + r1 * dX;
		x1.y = p1Y + r1 * dY;
		
		x2.x = p2X - r2 * dX;
		x2.y = p2Y - r2 * dY;
		return distance;
	}
	else if (dSqr > Number.MIN_VALUE * Number.MIN_VALUE)
	{
		
		dLen = Math.sqrt(dSqr);
		dX /= dLen;
		dY /= dLen;
		
		x1.x = p1X + r1 * dX;
		x1.y = p1Y + r1 * dY;
		
		x2.x = x1.x;
		x2.y = x1.y;
		return 0.0;
	}

	
	x1.x = p1X;
	x1.y = p1Y;
	
	x2.x = x1.x;
	x2.y = x1.y;
	return 0.0;
}
b2Distance.DistancePC = function (
	x1, x2,
	polygon, xf1,
	circle, xf2) {
	
	var tMat;
	var tVec;
	
	var point = b2Distance.gPoint;
	
	tVec = circle.m_localPosition;
	tMat = xf2.R;
	point.p.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
	point.p.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

	
	var distance = b2Distance.DistanceGeneric(x1, x2, polygon, xf1, point, b2Math.b2XForm_identity);

	var r = circle.m_radius - b2Settings.b2_toiSlop;

	if (distance > r)
	{
		distance -= r;
		
		var dX = x2.x - x1.x;
		var dY = x2.y - x1.y;
		
		var dLen = Math.sqrt(dX*dX + dY*dY);
		dX /= dLen;
		dY /= dLen;
		
		x2.x -= r * dX;
		x2.y -= r * dY;
	}
	else
	{
		distance = 0.0;
		
		x2.x = x1.x;
		x2.y = x1.y;
	}
	
	return distance;
}
b2Distance.Distance = function (x1, x2,
				 shape1, xf1,
				 shape2, xf2) {
	
	var type1 = shape1.m_type;
	
	var type2 = shape2.m_type;

	if (type1 == b2Shape.e_circleShape && type2 == b2Shape.e_circleShape)
	{
		
		return b2Distance.DistanceCC(x1, x2, shape1, xf1, shape2, xf2);
	}
	
	if (type1 == b2Shape.e_polygonShape && type2 == b2Shape.e_circleShape)
	{
		
		return b2Distance.DistancePC(x1, x2, shape1, xf1, shape2, xf2);
	}

	if (type1 == b2Shape.e_circleShape && type2 == b2Shape.e_polygonShape)
	{
		return b2Distance.DistancePC(x2, x1, shape2, xf2, shape1, xf1);
	}

	if (type1 == b2Shape.e_polygonShape && type2 == b2Shape.e_polygonShape)
	{
		return b2Distance.DistanceGeneric(x1, x2, shape1, xf1, shape2, xf2);
	}
	
	return 0.0;
}
// attributes
// methods