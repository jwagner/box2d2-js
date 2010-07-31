var b2TimeOfImpact = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2TimeOfImpact.prototype.__constructor = function(){}
b2TimeOfImpact.prototype.__varz = function(){
}
// static attributes
b2TimeOfImpact.s_p1 =  new b2Vec2();
b2TimeOfImpact.s_p2 =  new b2Vec2();
b2TimeOfImpact.s_xf1 =  new b2XForm();
b2TimeOfImpact.s_xf2 =  new b2XForm();
// static methods
b2TimeOfImpact.TimeOfImpact = function (	shape1, sweep1,
								shape2, sweep2) {
	var math1;
	var math2;
	
	var r1 = shape1.m_sweepRadius;
	var r2 = shape2.m_sweepRadius;

	
	

	var t0 = sweep1.t0;
	
	var v1X = sweep1.c.x - sweep1.c0.x;
	var v1Y = sweep1.c.y - sweep1.c0.y;
	
	var v2X = sweep2.c.x - sweep2.c0.x;
	var v2Y = sweep2.c.y - sweep2.c0.y;
	var omega1 = sweep1.a - sweep1.a0;
	var omega2 = sweep2.a - sweep2.a0;

	var alpha = 0.0;

	var p1 = b2TimeOfImpact.s_p1;
	var p2 = b2TimeOfImpact.s_p2;
	var k_maxIterations = 20;	
	var iter = 0;
	
	var normalX = 0.0;
	var normalY = 0.0;
	var distance = 0.0;
	var targetDistance = 0.0;
	for(;;)
	{
		var t = (1.0 - alpha) * t0 + alpha;
		
		var xf1 = b2TimeOfImpact.s_xf1;
		var xf2 = b2TimeOfImpact.s_xf2;
		sweep1.GetXForm(xf1, t);
		sweep2.GetXForm(xf2, t);
		
		
		distance = b2Distance.Distance(p1, p2, shape1, xf1, shape2, xf2);
		
		if (iter == 0)
		{
			
			
			if (distance > 2.0 * b2Settings.b2_toiSlop)
			{
				targetDistance = 1.5 * b2Settings.b2_toiSlop;
			}
			else
			{
				
				math1 = 0.05 * b2Settings.b2_toiSlop;
				math2 = distance - 0.5 * b2Settings.b2_toiSlop;
				targetDistance = math1 > math2 ? math1 : math2;
			}
		}
		
		if (distance - targetDistance < 0.05 * b2Settings.b2_toiSlop || iter == k_maxIterations)
		{
			break;
		}
		
		
		normalX = p2.x - p1.x;
		normalY = p2.y - p1.y;
		
		var nLen = Math.sqrt(normalX*normalX + normalY*normalY);
		normalX /= nLen;
		normalY /= nLen;
		
		
		
		var approachVelocityBound = 	(normalX*(v1X - v2X) + normalY*(v1Y - v2Y))
											+ (omega1 < 0 ? -omega1 : omega1) * r1 
											+ (omega2 < 0 ? -omega2 : omega2) * r2;
		
		if (approachVelocityBound == 0)
		{
			alpha = 1.0;
			break;
		}
		
		
		var dAlpha = (distance - targetDistance) / approachVelocityBound;
		
		var newAlpha = alpha + dAlpha;
		
		
		if (newAlpha < 0.0 || 1.0 < newAlpha)
		{
			alpha = 1.0;
			break;
		}
		
		
		if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha)
		{
			break;
		}
		
		alpha = newAlpha;
		
		++iter;
	}

	return alpha;
}
// attributes
// methods