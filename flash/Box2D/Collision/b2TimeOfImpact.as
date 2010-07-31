/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

package Box2D.Collision{
	
import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Collision.Shapes.*;
import Box2D.Collision.*;

public class b2TimeOfImpact
{

// This algorithm uses conservative advancement to compute the time of
// impact (TOI) of two shapes.
// Refs: Bullet, Young Kim
//
static public var s_p1:b2Vec2 = new b2Vec2();
static public var s_p2:b2Vec2 = new b2Vec2();
static public var s_xf1:b2XForm = new b2XForm();
static public var s_xf2:b2XForm = new b2XForm();
//
static public function TimeOfImpact(	shape1:b2Shape, sweep1:b2Sweep,
								shape2:b2Shape, sweep2:b2Sweep) : Number
{
	var math1:Number;
	var math2:Number;
	
	var r1:Number = shape1.m_sweepRadius;
	var r2:Number = shape2.m_sweepRadius;

	//b2Settings.b2Assert(sweep1.t0 == sweep2.t0);
	//b2Settings.b2Assert(1.0 - sweep1.t0 > Number.MIN_VALUE);

	var t0:Number = sweep1.t0;
	//b2Vec2 v1 = sweep1.c - sweep1.c0;
	var v1X:Number = sweep1.c.x - sweep1.c0.x;
	var v1Y:Number = sweep1.c.y - sweep1.c0.y;
	//b2Vec2 v2 = sweep2.c - sweep2.c0;
	var v2X:Number = sweep2.c.x - sweep2.c0.x;
	var v2Y:Number = sweep2.c.y - sweep2.c0.y;
	var omega1:Number = sweep1.a - sweep1.a0;
	var omega2:Number = sweep2.a - sweep2.a0;

	var alpha:Number = 0.0;

	var p1:b2Vec2 = s_p1;
	var p2:b2Vec2 = s_p2;
	var k_maxIterations:int = 20;	// TODO_ERIN b2Settings
	var iter:int = 0;
	//b2Vec2 normal = b2Vec2_zero;
	var normalX:Number = 0.0;
	var normalY:Number = 0.0;
	var distance:Number = 0.0;
	var targetDistance:Number = 0.0;
	for(;;)
	{
		var t:Number = (1.0 - alpha) * t0 + alpha;
		//b2XForm xf1, xf2;
		var xf1:b2XForm = s_xf1;
		var xf2:b2XForm = s_xf2;
		sweep1.GetXForm(xf1, t);
		sweep2.GetXForm(xf2, t);
		
		// Get the distance between shapes.
		distance = b2Distance.Distance(p1, p2, shape1, xf1, shape2, xf2);
		
		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement.
			if (distance > 2.0 * b2Settings.b2_toiSlop)
			{
				targetDistance = 1.5 * b2Settings.b2_toiSlop;
			}
			else
			{
				//targetDistance = Math.max(0.05 * b2Settings.b2_toiSlop, distance - 0.5 * b2Settings.b2_toiSlop);
				math1 = 0.05 * b2Settings.b2_toiSlop;
				math2 = distance - 0.5 * b2Settings.b2_toiSlop;
				targetDistance = math1 > math2 ? math1 : math2;
			}
		}
		
		if (distance - targetDistance < 0.05 * b2Settings.b2_toiSlop || iter == k_maxIterations)
		{
			break;
		}
		
		//normal = p2 - p1;
		normalX = p2.x - p1.x;
		normalY = p2.y - p1.y;
		//normal.Normalize();
		var nLen:Number = Math.sqrt(normalX*normalX + normalY*normalY);
		normalX /= nLen;
		normalY /= nLen;
		
		// Compute upper bound on remaining movement.
		//float32 approachVelocityBound = b2Dot(normal, v1 - v2) + b2Abs(omega1) * r1 + b2Abs(omega2) * r2;
		var approachVelocityBound:Number = 	(normalX*(v1X - v2X) + normalY*(v1Y - v2Y))
											+ (omega1 < 0 ? -omega1 : omega1) * r1 
											+ (omega2 < 0 ? -omega2 : omega2) * r2;
		//if (Math.abs(approachVelocityBound) < Number.MIN_VALUE)
		if (approachVelocityBound == 0)
		{
			alpha = 1.0;
			break;
		}
		
		// Get the conservative time increment. Don't advance all the way.
		var dAlpha:Number = (distance - targetDistance) / approachVelocityBound;
		//float32 dt = (distance - 0.5f * b2_linearSlop) / approachVelocityBound;
		var newAlpha:Number = alpha + dAlpha;
		
		// The shapes may be moving apart or a safe distance apart.
		if (newAlpha < 0.0 || 1.0 < newAlpha)
		{
			alpha = 1.0;
			break;
		}
		
		// Ensure significant advancement.
		if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha)
		{
			break;
		}
		
		alpha = newAlpha;
		
		++iter;
	}

	return alpha;
}


}


}