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

package Box2D.Dynamics.Joints{
	
	
import Box2D.Common.Math.*;
import Box2D.Dynamics.*;



/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, max lengths for each side,
/// and a pulley ratio.

public class b2PulleyJointDef extends b2JointDef
{
	public function b2PulleyJointDef()
	{
		type = b2Joint.e_pulleyJoint;
		groundAnchor1.Set(-1.0, 1.0);
		groundAnchor2.Set(1.0, 1.0);
		localAnchor1.Set(-1.0, 0.0);
		localAnchor2.Set(1.0, 0.0);
		length1 = 0.0;
		maxLength1 = 0.0;
		length2 = 0.0;
		maxLength2 = 0.0;
		ratio = 1.0;
		collideConnected = true;
	}
	
	public function Initialize(b1:b2Body, b2:b2Body,
				ga1:b2Vec2, ga2:b2Vec2,
				anchor1:b2Vec2, anchor2:b2Vec2,
				r:Number) : void
	{
		body1 = b1;
		body2 = b2;
		groundAnchor1.SetV( ga1 );
		groundAnchor2.SetV( ga2 );
		localAnchor1 = body1.GetLocalPoint(anchor1);
		localAnchor2 = body2.GetLocalPoint(anchor2);
		//b2Vec2 d1 = anchor1 - ga1;
		var d1X:Number = anchor1.x - ga1.x;
		var d1Y:Number = anchor1.y - ga1.y;
		//length1 = d1.Length();
		length1 = Math.sqrt(d1X*d1X + d1Y*d1Y);
		
		//b2Vec2 d2 = anchor2 - ga2;
		var d2X:Number = anchor2.x - ga2.x;
		var d2Y:Number = anchor2.y - ga2.y;
		//length2 = d2.Length();
		length2 = Math.sqrt(d2X*d2X + d2Y*d2Y);
		
		ratio = r;
		//b2Settings.b2Assert(ratio > Number.MIN_VALUE);
		var C:Number = length1 + ratio * length2;
		maxLength1 = C - ratio * b2PulleyJoint.b2_minPulleyLength;
		maxLength2 = (C - b2PulleyJoint.b2_minPulleyLength) / ratio;
	}

	/// The first ground anchor in world coordinates. This point never moves.
	public var groundAnchor1:b2Vec2 = new b2Vec2();
	
	/// The second ground anchor in world coordinates. This point never moves.
	public var groundAnchor2:b2Vec2 = new b2Vec2();
	
	/// The local anchor point relative to body1's origin.
	public var localAnchor1:b2Vec2 = new b2Vec2();
	
	/// The local anchor point relative to body2's origin.
	public var localAnchor2:b2Vec2 = new b2Vec2();
	
	/// The a reference length for the segment attached to body1.
	public var length1:Number;
	
	/// The maximum length of the segment attached to body1.
	public var maxLength1:Number;
	
	/// The a reference length for the segment attached to body2.
	public var length2:Number;
	
	/// The maximum length of the segment attached to body2.
	public var maxLength2:Number;
	
	/// The pulley ratio, used to simulate a block-and-tackle.
	public var ratio:Number;
	
};

}