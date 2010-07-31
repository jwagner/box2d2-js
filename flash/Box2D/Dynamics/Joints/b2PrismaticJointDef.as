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
	

public class b2PrismaticJointDef extends b2JointDef
{
	public function b2PrismaticJointDef()
	{
		type = b2Joint.e_prismaticJoint;
		//localAnchor1.SetZero();
		//localAnchor2.SetZero();
		localAxis1.Set(1.0, 0.0);
		referenceAngle = 0.0;
		enableLimit = false;
		lowerTranslation = 0.0;
		upperTranslation = 0.0;
		enableMotor = false;
		maxMotorForce = 0.0;
		motorSpeed = 0.0;
	}
	
	public function Initialize(b1:b2Body, b2:b2Body, anchor:b2Vec2, axis:b2Vec2) : void
	{
		body1 = b1;
		body2 = b2;
		localAnchor1 = body1.GetLocalPoint(anchor);
		localAnchor2 = body2.GetLocalPoint(anchor);
		localAxis1 = body1.GetLocalVector(axis);
		referenceAngle = body2.GetAngle() - body1.GetAngle();
	}

	/// The local anchor point relative to body1's origin.
	public var localAnchor1:b2Vec2 = new b2Vec2();

	/// The local anchor point relative to body2's origin.
	public var localAnchor2:b2Vec2 = new b2Vec2();

	/// The local translation axis in body1.
	public var localAxis1:b2Vec2 = new b2Vec2();

	/// The constrained angle between the bodies: body2_angle - body1_angle.
	public var referenceAngle:Number;

	/// Enable/disable the joint limit.
	public var enableLimit:Boolean;

	/// The lower translation limit, usually in meters.
	public var lowerTranslation:Number;

	/// The upper translation limit, usually in meters.
	public var upperTranslation:Number;

	/// Enable/disable the joint motor.
	public var enableMotor:Boolean;

	/// The maximum motor torque, usually in N-m.
	public var maxMotorForce:Number;

	/// The desired motor speed in radians per second.
	public var motorSpeed:Number;
};

}