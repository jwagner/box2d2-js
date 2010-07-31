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
import Box2D.Common.*;
import Box2D.Dynamics.*;


/// A revolute joint constrains to bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with
/// a joint limit that specifies a lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

public class b2RevoluteJoint extends b2Joint
{
	public override function GetAnchor1() :b2Vec2{
		return m_body1.GetWorldPoint(m_localAnchor1);
	}
	public override function GetAnchor2() :b2Vec2{
		return m_body2.GetWorldPoint(m_localAnchor2);
	}

	public override function GetReactionForce() :b2Vec2{
		return m_pivotForce;
	}
	public override function GetReactionTorque() :Number{
		return m_limitForce;
	}

	/// Get the current joint angle in radians.
	public function GetJointAngle() :Number{
		//b2Body* b1 = m_body1;
		//b2Body* b2 = m_body2;
		return m_body2.m_sweep.a - m_body1.m_sweep.a - m_referenceAngle;
	}

	/// Get the current joint angle speed in radians per second.
	public function GetJointSpeed() :Number{
		//b2Body* b1 = m_body1;
		//b2Body* b2 = m_body2;
		return m_body2.m_angularVelocity - m_body1.m_angularVelocity;
	}

	/// Is the joint limit enabled?
	public function IsLimitEnabled() :Boolean{
		return m_enableLimit;
	}

	/// Enable/disable the joint limit.
	public function EnableLimit(flag:Boolean) :void{
		m_enableLimit = flag;
	}

	/// Get the lower joint limit in radians.
	public function GetLowerLimit() :Number{
		return m_lowerAngle;
	}

	/// Get the upper joint limit in radians.
	public function GetUpperLimit() :Number{
		return m_upperAngle;
	}

	/// Set the joint limits in radians.
	public function SetLimits(lower:Number, upper:Number) : void{
		//b2Settings.b2Assert(lower <= upper);
		m_lowerAngle = lower;
		m_upperAngle = upper;
	}

	/// Is the joint motor enabled?
	public function IsMotorEnabled() :Boolean{
		return m_enableMotor;
	}

	/// Enable/disable the joint motor.
	public function EnableMotor(flag:Boolean) :void{
		m_enableMotor = flag;
	}

	/// Set the motor speed in radians per second.
	public function SetMotorSpeed(speed:Number) : void{
		m_motorSpeed = speed;
	}

	/// Get the motor speed in radians per second.
	public function GetMotorSpeed() :Number{
		return m_motorSpeed;
	}

	/// Set the maximum motor torque, usually in N-m.
	public function SetMaxMotorTorque(torque:Number) : void{
		m_maxMotorTorque = torque;
	}

	/// Get the current motor torque, usually in N-m.
	public function GetMotorTorque() :Number{
		return m_motorForce;
	}

	//--------------- Internals Below -------------------

	public function b2RevoluteJoint(def:b2RevoluteJointDef){
		super(def);
		
		//m_localAnchor1 = def->localAnchor1;
		m_localAnchor1.SetV(def.localAnchor1);
		//m_localAnchor2 = def->localAnchor2;
		m_localAnchor2.SetV(def.localAnchor2);
		
		m_referenceAngle = def.referenceAngle;
		
		m_pivotForce.Set(0.0, 0.0);
		m_motorForce = 0.0;
		m_limitForce = 0.0;
		m_limitPositionImpulse = 0.0;
		
		m_lowerAngle = def.lowerAngle;
		m_upperAngle = def.upperAngle;
		m_maxMotorTorque = def.maxMotorTorque;
		m_motorSpeed = def.motorSpeed;
		m_enableLimit = def.enableLimit;
		m_enableMotor = def.enableMotor;
	}

	// internal vars
	private var K:b2Mat22 = new b2Mat22();
	private var K1:b2Mat22 = new b2Mat22();
	private var K2:b2Mat22 = new b2Mat22();
	private var K3:b2Mat22 = new b2Mat22();
	public override function InitVelocityConstraints(step:b2TimeStep) : void{
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var tMat:b2Mat22;
		var tX:Number;
		
		// Compute the effective mass matrix.
		//b2Vec2 r1 = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
		tMat = b1.m_xf.R;
		var r1X:Number = m_localAnchor1.x - b1.m_sweep.localCenter.x;
		var r1Y:Number = m_localAnchor1.y - b1.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
		tMat = b2.m_xf.R;
		var r2X:Number = m_localAnchor2.x - b2.m_sweep.localCenter.x;
		var r2Y:Number = m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		
		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		var invMass1:Number = b1.m_invMass;
		var invMass2:Number = b2.m_invMass;
		var invI1:Number = b1.m_invI;
		var invI2:Number = b2.m_invI;
		
		//var K1:b2Mat22 = new b2Mat22();
		K1.col1.x = invMass1 + invMass2;	K1.col2.x = 0.0;
		K1.col1.y = 0.0;					K1.col2.y = invMass1 + invMass2;
		
		//var K2:b2Mat22 = new b2Mat22();
		K2.col1.x =  invI1 * r1Y * r1Y;	K2.col2.x = -invI1 * r1X * r1Y;
		K2.col1.y = -invI1 * r1X * r1Y;	K2.col2.y =  invI1 * r1X * r1X;
		
		//var K3:b2Mat22 = new b2Mat22();
		K3.col1.x =  invI2 * r2Y * r2Y;	K3.col2.x = -invI2 * r2X * r2Y;
		K3.col1.y = -invI2 * r2X * r2Y;	K3.col2.y =  invI2 * r2X * r2X;
		
		//var K:b2Mat22 = b2Math.AddMM(b2Math.AddMM(K1, K2), K3);
		K.SetM(K1);
		K.AddM(K2);
		K.AddM(K3);
		
		//m_pivotMass = K.Invert();
		K.Invert(m_pivotMass);
		
		m_motorMass = 1.0 / (invI1 + invI2);
		
		if (m_enableMotor == false)
		{
			m_motorForce = 0.0;
		}
		
		if (m_enableLimit)
		{
			//float32 jointAngle = b2->m_sweep.a - b1->m_sweep.a - m_referenceAngle;
			var jointAngle:Number = b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
			if (b2Math.b2Abs(m_upperAngle - m_lowerAngle) < 2.0 * b2Settings.b2_angularSlop)
			{
				m_limitState = e_equalLimits;
			}
			else if (jointAngle <= m_lowerAngle)
			{
				if (m_limitState != e_atLowerLimit)
				{
					m_limitForce = 0.0;
				}
				m_limitState = e_atLowerLimit;
			}
			else if (jointAngle >= m_upperAngle)
			{
				if (m_limitState != e_atUpperLimit)
				{
					m_limitForce = 0.0;
				}
				m_limitState = e_atUpperLimit;
			}
			else
			{
				m_limitState = e_inactiveLimit;
				m_limitForce = 0.0;
			}
		}
		else
		{
			m_limitForce = 0.0;
		}
		
		// Warm starting.
		if (step.warmStarting)
		{
			//b1->m_linearVelocity -= step.dt * invMass1 * m_pivotForce;
			b1.m_linearVelocity.x -= step.dt * invMass1 * m_pivotForce.x;
			b1.m_linearVelocity.y -= step.dt * invMass1 * m_pivotForce.y;
			//b1->m_angularVelocity -= step.dt * invI1 * (b2Cross(r1, m_pivotForce) + m_motorForce + m_limitForce);
			b1.m_angularVelocity -= step.dt * invI1 * ((r1X * m_pivotForce.y - r1Y * m_pivotForce.x) + m_motorForce + m_limitForce);
			
			//b2->m_linearVelocity += step.dt * invMass2 * m_pivotForce;
			b2.m_linearVelocity.x += step.dt * invMass2 * m_pivotForce.x;
			b2.m_linearVelocity.y += step.dt * invMass2 * m_pivotForce.y;
			//b2->m_angularVelocity += step.dt * invI2 * (b2Cross(r2, m_pivotForce) + m_motorForce + m_limitForce);
			b2.m_angularVelocity += step.dt * invI2 * ((r2X * m_pivotForce.y - r2Y * m_pivotForce.x) + m_motorForce + m_limitForce);
		}
		else{
			m_pivotForce.SetZero();
			m_motorForce = 0.0;
			m_limitForce = 0.0;
		}
		
		m_limitPositionImpulse = 0.0;
	}
	
	
	public override function SolveVelocityConstraints(step:b2TimeStep) : void{
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var tMat:b2Mat22;
		var tX:Number;
		
		//b2Vec2 r1 = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
		tMat = b1.m_xf.R;
		var r1X:Number = m_localAnchor1.x - b1.m_sweep.localCenter.x;
		var r1Y:Number = m_localAnchor1.y - b1.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
		tMat = b2.m_xf.R;
		var r2X:Number = m_localAnchor2.x - b2.m_sweep.localCenter.x;
		var r2Y:Number = m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		
		var oldLimitForce:Number;
		
		// Solve point-to-point constraint
		//b2Vec2 pivotCdot = b2.m_linearVelocity + b2Cross(b2.m_angularVelocity, r2) - b1.m_linearVelocity - b2Cross(b1.m_angularVelocity, r1);
		var pivotCdotX:Number = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y) - b1.m_linearVelocity.x - (-b1.m_angularVelocity * r1Y);
		var pivotCdotY:Number = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X) - b1.m_linearVelocity.y - (b1.m_angularVelocity * r1X);
		
		//b2Vec2 pivotForce = -step.inv_dt * b2Mul(m_pivotMass, pivotCdot);
		var pivotForceX:Number = -step.inv_dt * (m_pivotMass.col1.x * pivotCdotX + m_pivotMass.col2.x * pivotCdotY);
		var pivotForceY:Number = -step.inv_dt * (m_pivotMass.col1.y * pivotCdotX + m_pivotMass.col2.y * pivotCdotY);
		m_pivotForce.x += pivotForceX;
		m_pivotForce.y += pivotForceY;
		
		//b2Vec2 P = step.dt * pivotForce;
		var PX:Number = step.dt * pivotForceX;
		var PY:Number = step.dt * pivotForceY;
		
		//b1->m_linearVelocity -= b1->m_invMass * P;
		b1.m_linearVelocity.x -= b1.m_invMass * PX;
		b1.m_linearVelocity.y -= b1.m_invMass * PY;
		//b1->m_angularVelocity -= b1->m_invI * b2Cross(r1, P);
		b1.m_angularVelocity -= b1.m_invI * (r1X * PY - r1Y * PX);
		
		//b2->m_linearVelocity += b2->m_invMass * P;
		b2.m_linearVelocity.x += b2.m_invMass * PX;
		b2.m_linearVelocity.y += b2.m_invMass * PY;
		//b2->m_angularVelocity += b2->m_invI * b2Cross(r2, P);
		b2.m_angularVelocity += b2.m_invI * (r2X * PY - r2Y * PX);
		
		if (m_enableMotor && m_limitState != e_equalLimits)
		{
			var motorCdot:Number = b2.m_angularVelocity - b1.m_angularVelocity - m_motorSpeed;
			var motorForce:Number = -step.inv_dt * m_motorMass * motorCdot;
			var oldMotorForce:Number = m_motorForce;
			m_motorForce = b2Math.b2Clamp(m_motorForce + motorForce, -m_maxMotorTorque, m_maxMotorTorque);
			motorForce = m_motorForce - oldMotorForce;
			
			b1.m_angularVelocity -= b1.m_invI * step.dt * motorForce;
			b2.m_angularVelocity += b2.m_invI * step.dt * motorForce;
		}
		
		if (m_enableLimit && m_limitState != e_inactiveLimit)
		{
			var limitCdot:Number = b2.m_angularVelocity - b1.m_angularVelocity;
			var limitForce:Number = -step.inv_dt * m_motorMass * limitCdot;
			
			if (m_limitState == e_equalLimits)
			{
				m_limitForce += limitForce;
			}
			else if (m_limitState == e_atLowerLimit)
			{
				oldLimitForce = m_limitForce;
				m_limitForce = b2Math.b2Max(m_limitForce + limitForce, 0.0);
				limitForce = m_limitForce - oldLimitForce;
			}
			else if (m_limitState == e_atUpperLimit)
			{
				oldLimitForce = m_limitForce;
				m_limitForce = b2Math.b2Min(m_limitForce + limitForce, 0.0);
				limitForce = m_limitForce - oldLimitForce;
			}
			
			b1.m_angularVelocity -= b1.m_invI * step.dt * limitForce;
			b2.m_angularVelocity += b2.m_invI * step.dt * limitForce;
		}
	}
	
	
	public static var tImpulse:b2Vec2 = new b2Vec2();
	public override function SolvePositionConstraints():Boolean{
		
		var oldLimitImpulse:Number;
		var limitC:Number;
		
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var positionError:Number = 0.0;
		
		var tMat:b2Mat22;
		
		// Solve point-to-point position error.
		//b2Vec2 r1 = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
		tMat = b1.m_xf.R;
		var r1X:Number = m_localAnchor1.x - b1.m_sweep.localCenter.x;
		var r1Y:Number = m_localAnchor1.y - b1.m_sweep.localCenter.y;
		var tX:Number =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
		tMat = b2.m_xf.R;
		var r2X:Number = m_localAnchor2.x - b2.m_sweep.localCenter.x;
		var r2Y:Number = m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		
		//b2Vec2 p1 = b1->m_sweep.c + r1;
		var p1X:Number = b1.m_sweep.c.x + r1X;
		var p1Y:Number = b1.m_sweep.c.y + r1Y;
		//b2Vec2 p2 = b2->m_sweep.c + r2;
		var p2X:Number = b2.m_sweep.c.x + r2X;
		var p2Y:Number = b2.m_sweep.c.y + r2Y;
		
		//b2Vec2 ptpC = p2 - p1;
		var ptpCX:Number = p2X - p1X;
		var ptpCY:Number = p2Y - p1Y;
		
		//float32 positionError = ptpC.Length();
		positionError = Math.sqrt(ptpCX*ptpCX + ptpCY*ptpCY);
		
		// Prevent overly large corrections.
		//b2Vec2 dpMax(b2_maxLinearCorrection, b2_maxLinearCorrection);
		//ptpC = b2Clamp(ptpC, -dpMax, dpMax);
		
		//float32 invMass1 = b1->m_invMass, invMass2 = b2->m_invMass;
		var invMass1:Number = b1.m_invMass;
		var invMass2:Number = b2.m_invMass;
		//float32 invI1 = b1->m_invI, invI2 = b2->m_invI;
		var invI1:Number = b1.m_invI;
		var invI2:Number = b2.m_invI;
		
		//b2Mat22 K1;
		K1.col1.x = invMass1 + invMass2;	K1.col2.x = 0.0;
		K1.col1.y = 0.0;					K1.col2.y = invMass1 + invMass2;
		
		//b2Mat22 K2;
		K2.col1.x =  invI1 * r1Y * r1Y;	K2.col2.x = -invI1 * r1X * r1Y;
		K2.col1.y = -invI1 * r1X * r1Y;	K2.col2.y =  invI1 * r1X * r1X;
		
		//b2Mat22 K3;
		K3.col1.x =  invI2 * r2Y * r2Y;		K3.col2.x = -invI2 * r2X * r2Y;
		K3.col1.y = -invI2 * r2X * r2Y;		K3.col2.y =  invI2 * r2X * r2X;
		
		//b2Mat22 K = K1 + K2 + K3;
		K.SetM(K1);
		K.AddM(K2);
		K.AddM(K3);
		//b2Vec2 impulse = K.Solve(-ptpC);
		K.Solve(tImpulse, -ptpCX, -ptpCY);
		var impulseX:Number = tImpulse.x;
		var impulseY:Number = tImpulse.y;
		
		//b1.m_sweep.c -= b1.m_invMass * impulse;
		b1.m_sweep.c.x -= b1.m_invMass * impulseX;
		b1.m_sweep.c.y -= b1.m_invMass * impulseY;
		//b1.m_sweep.a -= b1.m_invI * b2Cross(r1, impulse);
		b1.m_sweep.a -= b1.m_invI * (r1X * impulseY - r1Y * impulseX);
		
		//b2.m_sweep.c += b2.m_invMass * impulse;
		b2.m_sweep.c.x += b2.m_invMass * impulseX;
		b2.m_sweep.c.y += b2.m_invMass * impulseY;
		//b2.m_sweep.a += b2.m_invI * b2Cross(r2, impulse);
		b2.m_sweep.a += b2.m_invI * (r2X * impulseY - r2Y * impulseX);
		
		b1.SynchronizeTransform();
		b2.SynchronizeTransform();
		
		
		// Handle limits.
		var angularError:Number = 0.0;
		
		if (m_enableLimit && m_limitState != e_inactiveLimit)
		{
			var angle:Number = b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
			var limitImpulse:Number = 0.0;
			
			if (m_limitState == e_equalLimits)
			{
				// Prevent large angular corrections
				limitC = b2Math.b2Clamp(angle, -b2Settings.b2_maxAngularCorrection, b2Settings.b2_maxAngularCorrection);
				limitImpulse = -m_motorMass * limitC;
				angularError = b2Math.b2Abs(limitC);
			}
			else if (m_limitState == e_atLowerLimit)
			{
				limitC = angle - m_lowerAngle;
				angularError = b2Math.b2Max(0.0, -limitC);
				
				// Prevent large angular corrections and allow some slop.
				limitC = b2Math.b2Clamp(limitC + b2Settings.b2_angularSlop, -b2Settings.b2_maxAngularCorrection, 0.0);
				limitImpulse = -m_motorMass * limitC;
				oldLimitImpulse = m_limitPositionImpulse;
				m_limitPositionImpulse = b2Math.b2Max(m_limitPositionImpulse + limitImpulse, 0.0);
				limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			}
			else if (m_limitState == e_atUpperLimit)
			{
				limitC = angle - m_upperAngle;
				angularError = b2Math.b2Max(0.0, limitC);
				
				// Prevent large angular corrections and allow some slop.
				limitC = b2Math.b2Clamp(limitC - b2Settings.b2_angularSlop, 0.0, b2Settings.b2_maxAngularCorrection);
				limitImpulse = -m_motorMass * limitC;
				oldLimitImpulse = m_limitPositionImpulse;
				m_limitPositionImpulse = b2Math.b2Min(m_limitPositionImpulse + limitImpulse, 0.0);
				limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			}
			
			b1.m_sweep.a -= b1.m_invI * limitImpulse;
			b2.m_sweep.a += b2.m_invI * limitImpulse;
			
			b1.SynchronizeTransform();
			b2.SynchronizeTransform();
		}
		
		return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
	}

	public var m_localAnchor1:b2Vec2 = new b2Vec2(); // relative
	public var m_localAnchor2:b2Vec2 = new b2Vec2();
	public var m_pivotForce:b2Vec2 = new b2Vec2();
	public var m_motorForce:Number;
	public var m_limitForce:Number;
	public var m_limitPositionImpulse:Number;

	public var m_pivotMass:b2Mat22 = new b2Mat22();		// effective mass for point-to-point constraint.
	public var m_motorMass:Number;	// effective mass for motor/limit angular constraint.
	public var m_enableMotor:Boolean;
	public var m_maxMotorTorque:Number;
	public var m_motorSpeed:Number;

	public var m_enableLimit:Boolean;
	public var m_referenceAngle:Number;
	public var m_lowerAngle:Number;
	public var m_upperAngle:Number;
	public var m_limitState:int;
};

}
