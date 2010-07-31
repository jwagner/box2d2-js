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

	
/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// The pulley also enforces a maximum length limit on both sides. This is
/// useful to prevent one side of the pulley hitting the top.
	
public class b2PulleyJoint extends b2Joint
{
	public override function GetAnchor1():b2Vec2{
		return m_body1.GetWorldPoint(m_localAnchor1);
	}
	public override function GetAnchor2():b2Vec2{
		return m_body2.GetWorldPoint(m_localAnchor2);
	}

	public override function GetReactionForce() :b2Vec2
	{
		//b2Vec2 F = m_force * m_u2;
		var F:b2Vec2 = m_u2.Copy();
		F.Multiply(m_force);
		return F;
	}

	public override function GetReactionTorque() :Number
	{
		return 0.0;
	}

	public function GetGroundAnchor1() :b2Vec2
	{
		//return m_ground.m_xf.position + m_groundAnchor1;
		var a:b2Vec2 = m_ground.m_xf.position.Copy();
		a.Add(m_groundAnchor1);
		return a;
	}

	public function GetGroundAnchor2() :b2Vec2
	{
		//return m_ground.m_xf.position + m_groundAnchor2;
		var a:b2Vec2 = m_ground.m_xf.position.Copy();
		a.Add(m_groundAnchor2);
		return a;
	}

	public function GetLength1() :Number
	{
		var p:b2Vec2 = m_body1.GetWorldPoint(m_localAnchor1);
		//b2Vec2 s = m_ground->m_xf.position + m_groundAnchor1;
		var sX:Number = m_ground.m_xf.position.x + m_groundAnchor1.x;
		var sY:Number = m_ground.m_xf.position.y + m_groundAnchor1.y;
		//b2Vec2 d = p - s;
		var dX:Number = p.x - sX;
		var dY:Number = p.y - sY;
		//return d.Length();
		return Math.sqrt(dX*dX + dY*dY);
	}

	public function GetLength2() :Number
	{
		var p:b2Vec2 = m_body2.GetWorldPoint(m_localAnchor2);
		//b2Vec2 s = m_ground->m_xf.position + m_groundAnchor2;
		var sX:Number = m_ground.m_xf.position.x + m_groundAnchor2.x;
		var sY:Number = m_ground.m_xf.position.y + m_groundAnchor2.y;
		//b2Vec2 d = p - s;
		var dX:Number = p.x - sX;
		var dY:Number = p.y - sY;
		//return d.Length();
		return Math.sqrt(dX*dX + dY*dY);
	}

	public function GetRatio():Number{
		return m_ratio;
	}

	//--------------- Internals Below -------------------

	public function b2PulleyJoint(def:b2PulleyJointDef){
		
		// parent
		super(def);
		
		var tMat:b2Mat22;
		var tX:Number;
		var tY:Number;
		
		m_ground = m_body1.m_world.m_groundBody;
		//m_groundAnchor1 = def->groundAnchor1 - m_ground->m_xf.position;
		m_groundAnchor1.x = def.groundAnchor1.x - m_ground.m_xf.position.x;
		m_groundAnchor1.y = def.groundAnchor1.y - m_ground.m_xf.position.y;
		//m_groundAnchor2 = def->groundAnchor2 - m_ground->m_xf.position;
		m_groundAnchor2.x = def.groundAnchor2.x - m_ground.m_xf.position.x;
		m_groundAnchor2.y = def.groundAnchor2.y - m_ground.m_xf.position.y;
		//m_localAnchor1 = def->localAnchor1;
		m_localAnchor1.SetV(def.localAnchor1);
		//m_localAnchor2 = def->localAnchor2;
		m_localAnchor2.SetV(def.localAnchor2);
		
		//b2Settings.b2Assert(def.ratio != 0.0);
		m_ratio = def.ratio;
		
		m_constant = def.length1 + m_ratio * def.length2;
		
		m_maxLength1 = b2Math.b2Min(def.maxLength1, m_constant - m_ratio * b2_minPulleyLength);
		m_maxLength2 = b2Math.b2Min(def.maxLength2, (m_constant - b2_minPulleyLength) / m_ratio);
		
		m_force = 0.0;
		m_limitForce1 = 0.0;
		m_limitForce2 = 0.0;
		
	}

	public override function InitVelocityConstraints(step:b2TimeStep) : void{
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var tMat:b2Mat22;
		
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
		
		//b2Vec2 s1 = m_ground->m_xf.position + m_groundAnchor1;
		var s1X:Number = m_ground.m_xf.position.x + m_groundAnchor1.x;
		var s1Y:Number = m_ground.m_xf.position.y + m_groundAnchor1.y;
		//b2Vec2 s2 = m_ground->m_xf.position + m_groundAnchor2;
		var s2X:Number = m_ground.m_xf.position.x + m_groundAnchor2.x;
		var s2Y:Number = m_ground.m_xf.position.y + m_groundAnchor2.y;
		
		// Get the pulley axes.
		//m_u1 = p1 - s1;
		m_u1.Set(p1X - s1X, p1Y - s1Y);
		//m_u2 = p2 - s2;
		m_u2.Set(p2X - s2X, p2Y - s2Y);
		
		var length1:Number = m_u1.Length();
		var length2:Number = m_u2.Length();
		
		if (length1 > b2Settings.b2_linearSlop)
		{
			//m_u1 *= 1.0f / length1;
			m_u1.Multiply(1.0 / length1);
		}
		else
		{
			m_u1.SetZero();
		}
		
		if (length2 > b2Settings.b2_linearSlop)
		{
			//m_u2 *= 1.0f / length2;
			m_u2.Multiply(1.0 / length2);
		}
		else
		{
			m_u2.SetZero();
		}
		
		var C:Number = m_constant - length1 - m_ratio * length2;
		if (C > 0.0)
		{
			m_state = e_inactiveLimit;
			m_force = 0.0;
		}
		else
		{
			m_state = e_atUpperLimit;
			m_positionImpulse = 0.0;
		}
		
		if (length1 < m_maxLength1)
		{
			m_limitState1 = e_inactiveLimit;
			m_limitForce1 = 0.0;
		}
		else
		{
			m_limitState1 = e_atUpperLimit;
			m_limitPositionImpulse1 = 0.0;
		}
		
		if (length2 < m_maxLength2)
		{
			m_limitState2 = e_inactiveLimit;
			m_limitForce2 = 0.0;
		}
		else
		{
			m_limitState2 = e_atUpperLimit;
			m_limitPositionImpulse2 = 0.0;
		}
		
		// Compute effective mass.
		//var cr1u1:Number = b2Cross(r1, m_u1);
		var cr1u1:Number = r1X * m_u1.y - r1Y * m_u1.x;
		//var cr2u2:Number = b2Cross(r2, m_u2);
		var cr2u2:Number = r2X * m_u2.y - r2Y * m_u2.x;
		
		m_limitMass1 = b1.m_invMass + b1.m_invI * cr1u1 * cr1u1;
		m_limitMass2 = b2.m_invMass + b2.m_invI * cr2u2 * cr2u2;
		m_pulleyMass = m_limitMass1 + m_ratio * m_ratio * m_limitMass2;
		//b2Settings.b2Assert(m_limitMass1 > Number.MIN_VALUE);
		//b2Settings.b2Assert(m_limitMass2 > Number.MIN_VALUE);
		//b2Settings.b2Assert(m_pulleyMass > Number.MIN_VALUE);
		m_limitMass1 = 1.0 / m_limitMass1;
		m_limitMass2 = 1.0 / m_limitMass2;
		m_pulleyMass = 1.0 / m_pulleyMass;
		
		if (step.warmStarting)
		{
			// Warm starting.
			//b2Vec2 P1 = step.dt * (-m_force - m_limitForce1) * m_u1;
			//b2Vec2 P1 = step.dt * (-m_force - m_limitForce1) * m_u1;
			var P1X:Number = step.dt * (-m_force - m_limitForce1) * m_u1.x;
			var P1Y:Number = step.dt * (-m_force - m_limitForce1) * m_u1.y;
			//b2Vec2 P2 = step.dt * (-m_ratio * m_force - m_limitForce2) * m_u2;
			//b2Vec2 P2 = step.dt * (-m_ratio * m_force - m_limitForce2) * m_u2;
			var P2X:Number = step.dt * (-m_ratio * m_force - m_limitForce2) * m_u2.x;
			var P2Y:Number = step.dt * (-m_ratio * m_force - m_limitForce2) * m_u2.y;
			//b1.m_linearVelocity += b1.m_invMass * P1;
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			//b1.m_angularVelocity += b1.m_invI * b2Cross(r1, P1);
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
			//b2.m_linearVelocity += b2.m_invMass * P2;
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			//b2.m_angularVelocity += b2.m_invI * b2Cross(r2, P2);
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		}
		else
		{
			m_force = 0.0;
			m_limitForce1 = 0.0;
			m_limitForce2 = 0.0;
		}
	}
	
	public override function SolveVelocityConstraints(step:b2TimeStep) : void{
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var tMat:b2Mat22;
		
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
		
		// temp vars
		var v1X:Number;
		var v1Y:Number;
		var v2X:Number;
		var v2Y:Number;
		var P1X:Number;
		var P1Y:Number;
		var P2X:Number;
		var P2Y:Number;
		var Cdot:Number;
		var force:Number;
		var oldForce:Number;
		
		if (m_state == e_atUpperLimit)
		{
			//b2Vec2 v1 = b1->m_linearVelocity + b2Cross(b1->m_angularVelocity, r1);
			v1X = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
			v1Y = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
			//b2Vec2 v2 = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2);
			v2X = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
			v2Y = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);
			
			//Cdot = -b2Dot(m_u1, v1) - m_ratio * b2Dot(m_u2, v2);
			Cdot = -(m_u1.x * v1X + m_u1.y * v1Y) - m_ratio * (m_u2.x * v2X + m_u2.y * v2Y);
			force = -step.inv_dt * m_pulleyMass * Cdot;
			oldForce = m_force;
			m_force = b2Math.b2Max(0.0, m_force + force);
			force = m_force - oldForce;
			
			//b2Vec2 P1 = -step.dt * force * m_u1;
			P1X = -step.dt * force * m_u1.x;
			P1Y = -step.dt * force * m_u1.y;
			//b2Vec2 P2 = -step.dt * m_ratio * force * m_u2;
			P2X = -step.dt * m_ratio * force * m_u2.x;
			P2Y = -step.dt * m_ratio * force * m_u2.y;
			//b1.m_linearVelocity += b1.m_invMass * P1;
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			//b1.m_angularVelocity += b1.m_invI * b2Cross(r1, P1);
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
			//b2.m_linearVelocity += b2.m_invMass * P2;
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			//b2.m_angularVelocity += b2.m_invI * b2Cross(r2, P2);
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		}
		
		if (m_limitState1 == e_atUpperLimit)
		{
			//b2Vec2 v1 = b1->m_linearVelocity + b2Cross(b1->m_angularVelocity, r1);
			v1X = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
			v1Y = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
			
			//float32 Cdot = -b2Dot(m_u1, v1);
			Cdot = -(m_u1.x * v1X + m_u1.y * v1Y);
			force = -step.inv_dt * m_limitMass1 * Cdot;
			oldForce = m_limitForce1;
			m_limitForce1 = b2Math.b2Max(0.0, m_limitForce1 + force);
			force = m_limitForce1 - oldForce;
			
			//b2Vec2 P1 = -step.dt * force * m_u1;
			P1X = -step.dt * force * m_u1.x;
			P1Y = -step.dt * force * m_u1.y;
			//b1.m_linearVelocity += b1->m_invMass * P1;
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			//b1.m_angularVelocity += b1->m_invI * b2Cross(r1, P1);
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
		}
		
		if (m_limitState2 == e_atUpperLimit)
		{
			//b2Vec2 v2 = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2);
			v2X = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
			v2Y = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);
			
			//float32 Cdot = -b2Dot(m_u2, v2);
			Cdot = -(m_u2.x * v2X + m_u2.y * v2Y);
			force = -step.inv_dt * m_limitMass2 * Cdot;
			oldForce = m_limitForce2;
			m_limitForce2 = b2Math.b2Max(0.0, m_limitForce2 + force);
			force = m_limitForce2 - oldForce;
			
			//b2Vec2 P2 = -step.dt * force * m_u2;
			P2X = -step.dt * force * m_u2.x;
			P2Y = -step.dt * force * m_u2.y;
			//b2->m_linearVelocity += b2->m_invMass * P2;
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			//b2->m_angularVelocity += b2->m_invI * b2Cross(r2, P2);
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		}
	}
	
	
	
	public override function SolvePositionConstraints():Boolean{
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var tMat:b2Mat22;
		
		//b2Vec2 s1 = m_ground->m_xf.position + m_groundAnchor1;
		var s1X:Number = m_ground.m_xf.position.x + m_groundAnchor1.x;
		var s1Y:Number = m_ground.m_xf.position.y + m_groundAnchor1.y;
		//b2Vec2 s2 = m_ground->m_xf.position + m_groundAnchor2;
		var s2X:Number = m_ground.m_xf.position.x + m_groundAnchor2.x;
		var s2Y:Number = m_ground.m_xf.position.y + m_groundAnchor2.y;
		
		// temp vars
		var r1X:Number;
		var r1Y:Number;
		var r2X:Number;
		var r2Y:Number;
		var p1X:Number;
		var p1Y:Number;
		var p2X:Number;
		var p2Y:Number;
		var length1:Number;
		var length2:Number;
		var C:Number;
		var impulse:Number;
		var oldImpulse:Number;
		var oldLimitPositionImpulse:Number;
		
		var tX:Number;
		
		var linearError:Number = 0.0;
		
		if (m_state == e_atUpperLimit)
		{
			//b2Vec2 r1 = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
			tMat = b1.m_xf.R;
			r1X = m_localAnchor1.x - b1.m_sweep.localCenter.x;
			r1Y = m_localAnchor1.y - b1.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			//b2Vec2 r2 = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
			tMat = b2.m_xf.R;
			r2X = m_localAnchor2.x - b2.m_sweep.localCenter.x;
			r2Y = m_localAnchor2.y - b2.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			
			//b2Vec2 p1 = b1->m_sweep.c + r1;
			p1X = b1.m_sweep.c.x + r1X;
			p1Y = b1.m_sweep.c.y + r1Y;
			//b2Vec2 p2 = b2->m_sweep.c + r2;
			p2X = b2.m_sweep.c.x + r2X;
			p2Y = b2.m_sweep.c.y + r2Y;
			
			// Get the pulley axes.
			//m_u1 = p1 - s1;
			m_u1.Set(p1X - s1X, p1Y - s1Y);
			//m_u2 = p2 - s2;
			m_u2.Set(p2X - s2X, p2Y - s2Y);
			
			length1 = m_u1.Length();
			length2 = m_u2.Length();
			
			if (length1 > b2Settings.b2_linearSlop)
			{
				//m_u1 *= 1.0f / length1;
				m_u1.Multiply( 1.0 / length1 );
			}
			else
			{
				m_u1.SetZero();
			}
			
			if (length2 > b2Settings.b2_linearSlop)
			{
				//m_u2 *= 1.0f / length2;
				m_u2.Multiply( 1.0 / length2 );
			}
			else
			{
				m_u2.SetZero();
			}
			
			C = m_constant - length1 - m_ratio * length2;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -m_pulleyMass * C;
			
			oldImpulse = m_positionImpulse;
			m_positionImpulse = b2Math.b2Max(0.0, m_positionImpulse + impulse);
			impulse = m_positionImpulse - oldImpulse;
			
			p1X = -impulse * m_u1.x;
			p1Y = -impulse * m_u1.y;
			p2X = -m_ratio * impulse * m_u2.x;
			p2Y = -m_ratio * impulse * m_u2.y;
			
			b1.m_sweep.c.x += b1.m_invMass * p1X;
			b1.m_sweep.c.y += b1.m_invMass * p1Y;
			b1.m_sweep.a += b1.m_invI * (r1X * p1Y - r1Y * p1X);
			b2.m_sweep.c.x += b2.m_invMass * p2X;
			b2.m_sweep.c.y += b2.m_invMass * p2Y;
			b2.m_sweep.a += b2.m_invI * (r2X * p2Y - r2Y * p2X);
			
			b1.SynchronizeTransform();
			b2.SynchronizeTransform();
		}
		
		if (m_limitState1 == e_atUpperLimit)
		{
			//b2Vec2 r1 = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
			tMat = b1.m_xf.R;
			r1X = m_localAnchor1.x - b1.m_sweep.localCenter.x;
			r1Y = m_localAnchor1.y - b1.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			//b2Vec2 p1 = b1->m_sweep.c + r1;
			p1X = b1.m_sweep.c.x + r1X;
			p1Y = b1.m_sweep.c.y + r1Y;
			
			//m_u1 = p1 - s1;
			m_u1.Set(p1X - s1X, p1Y - s1Y);
			
			length1 = m_u1.Length();
			
			if (length1 > b2Settings.b2_linearSlop)
			{
				//m_u1 *= 1.0 / length1;
				m_u1.x *= 1.0 / length1;
				m_u1.y *= 1.0 / length1;
			}
			else
			{
				m_u1.SetZero();
			}
			
			C = m_maxLength1 - length1;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -m_limitMass1 * C;
			oldLimitPositionImpulse = m_limitPositionImpulse1;
			m_limitPositionImpulse1 = b2Math.b2Max(0.0, m_limitPositionImpulse1 + impulse);
			impulse = m_limitPositionImpulse1 - oldLimitPositionImpulse;
			
			//P1 = -impulse * m_u1;
			p1X = -impulse * m_u1.x;
			p1Y = -impulse * m_u1.y;
			
			b1.m_sweep.c.x += b1.m_invMass * p1X;
			b1.m_sweep.c.y += b1.m_invMass * p1Y;
			//b1.m_rotation += b1.m_invI * b2Cross(r1, P1);
			b1.m_sweep.a += b1.m_invI * (r1X * p1Y - r1Y * p1X);
			
			b1.SynchronizeTransform();
		}
		
		if (m_limitState2 == e_atUpperLimit)
		{
			//b2Vec2 r2 = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
			tMat = b2.m_xf.R;
			r2X = m_localAnchor2.x - b2.m_sweep.localCenter.x;
			r2Y = m_localAnchor2.y - b2.m_sweep.localCenter.y;
			tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			//b2Vec2 p2 = b2->m_position + r2;
			p2X = b2.m_sweep.c.x + r2X;
			p2Y = b2.m_sweep.c.y + r2Y;
			
			//m_u2 = p2 - s2;
			m_u2.Set(p2X - s2X, p2Y - s2Y);
			
			length2 = m_u2.Length();
			
			if (length2 > b2Settings.b2_linearSlop)
			{
				//m_u2 *= 1.0 / length2;
				m_u2.x *= 1.0 / length2;
				m_u2.y *= 1.0 / length2;
			}
			else
			{
				m_u2.SetZero();
			}
			
			C = m_maxLength2 - length2;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -m_limitMass2 * C;
			oldLimitPositionImpulse = m_limitPositionImpulse2;
			m_limitPositionImpulse2 = b2Math.b2Max(0.0, m_limitPositionImpulse2 + impulse);
			impulse = m_limitPositionImpulse2 - oldLimitPositionImpulse;
			
			//P2 = -impulse * m_u2;
			p2X = -impulse * m_u2.x;
			p2Y = -impulse * m_u2.y;
			
			//b2.m_sweep.c += b2.m_invMass * P2;
			b2.m_sweep.c.x += b2.m_invMass * p2X;
			b2.m_sweep.c.y += b2.m_invMass * p2Y;
			//b2.m_sweep.a += b2.m_invI * b2Cross(r2, P2);
			b2.m_sweep.a += b2.m_invI * (r2X * p2Y - r2Y * p2X);
			
			b2.SynchronizeTransform();
		}
		
		return linearError < b2Settings.b2_linearSlop;
	}
	
	

	public var m_ground:b2Body;
	public var m_groundAnchor1:b2Vec2 = new b2Vec2();
	public var m_groundAnchor2:b2Vec2 = new b2Vec2();
	public var m_localAnchor1:b2Vec2 = new b2Vec2();
	public var m_localAnchor2:b2Vec2 = new b2Vec2();

	public var m_u1:b2Vec2 = new b2Vec2();
	public var m_u2:b2Vec2 = new b2Vec2();
	
	public var m_constant:Number;
	public var m_ratio:Number;
	
	public var m_maxLength1:Number;
	public var m_maxLength2:Number;

	// Effective masses
	public var m_pulleyMass:Number;
	public var m_limitMass1:Number;
	public var m_limitMass2:Number;

	// Impulses for accumulation/warm starting.
	public var m_force:Number;
	public var m_limitForce1:Number;
	public var m_limitForce2:Number;

	// Position impulses for accumulation.
	public var m_positionImpulse:Number;
	public var m_limitPositionImpulse1:Number;
	public var m_limitPositionImpulse2:Number;

	public var m_state:int;
	public var m_limitState1:int;
	public var m_limitState2:int;
	
	// static
	static public const b2_minPulleyLength:Number = 2.0;
};
	
	
}