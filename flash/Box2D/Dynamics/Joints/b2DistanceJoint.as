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

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k * 

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.

public class b2DistanceJoint extends b2Joint
{
	//--------------- Internals Below -------------------

	public function b2DistanceJoint(def:b2DistanceJointDef){
		super(def);
		
		var tMat:b2Mat22;
		var tX:Number;
		var tY:Number;
		//m_localAnchor1 = def->localAnchor1;
		m_localAnchor1.SetV(def.localAnchor1);
		//m_localAnchor2 = def->localAnchor2;
		m_localAnchor2.SetV(def.localAnchor2);
		
		m_length = def.length;
		m_frequencyHz = def.frequencyHz;
		m_dampingRatio = def.dampingRatio;
		m_impulse = 0.0;
		m_gamma = 0.0;
		m_bias = 0.0;
		m_inv_dt = 0.0;
	}

	public override function InitVelocityConstraints(step:b2TimeStep) : void{
		
		var tMat:b2Mat22;
		var tX:Number;
		
		m_inv_dt = step.inv_dt;

		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
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
		
		//m_u = b2->m_sweep.c + r2 - b1->m_sweep.c - r1;
		m_u.x = b2.m_sweep.c.x + r2X - b1.m_sweep.c.x - r1X;
		m_u.y = b2.m_sweep.c.y + r2Y - b1.m_sweep.c.y - r1Y;
		
		// Handle singularity.
		//float32 length = m_u.Length();
		var length:Number = Math.sqrt(m_u.x*m_u.x + m_u.y*m_u.y);
		if (length > b2Settings.b2_linearSlop)
		{
			//m_u *= 1.0 / length;
			m_u.Multiply( 1.0 / length );
		}
		else
		{
			m_u.SetZero();
		}
		
		//float32 cr1u = b2Cross(r1, m_u);
		var cr1u:Number = (r1X * m_u.y - r1Y * m_u.x);
		//float32 cr2u = b2Cross(r2, m_u);
		var cr2u:Number = (r2X * m_u.y - r2Y * m_u.x);
		//m_mass = b1->m_invMass + b1->m_invI * cr1u * cr1u + b2->m_invMass + b2->m_invI * cr2u * cr2u;
		var invMass:Number = b1.m_invMass + b1.m_invI * cr1u * cr1u + b2.m_invMass + b2.m_invI * cr2u * cr2u;
		//b2Settings.b2Assert(invMass > Number.MIN_VALUE);
		m_mass = 1.0 / invMass;
		
		if (m_frequencyHz > 0.0)
		{
			var C:Number = length - m_length;
	
			// Frequency
			var omega:Number = 2.0 * Math.PI * m_frequencyHz;
	
			// Damping coefficient
			var d:Number = 2.0 * m_mass * m_dampingRatio * omega;
	
			// Spring stiffness
			var k:Number = m_mass * omega * omega;
	
			// magic formulas
			m_gamma = 1.0 / (step.dt * (d + step.dt * k));
			m_bias = C * step.dt * k * m_gamma;
	
			m_mass = 1.0 / (invMass + m_gamma);
		}
		
		if (step.warmStarting)
		{
			m_impulse *= step.dtRatio;
			//b2Vec2 P = m_impulse * m_u;
			var PX:Number = m_impulse * m_u.x;
			var PY:Number = m_impulse * m_u.y;
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
		}
		else
		{
			m_impulse = 0.0;
		}
	}
	
	
	
	public override function SolveVelocityConstraints(step:b2TimeStep): void{
		
		var tMat:b2Mat22;
		
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
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
		
		// Cdot = dot(u, v + cross(w, r))
		//b2Vec2 v1 = b1->m_linearVelocity + b2Cross(b1->m_angularVelocity, r1);
		var v1X:Number = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
		var v1Y:Number = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
		//b2Vec2 v2 = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2);
		var v2X:Number = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
		var v2Y:Number = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);
		//float32 Cdot = b2Dot(m_u, v2 - v1);
		var Cdot:Number = (m_u.x * (v2X - v1X) + m_u.y * (v2Y - v1Y));
		
		var impulse:Number = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
		m_impulse += impulse;
		
		//b2Vec2 P = impulse * m_u;
		var PX:Number = impulse * m_u.x;
		var PY:Number = impulse * m_u.y;
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
	}
	
	public override function SolvePositionConstraints():Boolean{
		
		var tMat:b2Mat22;
		
		if (m_frequencyHz > 0.0)
		{
			return true;
		}
		
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
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
		
		//b2Vec2 d = b2->m_sweep.c + r2 - b1->m_sweep.c - r1;
		var dX:Number = b2.m_sweep.c.x + r2X - b1.m_sweep.c.x - r1X;
		var dY:Number = b2.m_sweep.c.y + r2Y - b1.m_sweep.c.y - r1Y;
		
		//float32 length = d.Normalize();
		var length:Number = Math.sqrt(dX*dX + dY*dY);
		dX /= length;
		dY /= length;
		//float32 C = length - m_length;
		var C:Number = length - m_length;
		C = b2Math.b2Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
		
		var impulse:Number = -m_mass * C;
		//m_u = d;
		m_u.Set(dX, dY);
		//b2Vec2 P = impulse * m_u;
		var PX:Number = impulse * m_u.x;
		var PY:Number = impulse * m_u.y;
		
		//b1->m_sweep.c -= b1->m_invMass * P;
		b1.m_sweep.c.x -= b1.m_invMass * PX;
		b1.m_sweep.c.y -= b1.m_invMass * PY;
		//b1->m_sweep.a -= b1->m_invI * b2Cross(r1, P);
		b1.m_sweep.a -= b1.m_invI * (r1X * PY - r1Y * PX);
		//b2->m_sweep.c += b2->m_invMass * P;
		b2.m_sweep.c.x += b2.m_invMass * PX;
		b2.m_sweep.c.y += b2.m_invMass * PY;
		//b2->m_sweep.a -= b2->m_invI * b2Cross(r2, P);
		b2.m_sweep.a += b2.m_invI * (r2X * PY - r2Y * PX);
		
		b1.SynchronizeTransform();
		b2.SynchronizeTransform();
		
		return b2Math.b2Abs(C) < b2Settings.b2_linearSlop;
		
	}
	
	public override function GetAnchor1():b2Vec2{
		return m_body1.GetWorldPoint(m_localAnchor1);
	}
	public override function GetAnchor2():b2Vec2{
		return m_body2.GetWorldPoint(m_localAnchor2);
	}
	
	public override function GetReactionForce():b2Vec2
	{
		//b2Vec2 F = (m_inv_dt * m_impulse) * m_u;
		var F:b2Vec2 = new b2Vec2();
		F.SetV(m_u);
		F.Multiply(m_inv_dt * m_impulse);
		return F;
	}

	public override function GetReactionTorque():Number
	{
		//NOT_USED(invTimeStep);
		return 0.0;
	}

	public var m_localAnchor1:b2Vec2 = new b2Vec2();
	public var m_localAnchor2:b2Vec2 = new b2Vec2();
	public var m_u:b2Vec2 = new b2Vec2();
	public var m_frequencyHz:Number;
	public var m_dampingRatio:Number;
	public var m_gamma:Number;
	public var m_bias:Number;
	public var m_impulse:Number;
	public var m_mass:Number;	// effective mass for the constraint.
	public var m_length:Number;
};

}
