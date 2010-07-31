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


// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.

public class b2MouseJoint extends b2Joint
{
	/// Implements b2Joint.
	public override function GetAnchor1():b2Vec2{
		return m_target;
	}
	/// Implements b2Joint.
	public override function GetAnchor2():b2Vec2{
		return m_body2.GetWorldPoint(m_localAnchor);
	}
	/// Implements b2Joint.
	public override function GetReactionForce():b2Vec2
	{
		return m_impulse;
	}
	/// Implements b2Joint.
	public override function GetReactionTorque():Number
	{
		return 0.0;
	}
	/// Use this to update the target point.
	public function SetTarget(target:b2Vec2) : void{
		if (m_body2.IsSleeping()){
			m_body2.WakeUp();
		}
		m_target = target;
	}

	//--------------- Internals Below -------------------

	public function b2MouseJoint(def:b2MouseJointDef){
		super(def);
		
		m_target.SetV(def.target);
		//m_localAnchor = b2MulT(m_body2.m_xf, m_target);
		var tX:Number = m_target.x - m_body2.m_xf.position.x;
		var tY:Number = m_target.y - m_body2.m_xf.position.y;
		var tMat:b2Mat22 = m_body2.m_xf.R;
		m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
		m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);
		
		m_maxForce = def.maxForce;
		m_impulse.SetZero();
		
		var mass:Number = m_body2.m_mass;
		
		// Frequency
		var omega:Number = 2.0 * b2Settings.b2_pi * def.frequencyHz;
		
		// Damping coefficient
		var d:Number = 2.0 * mass * def.dampingRatio * omega;
		
		// Spring stiffness
		var k:Number = (def.timeStep * mass) * (omega * omega);
		
		// magic formulas
		//b2Assert(d + k > B2_FLT_EPSILON);
		m_gamma = 1.0 / (d + k);
		m_beta = k / (d + k);
	}

	// Presolve vars
	private var K:b2Mat22 = new b2Mat22();
	private var K1:b2Mat22 = new b2Mat22();
	private var K2:b2Mat22 = new b2Mat22();
	public override function InitVelocityConstraints(step:b2TimeStep): void{
		var b:b2Body = m_body2;
		
		var tMat:b2Mat22;
		
		// Compute the effective mass matrix.
		//b2Vec2 r = b2Mul(b->m_xf.R, m_localAnchor - b->GetLocalCenter());
		tMat = b.m_xf.R;
		var rX:Number = m_localAnchor.x - b.m_sweep.localCenter.x;
		var rY:Number = m_localAnchor.y - b.m_sweep.localCenter.y;
		var tX:Number = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;
		
		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		var invMass:Number = b.m_invMass;
		var invI:Number = b.m_invI;
		
		//b2Mat22 K1;
		K1.col1.x = invMass;	K1.col2.x = 0.0;
		K1.col1.y = 0.0;		K1.col2.y = invMass;
		
		//b2Mat22 K2;
		K2.col1.x =  invI * rY * rY;	K2.col2.x = -invI * rX * rY;
		K2.col1.y = -invI * rX * rY;	K2.col2.y =  invI * rX * rX;
		
		//b2Mat22 K = K1 + K2;
		K.SetM(K1);
		K.AddM(K2);
		K.col1.x += m_gamma;
		K.col2.y += m_gamma;
		
		//m_ptpMass = K.Invert();
		K.Invert(m_mass);
		
		//m_C = b.m_position + r - m_target;
		m_C.x = b.m_sweep.c.x + rX - m_target.x;
		m_C.y = b.m_sweep.c.y + rY - m_target.y;
		
		// Cheat with some damping
		b.m_angularVelocity *= 0.98;
		
		// Warm starting.
		//b2Vec2 P = m_impulse;
		var PX:Number = step.dt * m_impulse.x;
		var PY:Number = step.dt * m_impulse.y;
		//b.m_linearVelocity += invMass * P;
		b.m_linearVelocity.x += invMass * PX;
		b.m_linearVelocity.y += invMass * PY;
		//b.m_angularVelocity += invI * b2Cross(r, P);
		b.m_angularVelocity += invI * (rX * PY - rY * PX);
	}
	
	
	public override function SolveVelocityConstraints(step:b2TimeStep) : void{
		var b:b2Body = m_body2;
		
		var tMat:b2Mat22;
		var tX:Number;
		var tY:Number;
		
		// Compute the effective mass matrix.
		//b2Vec2 r = b2Mul(b->m_xf.R, m_localAnchor - b->GetLocalCenter());
		tMat = b.m_xf.R;
		var rX:Number = m_localAnchor.x - b.m_sweep.localCenter.x;
		var rY:Number = m_localAnchor.y - b.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;
		
		// Cdot = v + cross(w, r)
		//b2Vec2 Cdot = b->m_linearVelocity + b2Cross(b->m_angularVelocity, r);
		var CdotX:Number = b.m_linearVelocity.x + (-b.m_angularVelocity * rY);
		var CdotY:Number = b.m_linearVelocity.y + (b.m_angularVelocity * rX);
		//b2Vec2 force = -step.inv_dt * b2Mul(m_mass, Cdot + (m_beta * step.inv_dt) * m_C + m_gamma * step.dt * m_force);
		tMat = m_mass;
		tX = CdotX + (m_beta * step.inv_dt) * m_C.x + m_gamma * step.dt * m_impulse.x;
		tY = CdotY + (m_beta * step.inv_dt) * m_C.y + m_gamma * step.dt * m_impulse.y;
		var forceX:Number = -step.inv_dt * (tMat.col1.x * tX + tMat.col2.x * tY);
		var forceY:Number = -step.inv_dt * (tMat.col1.y * tX + tMat.col2.y * tY);
		
		var oldForceX:Number = m_impulse.x;
		var oldForceY:Number = m_impulse.y;
		//m_force += force;
		m_impulse.x += forceX;
		m_impulse.y += forceY;
		var forceMagnitude:Number = m_impulse.Length();
		if (forceMagnitude > m_maxForce)
		{
			//m_impulse *= m_maxForce / forceMagnitude;
			m_impulse.Multiply(m_maxForce / forceMagnitude);
		}
		//force = m_impulse - oldForce;
		forceX = m_impulse.x - oldForceX;
		forceY = m_impulse.y - oldForceY;
		
		//b2Vec2 P = step.dt * force;
		var PX:Number = step.dt * forceX;
		var PY:Number = step.dt * forceY;
		//b->m_linearVelocity += b->m_invMass * P;
		b.m_linearVelocity.x += b.m_invMass * PX;
		b.m_linearVelocity.y += b.m_invMass * PY;
		//b->m_angularVelocity += b->m_invI * b2Cross(r, P);
		b.m_angularVelocity += b.m_invI * (rX * PY - rY * PX);
	}
	public override function SolvePositionConstraints():Boolean { 
		return true; 
	}

	public var m_localAnchor:b2Vec2 = new b2Vec2();
	public var m_target:b2Vec2 = new b2Vec2();
	public var m_impulse:b2Vec2 = new b2Vec2();

	public var m_mass:b2Mat22 = new b2Mat22();	// effective mass for point-to-point constraint.
	public var m_C:b2Vec2 = new b2Vec2();			// position error
	public var m_maxForce:Number;
	public var m_beta:Number;						// bias factor
	public var m_gamma:Number;						// softness
};

}
