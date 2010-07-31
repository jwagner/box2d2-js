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




/// A gear joint is used to connect two joints together. Either joint
/// can be a revolute or prismatic joint. You specify a gear ratio
/// to bind the motions together:
/// coordinate1 + ratio * coordinate2 = constant
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
/// @warning The revolute and prismatic joints must be attached to
/// fixed bodies (which must be body1 on those joints).

public class b2GearJoint extends b2Joint
{
	public override function GetAnchor1():b2Vec2{
		//return m_body1->GetWorldPoint(m_localAnchor1);
		return m_body1.GetWorldPoint(m_localAnchor1);
	}
	public override function GetAnchor2():b2Vec2{
		//return m_body2->GetWorldPoint(m_localAnchor2);
		return m_body2.GetWorldPoint(m_localAnchor2);
	}

	public override function GetReactionForce():b2Vec2{
		// TODO_ERIN not tested
		var F:b2Vec2 = new b2Vec2(m_force * m_J.linear2.x, m_force * m_J.linear2.y);
		return F;
	}
	public override function GetReactionTorque():Number{
		// TODO_ERIN not tested
		//b2Vec2 r = b2Mul(m_body2->m_xf.R, m_localAnchor2 - m_body2->GetLocalCenter());
		var tMat:b2Mat22 = m_body2.m_xf.R;
		var rX:Number = m_localAnchor1.x - m_body2.m_sweep.localCenter.x;
		var rY:Number = m_localAnchor1.y - m_body2.m_sweep.localCenter.y;
		var tX:Number = tMat.col1.x * rX + tMat.col2.x * rY;
		rY = tMat.col1.y * rX + tMat.col2.y * rY;
		rX = tX;
		//b2Vec2 F = m_force * m_J.linear2;
		//float32 T = m_force * m_J.angular2 - b2Cross(r, F);
		tX = m_force * m_J.angular2 - (rX * (m_force * m_J.linear2.y) - rY * (m_force * m_J.linear2.x));
		return tX;
	}

	public function GetRatio():Number{
		return m_ratio;
	}

	//--------------- Internals Below -------------------

	public function b2GearJoint(def:b2GearJointDef){
		// parent constructor
		super(def);
		
		var type1:int = def.joint1.m_type;
		var type2:int = def.joint2.m_type;
		
		//b2Settings.b2Assert(type1 == b2Joint.e_revoluteJoint || type1 == b2Joint.e_prismaticJoint);
		//b2Settings.b2Assert(type2 == b2Joint.e_revoluteJoint || type2 == b2Joint.e_prismaticJoint);
		//b2Settings.b2Assert(def.joint1.m_body1.IsStatic());
		//b2Settings.b2Assert(def.joint2.m_body1.IsStatic());
		
		m_revolute1 = null;
		m_prismatic1 = null;
		m_revolute2 = null;
		m_prismatic2 = null;
		
		var coordinate1:Number;
		var coordinate2:Number;
		
		m_ground1 = def.joint1.m_body1;
		m_body1 = def.joint1.m_body2;
		if (type1 == b2Joint.e_revoluteJoint)
		{
			m_revolute1 = def.joint1 as b2RevoluteJoint;
			m_groundAnchor1.SetV( m_revolute1.m_localAnchor1 );
			m_localAnchor1.SetV( m_revolute1.m_localAnchor2 );
			coordinate1 = m_revolute1.GetJointAngle();
		}
		else
		{
			m_prismatic1 = def.joint1 as b2PrismaticJoint;
			m_groundAnchor1.SetV( m_prismatic1.m_localAnchor1 );
			m_localAnchor1.SetV( m_prismatic1.m_localAnchor2 );
			coordinate1 = m_prismatic1.GetJointTranslation();
		}
		
		m_ground2 = def.joint2.m_body1;
		m_body2 = def.joint2.m_body2;
		if (type2 == b2Joint.e_revoluteJoint)
		{
			m_revolute2 = def.joint2 as b2RevoluteJoint;
			m_groundAnchor2.SetV( m_revolute2.m_localAnchor1 );
			m_localAnchor2.SetV( m_revolute2.m_localAnchor2 );
			coordinate2 = m_revolute2.GetJointAngle();
		}
		else
		{
			m_prismatic2 = def.joint2 as b2PrismaticJoint;
			m_groundAnchor2.SetV( m_prismatic2.m_localAnchor1 );
			m_localAnchor2.SetV( m_prismatic2.m_localAnchor2 );
			coordinate2 = m_prismatic2.GetJointTranslation();
		}
		
		m_ratio = def.ratio;
		
		m_constant = coordinate1 + m_ratio * coordinate2;
		
		m_force = 0.0;
		
	}

	public override function InitVelocityConstraints(step:b2TimeStep) : void{
		var g1:b2Body = m_ground1;
		var g2:b2Body = m_ground2;
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		// temp vars
		var ugX:Number;
		var ugY:Number;
		var rX:Number;
		var rY:Number;
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		var crug:Number;
		var tX:Number;
		
		var K:Number = 0.0;
		m_J.SetZero();
		
		if (m_revolute1)
		{
			m_J.angular1 = -1.0;
			K += b1.m_invI;
		}
		else
		{
			//b2Vec2 ug = b2MulMV(g1->m_xf.R, m_prismatic1->m_localXAxis1);
			tMat = g1.m_xf.R;
			tVec = m_prismatic1.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//b2Vec2 r = b2Mul(b1->m_xf.R, m_localAnchor1 - b1->GetLocalCenter());
			tMat = b1.m_xf.R;
			rX = m_localAnchor1.x - b1.m_sweep.localCenter.x;
			rY = m_localAnchor1.y - b1.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;
			
			//var crug:Number = b2Cross(r, ug);
			crug = rX * ugY - rY * ugX;
			//m_J.linear1 = -ug;
			m_J.linear1.Set(-ugX, -ugY);
			m_J.angular1 = -crug;
			K += b1.m_invMass + b1.m_invI * crug * crug;
		}
		
		if (m_revolute2)
		{
			m_J.angular2 = -m_ratio;
			K += m_ratio * m_ratio * b2.m_invI;
		}
		else
		{
			//b2Vec2 ug = b2Mul(g2->m_xf.R, m_prismatic2->m_localXAxis1);
			tMat = g2.m_xf.R;
			tVec = m_prismatic2.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//b2Vec2 r = b2Mul(b2->m_xf.R, m_localAnchor2 - b2->GetLocalCenter());
			tMat = b2.m_xf.R;
			rX = m_localAnchor2.x - b2.m_sweep.localCenter.x;
			rY = m_localAnchor2.y - b2.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;
			
			//float32 crug = b2Cross(r, ug);
			crug = rX * ugY - rY * ugX;
			//m_J.linear2 = -m_ratio * ug;
			m_J.linear2.Set(-m_ratio*ugX, -m_ratio*ugY);
			m_J.angular2 = -m_ratio * crug;
			K += m_ratio * m_ratio * (b2.m_invMass + b2.m_invI * crug * crug);
		}
		
		// Compute effective mass.
		//b2Settings.b2Assert(K > 0.0);
		m_mass = 1.0 / K;
		
		if (step.warmStarting)
		{
			// Warm starting.
			var P:Number = step.dt * m_force;
			//b1.m_linearVelocity += b1.m_invMass * P * m_J.linear1;
			b1.m_linearVelocity.x += b1.m_invMass * P * m_J.linear1.x;
			b1.m_linearVelocity.y += b1.m_invMass * P * m_J.linear1.y;
			b1.m_angularVelocity += b1.m_invI * P * m_J.angular1;
			//b2.m_linearVelocity += b2.m_invMass * P * m_J.linear2;
			b2.m_linearVelocity.x += b2.m_invMass * P * m_J.linear2.x;
			b2.m_linearVelocity.y += b2.m_invMass * P * m_J.linear2.y;
			b2.m_angularVelocity += b2.m_invI * P * m_J.angular2;
		}
		else
		{
			m_force = 0.0;
		}
	}
	
	
	public override function SolveVelocityConstraints(step:b2TimeStep): void{
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var Cdot:Number = m_J.Compute(	b1.m_linearVelocity, b1.m_angularVelocity,
										b2.m_linearVelocity, b2.m_angularVelocity);
		
		var force:Number = -step.inv_dt * m_mass * Cdot;
		m_force += force;
		
		var P:Number = step.dt * force;
		b1.m_linearVelocity.x += b1.m_invMass * P * m_J.linear1.x;
		b1.m_linearVelocity.y += b1.m_invMass * P * m_J.linear1.y;
		b1.m_angularVelocity  += b1.m_invI * P * m_J.angular1;
		b2.m_linearVelocity.x += b2.m_invMass * P * m_J.linear2.x;
		b2.m_linearVelocity.y += b2.m_invMass * P * m_J.linear2.y;
		b2.m_angularVelocity  += b2.m_invI * P * m_J.angular2;
	}
	
	public override function SolvePositionConstraints():Boolean{
		var linearError:Number = 0.0;
		
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var coordinate1:Number;
		var coordinate2:Number;
		if (m_revolute1)
		{
			coordinate1 = m_revolute1.GetJointAngle();
		}
		else
		{
			coordinate1 = m_prismatic1.GetJointTranslation();
		}
		
		if (m_revolute2)
		{
			coordinate2 = m_revolute2.GetJointAngle();
		}
		else
		{
			coordinate2 = m_prismatic2.GetJointTranslation();
		}
		
		var C:Number = m_constant - (coordinate1 + m_ratio * coordinate2);
		
		var impulse:Number = -m_mass * C;
		
		b1.m_sweep.c.x += b1.m_invMass * impulse * m_J.linear1.x;
		b1.m_sweep.c.y += b1.m_invMass * impulse * m_J.linear1.y;
		b1.m_sweep.a += b1.m_invI * impulse * m_J.angular1;
		b2.m_sweep.c.x += b2.m_invMass * impulse * m_J.linear2.x;
		b2.m_sweep.c.y += b2.m_invMass * impulse * m_J.linear2.y;
		b2.m_sweep.a += b2.m_invI * impulse * m_J.angular2;
		
		b1.SynchronizeTransform();
		b2.SynchronizeTransform();
		
		return linearError < b2Settings.b2_linearSlop;
	}

	public var m_ground1:b2Body;
	public var m_ground2:b2Body;

	// One of these is NULL.
	public var m_revolute1:b2RevoluteJoint;
	public var m_prismatic1:b2PrismaticJoint;

	// One of these is NULL.
	public var m_revolute2:b2RevoluteJoint;
	public var m_prismatic2:b2PrismaticJoint;

	public var m_groundAnchor1:b2Vec2 = new b2Vec2();
	public var m_groundAnchor2:b2Vec2 = new b2Vec2();

	public var m_localAnchor1:b2Vec2 = new b2Vec2();
	public var m_localAnchor2:b2Vec2 = new b2Vec2();

	public var m_J:b2Jacobian = new b2Jacobian();

	public var m_constant:Number;
	public var m_ratio:Number;

	// Effective mass
	public var m_mass:Number;

	// Impulse for accumulation/warm starting.
	public var m_force:Number;
};


}