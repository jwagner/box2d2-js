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

package Box2D.Dynamics.Contacts{

	
import Box2D.Dynamics.*;
import Box2D.Collision.*;
import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Dynamics.Contacts.*;


public class b2ContactSolver
{
	public function b2ContactSolver(step:b2TimeStep, contacts:Array, contactCount:int, allocator:*){
		var contact:b2Contact;
		
		//m_step = step;
		m_step.dt = step.dt;
		m_step.inv_dt = step.inv_dt;
		m_step.maxIterations = step.maxIterations;
		
		m_allocator = allocator;
		
		var i:int;
		var tVec:b2Vec2;
		var tMat:b2Mat22;
		
		m_constraintCount = 0;
		for (i = 0; i < contactCount; ++i)
		{
			// b2Assert(contacts[i].IsSolid());
			contact = contacts[i];
			m_constraintCount += contact.m_manifoldCount;
		}
		
		// fill array
		for (i = 0; i < m_constraintCount; i++){
			m_constraints[i] = new b2ContactConstraint();
		}
		
		var count:int = 0;
		for (i = 0; i < contactCount; ++i)
		{
			contact = contacts[i];
			var b1:b2Body = contact.m_shape1.m_body;
			var b2:b2Body = contact.m_shape2.m_body;
			var manifoldCount:int = contact.m_manifoldCount;
			var manifolds:Array = contact.GetManifolds();
			var friction:Number = contact.m_friction;
			var restitution:Number = contact.m_restitution;
			
			//var v1:b2Vec2 = b1.m_linearVelocity.Copy();
			var v1X:Number = b1.m_linearVelocity.x;
			var v1Y:Number = b1.m_linearVelocity.y;
			//var v2:b2Vec2 = b2.m_linearVelocity.Copy();
			var v2X:Number = b2.m_linearVelocity.x;
			var v2Y:Number = b2.m_linearVelocity.y;
			var w1:Number = b1.m_angularVelocity;
			var w2:Number = b2.m_angularVelocity;
			
			for (var j:int = 0; j < manifoldCount; ++j)
			{
				var manifold:b2Manifold = manifolds[ j ];
				
				//b2Settings.b2Assert(manifold.pointCount > 0);
				
				//var normal:b2Vec2 = manifold.normal.Copy();
				var normalX:Number = manifold.normal.x;
				var normalY:Number = manifold.normal.y;
				
				//b2Settings.b2Assert(count < m_constraintCount);
				var c:b2ContactConstraint = m_constraints[ count ];
				c.body1 = b1; //p
				c.body2 = b2; //p
				c.manifold = manifold; //p
				//c.normal = normal;
				c.normal.x = normalX;
				c.normal.y = normalY;
				c.pointCount = manifold.pointCount;
				c.friction = friction;
				c.restitution = restitution;
				
				for (var k:uint = 0; k < c.pointCount; ++k)
				{
					var cp:b2ManifoldPoint = manifold.points[ k ];
					var ccp:b2ContactConstraintPoint = c.points[ k ];
					
					ccp.normalImpulse = cp.normalImpulse;
					ccp.tangentImpulse = cp.tangentImpulse;
					ccp.separation = cp.separation;
					ccp.positionImpulse = 0.0;
					
					ccp.localAnchor1.SetV(cp.localPoint1);
					ccp.localAnchor2.SetV(cp.localPoint2);
					
					var tX:Number;
					var tY:Number;
					
					//ccp->r1 = b2Mul(b1->GetXForm().R, cp->localPoint1 - b1->GetLocalCenter());
					tMat = b1.m_xf.R;
					var r1X:Number = cp.localPoint1.x - b1.m_sweep.localCenter.x;
					var r1Y:Number = cp.localPoint1.y - b1.m_sweep.localCenter.y;
					tX  = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
					r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
					r1X = tX;
					ccp.r1.Set(r1X,r1Y);
					//ccp->r2 = b2Mul(b2->GetXForm().R, cp->localPoint2 - b2->GetLocalCenter());
					tMat = b2.m_xf.R;
					var r2X:Number = cp.localPoint2.x - b2.m_sweep.localCenter.x;
					var r2Y:Number = cp.localPoint2.y - b2.m_sweep.localCenter.y;
					tX  = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
					r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
					r2X = tX;
					ccp.r2.Set(r2X,r2Y);
					
					var r1Sqr:Number = r1X * r1X + r1Y * r1Y;//b2Math.b2Dot(r1, r1);
					var r2Sqr:Number = r2X * r2X + r2Y * r2Y;//b2Math.b2Dot(r2, r2);
					
					//var rn1:Number = b2Math.b2Dot(r1, normal);
					var rn1:Number = r1X*normalX + r1Y*normalY;
					//var rn2:Number = b2Math.b2Dot(r2, normal);
					var rn2:Number = r2X*normalX + r2Y*normalY;
					var kNormal:Number = b1.m_invMass + b2.m_invMass;
					kNormal += b1.m_invI * (r1Sqr - rn1 * rn1) + b2.m_invI * (r2Sqr - rn2 * rn2);
					//b2Settings.b2Assert(kNormal > Number.MIN_VALUE);
					ccp.normalMass = 1.0 / kNormal;
					
					var kEqualized:Number = b1.m_mass * b1.m_invMass + b2.m_mass * b2.m_invMass;
					kEqualized += b1.m_mass * b1.m_invI * (r1Sqr - rn1 * rn1) + b2.m_mass * b2.m_invI * (r2Sqr - rn2 * rn2);
					//b2Assert(kEqualized > Number.MIN_VALUE);
					ccp.equalizedMass = 1.0 / kEqualized;
					
					//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
					var tangentX:Number = normalY
					var tangentY:Number = -normalX;
					
					//var rt1:Number = b2Math.b2Dot(r1, tangent);
					var rt1:Number = r1X*tangentX + r1Y*tangentY;
					//var rt2:Number = b2Math.b2Dot(r2, tangent);
					var rt2:Number = r2X*tangentX + r2Y*tangentY;
					var kTangent:Number = b1.m_invMass + b2.m_invMass;
					kTangent += b1.m_invI * (r1Sqr - rt1 * rt1) + b2.m_invI * (r2Sqr - rt2 * rt2);
					//b2Settings.b2Assert(kTangent > Number.MIN_VALUE);
					ccp.tangentMass = 1.0 /  kTangent;
					
					// Setup a velocity bias for restitution.
					ccp.velocityBias = 0.0;
					if (ccp.separation > 0.0)
					{
						ccp.velocityBias = -60.0 * ccp.separation; // TODO_ERIN b2TimeStep
					}
					//b2Dot(c.normal, v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1));
					tX = v2X + (-w2*r2Y) - v1X - (-w1*r1Y);
					tY = v2Y + (w2*r2X) - v1Y - (w1*r1X);
					//var vRel:Number = b2Dot(c.normal, t);
					var vRel:Number = c.normal.x*tX + c.normal.y*tY;
					if (vRel < -b2Settings.b2_velocityThreshold)
					{
						ccp.velocityBias += -c.restitution * vRel;
					}
				}
				
				++count;
			}
		}
		
		//b2Settings.b2Assert(count == m_constraintCount);
	}
	//~b2ContactSolver();

	public function InitVelocityConstraints(step: b2TimeStep) : void{
		var tVec:b2Vec2;
		var tVec2:b2Vec2;
		var tMat:b2Mat22;
		
		// Warm start.
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			
			var b1:b2Body = c.body1;
			var b2:b2Body = c.body2;
			var invMass1:Number = b1.m_invMass;
			var invI1:Number = b1.m_invI;
			var invMass2:Number = b2.m_invMass;
			var invI2:Number = b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			var normalX:Number = c.normal.x;
			var normalY:Number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			var tangentX:Number = normalY;
			var tangentY:Number = -normalX;
			
			var tX:Number;
			
			var j:int;
			var tCount:int;
			if (step.warmStarting)
			{
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j)
				{
					var ccp:b2ContactConstraintPoint = c.points[ j ];
					ccp.normalImpulse *= step.dtRatio;
					ccp.tangentImpulse *= step.dtRatio;
					//b2Vec2 P = ccp->normalImpulse * normal + ccp->tangentImpulse * tangent;
					var PX:Number = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
					var PY:Number = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
					
					//b1.m_angularVelocity -= invI1 * b2Math.b2CrossVV(r1, P);
					b1.m_angularVelocity -= invI1 * (ccp.r1.x * PY - ccp.r1.y * PX);
					//b1.m_linearVelocity.Subtract( b2Math.MulFV(invMass1, P) );
					b1.m_linearVelocity.x -= invMass1 * PX;
					b1.m_linearVelocity.y -= invMass1 * PY;
					//b2.m_angularVelocity += invI2 * b2Math.b2CrossVV(r2, P);
					b2.m_angularVelocity += invI2 * (ccp.r2.x * PY - ccp.r2.y * PX);
					//b2.m_linearVelocity.Add( b2Math.MulFV(invMass2, P) );
					b2.m_linearVelocity.x += invMass2 * PX;
					b2.m_linearVelocity.y += invMass2 * PY;
				}
			}
			else{
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j)
				{
					var ccp2:b2ContactConstraintPoint = c.points[ j ];
					ccp2.normalImpulse = 0.0;
					ccp2.tangentImpulse = 0.0;
				}
			}
		}
	}
	public function SolveVelocityConstraints() : void{
		var j:int;
		var ccp:b2ContactConstraintPoint;
		var r1X:Number;
		var r1Y:Number;
		var r2X:Number;
		var r2Y:Number;
		var dvX:Number;
		var dvY:Number;
		var vn:Number;
		var vt:Number;
		var lambda_n:Number;
		var lambda_t:Number;
		var newImpulse_n:Number;
		var newImpulse_t:Number;
		var PX:Number;
		var PY:Number;
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			var b1:b2Body = c.body1;
			var b2:b2Body = c.body2;
			var w1:Number = b1.m_angularVelocity;
			var w2:Number = b2.m_angularVelocity;
			var v1:b2Vec2 = b1.m_linearVelocity;
			var v2:b2Vec2 = b2.m_linearVelocity;
			
			var invMass1:Number = b1.m_invMass;
			var invI1:Number = b1.m_invI;
			var invMass2:Number = b2.m_invMass;
			var invI2:Number = b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			var normalX:Number = c.normal.x;
			var normalY:Number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			var tangentX:Number = normalY;
			var tangentY:Number = -normalX;
			var friction:Number = c.friction;
			
			var tX:Number;
			
			var tCount:int = c.pointCount;
			for (j = 0; j < tCount; ++j)
			{
				ccp = c.points[ j ];
				
				// Relative velocity at contact
				//b2Vec2 dv = v2 + b2Cross(w2, ccp->r2) - v1 - b2Cross(w1, ccp->r1);
				dvX = v2.x + (-w2 * ccp.r2.y) - v1.x - (-w1 * ccp.r1.y);
				dvY = v2.y + (w2 * ccp.r2.x) - v1.y - (w1 * ccp.r1.x);
				
				// Compute normal impulse
				//var vn:Number = b2Math.b2Dot(dv, normal);
				vn = dvX * normalX + dvY * normalY;
				lambda_n = -ccp.normalMass * (vn - ccp.velocityBias);
				
				// Compute tangent impulse - normal
				vt = dvX*tangentX + dvY*tangentY;//b2Math.b2Dot(dv, tangent);
				lambda_t = ccp.tangentMass * (-vt);
				
				// b2Clamp the accumulated impulse - tangent
				newImpulse_n = b2Math.b2Max(ccp.normalImpulse + lambda_n, 0.0);
				lambda_n = newImpulse_n - ccp.normalImpulse;
				
				// b2Clamp the accumulated force
				var maxFriction:Number = friction * ccp.normalImpulse;
				newImpulse_t = b2Math.b2Clamp(ccp.tangentImpulse + lambda_t, -maxFriction, maxFriction);
				lambda_t = newImpulse_t - ccp.tangentImpulse;
				
				// Apply contact impulse
				//b2Vec2 P = lambda * normal;
				PX = lambda_n * normalX + lambda_t * tangentX;
				PY = lambda_n * normalY + lambda_t * tangentY;
				
				//v1.Subtract( b2Math.MulFV( invMass1, P ) );
				v1.x -= invMass1 * PX;
				v1.y -= invMass1 * PY;
				w1 -= invI1 * (ccp.r1.x * PY - ccp.r1.y * PX);//invI1 * b2Math.b2CrossVV(ccp.r1, P);
				
				//v2.Add( b2Math.MulFV( invMass2, P ) );
				v2.x += invMass2 * PX;
				v2.y += invMass2 * PY;
				w2 += invI2 * (ccp.r2.x * PY - ccp.r2.y * PX);//invI2 * b2Math.b2CrossVV(ccp.r2, P);
				
				ccp.normalImpulse = newImpulse_n;
				ccp.tangentImpulse = newImpulse_t;
			}
			
			// b2Vec2s in AS3 are copied by reference. The originals are 
			// references to the same things here and there is no need to 
			// copy them back, unlike in C++ land where b2Vec2s are 
			// copied by value.
			/*b1->m_linearVelocity = v1;
			b2->m_linearVelocity = v2;*/
			b1.m_angularVelocity = w1;
			b2.m_angularVelocity = w2;
		}
	}
	
	public function FinalizeVelocityConstraints() : void
	{
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			var m:b2Manifold = c.manifold;
			
			for (var j:int = 0; j < c.pointCount; ++j)
			{
				var point1:b2ManifoldPoint = m.points[j];
				var point2:b2ContactConstraintPoint = c.points[j];
				point1.normalImpulse = point2.normalImpulse;
				point1.tangentImpulse = point2.tangentImpulse;
			}
		}
	}
	
	
	public function SolvePositionConstraints(baumgarte:Number):Boolean{
		var minSeparation:Number = 0.0;
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			var b1:b2Body = c.body1;
			var b2:b2Body = c.body2;
			var b1_sweep_c:b2Vec2 = b1.m_sweep.c;
			var b1_sweep_a:Number = b1.m_sweep.a;
			var b2_sweep_c:b2Vec2 = b2.m_sweep.c;
			var b2_sweep_a:Number = b2.m_sweep.a;
			
			var invMass1:Number = b1.m_mass * b1.m_invMass;
			var invI1:Number = b1.m_mass * b1.m_invI;
			var invMass2:Number = b2.m_mass * b2.m_invMass;
			var invI2:Number = b2.m_mass * b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			var normalX:Number = c.normal.x;
			var normalY:Number = c.normal.y;
			
			// Solver normal constraints
			var tCount:int = c.pointCount;
			for (var j:int = 0; j < tCount; ++j)
			{
				var ccp:b2ContactConstraintPoint = c.points[ j ];
				
				//r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1 - b1->GetLocalCenter());
				tMat = b1.m_xf.R;
				tVec = b1.m_sweep.localCenter;
				var r1X:Number = ccp.localAnchor1.x - tVec.x;
				var r1Y:Number = ccp.localAnchor1.y - tVec.y;
				tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
				r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
				r1X = tX;
				
				//r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2 - b2->GetLocalCenter());
				tMat = b2.m_xf.R;
				tVec = b2.m_sweep.localCenter;
				var r2X:Number = ccp.localAnchor2.x - tVec.x;
				var r2Y:Number = ccp.localAnchor2.y - tVec.y;
				var tX:Number =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
				r2Y = 			 (tMat.col1.y * r2X + tMat.col2.y * r2Y);
				r2X = tX;
				
				//b2Vec2 p1 = b1->m_sweep.c + r1;
				var p1X:Number = b1_sweep_c.x + r1X;
				var p1Y:Number = b1_sweep_c.y + r1Y;
				
				//b2Vec2 p2 = b2->m_sweep.c + r2;
				var p2X:Number = b2_sweep_c.x + r2X;
				var p2Y:Number = b2_sweep_c.y + r2Y;
				
				//var dp:b2Vec2 = b2Math.SubtractVV(p2, p1);
				var dpX:Number = p2X - p1X;
				var dpY:Number = p2Y - p1Y;
				
				// Approximate the current separation.
				//var separation:Number = b2Math.b2Dot(dp, normal) + ccp.separation;
				var separation:Number = (dpX*normalX + dpY*normalY) + ccp.separation;
				
				// Track max constraint error.
				minSeparation = b2Math.b2Min(minSeparation, separation);
				
				// Prevent large corrections and allow slop.
				var C:Number = baumgarte * b2Math.b2Clamp(separation + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
				
				// Compute normal impulse
				var dImpulse:Number = -ccp.equalizedMass * C;
				
				// b2Clamp the accumulated impulse
				var impulse0:Number = ccp.positionImpulse;
				ccp.positionImpulse = b2Math.b2Max(impulse0 + dImpulse, 0.0);
				dImpulse = ccp.positionImpulse - impulse0;
				
				//var impulse:b2Vec2 = b2Math.MulFV( dImpulse, normal );
				var impulseX:Number = dImpulse * normalX;
				var impulseY:Number = dImpulse * normalY;
				
				//b1.m_position.Subtract( b2Math.MulFV( invMass1, impulse ) );
				b1_sweep_c.x -= invMass1 * impulseX;
				b1_sweep_c.y -= invMass1 * impulseY;
				b1_sweep_a -= invI1 * (r1X * impulseY - r1Y * impulseX);//b2Math.b2CrossVV(r1, impulse);
				b1.m_sweep.a = b1_sweep_a;
				b1.SynchronizeTransform();
				
				//b2.m_position.Add( b2Math.MulFV( invMass2, impulse ) );
				b2_sweep_c.x += invMass2 * impulseX;
				b2_sweep_c.y += invMass2 * impulseY;
				b2_sweep_a += invI2 * (r2X * impulseY - r2Y * impulseX);//b2Math.b2CrossVV(r2, impulse);
				b2.m_sweep.a = b2_sweep_a;
				b2.SynchronizeTransform();
			}
			// Update body rotations
			//b1.m_sweep.a = b1_sweep_a;
			//b2.m_sweep.a = b2_sweep_a;
		}
		
		// We can't expect minSpeparation >= -b2_linearSlop because we don't
		// push the separation above -b2_linearSlop.
		return minSeparation >= -1.5 * b2Settings.b2_linearSlop;
	}

	public var m_step:b2TimeStep = new b2TimeStep();
	public var m_allocator:*;
	public var m_constraints:Array = new Array();
	public var m_constraintCount:int;
};

}