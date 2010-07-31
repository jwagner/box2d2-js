var b2Island = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Island.prototype.__constructor = function (
	bodyCapacity,
	contactCapacity,
	jointCapacity,
	allocator,
	listener) {
		var i = 0;
		
		this.m_bodyCapacity = bodyCapacity;
		this.m_contactCapacity = contactCapacity;
		this.m_jointCapacity	 = jointCapacity;
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
		
		this.m_allocator = allocator;
		this.m_listener = listener;
		
		
		this.m_bodies = new Array(bodyCapacity);
		for (i = 0; i < bodyCapacity; i++)
			this.m_bodies[i] = null;
		
		
		this.m_contacts = new Array(contactCapacity);
		for (i = 0; i < contactCapacity; i++)
			this.m_contacts[i] = null;
		
		
		this.m_joints = new Array(jointCapacity);
		for (i = 0; i < jointCapacity; i++)
			this.m_joints[i] = null;
		
		this.m_positionIterationCount = 0;
		
	}
b2Island.prototype.__varz = function(){
}
// static attributes
b2Island.s_reportCR =  new b2ContactResult();
// static methods
// attributes
b2Island.prototype.m_allocator =  null;
b2Island.prototype.m_listener =  null;
b2Island.prototype.m_bodies =  null;
b2Island.prototype.m_contacts =  null;
b2Island.prototype.m_joints =  null;
b2Island.prototype.m_bodyCount =  0;
b2Island.prototype.m_jointCount =  0;
b2Island.prototype.m_contactCount =  0;
b2Island.prototype.m_bodyCapacity =  0;
b2Island.prototype.m_contactCapacity =  0;
b2Island.prototype.m_jointCapacity =  0;
b2Island.prototype.m_positionIterationCount =  0;
// methods
b2Island.prototype.Clear = function () {
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
	}
b2Island.prototype.Solve = function (step, gravity, correctPositions, allowSleep) {
		var i = 0;
		var b;
		var joint;
		
		
		for (i = 0; i < this.m_bodyCount; ++i)
		{
			b = this.m_bodies[i];
			
			if (b.IsStatic())
				continue;
			
			
			
			b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
			b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
			b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
			
			
			b.m_force.SetZero();
			b.m_torque = 0.0;
			
			
			
			
			
			
			
			
			b.m_linearVelocity.Multiply( b2Math.b2Clamp(1.0 - step.dt * b.m_linearDamping, 0.0, 1.0) );
			b.m_angularVelocity *= b2Math.b2Clamp(1.0 - step.dt * b.m_angularDamping, 0.0, 1.0);
			
			
			
			if ((b.m_linearVelocity.LengthSquared()) > b2Settings.b2_maxLinearVelocitySquared)
			{
				b.m_linearVelocity.Normalize();
				b.m_linearVelocity.x *= b2Settings.b2_maxLinearVelocity;
				b.m_linearVelocity.y *= b2Settings.b2_maxLinearVelocity;
			}
			
			if (b.m_angularVelocity * b.m_angularVelocity > b2Settings.b2_maxAngularVelocitySquared)
			{
				if (b.m_angularVelocity < 0.0)
				{
					b.m_angularVelocity = -b2Settings.b2_maxAngularVelocity;
				}
				else
				{
					b.m_angularVelocity = b2Settings.b2_maxAngularVelocity;
				}
			}
		}
		
		var contactSolver = new b2ContactSolver(step, this.m_contacts, this.m_contactCount, this.m_allocator);
		
		
		contactSolver.InitVelocityConstraints(step);
		
		for (i = 0; i < this.m_jointCount; ++i)
		{
			joint = this.m_joints[i];
			joint.InitVelocityConstraints(step);
		}
		
		
		for (i = 0; i < step.maxIterations; ++i)
		{
			contactSolver.SolveVelocityConstraints();
			
			for (var j = 0; j < this.m_jointCount; ++j)
			{
				joint = this.m_joints[j];
				joint.SolveVelocityConstraints(step);
			}
		}
		
		
		contactSolver.FinalizeVelocityConstraints();
		
		
		for (i = 0; i < this.m_bodyCount; ++i)
		{
			b = this.m_bodies[i];
			
			if (b.IsStatic())
				continue;
			
			
			b.m_sweep.c0.SetV(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;
			
			
			
			b.m_sweep.c.x += step.dt * b.m_linearVelocity.x;
			b.m_sweep.c.y += step.dt * b.m_linearVelocity.y;
			b.m_sweep.a += step.dt * b.m_angularVelocity;
			
			
			b.SynchronizeTransform();
			
			
		}
		
		if (correctPositions)
		{
			
			
			for (i = 0; i < this.m_jointCount; ++i)
			{
				joint = this.m_joints[i];
				joint.InitPositionConstraints();
			}
			
			
			for (this.m_positionIterationCount = 0; this.m_positionIterationCount < step.maxIterations; ++this.m_positionIterationCount)
			{
				var contactsOkay = contactSolver.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
				
				var jointsOkay = true;
				for (i = 0; i < this.m_jointCount; ++i)
				{
					joint = this.m_joints[i];
					var jointOkay = joint.SolvePositionConstraints();
					jointsOkay = jointsOkay && jointOkay;
				}
				
				if (contactsOkay && jointsOkay)
				{
					break;
				}
			}
		}
		
		this.Report(contactSolver.m_constraints);
		
		if (allowSleep){
			
			var minSleepTime = Number.MAX_VALUE;
			
			var linTolSqr = b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance;
			var angTolSqr = b2Settings.b2_angularSleepTolerance * b2Settings.b2_angularSleepTolerance;
			
			for (i = 0; i < this.m_bodyCount; ++i)
			{
				b = this.m_bodies[i];
				if (b.m_invMass == 0.0)
				{
					continue;
				}
				
				if ((b.m_flags & b2Body.e_allowSleepFlag) == 0)
				{
					b.m_sleepTime = 0.0;
					minSleepTime = 0.0;
				}
				
				if ((b.m_flags & b2Body.e_allowSleepFlag) == 0 ||
					b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
					b2Math.b2Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr)
				{
					b.m_sleepTime = 0.0;
					minSleepTime = 0.0;
				}
				else
				{
					b.m_sleepTime += step.dt;
					minSleepTime = b2Math.b2Min(minSleepTime, b.m_sleepTime);
				}
			}
			
			if (minSleepTime >= b2Settings.b2_timeToSleep)
			{
				for (i = 0; i < this.m_bodyCount; ++i)
				{
					b = this.m_bodies[i];
					b.m_flags |= b2Body.e_sleepFlag;
					b.m_linearVelocity.SetZero();
					b.m_angularVelocity = 0.0;
				}
			}
		}
	}
b2Island.prototype.SolveTOI = function (subStep) {
		var i = 0;
		var contactSolver = new b2ContactSolver(subStep, this.m_contacts, this.m_contactCount, this.m_allocator);
		
		
		
		
		for (i = 0; i < subStep.maxIterations; ++i)
		{
			contactSolver.SolveVelocityConstraints();
		}
		
		
		
		
		
		for (i = 0; i < this.m_bodyCount; ++i)
		{
			var b = this.m_bodies[i];
			
			if (b.IsStatic())
				continue;
			
			
			b.m_sweep.c0.SetV(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;
			
			
			b.m_sweep.c.x += subStep.dt * b.m_linearVelocity.x;
			b.m_sweep.c.y += subStep.dt * b.m_linearVelocity.y;
			b.m_sweep.a += subStep.dt * b.m_angularVelocity;
			
			
			b.SynchronizeTransform();
			
			
		}
		
		
		var k_toiBaumgarte = 0.75;
		for (i = 0; i < subStep.maxIterations; ++i)
		{
			var contactsOkay = contactSolver.SolvePositionConstraints(k_toiBaumgarte);
			if (contactsOkay)
			{
				break;
			}
		}
		
		this.Report(contactSolver.m_constraints);
	}
b2Island.prototype.Report = function (constraints) {
		var tMat;
		var tVec;
		if (this.m_listener == null)
		{
			return;
		}
		
		for (var i = 0; i < this.m_contactCount; ++i)
		{
			var c = this.m_contacts[i];
			var cc = constraints[ i ];
			var cr = b2Island.s_reportCR;
			cr.shape1 = c.m_shape1;
			cr.shape2 = c.m_shape2;
			var b1 = cr.shape1.m_body;
			var manifoldCount = c.m_manifoldCount;
			var manifolds = c.GetManifolds();
			for (var j = 0; j < manifoldCount; ++j)
			{
				var manifold = manifolds[ j ];
				cr.normal.SetV( manifold.normal );
				for (var k = 0; k < manifold.pointCount; ++k)
				{
					var point = manifold.points[ k ];
					var ccp = cc.points[ k ];
					cr.position = b1.GetWorldPoint(point.localPoint1);
					
					
					
					cr.normalImpulse = ccp.normalImpulse;
					cr.tangentImpulse = ccp.tangentImpulse;
					cr.id.key = point.id.key;
					
					this.m_listener.Result(cr);
				}
			}
		}
	}
b2Island.prototype.AddBody = function (body) {
		
		this.m_bodies[this.m_bodyCount++] = body;
	}
b2Island.prototype.AddContact = function (contact) {
		
		this.m_contacts[this.m_contactCount++] = contact;
	}
b2Island.prototype.AddJoint = function (joint) {
		
		this.m_joints[this.m_jointCount++] = joint;
	}