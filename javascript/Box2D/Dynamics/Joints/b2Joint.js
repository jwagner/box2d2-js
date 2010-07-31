var b2Joint = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Joint.prototype.__constructor = function (def) {
		this.m_type = def.type;
		this.m_prev = null;
		this.m_next = null;
		this.m_body1 = def.body1;
		this.m_body2 = def.body2;
		this.m_collideConnected = def.collideConnected;
		this.m_islandFlag = false;
		this.m_userData = def.userData;
	}
b2Joint.prototype.__varz = function(){
this.m_node1 =  new b2JointEdge();
this.m_node2 =  new b2JointEdge();
}
// static attributes
b2Joint.e_unknownJoint =  0;
b2Joint.e_revoluteJoint =  1;
b2Joint.e_prismaticJoint =  2;
b2Joint.e_distanceJoint =  3;
b2Joint.e_pulleyJoint =  4;
b2Joint.e_mouseJoint =  5;
b2Joint.e_gearJoint =  6;
b2Joint.e_inactiveLimit =  0;
b2Joint.e_atLowerLimit =  1;
b2Joint.e_atUpperLimit =  2;
b2Joint.e_equalLimits =  3;
// static methods
b2Joint.Create = function (def, allocator) {
		var joint = null;
		
		switch (def.type)
		{
		case b2Joint.e_distanceJoint:
			{
				
				joint = new b2DistanceJoint(def);
			}
			break;
		
		case b2Joint.e_mouseJoint:
			{
				
				joint = new b2MouseJoint(def);
			}
			break;
		
		case b2Joint.e_prismaticJoint:
			{
				
				joint = new b2PrismaticJoint(def);
			}
			break;
		
		case b2Joint.e_revoluteJoint:
			{
				
				joint = new b2RevoluteJoint(def);
			}
			break;
		
		case b2Joint.e_pulleyJoint:
			{
				
				joint = new b2PulleyJoint(def);
			}
			break;
		
		case b2Joint.e_gearJoint:
			{
				
				joint = new b2GearJoint(def);
			}
			break;
		
		default:
			
			break;
		}
		
		return joint;
	}
b2Joint.Destroy = function (joint, allocator) {
		
	}
// attributes
b2Joint.prototype.m_type =  0;
b2Joint.prototype.m_prev =  null;
b2Joint.prototype.m_next =  null;
b2Joint.prototype.m_node1 =  new b2JointEdge();
b2Joint.prototype.m_node2 =  new b2JointEdge();
b2Joint.prototype.m_body1 =  null;
b2Joint.prototype.m_body2 =  null;
b2Joint.prototype.m_inv_dt =  null;
b2Joint.prototype.m_islandFlag =  null;
b2Joint.prototype.m_collideConnected =  null;
b2Joint.prototype.m_userData =  null;
// methods
b2Joint.prototype.GetType = function () {
		return this.m_type;
	}
b2Joint.prototype.GetAnchor1 = function () {return null}
b2Joint.prototype.GetAnchor2 = function () {return null}
b2Joint.prototype.GetReactionForce = function () {return null}
b2Joint.prototype.GetReactionTorque = function () {return 0.0}
b2Joint.prototype.GetBody1 = function () {
		return this.m_body1;
	}
b2Joint.prototype.GetBody2 = function () {
		return this.m_body2;
	}
b2Joint.prototype.GetNext = function () {
		return this.m_next;
	}
b2Joint.prototype.GetUserData = function () {
		return this.m_userData;
	}
b2Joint.prototype.SetUserData = function (data) {
		this.m_userData = data;
	}
b2Joint.prototype.InitVelocityConstraints = function (step) {}
b2Joint.prototype.SolveVelocityConstraints = function (step) {}
b2Joint.prototype.InitPositionConstraints = function () {}
b2Joint.prototype.SolvePositionConstraints = function () {return false}