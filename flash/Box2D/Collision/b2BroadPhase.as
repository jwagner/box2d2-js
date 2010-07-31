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

package Box2D.Collision{
	
	
import Box2D.Common.*;
import Box2D.Collision.*;
import Box2D.Common.Math.*;


/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/


// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
//   overlap query results.
// - where possible, we compare bound indices instead of values to reduce
//   cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for huge
//   worlds (use a multi-SAP instead), it is not great for large objects.

public class b2BroadPhase
{
//public:
	public function b2BroadPhase(worldAABB:b2AABB, callback:b2PairCallback){
		//b2Settings.b2Assert(worldAABB.IsValid());
		var i:int;
		
		m_pairManager.Initialize(this, callback);
		
		m_worldAABB = worldAABB;
		
		m_proxyCount = 0;
		
		// query results
		for (i = 0; i < b2Settings.b2_maxProxies; i++){
			m_queryResults[i] = 0;
		}
		
		// bounds array
		m_bounds = new Array(2);
		for (i = 0; i < 2; i++){
			m_bounds[i] = new Array(2*b2Settings.b2_maxProxies);
			for (var j:int = 0; j < 2*b2Settings.b2_maxProxies; j++){
				m_bounds[i][j] = new b2Bound();
			}
		}
		
		//b2Vec2 d = worldAABB.upperBound - worldAABB.lowerBound;
		var dX:Number = worldAABB.upperBound.x - worldAABB.lowerBound.x;;
		var dY:Number = worldAABB.upperBound.y - worldAABB.lowerBound.y;
		
		m_quantizationFactor.x = b2Settings.USHRT_MAX / dX;
		m_quantizationFactor.y = b2Settings.USHRT_MAX / dY;
		
		var tProxy:b2Proxy;
		for (i = 0; i < b2Settings.b2_maxProxies - 1; ++i)
		{
			tProxy = new b2Proxy();
			m_proxyPool[i] = tProxy;
			tProxy.SetNext(i + 1);
			tProxy.timeStamp = 0;
			tProxy.overlapCount = b2_invalid;
			tProxy.userData = null;
		}
		tProxy = new b2Proxy();
		m_proxyPool[int(b2Settings.b2_maxProxies-1)] = tProxy;
		tProxy.SetNext(b2Pair.b2_nullProxy);
		tProxy.timeStamp = 0;
		tProxy.overlapCount = b2_invalid;
		tProxy.userData = null;
		m_freeProxy = 0;
		
		m_timeStamp = 1;
		m_queryResultCount = 0;
	}
	//~b2BroadPhase();
	
	// Use this to see if your proxy is in range. If it is not in range,
	// it should be destroyed. Otherwise you may get O(m^2) pairs, where m
	// is the number of proxies that are out of range.
	public function InRange(aabb:b2AABB):Boolean{
		//b2Vec2 d = b2Max(aabb.lowerBound - m_worldAABB.upperBound, m_worldAABB.lowerBound - aabb.upperBound);
		var dX:Number;
		var dY:Number;
		var d2X:Number;
		var d2Y:Number;
		
		dX = aabb.lowerBound.x;
		dY = aabb.lowerBound.y;
		dX -= m_worldAABB.upperBound.x;
		dY -= m_worldAABB.upperBound.y;
		
		d2X = m_worldAABB.lowerBound.x;
		d2Y = m_worldAABB.lowerBound.y;
		d2X -= aabb.upperBound.x;
		d2Y -= aabb.upperBound.y;
		
		dX = b2Math.b2Max(dX, d2X);
		dY = b2Math.b2Max(dY, d2Y);
		
		return b2Math.b2Max(dX, dY) < 0.0;
	}
	
	// Get a single proxy. Returns NULL if the id is invalid.
	public function GetProxy(proxyId:int):b2Proxy{
		var proxy: b2Proxy = m_proxyPool[proxyId];
		if (proxyId == b2Pair.b2_nullProxy || proxy.IsValid() == false)
		{
			return null;
		}
		
		return proxy;
	}

	// Create and destroy proxies. These call Flush first.
	public function CreateProxy(aabb:b2AABB, userData:*):uint{
		var index:uint;
		var proxy:b2Proxy;
		
		//b2Settings.b2Assert(m_proxyCount < b2_maxProxies);
		//b2Settings.b2Assert(m_freeProxy != b2Pair.b2_nullProxy);
		
		var proxyId:uint = m_freeProxy;
		proxy = m_proxyPool[ proxyId ];
		m_freeProxy = proxy.GetNext();
		
		proxy.overlapCount = 0;
		proxy.userData = userData;
		
		var boundCount:uint = 2 * m_proxyCount;
		
		var lowerValues:Array = new Array();
		var upperValues:Array = new Array();
		ComputeBounds(lowerValues, upperValues, aabb);
		
		for (var axis:int = 0; axis < 2; ++axis)
		{
			var bounds:Array = m_bounds[axis];
			var lowerIndex:uint;
			var upperIndex:uint;
			var lowerIndexOut:Array = [lowerIndex];
			var upperIndexOut:Array = [upperIndex];
			Query(lowerIndexOut, upperIndexOut, lowerValues[axis], upperValues[axis], bounds, boundCount, axis);
			lowerIndex = lowerIndexOut[0];
			upperIndex = upperIndexOut[0];
			
			// Replace memmove calls
			//memmove(bounds + upperIndex + 2, bounds + upperIndex, (edgeCount - upperIndex) * sizeof(b2Bound));
			var tArr:Array = new Array();
			var j:int;
			var tEnd:int = boundCount - upperIndex
			var tBound1:b2Bound;
			var tBound2:b2Bound;
			var tBoundAS3:b2Bound;
			// make temp array
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[int(upperIndex+j)];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			// move temp array back in to bounds
			tEnd = tArr.length;
			var tIndex:int = upperIndex+2;
			for (j = 0; j < tEnd; j++){
				//bounds[tIndex+j] = tArr[j];
				tBound2 = tArr[j];
				tBound1 = bounds[int(tIndex+j)]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			//memmove(bounds + lowerIndex + 1, bounds + lowerIndex, (upperIndex - lowerIndex) * sizeof(b2Bound));
			// make temp array
			tArr = new Array();
			tEnd = upperIndex - lowerIndex;
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[int(lowerIndex+j)];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			// move temp array back in to bounds
			tEnd = tArr.length;
			tIndex = lowerIndex+1;
			for (j = 0; j < tEnd; j++){
				//bounds[tIndex+j] = tArr[j];
				tBound2 = tArr[j];
				tBound1 = bounds[int(tIndex+j)]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			// The upper index has increased because of the lower bound insertion.
			++upperIndex;
			
			// Copy in the new bounds.
			tBound1 = bounds[lowerIndex];
			tBound2 = bounds[upperIndex];
			tBound1.value = lowerValues[axis];
			tBound1.proxyId = proxyId;
			tBound2.value = upperValues[axis];
			tBound2.proxyId = proxyId;
			
			tBoundAS3 = bounds[int(lowerIndex-1)];
			tBound1.stabbingCount = lowerIndex == 0 ? 0 : tBoundAS3.stabbingCount;
			tBoundAS3 = bounds[int(upperIndex-1)];
			tBound2.stabbingCount = tBoundAS3.stabbingCount;
			
			// Adjust the stabbing count between the new bounds.
			for (index = lowerIndex; index < upperIndex; ++index)
			{
				tBoundAS3 = bounds[index];
				tBoundAS3.stabbingCount++;
			}
			
			// Adjust the all the affected bound indices.
			for (index = lowerIndex; index < boundCount + 2; ++index)
			{
				tBound1 = bounds[index];
				var proxy2:b2Proxy = m_proxyPool[ tBound1.proxyId ];
				if (tBound1.IsLower())
				{
					proxy2.lowerBounds[axis] = index;
				}
				else
				{
					proxy2.upperBounds[axis] = index;
				}
			}
		}
		
		++m_proxyCount;
		
		//b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);
		
		for (var i:int = 0; i < m_queryResultCount; ++i)
		{
			//b2Settings.b2Assert(m_queryResults[i] < b2_maxProxies);
			//b2Settings.b2Assert(m_proxyPool[m_queryResults[i]].IsValid());
			
			m_pairManager.AddBufferedPair(proxyId, m_queryResults[i]);
		}
		
		m_pairManager.Commit();
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
		
		return proxyId;
	}
	
	public function DestroyProxy(proxyId:uint) : void{
		var tBound1:b2Bound;
		var tBound2:b2Bound;
		
		//b2Settings.b2Assert(0 < m_proxyCount && m_proxyCount <= b2_maxProxies);
		
		var proxy:b2Proxy = m_proxyPool[ proxyId ];
		//b2Settings.b2Assert(proxy.IsValid());
		
		var boundCount:int = 2 * m_proxyCount;
		
		for (var axis:int = 0; axis < 2; ++axis)
		{
			var bounds:Array = m_bounds[axis];
			
			var lowerIndex:uint = proxy.lowerBounds[axis];
			var upperIndex:uint = proxy.upperBounds[axis];
			tBound1 = bounds[lowerIndex];
			var lowerValue:uint = tBound1.value;
			tBound2 = bounds[upperIndex];
			var upperValue:uint = tBound2.value;
			
			// replace memmove calls
			//memmove(bounds + lowerIndex, bounds + lowerIndex + 1, (upperIndex - lowerIndex - 1) * sizeof(b2Bound));
			var tArr:Array = new Array();
			var j:int;
			var tEnd:int = upperIndex - lowerIndex - 1;
			// make temp array
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[int(lowerIndex+1+j)];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			// move temp array back in to bounds
			tEnd = tArr.length;
			var tIndex:int = lowerIndex;
			for (j = 0; j < tEnd; j++){
				//bounds[tIndex+j] = tArr[j];
				tBound2 = tArr[j];
				tBound1 = bounds[int(tIndex+j)]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			//memmove(bounds + upperIndex-1, bounds + upperIndex + 1, (edgeCount - upperIndex - 1) * sizeof(b2Bound));
			// make temp array
			tArr = new Array();
			tEnd = boundCount - upperIndex - 1;
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[int(upperIndex+1+j)];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			// move temp array back in to bounds
			tEnd = tArr.length;
			tIndex = upperIndex-1;
			for (j = 0; j < tEnd; j++){
				//bounds[tIndex+j] = tArr[j];
				tBound2 = tArr[j];
				tBound1 = bounds[int(tIndex+j)]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			// Fix bound indices.
			tEnd = boundCount - 2;
			for (var index:uint = lowerIndex; index < tEnd; ++index)
			{
				tBound1 = bounds[index];
				var proxy2:b2Proxy = m_proxyPool[ tBound1.proxyId ];
				if (tBound1.IsLower())
				{
					proxy2.lowerBounds[axis] = index;
				}
				else
				{
					proxy2.upperBounds[axis] = index;
				}
			}
			
			// Fix stabbing count.
			tEnd = upperIndex - 1;
			for (var index2:int = lowerIndex; index2 < tEnd; ++index2)
			{
				tBound1 = bounds[index2];
				tBound1.stabbingCount--;
			}
			
			// Query for pairs to be removed. lowerIndex and upperIndex are not needed.
			// make lowerIndex and upper output using an array and do this for others if compiler doesn't pick them up
			Query([0], [0], lowerValue, upperValue, bounds, boundCount - 2, axis);
		}
		
		//b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);
		
		for (var i:int = 0; i < m_queryResultCount; ++i)
		{
			//b2Settings.b2Assert(m_proxyPool[m_queryResults[i]].IsValid());
			
			m_pairManager.RemoveBufferedPair(proxyId, m_queryResults[i]);
		}
		
		m_pairManager.Commit();
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
		
		// Return the proxy to the pool.
		proxy.userData = null;
		proxy.overlapCount = b2_invalid;
		proxy.lowerBounds[0] = b2_invalid;
		proxy.lowerBounds[1] = b2_invalid;
		proxy.upperBounds[0] = b2_invalid;
		proxy.upperBounds[1] = b2_invalid;
		
		proxy.SetNext(m_freeProxy);
		m_freeProxy = proxyId;
		--m_proxyCount;
	}


	// Call MoveProxy as many times as you like, then when you are done
	// call Commit to finalized the proxy pairs (for your time step).
	public function MoveProxy(proxyId:uint, aabb:b2AABB) : void{
		var as3arr: Array;
		var as3int: int;
		
		var axis:uint;
		var index:uint;
		var bound:b2Bound;
		var prevBound:b2Bound;
		var nextBound:b2Bound;
		var nextProxyId:uint;
		var nextProxy:b2Proxy;
		
		if (proxyId == b2Pair.b2_nullProxy || b2Settings.b2_maxProxies <= proxyId)
		{
			//b2Settings.b2Assert(false);
			return;
		}
		
		if (aabb.IsValid() == false)
		{
			//b2Settings.b2Assert(false);
			return;
		}
		
		var boundCount:uint = 2 * m_proxyCount;
		
		var proxy:b2Proxy = m_proxyPool[ proxyId ];
		// Get new bound values
		var newValues:b2BoundValues = new b2BoundValues();
		ComputeBounds(newValues.lowerValues, newValues.upperValues, aabb);
		
		// Get old bound values
		var oldValues:b2BoundValues = new b2BoundValues();
		for (axis = 0; axis < 2; ++axis)
		{
			bound = m_bounds[axis][proxy.lowerBounds[axis]];
			oldValues.lowerValues[axis] = bound.value;
			bound = m_bounds[axis][proxy.upperBounds[axis]];
			oldValues.upperValues[axis] = bound.value;
		}
		
		for (axis = 0; axis < 2; ++axis)
		{
			var bounds:Array = m_bounds[axis];
			
			var lowerIndex:uint = proxy.lowerBounds[axis];
			var upperIndex:uint = proxy.upperBounds[axis];
			
			var lowerValue:uint = newValues.lowerValues[axis];
			var upperValue:uint = newValues.upperValues[axis];
			
			bound = bounds[lowerIndex];
			var deltaLower:int = lowerValue - bound.value;
			bound.value = lowerValue;
			
			bound = bounds[upperIndex];
			var deltaUpper:int = upperValue - bound.value;
			bound.value = upperValue;
			
			//
			// Expanding adds overlaps
			//
			
			// Should we move the lower bound down?
			if (deltaLower < 0)
			{
				index = lowerIndex;
				while (index > 0 && lowerValue < (bounds[int(index-1)] as b2Bound).value)
				{
					bound = bounds[index];
					prevBound = bounds[int(index - 1)];
					
					var prevProxyId:uint = prevBound.proxyId;
					var prevProxy:b2Proxy = m_proxyPool[ prevBound.proxyId ];
					
					prevBound.stabbingCount++;
					
					if (prevBound.IsUpper() == true)
					{
						if (TestOverlap(newValues, prevProxy))
						{
							m_pairManager.AddBufferedPair(proxyId, prevProxyId);
						}
						
						//prevProxy.upperBounds[axis]++;
						as3arr = prevProxy.upperBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					else
					{
						//prevProxy.lowerBounds[axis]++;
						as3arr = prevProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					
					//proxy.lowerBounds[axis]--;
					as3arr = proxy.lowerBounds;
					as3int = as3arr[axis];
					as3int--;
					as3arr[axis] = as3int;
					
					// swap
					//var temp:b2Bound = bound;
					//bound = prevEdge;
					//prevEdge = temp;
					bound.Swap(prevBound);
					//b2Math.b2Swap(bound, prevEdge);
					--index;
				}
			}
			
			// Should we move the upper bound up?
			if (deltaUpper > 0)
			{
				index = upperIndex;
				while (index < boundCount-1 && (bounds[int(index+1)] as b2Bound).value <= upperValue)
				{
					bound = bounds[ index ];
					nextBound = bounds[ int(index + 1) ];
					nextProxyId = nextBound.proxyId;
					nextProxy = m_proxyPool[ nextProxyId ];
					
					nextBound.stabbingCount++;
					
					if (nextBound.IsLower() == true)
					{
						if (TestOverlap(newValues, nextProxy))
						{
							m_pairManager.AddBufferedPair(proxyId, nextProxyId);
						}
						
						//nextProxy.lowerBounds[axis]--;
						as3arr = nextProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					else
					{
						//nextProxy.upperBounds[axis]--;
						as3arr = nextProxy.upperBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					
					//proxy.upperBounds[axis]++;
					as3arr = proxy.upperBounds;
					as3int = as3arr[axis];
					as3int++;
					as3arr[axis] = as3int;
					
					// swap
					//var temp:b2Bound = bound;
					//bound = nextEdge;
					//nextEdge = temp;
					bound.Swap(nextBound);
					//b2Math.b2Swap(bound, nextEdge);
					index++;
				}
			}
			
			//
			// Shrinking removes overlaps
			//
			
			// Should we move the lower bound up?
			if (deltaLower > 0)
			{
				index = lowerIndex;
				while (index < boundCount-1 && (bounds[int(index+1)] as b2Bound).value <= lowerValue)
				{
					bound = bounds[ index ];
					nextBound = bounds[ int(index + 1) ];
					
					nextProxyId = nextBound.proxyId;
					nextProxy = m_proxyPool[ nextProxyId ];
					
					nextBound.stabbingCount--;
					
					if (nextBound.IsUpper())
					{
						if (TestOverlap(oldValues, nextProxy))
						{
							m_pairManager.RemoveBufferedPair(proxyId, nextProxyId);
						}
						
						//nextProxy.upperBounds[axis]--;
						as3arr = nextProxy.upperBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					else
					{
						//nextProxy.lowerBounds[axis]--;
						as3arr = nextProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					
					//proxy.lowerBounds[axis]++;
					as3arr = proxy.lowerBounds;
					as3int = as3arr[axis];
					as3int++;
					as3arr[axis] = as3int;
					
					// swap
					//var temp:b2Bound = bound;
					//bound = nextEdge;
					//nextEdge = temp;
					bound.Swap(nextBound);
					//b2Math.b2Swap(bound, nextEdge);
					index++;
				}
			}
			
			// Should we move the upper bound down?
			if (deltaUpper < 0)
			{
				index = upperIndex;
				while (index > 0 && upperValue < (bounds[int(index-1)] as b2Bound).value)
				{
					bound = bounds[index];
					prevBound = bounds[int(index - 1)];
					
					prevProxyId = prevBound.proxyId;
					prevProxy = m_proxyPool[ prevProxyId ];
					
					prevBound.stabbingCount--;
					
					if (prevBound.IsLower() == true)
					{
						if (TestOverlap(oldValues, prevProxy))
						{
							m_pairManager.RemoveBufferedPair(proxyId, prevProxyId);
						}
						
						//prevProxy.lowerBounds[axis]++;
						as3arr = prevProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					else
					{
						//prevProxy.upperBounds[axis]++;
						as3arr = prevProxy.upperBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					
					//proxy.upperBounds[axis]--;
					as3arr = proxy.upperBounds;
					as3int = as3arr[axis];
					as3int--;
					as3arr[axis] = as3int;
					
					// swap
					//var temp:b2Bound = bound;
					//bound = prevEdge;
					//prevEdge = temp;
					bound.Swap(prevBound);
					//b2Math.b2Swap(bound, prevEdge);
					index--;
				}
			}
		}
	}
	
	public function Commit() : void{
		m_pairManager.Commit();
	}

	// Query an AABB for overlapping proxies, returns the user data and
	// the count, up to the supplied maximum count.
	public function QueryAABB(aabb:b2AABB, userData:*, maxCount:int):int{
		var lowerValues:Array = new Array();
		var upperValues:Array = new Array();
		ComputeBounds(lowerValues, upperValues, aabb);
		
		var lowerIndex:uint;
		var upperIndex:uint;
		var lowerIndexOut:Array = [lowerIndex];
		var upperIndexOut:Array = [upperIndex];
		Query(lowerIndexOut, upperIndexOut, lowerValues[0], upperValues[0], m_bounds[0], 2*m_proxyCount, 0);
		Query(lowerIndexOut, upperIndexOut, lowerValues[1], upperValues[1], m_bounds[1], 2*m_proxyCount, 1);
		
		//b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);
		
		var count:int = 0;
		for (var i:int = 0; i < m_queryResultCount && count < maxCount; ++i, ++count)
		{
			//b2Settings.b2Assert(m_queryResults[i] < b2Settings.b2_maxProxies);
			var proxy:b2Proxy = m_proxyPool[ m_queryResults[i] ];
			//b2Settings.b2Assert(proxy.IsValid());
			userData[i] = proxy.userData;
		}
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
		
		return count;
	}

	public function Validate() : void{
		var pair:b2Pair;
		var proxy1:b2Proxy;
		var proxy2:b2Proxy;
		var overlap:Boolean;
		
		for (var axis:int = 0; axis < 2; ++axis)
		{
			var bounds:b2Bound = m_bounds[axis];
			
			var boundCount:uint = 2 * m_proxyCount;
			var stabbingCount:uint = 0;
			
			for (var i:uint = 0; i < boundCount; ++i)
			{
				var bound:b2Bound = bounds[i];
				//b2Settings.b2Assert(i == 0 || bounds[i-1].value <= bound->value);
				//b2Settings.b2Assert(bound->proxyId != b2_nullProxy);
				//b2Settings.b2Assert(m_proxyPool[bound->proxyId].IsValid());
				
				if (bound.IsLower() == true)
				{
					//b2Settings.b2Assert(m_proxyPool[bound.proxyId].lowerBounds[axis] == i);
					stabbingCount++;
				}
				else
				{
					//b2Settings.b2Assert(m_proxyPool[bound.proxyId].upperBounds[axis] == i);
					stabbingCount--;
				}
				
				//b2Settings.b2Assert(bound.stabbingCount == stabbingCount);
			}
		}
		
	}

//private:
	private function ComputeBounds(lowerValues:Array, upperValues:Array, aabb:b2AABB) : void
	{
		//b2Settings.b2Assert(aabb.upperBound.x > aabb.lowerBound.x);
		//b2Settings.b2Assert(aabb.upperBound.y > aabb.lowerBound.y);
		
		//var minVertex:b2Vec2 = b2Math.b2ClampV(aabb.minVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);
		var minVertexX:Number = aabb.lowerBound.x;
		var minVertexY:Number = aabb.lowerBound.y;
		minVertexX = b2Math.b2Min(minVertexX, m_worldAABB.upperBound.x);
		minVertexY = b2Math.b2Min(minVertexY, m_worldAABB.upperBound.y);
		minVertexX = b2Math.b2Max(minVertexX, m_worldAABB.lowerBound.x);
		minVertexY = b2Math.b2Max(minVertexY, m_worldAABB.lowerBound.y);
		
		//var maxVertex:b2Vec2 = b2Math.b2ClampV(aabb.maxVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);
		var maxVertexX:Number = aabb.upperBound.x;
		var maxVertexY:Number = aabb.upperBound.y;
		maxVertexX = b2Math.b2Min(maxVertexX, m_worldAABB.upperBound.x);
		maxVertexY = b2Math.b2Min(maxVertexY, m_worldAABB.upperBound.y);
		maxVertexX = b2Math.b2Max(maxVertexX, m_worldAABB.lowerBound.x);
		maxVertexY = b2Math.b2Max(maxVertexY, m_worldAABB.lowerBound.y);
		
		// Bump lower bounds downs and upper bounds up. This ensures correct sorting of
		// lower/upper bounds that would have equal values.
		// TODO_ERIN implement fast float to uint16 conversion.
		lowerValues[0] = uint(m_quantizationFactor.x * (minVertexX - m_worldAABB.lowerBound.x)) & (b2Settings.USHRT_MAX - 1);
		upperValues[0] = (uint(m_quantizationFactor.x * (maxVertexX - m_worldAABB.lowerBound.x))& 0x0000ffff) | 1;
		
		lowerValues[1] = uint(m_quantizationFactor.y * (minVertexY - m_worldAABB.lowerBound.y)) & (b2Settings.USHRT_MAX - 1);
		upperValues[1] = (uint(m_quantizationFactor.y * (maxVertexY - m_worldAABB.lowerBound.y))& 0x0000ffff) | 1;
	}

	// This one is only used for validation.
	private function TestOverlapValidate(p1:b2Proxy, p2:b2Proxy):Boolean{
		
		for (var axis:int = 0; axis < 2; ++axis)
		{
			var bounds:Array = m_bounds[axis];
			
			//b2Settings.b2Assert(p1.lowerBounds[axis] < 2 * m_proxyCount);
			//b2Settings.b2Assert(p1.upperBounds[axis] < 2 * m_proxyCount);
			//b2Settings.b2Assert(p2.lowerBounds[axis] < 2 * m_proxyCount);
			//b2Settings.b2Assert(p2.upperBounds[axis] < 2 * m_proxyCount);
			
			var bound1:b2Bound = bounds[p1.lowerBounds[axis]];
			var bound2:b2Bound = bounds[p2.upperBounds[axis]];
			if (bound1.value > bound2.value)
				return false;
			
			bound1 = bounds[p1.upperBounds[axis]];
			bound2 = bounds[p2.lowerBounds[axis]];
			if (bound1.value < bound2.value)
				return false;
		}
		
		return true;
	}
	
	public function TestOverlap(b:b2BoundValues, p:b2Proxy):Boolean
	{
		for (var axis:int = 0; axis < 2; ++axis)
		{
			var bounds:Array = m_bounds[axis];
			
			//b2Settings.b2Assert(p.lowerBounds[axis] < 2 * m_proxyCount);
			//b2Settings.b2Assert(p.upperBounds[axis] < 2 * m_proxyCount);
			
			var bound:b2Bound = bounds[p.upperBounds[axis]];
			if (b.lowerValues[axis] > bound.value)
				return false;
			
			bound = bounds[p.lowerBounds[axis]];
			if (b.upperValues[axis] < bound.value)
				return false;
		}
		
		return true;
	}

	private function Query(lowerQueryOut:Array, upperQueryOut:Array, lowerValue:uint, upperValue:uint, bounds:Array, boundCount:uint, axis:int) : void{
		
		var lowerQuery:uint = BinarySearch(bounds, boundCount, lowerValue);
		var upperQuery:uint = BinarySearch(bounds, boundCount, upperValue);
		var bound: b2Bound;
		
		// Easy case: lowerQuery <= lowerIndex(i) < upperQuery
		// Solution: search query range for min bounds.
		for (var j:uint = lowerQuery; j < upperQuery; ++j)
		{
			bound = bounds[j];
			if (bound.IsLower())
			{
				IncrementOverlapCount(bound.proxyId);
			}
		}
		
		// Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
		// Solution: use the stabbing count to search down the bound array.
		if (lowerQuery > 0)
		{
			var i:int = lowerQuery - 1;
			bound = bounds[i];
			var s:int = bound.stabbingCount;
			
			// Find the s overlaps.
			while (s)
			{
				//b2Settings.b2Assert(i >= 0);
				bound = bounds[i];
				if (bound.IsLower())
				{
					var proxy:b2Proxy = m_proxyPool[ bound.proxyId ];
					if (lowerQuery <= proxy.upperBounds[axis])
					{
						IncrementOverlapCount(bound.proxyId);
						--s;
					}
				}
				--i;
			}
		}
		
		lowerQueryOut[0] = lowerQuery;
		upperQueryOut[0] = upperQuery;
	}


	private function IncrementOverlapCount(proxyId:uint) : void{
		var proxy:b2Proxy = m_proxyPool[ proxyId ];
		if (proxy.timeStamp < m_timeStamp)
		{
			proxy.timeStamp = m_timeStamp;
			proxy.overlapCount = 1;
		}
		else
		{
			proxy.overlapCount = 2;
			//b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);
			m_queryResults[m_queryResultCount] = proxyId;
			++m_queryResultCount;
		}
	}
	private function IncrementTimeStamp() : void{
		if (m_timeStamp == b2Settings.USHRT_MAX)
		{
			for (var i:uint = 0; i < b2Settings.b2_maxProxies; ++i)
			{
				(m_proxyPool[i] as b2Proxy).timeStamp = 0;
			}
			m_timeStamp = 1;
		}
		else
		{
			++m_timeStamp;
		}
	}

//public:
	public var m_pairManager:b2PairManager = new b2PairManager();

	public var m_proxyPool:Array = new Array(b2Settings.b2_maxPairs);
	public var m_freeProxy:uint;

	public var m_bounds:Array = new Array(2*b2Settings.b2_maxProxies);

	public var m_queryResults:Array = new Array(b2Settings.b2_maxProxies);
	public var m_queryResultCount:int;

	public var m_worldAABB:b2AABB;
	public var m_quantizationFactor:b2Vec2 = new b2Vec2();
	public var m_proxyCount:int;
	public var m_timeStamp:uint;

	static public var s_validate:Boolean = false;
	
	static public const b2_invalid:uint = b2Settings.USHRT_MAX;
	static public const b2_nullEdge:uint = b2Settings.USHRT_MAX;


	static public function BinarySearch(bounds:Array, count:int, value:uint):uint
	{
		var low:int = 0;
		var high:int = count - 1;
		while (low <= high)
		{
			var mid:int = ((low + high) / 2);
			var bound:b2Bound = bounds[mid];
			if (bound.value > value)
			{
				high = mid - 1;
			}
			else if (bound.value < value)
			{
				low = mid + 1;
			}
			else
			{
				return uint(mid);
			}
		}
		
		return uint(low);
	}
	
	
};
}