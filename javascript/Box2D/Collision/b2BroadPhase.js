var b2BroadPhase = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2BroadPhase.prototype.__constructor = function (worldAABB, callback) {
		
		var i = 0;
		
		this.m_pairManager.Initialize(this, callback);
		
		this.m_worldAABB = worldAABB;
		
		this.m_proxyCount = 0;
		
		
		for (i = 0; i < b2Settings.b2_maxProxies; i++){
			this.m_queryResults[i] = 0;
		}
		
		
		this.m_bounds = new Array(2);
		for (i = 0; i < 2; i++){
			this.m_bounds[i] = new Array(2*b2Settings.b2_maxProxies);
			for (var j = 0; j < 2*b2Settings.b2_maxProxies; j++){
				this.m_bounds[i][j] = new b2Bound();
			}
		}
		
		
		var dX = worldAABB.upperBound.x - worldAABB.lowerBound.x;;
		var dY = worldAABB.upperBound.y - worldAABB.lowerBound.y;
		
		this.m_quantizationFactor.x = b2Settings.USHRT_MAX / dX;
		this.m_quantizationFactor.y = b2Settings.USHRT_MAX / dY;
		
		var tProxy;
		for (i = 0; i < b2Settings.b2_maxProxies - 1; ++i)
		{
			tProxy = new b2Proxy();
			this.m_proxyPool[i] = tProxy;
			tProxy.SetNext(i + 1);
			tProxy.timeStamp = 0;
			tProxy.overlapCount = b2BroadPhase.b2_invalid;
			tProxy.userData = null;
		}
		tProxy = new b2Proxy();
		this.m_proxyPool[parseInt(b2Settings.b2_maxProxies-1)] = tProxy;
		tProxy.SetNext(b2Pair.b2_nullProxy);
		tProxy.timeStamp = 0;
		tProxy.overlapCount = b2BroadPhase.b2_invalid;
		tProxy.userData = null;
		this.m_freeProxy = 0;
		
		this.m_timeStamp = 1;
		this.m_queryResultCount = 0;
	}
b2BroadPhase.prototype.__varz = function(){
this.m_pairManager =  new b2PairManager();
this.m_proxyPool =  new Array(b2Settings.b2_maxPairs);
this.m_bounds =  new Array(2*b2Settings.b2_maxProxies);
this.m_queryResults =  new Array(b2Settings.b2_maxProxies);
this.m_quantizationFactor =  new b2Vec2();
}
// static attributes
b2BroadPhase.s_validate =  false;
b2BroadPhase.b2_invalid =  b2Settings.USHRT_MAX;
b2BroadPhase.b2_nullEdge =  b2Settings.USHRT_MAX;
// static methods
b2BroadPhase.BinarySearch = function (bounds, count, value) {
		var low = 0;
		var high = count - 1;
		while (low <= high)
		{
			var mid = Math.round((low + high) / 2);
			var bound = bounds[mid];
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
				return parseInt(mid);
			}
		}
		
		return parseInt(low);
	}
// attributes
b2BroadPhase.prototype.m_pairManager =  new b2PairManager();
b2BroadPhase.prototype.m_proxyPool =  new Array(b2Settings.b2_maxPairs);
b2BroadPhase.prototype.m_freeProxy =  0;
b2BroadPhase.prototype.m_bounds =  new Array(2*b2Settings.b2_maxProxies);
b2BroadPhase.prototype.m_queryResults =  new Array(b2Settings.b2_maxProxies);
b2BroadPhase.prototype.m_queryResultCount =  0;
b2BroadPhase.prototype.m_worldAABB =  null;
b2BroadPhase.prototype.m_quantizationFactor =  new b2Vec2();
b2BroadPhase.prototype.m_proxyCount =  0;
b2BroadPhase.prototype.m_timeStamp =  0;
// methods
b2BroadPhase.prototype.ComputeBounds = function (lowerValues, upperValues, aabb) {
		
		
		
		
		var minVertexX = aabb.lowerBound.x;
		var minVertexY = aabb.lowerBound.y;
		minVertexX = b2Math.b2Min(minVertexX, this.m_worldAABB.upperBound.x);
		minVertexY = b2Math.b2Min(minVertexY, this.m_worldAABB.upperBound.y);
		minVertexX = b2Math.b2Max(minVertexX, this.m_worldAABB.lowerBound.x);
		minVertexY = b2Math.b2Max(minVertexY, this.m_worldAABB.lowerBound.y);
		
		
		var maxVertexX = aabb.upperBound.x;
		var maxVertexY = aabb.upperBound.y;
		maxVertexX = b2Math.b2Min(maxVertexX, this.m_worldAABB.upperBound.x);
		maxVertexY = b2Math.b2Min(maxVertexY, this.m_worldAABB.upperBound.y);
		maxVertexX = b2Math.b2Max(maxVertexX, this.m_worldAABB.lowerBound.x);
		maxVertexY = b2Math.b2Max(maxVertexY, this.m_worldAABB.lowerBound.y);
		
		
		
		
		lowerValues[0] = parseInt(this.m_quantizationFactor.x * (minVertexX - this.m_worldAABB.lowerBound.x)) & (b2Settings.USHRT_MAX - 1);
		upperValues[0] = (parseInt(this.m_quantizationFactor.x * (maxVertexX - this.m_worldAABB.lowerBound.x))% 65535) | 1;
		
		lowerValues[1] = parseInt(this.m_quantizationFactor.y * (minVertexY - this.m_worldAABB.lowerBound.y)) & (b2Settings.USHRT_MAX - 1);
		upperValues[1] = (parseInt(this.m_quantizationFactor.y * (maxVertexY - this.m_worldAABB.lowerBound.y))% 65535) | 1;
	}
b2BroadPhase.prototype.TestOverlapValidate = function (p1, p2) {
		
		for (var axis = 0; axis < 2; ++axis)
		{
			var bounds = this.m_bounds[axis];
			
			
			
			
			
			
			var bound1 = bounds[p1.lowerBounds[axis]];
			var bound2 = bounds[p2.upperBounds[axis]];
			if (bound1.value > bound2.value)
				return false;
			
			bound1 = bounds[p1.upperBounds[axis]];
			bound2 = bounds[p2.lowerBounds[axis]];
			if (bound1.value < bound2.value)
				return false;
		}
		
		return true;
	}
b2BroadPhase.prototype.Query = function (lowerQueryOut, upperQueryOut, lowerValue, upperValue, bounds, boundCount, axis) {
		
		var lowerQuery = b2BroadPhase.BinarySearch(bounds, boundCount, lowerValue);
		var upperQuery = b2BroadPhase.BinarySearch(bounds, boundCount, upperValue);
		var bound;
		
		
		
		for (var j = lowerQuery; j < upperQuery; ++j)
		{
			bound = bounds[j];
			if (bound.IsLower())
			{
				this.IncrementOverlapCount(bound.proxyId);
			}
		}
		
		
		
		if (lowerQuery > 0)
		{
			var i = lowerQuery - 1;
			bound = bounds[i];
			var s = bound.stabbingCount;
			
			
			while (s)
			{
				
				bound = bounds[i];
				if (bound.IsLower())
				{
					var proxy = this.m_proxyPool[ bound.proxyId ];
					if (lowerQuery <= proxy.upperBounds[axis])
					{
						this.IncrementOverlapCount(bound.proxyId);
						--s;
					}
				}
				--i;
			}
		}
		
		lowerQueryOut[0] = lowerQuery;
		upperQueryOut[0] = upperQuery;
	}
b2BroadPhase.prototype.IncrementOverlapCount = function (proxyId) {
		var proxy = this.m_proxyPool[ proxyId ];
		if (proxy.timeStamp < this.m_timeStamp)
		{
			proxy.timeStamp = this.m_timeStamp;
			proxy.overlapCount = 1;
		}
		else
		{
			proxy.overlapCount = 2;
			
			this.m_queryResults[this.m_queryResultCount] = proxyId;
			++this.m_queryResultCount;
		}
	}
b2BroadPhase.prototype.IncrementTimeStamp = function () {
		if (this.m_timeStamp == b2Settings.USHRT_MAX)
		{
			for (var i = 0; i < b2Settings.b2_maxProxies; ++i)
			{
				(this.m_proxyPool[i]).timeStamp = 0;
			}
			this.m_timeStamp = 1;
		}
		else
		{
			++this.m_timeStamp;
		}
	}
b2BroadPhase.prototype.InRange = function (aabb) {
		
		var dX;
		var dY;
		var d2X;
		var d2Y;
		
		dX = aabb.lowerBound.x;
		dY = aabb.lowerBound.y;
		dX -= this.m_worldAABB.upperBound.x;
		dY -= this.m_worldAABB.upperBound.y;
		
		d2X = this.m_worldAABB.lowerBound.x;
		d2Y = this.m_worldAABB.lowerBound.y;
		d2X -= aabb.upperBound.x;
		d2Y -= aabb.upperBound.y;
		
		dX = b2Math.b2Max(dX, d2X);
		dY = b2Math.b2Max(dY, d2Y);
		
		return b2Math.b2Max(dX, dY) < 0.0;
	}
b2BroadPhase.prototype.GetProxy = function (proxyId) {
		var proxy = this.m_proxyPool[proxyId];
		if (proxyId == b2Pair.b2_nullProxy || proxy.IsValid() == false)
		{
			return null;
		}
		
		return proxy;
	}
b2BroadPhase.prototype.CreateProxy = function (aabb, userData) {
		var index = 0;
		var proxy;
		
		
		
		
		var proxyId = this.m_freeProxy;
		proxy = this.m_proxyPool[ proxyId ];
		this.m_freeProxy = proxy.GetNext();
		
		proxy.overlapCount = 0;
		proxy.userData = userData;
		
		var boundCount = 2 * this.m_proxyCount;
		
		var lowerValues = new Array();
		var upperValues = new Array();
		this.ComputeBounds(lowerValues, upperValues, aabb);
		
		for (var axis = 0; axis < 2; ++axis)
		{
			var bounds = this.m_bounds[axis];
			var lowerIndex = 0;
			var upperIndex = 0;
			var lowerIndexOut = [lowerIndex];
			var upperIndexOut = [upperIndex];
			this.Query(lowerIndexOut, upperIndexOut, lowerValues[axis], upperValues[axis], bounds, boundCount, axis);
			lowerIndex = lowerIndexOut[0];
			upperIndex = upperIndexOut[0];
			
			
			
			var tArr = new Array();
			var j = 0;
			var tEnd = boundCount - upperIndex
			var tBound1;
			var tBound2;
			var tBoundAS3;
			
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[parseInt(upperIndex+j)];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			tEnd = tArr.length;
			var tIndex = upperIndex+2;
			for (j = 0; j < tEnd; j++){
				
				tBound2 = tArr[j];
				tBound1 = bounds[parseInt(tIndex+j)]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			
			tArr = new Array();
			tEnd = upperIndex - lowerIndex;
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[parseInt(lowerIndex+j)];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			tEnd = tArr.length;
			tIndex = lowerIndex+1;
			for (j = 0; j < tEnd; j++){
				
				tBound2 = tArr[j];
				tBound1 = bounds[parseInt(tIndex+j)]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			
			++upperIndex;
			
			
			tBound1 = bounds[lowerIndex];
			tBound2 = bounds[upperIndex];
			tBound1.value = lowerValues[axis];
			tBound1.proxyId = proxyId;
			tBound2.value = upperValues[axis];
			tBound2.proxyId = proxyId;
			
			tBoundAS3 = bounds[parseInt(lowerIndex-1)];
			tBound1.stabbingCount = lowerIndex == 0 ? 0 : tBoundAS3.stabbingCount;
			tBoundAS3 = bounds[parseInt(upperIndex-1)];
			tBound2.stabbingCount = tBoundAS3.stabbingCount;
			
			
			for (index = lowerIndex; index < upperIndex; ++index)
			{
				tBoundAS3 = bounds[index];
				tBoundAS3.stabbingCount++;
			}
			
			
			for (index = lowerIndex; index < boundCount + 2; ++index)
			{
				tBound1 = bounds[index];
				var proxy2 = this.m_proxyPool[ tBound1.proxyId ];
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
		
		++this.m_proxyCount;
		
		
		
		for (var i = 0; i < this.m_queryResultCount; ++i)
		{
			
			
			
			this.m_pairManager.AddBufferedPair(proxyId, this.m_queryResults[i]);
		}
		
		this.m_pairManager.Commit();
		
		
		this.m_queryResultCount = 0;
		this.IncrementTimeStamp();
		
		return proxyId;
	}
b2BroadPhase.prototype.DestroyProxy = function (proxyId) {
		var tBound1;
		var tBound2;
		
		
		
		var proxy = this.m_proxyPool[ proxyId ];
		
		
		var boundCount = 2 * this.m_proxyCount;
		
		for (var axis = 0; axis < 2; ++axis)
		{
			var bounds = this.m_bounds[axis];
			
			var lowerIndex = proxy.lowerBounds[axis];
			var upperIndex = proxy.upperBounds[axis];
			tBound1 = bounds[lowerIndex];
			var lowerValue = tBound1.value;
			tBound2 = bounds[upperIndex];
			var upperValue = tBound2.value;
			
			
			
			var tArr = new Array();
			var j = 0;
			var tEnd = upperIndex - lowerIndex - 1;
			
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[parseInt(lowerIndex+1+j)];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			tEnd = tArr.length;
			var tIndex = lowerIndex;
			for (j = 0; j < tEnd; j++){
				
				tBound2 = tArr[j];
				tBound1 = bounds[parseInt(tIndex+j)]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			
			tArr = new Array();
			tEnd = boundCount - upperIndex - 1;
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[parseInt(upperIndex+1+j)];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			tEnd = tArr.length;
			tIndex = upperIndex-1;
			for (j = 0; j < tEnd; j++){
				
				tBound2 = tArr[j];
				tBound1 = bounds[parseInt(tIndex+j)]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			
			tEnd = boundCount - 2;
			for (var index = lowerIndex; index < tEnd; ++index)
			{
				tBound1 = bounds[index];
				var proxy2 = this.m_proxyPool[ tBound1.proxyId ];
				if (tBound1.IsLower())
				{
					proxy2.lowerBounds[axis] = index;
				}
				else
				{
					proxy2.upperBounds[axis] = index;
				}
			}
			
			
			tEnd = upperIndex - 1;
			for (var index2 = lowerIndex; index2 < tEnd; ++index2)
			{
				tBound1 = bounds[index2];
				tBound1.stabbingCount--;
			}
			
			
			
			this.Query([0], [0], lowerValue, upperValue, bounds, boundCount - 2, axis);
		}
		
		
		
		for (var i = 0; i < this.m_queryResultCount; ++i)
		{
			
			
			this.m_pairManager.RemoveBufferedPair(proxyId, this.m_queryResults[i]);
		}
		
		this.m_pairManager.Commit();
		
		
		this.m_queryResultCount = 0;
		this.IncrementTimeStamp();
		
		
		proxy.userData = null;
		proxy.overlapCount = b2BroadPhase.b2_invalid;
		proxy.lowerBounds[0] = b2BroadPhase.b2_invalid;
		proxy.lowerBounds[1] = b2BroadPhase.b2_invalid;
		proxy.upperBounds[0] = b2BroadPhase.b2_invalid;
		proxy.upperBounds[1] = b2BroadPhase.b2_invalid;
		
		proxy.SetNext(this.m_freeProxy);
		this.m_freeProxy = proxyId;
		--this.m_proxyCount;
	}
b2BroadPhase.prototype.MoveProxy = function (proxyId, aabb) {
		var as3arr;
		var as3int;
		
		var axis = 0;
		var index = 0;
		var bound;
		var prevBound;
		var nextBound;
		var nextProxyId = 0;
		var nextProxy;
		
		if (proxyId == b2Pair.b2_nullProxy || b2Settings.b2_maxProxies <= proxyId)
		{
			
			return;
		}
		
		if (aabb.IsValid() == false)
		{
			
			return;
		}
		
		var boundCount = 2 * this.m_proxyCount;
		
		var proxy = this.m_proxyPool[ proxyId ];
		
		var newValues = new b2BoundValues();
		this.ComputeBounds(newValues.lowerValues, newValues.upperValues, aabb);
		
		
		var oldValues = new b2BoundValues();
		for (axis = 0; axis < 2; ++axis)
		{
			bound = this.m_bounds[axis][proxy.lowerBounds[axis]];
			oldValues.lowerValues[axis] = bound.value;
			bound = this.m_bounds[axis][proxy.upperBounds[axis]];
			oldValues.upperValues[axis] = bound.value;
		}
		
		for (axis = 0; axis < 2; ++axis)
		{
			var bounds = this.m_bounds[axis];
			
			var lowerIndex = proxy.lowerBounds[axis];
			var upperIndex = proxy.upperBounds[axis];
			
			var lowerValue = newValues.lowerValues[axis];
			var upperValue = newValues.upperValues[axis];
			
			bound = bounds[lowerIndex];
			var deltaLower = lowerValue - bound.value;
			bound.value = lowerValue;
			
			bound = bounds[upperIndex];
			var deltaUpper = upperValue - bound.value;
			bound.value = upperValue;
			
			
			
			
			
			
			if (deltaLower < 0)
			{
				index = lowerIndex;
				while (index > 0 && lowerValue < (bounds[parseInt(index-1)]).value)
				{
					bound = bounds[index];
					prevBound = bounds[parseInt(index - 1)];
					
					var prevProxyId = prevBound.proxyId;
					var prevProxy = this.m_proxyPool[ prevBound.proxyId ];
					
					prevBound.stabbingCount++;
					
					if (prevBound.IsUpper() == true)
					{
						if (this.TestOverlap(newValues, prevProxy))
						{
							this.m_pairManager.AddBufferedPair(proxyId, prevProxyId);
						}
						
						
						as3arr = prevProxy.upperBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					else
					{
						
						as3arr = prevProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					
					
					as3arr = proxy.lowerBounds;
					as3int = as3arr[axis];
					as3int--;
					as3arr[axis] = as3int;
					
					
					
					
					
					bound.Swap(prevBound);
					
					--index;
				}
			}
			
			
			if (deltaUpper > 0)
			{
				index = upperIndex;
				while (index < boundCount-1 && (bounds[parseInt(index+1)]).value <= upperValue)
				{
					bound = bounds[ index ];
					nextBound = bounds[ parseInt(index + 1) ];
					nextProxyId = nextBound.proxyId;
					nextProxy = this.m_proxyPool[ nextProxyId ];
					
					nextBound.stabbingCount++;
					
					if (nextBound.IsLower() == true)
					{
						if (this.TestOverlap(newValues, nextProxy))
						{
							this.m_pairManager.AddBufferedPair(proxyId, nextProxyId);
						}
						
						
						as3arr = nextProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					else
					{
						
						as3arr = nextProxy.upperBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					
					
					as3arr = proxy.upperBounds;
					as3int = as3arr[axis];
					as3int++;
					as3arr[axis] = as3int;
					
					
					
					
					
					bound.Swap(nextBound);
					
					index++;
				}
			}
			
			
			
			
			
			
			if (deltaLower > 0)
			{
				index = lowerIndex;
				while (index < boundCount-1 && (bounds[parseInt(index+1)]).value <= lowerValue)
				{
					bound = bounds[ index ];
					nextBound = bounds[ parseInt(index + 1) ];
					
					nextProxyId = nextBound.proxyId;
					nextProxy = this.m_proxyPool[ nextProxyId ];
					
					nextBound.stabbingCount--;
					
					if (nextBound.IsUpper())
					{
						if (this.TestOverlap(oldValues, nextProxy))
						{
							this.m_pairManager.RemoveBufferedPair(proxyId, nextProxyId);
						}
						
						
						as3arr = nextProxy.upperBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					else
					{
						
						as3arr = nextProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int--;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					
					
					as3arr = proxy.lowerBounds;
					as3int = as3arr[axis];
					as3int++;
					as3arr[axis] = as3int;
					
					
					
					
					
					bound.Swap(nextBound);
					
					index++;
				}
			}
			
			
			if (deltaUpper < 0)
			{
				index = upperIndex;
				while (index > 0 && upperValue < (bounds[parseInt(index-1)]).value)
				{
					bound = bounds[index];
					prevBound = bounds[parseInt(index - 1)];
					
					prevProxyId = prevBound.proxyId;
					prevProxy = this.m_proxyPool[ prevProxyId ];
					
					prevBound.stabbingCount--;
					
					if (prevBound.IsLower() == true)
					{
						if (this.TestOverlap(oldValues, prevProxy))
						{
							this.m_pairManager.RemoveBufferedPair(proxyId, prevProxyId);
						}
						
						
						as3arr = prevProxy.lowerBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount--;
					}
					else
					{
						
						as3arr = prevProxy.upperBounds;
						as3int = as3arr[axis];
						as3int++;
						as3arr[axis] = as3int;
						
						bound.stabbingCount++;
					}
					
					
					as3arr = proxy.upperBounds;
					as3int = as3arr[axis];
					as3int--;
					as3arr[axis] = as3int;
					
					
					
					
					
					bound.Swap(prevBound);
					
					index--;
				}
			}
		}
	}
b2BroadPhase.prototype.Commit = function () {
		this.m_pairManager.Commit();
	}
b2BroadPhase.prototype.QueryAABB = function (aabb, userData, maxCount) {
		var lowerValues = new Array();
		var upperValues = new Array();
		this.ComputeBounds(lowerValues, upperValues, aabb);
		
		var lowerIndex = 0;
		var upperIndex = 0;
		var lowerIndexOut = [lowerIndex];
		var upperIndexOut = [upperIndex];
		this.Query(lowerIndexOut, upperIndexOut, lowerValues[0], upperValues[0], this.m_bounds[0], 2*this.m_proxyCount, 0);
		this.Query(lowerIndexOut, upperIndexOut, lowerValues[1], upperValues[1], this.m_bounds[1], 2*this.m_proxyCount, 1);
		
		
		
		var count = 0;
		for (var i = 0; i < this.m_queryResultCount && count < maxCount; ++i, ++count)
		{
			
			var proxy = this.m_proxyPool[ this.m_queryResults[i] ];
			
			userData[i] = proxy.userData;
		}
		
		
		this.m_queryResultCount = 0;
		this.IncrementTimeStamp();
		
		return count;
	}
b2BroadPhase.prototype.Validate = function () {
		var pair;
		var proxy1;
		var proxy2;
		var overlap;
		
		for (var axis = 0; axis < 2; ++axis)
		{
			var bounds = this.m_bounds[axis];
			
			var boundCount = 2 * this.m_proxyCount;
			var stabbingCount = 0;
			
			for (var i = 0; i < boundCount; ++i)
			{
				var bound = bounds[i];
				
				
				
				
				if (bound.IsLower() == true)
				{
					
					stabbingCount++;
				}
				else
				{
					
					stabbingCount--;
				}
				
				
			}
		}
		
	}
b2BroadPhase.prototype.TestOverlap = function (b, p) {
		for (var axis = 0; axis < 2; ++axis)
		{
			var bounds = this.m_bounds[axis];
			
			
			
			
			var bound = bounds[p.upperBounds[axis]];
			if (b.lowerValues[axis] > bound.value)
				return false;
			
			bound = bounds[p.lowerBounds[axis]];
			if (b.upperValues[axis] < bound.value)
				return false;
		}
		
		return true;
	}