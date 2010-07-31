var b2PairManager = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2PairManager.prototype.__constructor = function () {
		var i = 0;
		
		
		this.m_hashTable = new Array(b2Pair.b2_tableCapacity);
		for (i = 0; i < b2Pair.b2_tableCapacity; ++i)
		{
			this.m_hashTable[i] = b2Pair.b2_nullPair;
		}
		this.m_pairs = new Array(b2Settings.b2_maxPairs);
		for (i = 0; i < b2Settings.b2_maxPairs; ++i)
		{
			this.m_pairs[i] = new b2Pair();
		}
		this.m_pairBuffer = new Array(b2Settings.b2_maxPairs);
		for (i = 0; i < b2Settings.b2_maxPairs; ++i)
		{
			this.m_pairBuffer[i] = new b2BufferedPair();
		}
		
		for (i = 0; i < b2Settings.b2_maxPairs; ++i)
		{
			this.m_pairs[i].proxyId1 = b2Pair.b2_nullProxy;
			this.m_pairs[i].proxyId2 = b2Pair.b2_nullProxy;
			this.m_pairs[i].userData = null;
			this.m_pairs[i].status = 0;
			this.m_pairs[i].next = (i + 1);
		}
		this.m_pairs[parseInt(b2Settings.b2_maxPairs-1)].next = b2Pair.b2_nullPair;
		this.m_pairCount = 0;
		this.m_pairBufferCount = 0;
	}
b2PairManager.prototype.__varz = function(){
}
// static attributes
// static methods
b2PairManager.Hash = function (proxyId1, proxyId2) {
		var key = ((proxyId2 << 16) & 0xffff0000) | proxyId1;
		key = ~key + ((key << 15) & 0xFFFF8000);
		key = key ^ ((key >> 12) & 0x000fffff);
		key = key + ((key << 2) & 0xFFFFFFFC);
		key = key ^ ((key >> 4) & 0x0fffffff);
		key = key * 2057;
		key = key ^ ((key >> 16) % 65535);
		return key;
	}
b2PairManager.Equals = function (pair, proxyId1, proxyId2) {
		return (pair.proxyId1 == proxyId1 && pair.proxyId2 == proxyId2);
	}
b2PairManager.EqualsPair = function (pair1, pair2) {
		return pair1.proxyId1 == pair2.proxyId1 && pair1.proxyId2 == pair2.proxyId2;
	}
// attributes
b2PairManager.prototype.m_broadPhase =  null;
b2PairManager.prototype.m_callback =  null;
b2PairManager.prototype.m_pairs =  null;
b2PairManager.prototype.m_freePair =  0;
b2PairManager.prototype.m_pairCount =  0;
b2PairManager.prototype.m_pairBuffer =  null;
b2PairManager.prototype.m_pairBufferCount =  0;
b2PairManager.prototype.m_hashTable =  null;
// methods
b2PairManager.prototype.AddPair = function (proxyId1, proxyId2) {
		
		if (proxyId1 > proxyId2){
			var temp = proxyId1;
			proxyId1 = proxyId2;
			proxyId2 = temp;
			
		}
		
		var hash = b2PairManager.Hash(proxyId1, proxyId2) & b2Pair.b2_tableMask;
		
		
		var pair = pair = this.FindHash(proxyId1, proxyId2, hash);
		
		if (pair != null)
		{
			return pair;
		}
		
		
		
		var pIndex = this.m_freePair;
		pair = this.m_pairs[pIndex];
		this.m_freePair = pair.next;
		
		pair.proxyId1 = proxyId1;
		pair.proxyId2 = proxyId2;
		pair.status = 0;
		pair.userData = null;
		pair.next = this.m_hashTable[hash];
		
		this.m_hashTable[hash] = pIndex;
		
		++this.m_pairCount;
		
		return pair;
	}
b2PairManager.prototype.RemovePair = function (proxyId1, proxyId2) {
		var pair;
		
		
		
		if (proxyId1 > proxyId2){
			var temp = proxyId1;
			proxyId1 = proxyId2;
			proxyId2 = temp;
			
		}
		
		var hash = b2PairManager.Hash(proxyId1, proxyId2) & b2Pair.b2_tableMask;
		
		var node = this.m_hashTable[hash];
		var pNode = null;
		
		while (node != b2Pair.b2_nullPair)
		{
			if (b2PairManager.Equals(this.m_pairs[node], proxyId1, proxyId2))
			{
				var index = node;
				
				
				pair = this.m_pairs[node];
				if (pNode){
					pNode.next = pair.next;
				}
				else{
					this.m_hashTable[hash] = pair.next;
				}
				
				pair = this.m_pairs[ index ];
				var userData = pair.userData;
				
				
				pair.next = this.m_freePair;
				pair.proxyId1 = b2Pair.b2_nullProxy;
				pair.proxyId2 = b2Pair.b2_nullProxy;
				pair.userData = null;
				pair.status = 0;
				
				this.m_freePair = index;
				--this.m_pairCount;
				return userData;
			}
			else
			{
				
				pNode = this.m_pairs[node];
				node = pNode.next;
			}
		}
		
		
		return null;
	}
b2PairManager.prototype.Find = function (proxyId1, proxyId2) {
		
		if (proxyId1 > proxyId2){
			var temp = proxyId1;
			proxyId1 = proxyId2;
			proxyId2 = temp;
			
		}
		
		var hash = b2PairManager.Hash(proxyId1, proxyId2) & b2Pair.b2_tableMask;
		
		return this.FindHash(proxyId1, proxyId2, hash);
	}
b2PairManager.prototype.FindHash = function (proxyId1, proxyId2, hash) {
		var pair;
		var index = this.m_hashTable[hash];
		
		pair = this.m_pairs[index];
		while( index != b2Pair.b2_nullPair && b2PairManager.Equals(pair, proxyId1, proxyId2) == false)
		{
			index = pair.next;
			pair = this.m_pairs[index];
		}
		
		if ( index == b2Pair.b2_nullPair )
		{
			return null;
		}
		
		
		
		return pair;
	}
b2PairManager.prototype.ValidateBuffer = function () {
		
	}
b2PairManager.prototype.ValidateTable = function () {
		
	}
b2PairManager.prototype.Initialize = function (broadPhase, callback) {
		this.m_broadPhase = broadPhase;
		this.m_callback = callback;
	}
b2PairManager.prototype.AddBufferedPair = function (proxyId1, proxyId2) {
		var bufferedPair;
		
		
		
		var pair = this.AddPair(proxyId1, proxyId2);
		
		
		if (pair.IsBuffered() == false)
		{
			
			
			
			
			pair.SetBuffered();
			bufferedPair = this.m_pairBuffer[this.m_pairBufferCount];
			bufferedPair.proxyId1 = pair.proxyId1;
			bufferedPair.proxyId2 = pair.proxyId2;
			++this.m_pairBufferCount;
			
			
		}
		
		
		pair.ClearRemoved();
		
		if (b2BroadPhase.s_validate)
		{
			this.ValidateBuffer();
		}
	}
b2PairManager.prototype.RemoveBufferedPair = function (proxyId1, proxyId2) {
		var bufferedPair;
		
		
		
		
		var pair = this.Find(proxyId1, proxyId2);
		
		if (pair == null)
		{
			
			return;
		}
		
		
		if (pair.IsBuffered() == false)
		{
			
			
			
			pair.SetBuffered();
			bufferedPair = this.m_pairBuffer[this.m_pairBufferCount];
			bufferedPair.proxyId1 = pair.proxyId1;
			bufferedPair.proxyId2 = pair.proxyId2;
			++this.m_pairBufferCount;
			
			
		}
		
		pair.SetRemoved();
		
		if (b2BroadPhase.s_validate)
		{
			this.ValidateBuffer();
		}
	}
b2PairManager.prototype.Commit = function () {
		var bufferedPair;
		var i = 0;
		
		var removeCount = 0;
		
		var proxies = this.m_broadPhase.m_proxyPool;
		
		for (i = 0; i < this.m_pairBufferCount; ++i)
		{
			bufferedPair = this.m_pairBuffer[i];
			var pair = this.Find(bufferedPair.proxyId1, bufferedPair.proxyId2);
			
			pair.ClearBuffered();
			
			
			
			var proxy1 = proxies[ pair.proxyId1 ];
			var proxy2 = proxies[ pair.proxyId2 ];
			
			
			
			
			if (pair.IsRemoved())
			{
				
				
				
				if (pair.IsFinal() == true)
				{
					this.m_callback.PairRemoved(proxy1.userData, proxy2.userData, pair.userData);
				}
				
				
				bufferedPair = this.m_pairBuffer[removeCount];
				bufferedPair.proxyId1 = pair.proxyId1;
				bufferedPair.proxyId2 = pair.proxyId2;
				++removeCount;
			}
			else
			{
				
				
				if (pair.IsFinal() == false)
				{
					pair.userData = this.m_callback.PairAdded(proxy1.userData, proxy2.userData);
					pair.SetFinal();
				}
			}
		}
		
		for (i = 0; i < removeCount; ++i)
		{
			bufferedPair = this.m_pairBuffer[i]
			this.RemovePair(bufferedPair.proxyId1, bufferedPair.proxyId2);
		}
		
		this.m_pairBufferCount = 0;
		
		if (b2BroadPhase.s_validate)
		{
			this.ValidateTable();
		}	
	}