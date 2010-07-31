var Features = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
Features.prototype = {
    get referenceEdge() {
        return this._referenceEdge;
    },
    set referenceEdge(value) {
		this._referenceEdge = value;
		this._m_id._key = (this._m_id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
    },
    get incidentEdge() {
        return this._incidentEdge;
    },
    set incidentEdge(value) {
		this._incidentEdge = value;
		this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
    },
    set incidentVertex(value) {
		this._incidentVertex = value;
		this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
    },
    get incidentVertex() {
        return this._incidentVertex;
    },
    set flip(value) {
		this._flip = value;
		this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
    },
    get flip() {
        return this._flip;
    }
}
Features.prototype.__constructor = function(){}
Features.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
Features.prototype._referenceEdge =  0;
Features.prototype._incidentEdge =  0;
Features.prototype._incidentVertex =  0;
Features.prototype._flip =  0;
Features.prototype._m_id =  null;
