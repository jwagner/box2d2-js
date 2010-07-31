var b2ContactID = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactID.prototype = {
    get key() {
        return this._key;
    },
    set key(value) {
		this._key = value;
		this.features._referenceEdge = this._key & 0x000000ff;
		this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
		this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
		this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
    }
}
b2ContactID.prototype.__constructor = function () {
		this.features._m_id = this;
}
b2ContactID.prototype.__varz = function(){
    this.features =  new Features();
}
// static attributes
// static methods
// attributes
b2ContactID.prototype.features =  new Features();
b2ContactID.prototype._key =  0;
// methods
b2ContactID.prototype.Set = function (id) {
    this.key = id._key;
}
b2ContactID.prototype.Copy = function () {
    var id = new b2ContactID();
    id.key = this._key;
    return id;
}
