
/*
 //TODO: [high] (nhat) - remove this file since it's pretty convoluted and unnecessary.
 */
function SocketMap() {
    this.wsMap = {};
    this.idMap = {};
}

SocketMap.prototype.get = function (key) {
    var ret = {};
    var found;
    if (typeof key === "string") {
        found = this.idMap[key];
        if (!found) {
            return null;
        }
        ret['ws'] = found;
        ret['id'] = key;
    } else {
        found = this.wsMap[key];
        if (!found) {
            return null;
        }
        ret['id'] = found;
        ret['ws'] = key;
    }

    return ret;
};

SocketMap.prototype.remove = function (key) {
    var found = this.get(key);
    if (found) {
        delete this.wsMap[found.ws];
        delete this.idMap[found.id];
    }
};

SocketMap.prototype.markAsVerified = function (key) {
    var obj = this.get(key);
    if (obj) {
        obj['ws']['castchat_verified'] = true;
    }
};

SocketMap.prototype.isVerified = function (key) {
    var obj = this.get(key);
    if (obj) {
        return obj['ws']['castchat_verified'];
    }
    return false;
};


SocketMap.prototype.cleanup = function () {
    for(var k in this.idMap){
        var item = this.idMap[k];
        var obj = this.get(item);
        if(obj){
            //actually close the socket.
            obj.ws.close();
            this.remove(item);
        }
    }
};



/**
 *
 * @param {string} id
 * @param {Object} ws
 * @param {boolean} verified
 */
SocketMap.prototype.put = function (id, ws, verified) {
    if (typeof verified !== "undefined") {
        ws['castchat_verified'] = verified;
    }
    this.wsMap[ws] = id;
    this.idMap[id] = ws;
};

module.exports = SocketMap;