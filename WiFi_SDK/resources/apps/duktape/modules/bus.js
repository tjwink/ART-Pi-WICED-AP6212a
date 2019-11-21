/*
 internal message bus via EventEmitter
 */

var EventEmitter = require('events').EventEmitter;
var util = require('util');

function Bus() {
    //TODO: [low] (nhat) - add some logging around the events or emits for debugging purpose?
    EventEmitter.call(this);
}

util.inherits(Bus, EventEmitter);


Bus.prototype.emit = function () {
    var arg = Array.prototype.slice.call(arguments, 0);
    EventEmitter.prototype.emit.apply(this, arg);
};


//TODO: [low] (nhat) - reference a 'config's hash of timestamp' for the constructor to protect the usage of this bus.
module.exports = exports = new Bus();