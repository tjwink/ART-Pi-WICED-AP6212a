var dpm = require('dpm')
var util = require('util'),
    EventEmitter = require('events').EventEmitter

function WebsocketConn(wsi) {
  this.handle = dpm.new_wss.call(this, wsi)

  EventEmitter.call(this)
  var self = this
  this.send = function(msg) {
      return dpm.wss_send(this.handle, msg)
  }
  this.close = function() {
      return dpm.wss_close(this.handle)
  }
  this.onMsg = function (msg) {
      self.emit('message', msg)
  }
  this.onClose = function () {
      self.emit('close')
  }
  dpm.wss_on_message(this.handle, this.onMsg)
  dpm.wss_on_close(this.handle, this.onClose)
}
util.inherits(WebsocketConn, EventEmitter)
module.exports = WebsocketConn
