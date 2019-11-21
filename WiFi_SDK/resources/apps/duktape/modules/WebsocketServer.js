var dpm = require('dpm')
function WebsocketServer(port) {
  this.bus = require('bus')
  this.handle = dpm.new_wss.call(this)

  var that = this
  var onConnection = function (wsi) {
      var conn = new (require('WebsocketConn'))(wsi)
      that.bus.emit('connection', conn)
  }
  dpm.wss_start(this.handle, port, onConnection)
}
module.exports = WebsocketServer
