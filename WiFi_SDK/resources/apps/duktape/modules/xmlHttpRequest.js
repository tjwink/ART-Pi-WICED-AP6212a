var dpm = require('dpm')

function xmlHttpRequest() {
  this.handle = dpm.new_xhr.call(this)

  var self = this
  this.status = 0
  this.responseText = null
  this.open = function(method, uri, async) {
      return dpm.xhr_open(this.handle, method, uri, async)
  }
  this.send = function(msg) {
      return dpm.xhr_send(this.handle, msg)
  }
  this.setRequestHeader = function(header, value) {
      return dpm.xhr_set_request_header(this.handle, header, value)
  }
  this.onLoad = function (status, msg) {
      self.status = status
      self.responseText = msg
      if (typeof self.onload === 'function') {
          self.onload.call(this)
      }
  }
  this.onError = function (errno) {
      if (typeof self.onerror === 'function') {
          self.onerror.call(this)
      }
  }
  dpm.xhr_on_load(this.handle, this.onLoad)
  dpm.xhr_on_error(this.handle, this.onError)
}
module.exports = xmlHttpRequest
