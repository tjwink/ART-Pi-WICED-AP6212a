var dpm = require('dpm')
var util = require('util'),
    EventEmitter = require('events').EventEmitter

function audio() {
  this.handle = dpm.new_audio.call(this)

  EventEmitter.call(this)
  var self = this
  this.load = function(msg) {
      return dpm.audio_load(this.handle)
  }
  this.play = function() {
      return dpm.audio_play(this.handle)
  }
  this.pause = function() {
      return dpm.audio_pause(this.handle)
  }
  function dpm_getter(key) {
      return dpm.audio_get_prop(this.handle, key)
  }
  function dpm_setter(val, key) {
      return dpm.audio_set_prop(this.handle, key, val)
  }
  Object.defineProperties(this, {
    duration: { enumerable: false, configurable: false, get: dpm_getter },
    currentTime: { enumerable: false, configurable: false, get: dpm_getter, set: dpm_setter },
    readyState: { enumerable: false, configurable: false, get: dpm_getter },
    ended: { enumerable: false, configurable: false, get: dpm_getter },
    paused: { enumerable: false, configurable: false, get: dpm_getter },
    src: { enumerable: false, configurable: false, get: dpm_getter },
    preload: { enumerable: false, configurable: false, get: dpm_getter },
    autoplay: { enumerable: false, configurable: false, get: dpm_getter },
    volume: { enumerable: false, configurable: false, get: dpm_getter, set: dpm_setter }
  });
  Object.defineProperty(this, 'seekable', {
    enumerable: false,
    configurable: false,
    get: function () {
        var dt = dpm.audio_get_prop(this.handle, 'duration')
        return {
            length: dt,
            start: function(i) {return 0;},
            end: function(i) {return dt;}
        };
    },
  });
  Object.defineProperty(this, 'played', {
    enumerable: false,
    configurable: false,
    get: function () {
        var ct = dpm.audio_get_prop(this.handle, 'currentTime')
        return {
            length: ct,
            start: function(i) {return 0;},
            end: function(i) {return ct;}
        };
    },
  });
  this.setAttribute = function(name, value) {
    return dpm.audio_attr(this.handle, name, value)
  }
  this.removeAttribute = function(name, value) {
    return dpm.audio_attr(this.handle, name, value)
  }
  this.onEvent = function (event, state) {
      self.emit(event, state)
  }
  dpm.audio_on_event(this.handle, this.onEvent)
}

util.inherits(audio, EventEmitter)
module.exports = audio
