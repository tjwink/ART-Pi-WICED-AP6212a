var dpm = require('dpm');

function settimer(callback, after, periodic) {
    this.handle = dpm.new_timer.call(this);
    dpm.start_timer(this.handle, after, periodic, callback);
}

module.exports.setTimeout = function(callback, after) {
    return new settimer(callback, after, !1);
}

module.exports.clearTimeout = function (timer) {
    dpm.clear_timer(timer.handle)
}

module.exports.setInterval = function (callback, after) {
    return new settimer(callback, after, !0);
}

module.exports.clearInterval = function (timer) {
    dpm.clear_timer(timer.handle)
}
