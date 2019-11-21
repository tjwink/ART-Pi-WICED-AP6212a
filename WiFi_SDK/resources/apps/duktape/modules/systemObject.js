var logger = new Duktape.Logger('SystemObj');
var bus = require('bus')

//wrapper for systemObject
function SystemObject(bus) {
    var self = this;
    bus.on('systemVolumeChanged', function (json) {
        logger.debug("systemVolumeChanged - " + JSON.stringify(json));
        if (json) {
            Object.defineProperty(self, "systemVolume", {
                value: json['volume'],
                writable: false,
                configurable: true
            });
        }
    });

    bus.on('muteChanged', function (json) {
        logger.debug("mutedChanged - " + JSON.stringify(json));
        if (json) {
            Object.defineProperty(self, "isMuted", {
                value: json['muted'],
                writable: false,
                configurable: true
            });
        }
    });

    bus.on('playingChanged', function (json) {
        logger.debug("playingChanged - " + JSON.stringify(json));
        if (json) {
            Object.defineProperty(self, "isPlaying", {
                value: json['playing'],
                writable: false,
                configurable: true
            });
        }
    });

    this.setMediaInfo = function (mediaInfo) {
        logger.debug("setMediaInfo - " + JSON.stringify(mediaInfo));
        bus.emit('mediaInfo', mediaInfo);
        return true;
    };

    this.shutdown = function () {
        logger.debug("shutdown");
        bus.emit('shutdown');
        return true;
    };

    this.close = function (socketID) {
        logger.debug("close - " + socketID);
        if (socketID) {
            bus.emit('ws.close', {"id": socketID});
            return true;
        }
        return false;
    };

    this.send = function (socketID, message) {
        logger.debug("send - " + socketID + " - " + JSON.stringify(message));

        if (socketID) {
            bus.emit('ws.send', {"id": socketID, "message": message});
            return true;
        }
        return false;
    };

    this.setMute = function (trueOrFalse) {
        logger.debug("setMute is called - " + trueOrFalse);
        Object.defineProperty(self, "isMuted", {
            value: trueOrFalse,
            writable: false,
            configurable: true
        });
    };

    //default properties

    Object.defineProperty(this, "systemVolume", {
        value: 0.5,
        writable: false,
        configurable: true
    });

    Object.defineProperty(this, "isMuted", {
        value: false,
        writable: false,
        configurable: true
    });

    Object.defineProperty(this, "isPlaying", {
        value: false,
        writable: false,
        configurable: true
    });
}

module.exports = new SystemObject(bus);