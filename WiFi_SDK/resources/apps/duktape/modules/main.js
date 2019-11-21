var logger = new Duktape.Logger('main');
var configuration = require("config");
var uuid = require("uuid");
var WebsocketServer = require('WebsocketServer')
var SocketMap = require('socketmap');
var socketMap = new SocketMap();
var currentTrackMediaInfo = {};
var dpm = require('dpm');
var XMLHttpRequest = require('xmlHttpRequest')
var Audio = require('audio')

//global error handler
//process.on('uncaughtException', function (err) {
//  // handle the error safely
//  logger.warn(err);
//});

// set the log level
logger.l=configuration.logging.console.level

var systemObject = require('systemObject');

//queue of events to delay and retry until the app is in a ready state to process the events.
var eventqueue = [];
var applaunched = false;
var myTimeout = 0;

var notify = function () {
    var args = Array.prototype.slice.call(arguments, 0);
    var callback = args.shift();
    var fn = window[callback];
    if (fn) {
        logger.info("calling " + callback);
        fn.apply(window, args);
    } else {
        logger.warn("Receiver did not define " + callback + " along with args: " + JSON.stringify(args));
    }
};

//web-socket event
require('bus').on('connection', function (ws) {
//cchu  logger.info("WS connected", ws);
    logger.warn("WS connected", ws);
    var genId = uuid.v4();
    ws.id = genId;

    socketMap.put(genId, ws);

    notify('onconnected', genId);

    ws.on('close', function () {
//cchu      logger.info("WS closed");
        logger.warn("WS closed");
        var obj = socketMap.get(this.id);
        if (obj) {
            notify('ondisconnected', obj.id, "close");
            //remove it
            socketMap.remove(obj.id);
        }
    });

    ws.on('message', function (msg) {
        logger.info("WS message");
        var obj = socketMap.get(this.id);
        if (obj) {
//cchu          logger.info("MSG:\n", msg);
            logger.warn("MSG:\n", msg);
            notify('onmessage', obj.id, msg);
        }
    });

    ws.on('error', function (e) {
        logger.warn("error encountered: " + JSON.stringify(e));
        var obj = socketMap.get(this.id);
        if (obj) {
            notify('ondisconnected', obj.id, "error");
            //remove it
            socketMap.remove(this);
        }
    });
});

require('bus').on('ws.close', function (obj) {
    var id = obj.id;
    var found = socketMap.get(id);
    if (found) {
        found.ws.close(); //this will trigger close event. //TODO: [low] (nhat) - consider silent or force close?
        socketMap.remove(found.id);
    }
});

require('bus').on('ws.send', function (obj) {
    var id = obj.id;
    var found = socketMap.get(id);
    if (found) {
            found.ws.send(obj.message);
    }
});

//audio progress
require('bus').on('audio.progress', function (progress) {
    var percentPlayed = Math.ceil(progress.percentPlayed);
});

//TODO: [medium] (nhat) - this is an attempt to get the mute/unmute to work.  We might need to remove it.
var currentAudio = null;
//audio started
require('bus').on('audio.started', function (audio) {
//cchu  logger.info("audio started");
    logger.warn("audio started");
    if (currentAudio != null) {
        if (currentAudio.src !== audio.src) {
            currentAudio = audio;
        }
    }
});

require('bus').on('audio.ended', function (audio) {
//cchu  logger.info("audio ended");
    logger.warn("audio ended");
    if (currentAudio && currentAudio.src === audio.src) {
        currentAudio = null;
    }
});

require('bus').on('shutdown', function (dial) {
    logger.warn("===>>> Shutting down timers <<<===")
    dpm.timer_shutdown()
    logger.warn("===>>> Shutting down Audio <<<===")
    dpm.audio_shutdown()
    logger.warn("===>>> Cleanup Websocket connection <<<===")
    socketMap.cleanup()
    logger.warn("===>>> Shutting down Websocket Server <<<===")
    dpm.wss_shutdown()
    logger.warn("===>>> Shutting down XHR <<<===")
    dpm.xhr_shutdown()
});

require('bus').on('mediaInfo', function (media) {
    if (media == null) {
        currentTrackMediaInfo = {};
        document.querySelector("#progress").style.width = "100%";
    }
    if (media != null) {
        if (media.trackTitle) {
            currentTrackMediaInfo.trackName = media.trackTitle;
        }

        if (media.albumTitle) {
            currentTrackMediaInfo.albumName = media.albumTitle;
        }

        if (media.artistName) {
            currentTrackMediaInfo.artistName = media.artistName;
        }

        if (media.rating != null) {
            currentTrackMediaInfo.rating = media.rating;
        }

        if (media.artUrl) {
            currentTrackMediaInfo.artUrl = media.artUrl;
        }
    }
});

logger.warn("=== start Websocket Server ===");
WebsocketServer(configuration.dial.port)

if ( 'undefined' === typeof window ) {
    window = {
        console : logger
    }
}

// Timer functions
function settimer(callback, after, periodic) {
    this.handle = dpm.new_timer.call(this);
    dpm.start_timer(this.handle, after, periodic, callback);
}

function setTimeout(callback, after) {
    return new settimer(callback, after, !1);
}

function clearTimeout(timer) {
    dpm.clear_timer(timer.handle)
}

function setInterval(callback, after) {
    return new settimer(callback, after, !0);
}

function clearInterval(timer) {
    dpm.clear_timer(timer.handle)
}

// === beginning of pandora.js ===
var _data = {
    "or": "true",
    "entrypoint": "ce",
    "vendor": "cypress",
    "model": "test",
    "audio": false,
    "type": "STEREO_CONNECTED",
    "modelYear": "2016"
};

function hex_md5(r) {
    return rstr2hex(rstr_md5(str2rstr_utf8(r)))
}

function hex_hmac_md5(r, d) {
    return rstr2hex(rstr_hmac_md5(str2rstr_utf8(r), str2rstr_utf8(d)))
}

function md5_vm_test() {
    return "900150983cd24fb0d6963f7d28e17f72" == hex_md5("abc").toLowerCase()
}

function rstr_md5(r) {
    return binl2rstr(binl_md5(rstr2binl(r), 8 * r.length))
}

function rstr_hmac_md5(r, d) {
    var _ = rstr2binl(r);
    _.length > 16 && (_ = binl_md5(_, 8 * r.length));
    for (var m = Array(16), t = Array(16), n = 0; n < 16; n++) m[n] = 909522486 ^ _[n], t[n] = 1549556828 ^ _[n];
    var f = binl_md5(m.concat(rstr2binl(d)), 512 + 8 * d.length);
    return binl2rstr(binl_md5(t.concat(f), 640))
}

function rstr2hex(r) {
    try {} catch (d) {
        hexcase = 0
    }
    for (var _, m = hexcase ? "0123456789ABCDEF" : "0123456789abcdef", t = "", n = 0; n < r.length; n++) _ = r.charCodeAt(n), t += m.charAt(_ >>> 4 & 15) + m.charAt(15 & _);
    return t
}

function str2rstr_utf8(r) {
    for (var d, _, m = "", t = -1; ++t < r.length;) d = r.charCodeAt(t), _ = t + 1 < r.length ? r.charCodeAt(t + 1) : 0, 55296 <= d && d <= 56319 && 56320 <= _ && _ <= 57343 && (d = 65536 + ((1023 & d) << 10) + (1023 & _), t++), d <= 127 ? m += String.fromCharCode(d) : d <= 2047 ? m += String.fromCharCode(192 | d >>> 6 & 31, 128 | 63 & d) : d <= 65535 ? m += String.fromCharCode(224 | d >>> 12 & 15, 128 | d >>> 6 & 63, 128 | 63 & d) : d <= 2097151 && (m += String.fromCharCode(240 | d >>> 18 & 7, 128 | d >>> 12 & 63, 128 | d >>> 6 & 63, 128 | 63 & d));
    return m
}

function rstr2binl(r) {
    for (var d = Array(r.length >> 2), _ = 0; _ < d.length; _++) d[_] = 0;
    for (var _ = 0; _ < 8 * r.length; _ += 8) d[_ >> 5] |= (255 & r.charCodeAt(_ / 8)) << _ % 32;
    return d
}

function binl2rstr(r) {
    for (var d = "", _ = 0; _ < 32 * r.length; _ += 8) d += String.fromCharCode(r[_ >> 5] >>> _ % 32 & 255);
    return d
}

function binl_md5(r, d) {
    r[d >> 5] |= 128 << d % 32, r[(d + 64 >>> 9 << 4) + 14] = d;
    for (var _ = 1732584193, m = -271733879, t = -1732584194, n = 271733878, f = 0; f < r.length; f += 16) {
        var h = _,
            i = m,
            e = t,
            a = n;
        _ = md5_ff(_, m, t, n, r[f + 0], 7, -680876936), n = md5_ff(n, _, m, t, r[f + 1], 12, -389564586), t = md5_ff(t, n, _, m, r[f + 2], 17, 606105819), m = md5_ff(m, t, n, _, r[f + 3], 22, -1044525330), _ = md5_ff(_, m, t, n, r[f + 4], 7, -176418897), n = md5_ff(n, _, m, t, r[f + 5], 12, 1200080426), t = md5_ff(t, n, _, m, r[f + 6], 17, -1473231341), m = md5_ff(m, t, n, _, r[f + 7], 22, -45705983), _ = md5_ff(_, m, t, n, r[f + 8], 7, 1770035416), n = md5_ff(n, _, m, t, r[f + 9], 12, -1958414417), t = md5_ff(t, n, _, m, r[f + 10], 17, -42063), m = md5_ff(m, t, n, _, r[f + 11], 22, -1990404162), _ = md5_ff(_, m, t, n, r[f + 12], 7, 1804603682), n = md5_ff(n, _, m, t, r[f + 13], 12, -40341101), t = md5_ff(t, n, _, m, r[f + 14], 17, -1502002290), m = md5_ff(m, t, n, _, r[f + 15], 22, 1236535329), _ = md5_gg(_, m, t, n, r[f + 1], 5, -165796510), n = md5_gg(n, _, m, t, r[f + 6], 9, -1069501632), t = md5_gg(t, n, _, m, r[f + 11], 14, 643717713), m = md5_gg(m, t, n, _, r[f + 0], 20, -373897302), _ = md5_gg(_, m, t, n, r[f + 5], 5, -701558691), n = md5_gg(n, _, m, t, r[f + 10], 9, 38016083), t = md5_gg(t, n, _, m, r[f + 15], 14, -660478335), m = md5_gg(m, t, n, _, r[f + 4], 20, -405537848), _ = md5_gg(_, m, t, n, r[f + 9], 5, 568446438), n = md5_gg(n, _, m, t, r[f + 14], 9, -1019803690), t = md5_gg(t, n, _, m, r[f + 3], 14, -187363961), m = md5_gg(m, t, n, _, r[f + 8], 20, 1163531501), _ = md5_gg(_, m, t, n, r[f + 13], 5, -1444681467), n = md5_gg(n, _, m, t, r[f + 2], 9, -51403784), t = md5_gg(t, n, _, m, r[f + 7], 14, 1735328473), m = md5_gg(m, t, n, _, r[f + 12], 20, -1926607734), _ = md5_hh(_, m, t, n, r[f + 5], 4, -378558), n = md5_hh(n, _, m, t, r[f + 8], 11, -2022574463), t = md5_hh(t, n, _, m, r[f + 11], 16, 1839030562), m = md5_hh(m, t, n, _, r[f + 14], 23, -35309556), _ = md5_hh(_, m, t, n, r[f + 1], 4, -1530992060), n = md5_hh(n, _, m, t, r[f + 4], 11, 1272893353), t = md5_hh(t, n, _, m, r[f + 7], 16, -155497632), m = md5_hh(m, t, n, _, r[f + 10], 23, -1094730640), _ = md5_hh(_, m, t, n, r[f + 13], 4, 681279174), n = md5_hh(n, _, m, t, r[f + 0], 11, -358537222), t = md5_hh(t, n, _, m, r[f + 3], 16, -722521979), m = md5_hh(m, t, n, _, r[f + 6], 23, 76029189), _ = md5_hh(_, m, t, n, r[f + 9], 4, -640364487), n = md5_hh(n, _, m, t, r[f + 12], 11, -421815835), t = md5_hh(t, n, _, m, r[f + 15], 16, 530742520), m = md5_hh(m, t, n, _, r[f + 2], 23, -995338651), _ = md5_ii(_, m, t, n, r[f + 0], 6, -198630844), n = md5_ii(n, _, m, t, r[f + 7], 10, 1126891415), t = md5_ii(t, n, _, m, r[f + 14], 15, -1416354905), m = md5_ii(m, t, n, _, r[f + 5], 21, -57434055), _ = md5_ii(_, m, t, n, r[f + 12], 6, 1700485571), n = md5_ii(n, _, m, t, r[f + 3], 10, -1894986606), t = md5_ii(t, n, _, m, r[f + 10], 15, -1051523), m = md5_ii(m, t, n, _, r[f + 1], 21, -2054922799), _ = md5_ii(_, m, t, n, r[f + 8], 6, 1873313359), n = md5_ii(n, _, m, t, r[f + 15], 10, -30611744), t = md5_ii(t, n, _, m, r[f + 6], 15, -1560198380), m = md5_ii(m, t, n, _, r[f + 13], 21, 1309151649), _ = md5_ii(_, m, t, n, r[f + 4], 6, -145523070), n = md5_ii(n, _, m, t, r[f + 11], 10, -1120210379), t = md5_ii(t, n, _, m, r[f + 2], 15, 718787259), m = md5_ii(m, t, n, _, r[f + 9], 21, -343485551), _ = safe_add(_, h), m = safe_add(m, i), t = safe_add(t, e), n = safe_add(n, a)
    }
    return Array(_, m, t, n)
}

function md5_cmn(r, d, _, m, t, n) {
    return safe_add(bit_rol(safe_add(safe_add(d, r), safe_add(m, n)), t), _)
}

function md5_ff(r, d, _, m, t, n, f) {
    return md5_cmn(d & _ | ~d & m, r, d, t, n, f)
}

function md5_gg(r, d, _, m, t, n, f) {
    return md5_cmn(d & m | _ & ~m, r, d, t, n, f)
}

function md5_hh(r, d, _, m, t, n, f) {
    return md5_cmn(d ^ _ ^ m, r, d, t, n, f)
}

function md5_ii(r, d, _, m, t, n, f) {
    return md5_cmn(_ ^ (d | ~m), r, d, t, n, f)
}

function safe_add(r, d) {
    var _ = (65535 & r) + (65535 & d),
        m = (r >> 16) + (d >> 16) + (_ >> 16);
    return m << 16 | 65535 & _
}

function bit_rol(r, d) {
    return r << d | r >>> 32 - d
}
var hexcase = 0;
var _ = function() {
    function n(t, r, e) {
        if (t === r) return 0 !== t || 1 / t == 1 / r;
        if (null == t || null == r) return t === r;
        if (t._chain && (t = t._wrapped), r._chain && (r = r._wrapped), t.isEqual && A.isFunction(t.isEqual)) return t.isEqual(r);
        if (r.isEqual && A.isFunction(r.isEqual)) return r.isEqual(t);
        var u = l.call(t);
        if (u != l.call(r)) return !1;
        switch (u) {
            case "[object String]":
                return t == String(r);
            case "[object Number]":
                return t != +t ? r != +r : 0 == t ? 1 / t == 1 / r : t == +r;
            case "[object Date]":
            case "[object Boolean]":
                return +t == +r;
            case "[object RegExp]":
                return t.source == r.source && t.global == r.global && t.multiline == r.multiline && t.ignoreCase == r.ignoreCase
        }
        if ("object" != typeof t || "object" != typeof r) return !1;
        for (var i = e.length; i--;)
            if (e[i] == t) return !0;
        e.push(t);
        var c = 0,
            a = !0;
        if ("[object Array]" == u) {
            if (c = t.length, a = c == r.length)
                for (; c-- && (a = c in t == c in r && n(t[c], r[c], e)););
        } else {
            if ("constructor" in t != "constructor" in r || t.constructor != r.constructor) return !1;
            for (var o in t)
                if (A.has(t, o) && (c++, !(a = A.has(r, o) && n(t[o], r[o], e)))) break;
            if (a) {
                for (o in r)
                    if (A.has(r, o) && !c--) break;
                a = !c
            }
        }
        return e.pop(), a
    }
    var t = this,
        r = t._,
        e = {},
        u = Array.prototype,
        i = Object.prototype,
        c = Function.prototype,
        a = u.slice,
        o = u.unshift,
        l = i.toString,
        f = i.hasOwnProperty,
        p = u.forEach,
        s = u.map,
        h = u.reduce,
        v = u.reduceRight,
        d = u.filter,
        y = u.every,
        g = u.some,
        m = u.indexOf,
        b = u.lastIndexOf,
        _ = Array.isArray,
        j = Object.keys,
        x = c.bind,
        A = function(n) {
            return new R(n)
        };
    t._ = A, A.VERSION = "1.3.1";
    var w = A.each = A.forEach = function(n, t, r) {
        if (null != n)
            if (p && n.forEach === p) n.forEach(t, r);
            else if (n.length === +n.length) {
            for (var u = 0, i = n.length; u < i; u++)
                if (u in n && t.call(r, n[u], u, n) === e) return
        } else
            for (var c in n)
                if (A.has(n, c) && t.call(r, n[c], c, n) === e) return
    };
    A.map = A.collect = function(n, t, r) {
        var e = [];
        return null == n ? e : s && n.map === s ? n.map(t, r) : (w(n, function(n, u, i) {
            e[e.length] = t.call(r, n, u, i)
        }), n.length === +n.length && (e.length = n.length), e)
    }, A.reduce = A.foldl = A.inject = function(n, t, r, e) {
        var u = arguments.length > 2;
        if (null == n && (n = []), h && n.reduce === h) return e && (t = A.bind(t, e)), u ? n.reduce(t, r) : n.reduce(t);
        if (w(n, function(n, i, c) {
                u ? r = t.call(e, r, n, i, c) : (r = n, u = !0)
            }), !u) throw new TypeError("Reduce of empty array with no initial value");
        return r
    }, A.reduceRight = A.foldr = function(n, t, r, e) {
        var u = arguments.length > 2;
        if (null == n && (n = []), v && n.reduceRight === v) return e && (t = A.bind(t, e)), u ? n.reduceRight(t, r) : n.reduceRight(t);
        var i = A.toArray(n).reverse();
        return e && !u && (t = A.bind(t, e)), u ? A.reduce(i, t, r, e) : A.reduce(i, t)
    }, A.find = A.detect = function(n, t, r) {
        var e;
        return E(n, function(n, u, i) {
            if (t.call(r, n, u, i)) return e = n, !0
        }), e
    }, A.filter = A.select = function(n, t, r) {
        var e = [];
        return null == n ? e : d && n.filter === d ? n.filter(t, r) : (w(n, function(n, u, i) {
            t.call(r, n, u, i) && (e[e.length] = n)
        }), e)
    }, A.reject = function(n, t, r) {
        var e = [];
        return null == n ? e : (w(n, function(n, u, i) {
            t.call(r, n, u, i) || (e[e.length] = n)
        }), e)
    }, A.every = A.all = function(n, t, r) {
        var u = !0;
        return null == n ? u : y && n.every === y ? n.every(t, r) : (w(n, function(n, i, c) {
            if (!(u = u && t.call(r, n, i, c))) return e
        }), u)
    };
    var E = A.some = A.any = function(n, t, r) {
        t || (t = A.identity);
        var u = !1;
        return null == n ? u : g && n.some === g ? n.some(t, r) : (w(n, function(n, i, c) {
            if (u || (u = t.call(r, n, i, c))) return e
        }), !!u)
    };
    A.include = A.contains = function(n, t) {
        var r = !1;
        return null == n ? r : m && n.indexOf === m ? n.indexOf(t) != -1 : r = E(n, function(n) {
            return n === t
        })
    }, A.invoke = function(n, t) {
        var r = a.call(arguments, 2);
        return A.map(n, function(n) {
            return (A.isFunction(t) ? t || n : n[t]).apply(n, r)
        })
    }, A.pluck = function(n, t) {
        return A.map(n, function(n) {
            return n[t]
        })
    }, A.max = function(n, t, r) {
        if (!t && A.isArray(n)) return Math.max.apply(Math, n);
        if (!t && A.isEmpty(n)) return -(1 / 0);
        var e = {
            computed: -(1 / 0)
        };
        return w(n, function(n, u, i) {
            var c = t ? t.call(r, n, u, i) : n;
            c >= e.computed && (e = {
                value: n,
                computed: c
            })
        }), e.value
    }, A.min = function(n, t, r) {
        if (!t && A.isArray(n)) return Math.min.apply(Math, n);
        if (!t && A.isEmpty(n)) return 1 / 0;
        var e = {
            computed: 1 / 0
        };
        return w(n, function(n, u, i) {
            var c = t ? t.call(r, n, u, i) : n;
            c < e.computed && (e = {
                value: n,
                computed: c
            })
        }), e.value
    }, A.shuffle = function(n) {
        var t, r = [];
        return w(n, function(n, e, u) {
            0 == e ? r[0] = n : (t = Math.floor(Math.random() * (e + 1)), r[e] = r[t], r[t] = n)
        }), r
    }, A.sortBy = function(n, t, r) {
        return A.pluck(A.map(n, function(n, e, u) {
            return {
                value: n,
                criteria: t.call(r, n, e, u)
            }
        }).sort(function(n, t) {
            var r = n.criteria,
                e = t.criteria;
            return r < e ? -1 : r > e ? 1 : 0
        }), "value")
    }, A.groupBy = function(n, t) {
        var r = {},
            e = A.isFunction(t) ? t : function(n) {
                return n[t]
            };
        return w(n, function(n, t) {
            var u = e(n, t);
            (r[u] || (r[u] = [])).push(n)
        }), r
    }, A.sortedIndex = function(n, t, r) {
        r || (r = A.identity);
        for (var e = 0, u = n.length; e < u;) {
            var i = e + u >> 1;
            r(n[i]) < r(t) ? e = i + 1 : u = i
        }
        return e
    }, A.toArray = function(n) {
        return n ? n.toArray ? n.toArray() : A.isArray(n) ? a.call(n) : A.isArguments(n) ? a.call(n) : A.values(n) : []
    }, A.size = function(n) {
        return A.toArray(n).length
    }, A.first = A.head = function(n, t, r) {
        return null == t || r ? n[0] : a.call(n, 0, t)
    }, A.initial = function(n, t, r) {
        return a.call(n, 0, n.length - (null == t || r ? 1 : t))
    }, A.last = function(n, t, r) {
        return null == t || r ? n[n.length - 1] : a.call(n, Math.max(n.length - t, 0))
    }, A.rest = A.tail = function(n, t, r) {
        return a.call(n, null == t || r ? 1 : t)
    }, A.compact = function(n) {
        return A.filter(n, function(n) {
            return !!n
        })
    }, A.flatten = function(n, t) {
        return A.reduce(n, function(n, r) {
            return A.isArray(r) ? n.concat(t ? r : A.flatten(r)) : (n[n.length] = r, n)
        }, [])
    }, A.without = function(n) {
        return A.difference(n, a.call(arguments, 1))
    }, A.uniq = A.unique = function(n, t, r) {
        var e = r ? A.map(n, r) : n,
            u = [];
        return A.reduce(e, function(r, e, i) {
            return 0 != i && (t === !0 ? A.last(r) == e : A.include(r, e)) || (r[r.length] = e, u[u.length] = n[i]), r
        }, []), u
    }, A.union = function() {
        return A.uniq(A.flatten(arguments, !0))
    }, A.intersection = A.intersect = function(n) {
        var t = a.call(arguments, 1);
        return A.filter(A.uniq(n), function(n) {
            return A.every(t, function(t) {
                return A.indexOf(t, n) >= 0
            })
        })
    }, A.difference = function(n) {
        var t = A.flatten(a.call(arguments, 1));
        return A.filter(n, function(n) {
            return !A.include(t, n)
        })
    }, A.zip = function() {
        for (var n = a.call(arguments), t = A.max(A.pluck(n, "length")), r = new Array(t), e = 0; e < t; e++) r[e] = A.pluck(n, "" + e);
        return r
    }, A.indexOf = function(n, t, r) {
        if (null == n) return -1;
        var e, u;
        if (r) return e = A.sortedIndex(n, t), n[e] === t ? e : -1;
        if (m && n.indexOf === m) return n.indexOf(t);
        for (e = 0, u = n.length; e < u; e++)
            if (e in n && n[e] === t) return e;
        return -1
    }, A.lastIndexOf = function(n, t) {
        if (null == n) return -1;
        if (b && n.lastIndexOf === b) return n.lastIndexOf(t);
        for (var r = n.length; r--;)
            if (r in n && n[r] === t) return r;
        return -1
    }, A.range = function(n, t, r) {
        arguments.length <= 1 && (t = n || 0, n = 0), r = arguments[2] || 1;
        for (var e = Math.max(Math.ceil((t - n) / r), 0), u = 0, i = new Array(e); u < e;) i[u++] = n, n += r;
        return i
    };
    var O = function() {};
    A.bind = function(n, t) {
        var r, e;
        if (n.bind === x && x) return x.apply(n, a.call(arguments, 1));
        if (!A.isFunction(n)) throw new TypeError;
        return e = a.call(arguments, 2), r = function() {
            if (!(this instanceof r)) return n.apply(t, e.concat(a.call(arguments)));
            O.prototype = n.prototype;
            var u = new O,
                i = n.apply(u, e.concat(a.call(arguments)));
            return Object(i) === i ? i : u
        }
    }, A.bindAll = function(n) {
        var t = a.call(arguments, 1);
        return 0 == t.length && (t = A.functions(n)), w(t, function(t) {
            n[t] = A.bind(n[t], n)
        }), n
    }, A.memoize = function(n, t) {
        var r = {};
        return t || (t = A.identity),
            function() {
                var e = t.apply(this, arguments);
                return A.has(r, e) ? r[e] : r[e] = n.apply(this, arguments)
            }
    }, A.delay = function(n, t) {
        var r = a.call(arguments, 2);
        return setTimeout(function() {
            return n.apply(n, r)
        }, t)
    }, A.defer = function(n) {
        return A.delay.apply(A, [n, 1].concat(a.call(arguments, 1)))
    }, A.throttle = function(n, t) {
        var r, e, u, i, c, a = A.debounce(function() {
            c = i = !1
        }, t);
        return function() {
            r = this, e = arguments;
            var o = function() {
                u = null, c && n.apply(r, e), a()
            };
            u || (u = setTimeout(o, t)), i ? c = !0 : n.apply(r, e), a(), i = !0
        }
    }, A.debounce = function(n, t) {
        var r;
        return function() {
            var e = this,
                u = arguments,
                i = function() {
                    r = null, n.apply(e, u)
                };
            clearTimeout(r), r = setTimeout(i, t)
        }
    }, A.once = function(n) {
        var t, r = !1;
        return function() {
            return r ? t : (r = !0, t = n.apply(this, arguments))
        }
    }, A.wrap = function(n, t) {
        return function() {
            var r = [n].concat(a.call(arguments, 0));
            return t.apply(this, r)
        }
    }, A.compose = function() {
        var n = arguments;
        return function() {
            for (var t = arguments, r = n.length - 1; r >= 0; r--) t = [n[r].apply(this, t)];
            return t[0]
        }
    }, A.after = function(n, t) {
        return n <= 0 ? t() : function() {
            if (--n < 1) return t.apply(this, arguments)
        }
    }, A.keys = j || function(n) {
        if (n !== Object(n)) throw new TypeError("Invalid object");
        var t = [];
        for (var r in n) A.has(n, r) && (t[t.length] = r);
        return t
    }, A.values = function(n) {
        return A.map(n, A.identity)
    }, A.functions = A.methods = function(n) {
        var t = [];
        for (var r in n) A.isFunction(n[r]) && t.push(r);
        return t.sort()
    }, A.extend = function(n) {
        return w(a.call(arguments, 1), function(t) {
            for (var r in t) n[r] = t[r]
        }), n
    }, A.defaults = function(n) {
        return w(a.call(arguments, 1), function(t) {
            for (var r in t) null == n[r] && (n[r] = t[r])
        }), n
    }, A.clone = function(n) {
        return A.isObject(n) ? A.isArray(n) ? n.slice() : A.extend({}, n) : n
    }, A.tap = function(n, t) {
        return t(n), n
    }, A.isEqual = function(t, r) {
        return n(t, r, [])
    }, A.isEmpty = function(n) {
        if (A.isArray(n) || A.isString(n)) return 0 === n.length;
        for (var t in n)
            if (A.has(n, t)) return !1;
        return !0
    }, A.isElement = function(n) {
        return !(!n || 1 != n.nodeType)
    }, A.isArray = _ || function(n) {
        return "[object Array]" == l.call(n)
    }, A.isObject = function(n) {
        return n === Object(n)
    }, A.isArguments = function(n) {
        return "[object Arguments]" == l.call(n)
    }, A.isArguments(arguments) || (A.isArguments = function(n) {
        return !(!n || !A.has(n, "callee"))
    }), A.isFunction = function(n) {
        return "[object Function]" == l.call(n)
    }, A.isString = function(n) {
        return "[object String]" == l.call(n)
    }, A.isNumber = function(n) {
        return "[object Number]" == l.call(n)
    }, A.isNaN = function(n) {
        return n !== n
    }, A.isBoolean = function(n) {
        return n === !0 || n === !1 || "[object Boolean]" == l.call(n)
    }, A.isDate = function(n) {
        return "[object Date]" == l.call(n)
    }, A.isRegExp = function(n) {
        return "[object RegExp]" == l.call(n)
    }, A.isNull = function(n) {
        return null === n
    }, A.isUndefined = function(n) {
        return void 0 === n
    }, A.has = function(n, t) {
        return f.call(n, t)
    }, A.noConflict = function() {
        return t._ = r, this
    }, A.identity = function(n) {
        return n
    }, A.times = function(n, t, r) {
        for (var e = 0; e < n; e++) t.call(r, e)
    }, A.escape = function(n) {
        return ("" + n).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;").replace(/'/g, "&#x27;").replace(/\//g, "&#x2F;")
    }, A.mixin = function(n) {
        w(A.functions(n), function(t) {
            I(t, A[t] = n[t])
        })
    };
    var q = 0;
    A.uniqueId = function(n) {
        var t = q++;
        return n ? n + t : t
    }, A.templateSettings = {
        evaluate: /<%([\s\S]+?)%>/g,
        interpolate: /<%=([\s\S]+?)%>/g,
        escape: /<%-([\s\S]+?)%>/g
    };
    var S = /.^/,
        F = function(n) {
            return n.replace(/\\\\/g, "\\").replace(/\\'/g, "'")
        };
    A.template = function(n, t) {
        var r = A.templateSettings,
            e = "var __p=[],print=function(){__p.push.apply(__p,arguments);};with(obj||{}){__p.push('" + n.replace(/\\/g, "\\\\").replace(/'/g, "\\'").replace(r.escape || S, function(n, t) {
                return "',_.escape(" + F(t) + "),'"
            }).replace(r.interpolate || S, function(n, t) {
                return "'," + F(t) + ",'"
            }).replace(r.evaluate || S, function(n, t) {
                return "');" + F(t).replace(/[\r\n\t]/g, " ") + ";__p.push('"
            }).replace(/\r/g, "\\r").replace(/\n/g, "\\n").replace(/\t/g, "\\t") + "');}return __p.join('');",
            u = new Function("obj", "_", e);
        return t ? u(t, A) : function(n) {
            return u.call(this, n, A)
        }
    }, A.chain = function(n) {
        return A(n).chain()
    };
    var R = function(n) {
        this._wrapped = n
    };
    A.prototype = R.prototype;
    var k = function(n, t) {
            return t ? A(n).chain() : n
        },
        I = function(n, t) {
            R.prototype[n] = function() {
                var n = a.call(arguments);
                return o.call(n, this._wrapped), k(t.apply(A, n), this._chain)
            }
        };
    return A.mixin(A), w(["pop", "push", "reverse", "shift", "sort", "splice", "unshift"], function(n) {
        var t = u[n];
        R.prototype[n] = function() {
            var r = this._wrapped;
            t.apply(r, arguments);
            var e = r.length;
            return "shift" != n && "splice" != n || 0 !== e || delete r[0], k(r, this._chain)
        }
    }), w(["concat", "join", "slice"], function(n) {
        var t = u[n];
        R.prototype[n] = function() {
            return k(t.apply(this._wrapped, arguments), this._chain)
        }
    }), R.prototype.chain = function() {
        return this._chain = !0, this
    }, R.prototype.value = function() {
        return this._wrapped
    }, A
}.call(this);
var Backbone = function() {
    var t, e = this;
    t = e.Backbone = {}, t.VERSION = "0.5.3", t.Events = {
        bind: function(t, e, i) {
            var r = this._callbacks || (this._callbacks = {}),
                s = r[t] || (r[t] = []);
            return s.push([e, i]), this
        },
        unbind: function(t, e) {
            var i;
            if (t) {
                if (i = this._callbacks)
                    if (e) {
                        var r = i[t];
                        if (!r) return this;
                        for (var s = 0, n = r.length; s < n; s++)
                            if (r[s] && e === r[s][0]) {
                                r[s] = null;
                                break
                            }
                    } else i[t] = []
            } else this._callbacks = {};
            return this
        },
        trigger: function(t) {
            var e, i, r, s, n, o = 2;
            if (!(i = this._callbacks)) return this;
            for (; o--;)
                if (r = o ? t : "all", e = i[r])
                    for (var h = 0, a = e.length; h < a; h++)(s = e[h]) ? (n = o ? Array.prototype.slice.call(arguments, 1) : arguments, s[0].apply(s[1] || this, n)) : (e.splice(h, 1), h--, a--);
            return this
        }
    }, t.Model = function(t, e) {
        var i;
        t || (t = {}), (i = this.defaults) && (_.isFunction(i) && (i = i.call(this)), t = _.extend({}, i, t)), this.attributes = {}, this._escapedAttributes = {}, this.cid = _.uniqueId("c"), this.set(t, {
            silent: !0
        }), this._changed = !1, this._previousAttributes = _.clone(this.attributes), e && e.collection && (this.collection = e.collection), this.initialize(t, e)
    }, _.extend(t.Model.prototype, t.Events, {
        _previousAttributes: null,
        _changed: !1,
        idAttribute: "id",
        initialize: function() {},
        toJSON: function() {
            return _.clone(this.attributes)
        },
        get: function(t) {
            return this.attributes[t]
        },
        escape: function(t) {
            var e;
            if (e = this._escapedAttributes[t]) return e;
            var i = this.attributes[t];
            return this._escapedAttributes[t] = h(null == i ? "" : "" + i)
        },
        has: function(t) {
            return null != this.attributes[t]
        },
        set: function(t, e) {
            if (e || (e = {}), !t) return this;
            t.attributes && (t = t.attributes);
            var i = this.attributes,
                r = this._escapedAttributes;
            if (!e.silent && this.validate && !this._performValidation(t, e)) return !1;
            this.idAttribute in t && (this.id = t[this.idAttribute]);
            var s = this._changing;
            this._changing = !0;
            for (var n in t) {
                var o = t[n];
                _.isEqual(i[n], o) || (i[n] = o, delete r[n], this._changed = !0, e.silent || this.trigger("change:" + n, this, o, e))
            }
            return s || e.silent || !this._changed || this.change(e), this._changing = !1, this
        },
        unset: function(t, e) {
            if (!(t in this.attributes)) return this;
            e || (e = {});
            var i = (this.attributes[t], {});
            return i[t] = void 0, !(!e.silent && this.validate && !this._performValidation(i, e)) && (delete this.attributes[t], delete this._escapedAttributes[t], t == this.idAttribute && delete this.id, this._changed = !0, e.silent || (this.trigger("change:" + t, this, void 0, e), this.change(e)), this)
        },
        clear: function(t) {
            t || (t = {});
            var e, i = this.attributes,
                r = {};
            for (e in i) r[e] = void 0;
            if (!t.silent && this.validate && !this._performValidation(r, t)) return !1;
            if (this.attributes = {}, this._escapedAttributes = {}, this._changed = !0, !t.silent) {
                for (e in i) this.trigger("change:" + e, this, void 0, t);
                this.change(t)
            }
            return this
        },
        destroy: function(e) {
            if (e || (e = {}), this.isNew()) return this.trigger("destroy", this, this.collection, e);
            var i = this,
                r = e.success;
            return e.success = function(t) {
                i.trigger("destroy", i, i.collection, e), r && r(i, t)
            }, e.error = o(e.error, i, e), (this.sync || t.sync).call(this, "delete", this, e)
        },
        clone: function() {
            return new this.constructor(this)
        },
        isNew: function() {
            return null == this.id
        },
        change: function(t) {
            this.trigger("change", this, t), this._previousAttributes = _.clone(this.attributes), this._changed = !1
        },
        hasChanged: function(t) {
            return t ? this._previousAttributes[t] != this.attributes[t] : this._changed
        },
        changedAttributes: function(t) {
            t || (t = this.attributes);
            var e = this._previousAttributes,
                i = !1;
            for (var r in t) _.isEqual(e[r], t[r]) || (i = i || {}, i[r] = t[r]);
            return i
        },
        previous: function(t) {
            return t && this._previousAttributes ? this._previousAttributes[t] : null
        },
        previousAttributes: function() {
            return _.clone(this._previousAttributes)
        },
        _performValidation: function(t, e) {
            var i = this.validate(t);
            return !i || (e.error ? e.error(this, i, e) : this.trigger("error", this, i, e), !1)
        }
    }), t.Collection = function(t, e) {
        e || (e = {}), e.comparator && (this.comparator = e.comparator), _.bindAll(this, "_onModelEvent", "_removeReference"), this._reset(), t && this.reset(t, {
            silent: !0
        }), this.initialize.apply(this, arguments)
    }, _.extend(t.Collection.prototype, t.Events, {
        model: t.Model,
        initialize: function() {},
        toJSON: function() {
            return this.map(function(t) {
                return t.toJSON()
            })
        },
        add: function(t, e) {
            if (_.isArray(t))
                for (var i = 0, r = t.length; i < r; i++) this._add(t[i], e);
            else this._add(t, e);
            return this
        },
        remove: function(t, e) {
            if (_.isArray(t))
                for (var i = 0, r = t.length; i < r; i++) this._remove(t[i], e);
            else this._remove(t, e);
            return this
        },
        get: function(t) {
            return null == t ? null : this._byId[null != t.id ? t.id : t]
        },
        getByCid: function(t) {
            return t && this._byCid[t.cid || t]
        },
        at: function(t) {
            return this.models[t]
        },
        sort: function(t) {
            if (t || (t = {}), !this.comparator) throw new Error("Cannot sort a set without a comparator");
            return this.models = this.sortBy(this.comparator), t.silent || this.trigger("reset", this, t), this
        },
        pluck: function(t) {
            return _.map(this.models, function(e) {
                return e.get(t)
            })
        },
        reset: function(t, e) {
            return t || (t = []), e || (e = {}), this.each(this._removeReference), this._reset(), this.add(t, {
                silent: !0
            }), e.silent || this.trigger("reset", this, e), this
        },
        _reset: function(t) {
            this.length = 0, this.models = [], this._byId = {}, this._byCid = {}
        },
        _prepareModel: function(e, i) {
            if (e instanceof t.Model) e.collection || (e.collection = this);
            else {
                var r = e;
                e = new this.model(r, {
                    collection: this
                }), e.validate && !e._performValidation(r, i) && (e = !1)
            }
            return e
        },
        _add: function(t, e) {
            if (e || (e = {}), t = this._prepareModel(t, e), !t) return !1;
            var i = this.getByCid(t);
            if (i) throw new Error(["Can't add the same model to a set twice", i.id]);
            this._byId[t.id] = t, this._byCid[t.cid] = t;
            var r = null != e.at ? e.at : this.comparator ? this.sortedIndex(t, this.comparator) : this.length;
            return this.models.splice(r, 0, t), t.bind("all", this._onModelEvent), this.length++, e.silent || t.trigger("add", t, this, e), t
        },
        _remove: function(t, e) {
            return e || (e = {}), (t = this.getByCid(t) || this.get(t)) ? (delete this._byId[t.id], delete this._byCid[t.cid], this.models.splice(this.indexOf(t), 1), this.length--, e.silent || t.trigger("remove", t, this, e), this._removeReference(t), t) : null
        },
        _removeReference: function(t) {
            this == t.collection && delete t.collection, t.unbind("all", this._onModelEvent)
        },
        _onModelEvent: function(t, e, i, r) {
            ("add" != t && "remove" != t || i == this) && ("destroy" == t && this._remove(e, r), e && t === "change:" + e.idAttribute && (delete this._byId[e.previous(e.idAttribute)], this._byId[e.id] = e), this.trigger.apply(this, arguments))
        }
    });
    var i = ["forEach", "each", "map", "reduce", "reduceRight", "find", "detect", "filter", "select", "reject", "every", "all", "some", "any", "include", "contains", "invoke", "max", "min", "sortBy", "sortedIndex", "toArray", "size", "first", "rest", "last", "without", "indexOf", "lastIndexOf", "isEmpty", "groupBy"];
    _.each(i, function(e) {
        t.Collection.prototype[e] = function() {
            return _[e].apply(_, [this.models].concat(_.toArray(arguments)))
        }
    });
    var r = function(t, e) {
        var i = n(this, t, e);
        return i.extend = this.extend, i
    };
    t.Model.extend = t.Collection.extend = r;
    var s = function() {},
        n = function(t, e, i) {
            var r;
            return r = e && e.hasOwnProperty("constructor") ? e.constructor : function() {
                return t.apply(this, arguments)
            }, _.extend(r, t), s.prototype = t.prototype, r.prototype = new s, e && _.extend(r.prototype, e), i && _.extend(r, i), r.prototype.constructor = r, r.__super__ = t.prototype, r
        },
        o = function(t, e, i) {
            return function(r) {
                t ? t(e, r, i) : e.trigger("error", e, r, i)
            }
        },
        h = function(t) {
            return t.replace(/&(?!\w+;|#\d+;|#x[\da-f]+;)/gi, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;").replace(/'/g, "&#x27;").replace(/\//g, "&#x2F;")
        };
    return {
        Model: t.Model,
        Events: t.Events,
        Collection: t.Collection
    }
}.call(this);
var $ = $ || {};
$.support = {};
var CryptoJS = CryptoJS || function(t, e) {
    var r = {},
        i = r.lib = {},
        n = function() {},
        o = i.Base = {
            extend: function(t) {
                n.prototype = this;
                var e = new n;
                return t && e.mixIn(t), e.hasOwnProperty("init") || (e.init = function() {
                    e.$super.init.apply(this, arguments)
                }), e.init.prototype = e, e.$super = this, e
            },
            create: function() {
                var t = this.extend();
                return t.init.apply(t, arguments), t
            },
            init: function() {},
            mixIn: function(t) {
                for (var e in t) t.hasOwnProperty(e) && (this[e] = t[e]);
                t.hasOwnProperty("toString") && (this.toString = t.toString)
            },
            clone: function() {
                return this.init.prototype.extend(this)
            }
        },
        s = i.WordArray = o.extend({
            init: function(t, r) {
                t = this.words = t || [], this.sigBytes = r != e ? r : 4 * t.length
            },
            toString: function(t) {
                return (t || a).stringify(this)
            },
            concat: function(t) {
                var e = this.words,
                    r = t.words,
                    i = this.sigBytes;
                if (t = t.sigBytes, this.clamp(), i % 4)
                    for (var n = 0; n < t; n++) e[i + n >>> 2] |= (r[n >>> 2] >>> 24 - 8 * (n % 4) & 255) << 24 - 8 * ((i + n) % 4);
                else if (65535 < r.length)
                    for (n = 0; n < t; n += 4) e[i + n >>> 2] = r[n >>> 2];
                else e.push.apply(e, r);
                return this.sigBytes += t, this
            },
            clamp: function() {
                var e = this.words,
                    r = this.sigBytes;
                e[r >>> 2] &= 4294967295 << 32 - 8 * (r % 4), e.length = t.ceil(r / 4)
            },
            clone: function() {
                var t = o.clone.call(this);
                return t.words = this.words.slice(0), t
            },
            random: function(e) {
                for (var r = [], i = 0; i < e; i += 4) r.push(4294967296 * t.random() | 0);
                return new s.init(r, e)
            }
        }),
        c = r.enc = {},
        a = c.Hex = {
            stringify: function(t) {
                var e = t.words;
                t = t.sigBytes;
                for (var r = [], i = 0; i < t; i++) {
                    var n = e[i >>> 2] >>> 24 - 8 * (i % 4) & 255;
                    r.push((n >>> 4).toString(16)), r.push((15 & n).toString(16))
                }
                return r.join("")
            },
            parse: function(t) {
                for (var e = t.length, r = [], i = 0; i < e; i += 2) r[i >>> 3] |= parseInt(t.substr(i, 2), 16) << 24 - 4 * (i % 8);
                return new s.init(r, e / 2)
            }
        },
        f = c.Latin1 = {
            stringify: function(t) {
                var e = t.words;
                t = t.sigBytes;
                for (var r = [], i = 0; i < t; i++) r.push(String.fromCharCode(e[i >>> 2] >>> 24 - 8 * (i % 4) & 255));
                return r.join("")
            },
            parse: function(t) {
                for (var e = t.length, r = [], i = 0; i < e; i++) r[i >>> 2] |= (255 & t.charCodeAt(i)) << 24 - 8 * (i % 4);
                return new s.init(r, e)
            }
        },
        h = c.Utf8 = {
            stringify: function(t) {
                try {
                    return decodeURIComponent(escape(f.stringify(t)))
                } catch (e) {
                    throw Error("Malformed UTF-8 data")
                }
            },
            parse: function(t) {
                return f.parse(unescape(encodeURIComponent(t)))
            }
        },
        u = i.BufferedBlockAlgorithm = o.extend({
            reset: function() {
                this._data = new s.init, this._nDataBytes = 0
            },
            _append: function(t) {
                "string" == typeof t && (t = h.parse(t)), this._data.concat(t), this._nDataBytes += t.sigBytes
            },
            _process: function(e) {
                var r = this._data,
                    i = r.words,
                    n = r.sigBytes,
                    o = this.blockSize,
                    c = n / (4 * o),
                    c = e ? t.ceil(c) : t.max((0 | c) - this._minBufferSize, 0);
                if (e = c * o, n = t.min(4 * e, n), e) {
                    for (var a = 0; a < e; a += o) this._doProcessBlock(i, a);
                    a = i.splice(0, e), r.sigBytes -= n
                }
                return new s.init(a, n)
            },
            clone: function() {
                var t = o.clone.call(this);
                return t._data = this._data.clone(), t
            },
            _minBufferSize: 0
        });
    i.Hasher = u.extend({
        cfg: o.extend(),
        init: function(t) {
            this.cfg = this.cfg.extend(t), this.reset()
        },
        reset: function() {
            u.reset.call(this), this._doReset()
        },
        update: function(t) {
            return this._append(t), this._process(), this
        },
        finalize: function(t) {
            return t && this._append(t), this._doFinalize()
        },
        blockSize: 16,
        _createHelper: function(t) {
            return function(e, r) {
                return new t.init(r).finalize(e)
            }
        },
        _createHmacHelper: function(t) {
            return function(e, r) {
                return new p.HMAC.init(t, r).finalize(e)
            }
        }
    });
    var p = r.algo = {};
    return r
}(Math);
! function() {
    var t = CryptoJS,
        e = t.lib.WordArray;
    t.enc.Base64 = {
        stringify: function(t) {
            var e = t.words,
                r = t.sigBytes,
                i = this._map;
            t.clamp(), t = [];
            for (var n = 0; n < r; n += 3)
                for (var o = (e[n >>> 2] >>> 24 - 8 * (n % 4) & 255) << 16 | (e[n + 1 >>> 2] >>> 24 - 8 * ((n + 1) % 4) & 255) << 8 | e[n + 2 >>> 2] >>> 24 - 8 * ((n + 2) % 4) & 255, s = 0; 4 > s && n + .75 * s < r; s++) t.push(i.charAt(o >>> 6 * (3 - s) & 63));
            if (e = i.charAt(64))
                for (; t.length % 4;) t.push(e);
            return t.join("")
        },
        parse: function(t) {
            var r = t.length,
                i = this._map,
                n = i.charAt(64);
            n && (n = t.indexOf(n), -1 != n && (r = n));
            for (var n = [], o = 0, s = 0; s < r; s++)
                if (s % 4) {
                    var c = i.indexOf(t.charAt(s - 1)) << 2 * (s % 4),
                        a = i.indexOf(t.charAt(s)) >>> 6 - 2 * (s % 4);
                    n[o >>> 2] |= (c | a) << 24 - 8 * (o % 4), o++
                }
            return e.create(n, o)
        },
        _map: "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/="
    }
}(),
function(t) {
    function e(t, e, r, i, n, o, s) {
        return t = t + (e & r | ~e & i) + n + s, (t << o | t >>> 32 - o) + e
    }

    function r(t, e, r, i, n, o, s) {
        return t = t + (e & i | r & ~i) + n + s, (t << o | t >>> 32 - o) + e
    }

    function i(t, e, r, i, n, o, s) {
        return t = t + (e ^ r ^ i) + n + s, (t << o | t >>> 32 - o) + e
    }

    function n(t, e, r, i, n, o, s) {
        return t = t + (r ^ (e | ~i)) + n + s, (t << o | t >>> 32 - o) + e
    }
    for (var o = CryptoJS, s = o.lib, c = s.WordArray, a = s.Hasher, s = o.algo, f = [], h = 0; 64 > h; h++) f[h] = 4294967296 * t.abs(t.sin(h + 1)) | 0;
    s = s.MD5 = a.extend({
        _doReset: function() {
            this._hash = new c.init([1732584193, 4023233417, 2562383102, 271733878])
        },
        _doProcessBlock: function(t, o) {
            for (var s = 0; 16 > s; s++) {
                var c = o + s,
                    a = t[c];
                t[c] = 16711935 & (a << 8 | a >>> 24) | 4278255360 & (a << 24 | a >>> 8)
            }
            var s = this._hash.words,
                c = t[o + 0],
                a = t[o + 1],
                h = t[o + 2],
                u = t[o + 3],
                p = t[o + 4],
                d = t[o + 5],
                l = t[o + 6],
                y = t[o + 7],
                _ = t[o + 8],
                v = t[o + 9],
                g = t[o + 10],
                B = t[o + 11],
                x = t[o + 12],
                S = t[o + 13],
                k = t[o + 14],
                m = t[o + 15],
                z = s[0],
                C = s[1],
                w = s[2],
                D = s[3],
                z = e(z, C, w, D, c, 7, f[0]),
                D = e(D, z, C, w, a, 12, f[1]),
                w = e(w, D, z, C, h, 17, f[2]),
                C = e(C, w, D, z, u, 22, f[3]),
                z = e(z, C, w, D, p, 7, f[4]),
                D = e(D, z, C, w, d, 12, f[5]),
                w = e(w, D, z, C, l, 17, f[6]),
                C = e(C, w, D, z, y, 22, f[7]),
                z = e(z, C, w, D, _, 7, f[8]),
                D = e(D, z, C, w, v, 12, f[9]),
                w = e(w, D, z, C, g, 17, f[10]),
                C = e(C, w, D, z, B, 22, f[11]),
                z = e(z, C, w, D, x, 7, f[12]),
                D = e(D, z, C, w, S, 12, f[13]),
                w = e(w, D, z, C, k, 17, f[14]),
                C = e(C, w, D, z, m, 22, f[15]),
                z = r(z, C, w, D, a, 5, f[16]),
                D = r(D, z, C, w, l, 9, f[17]),
                w = r(w, D, z, C, B, 14, f[18]),
                C = r(C, w, D, z, c, 20, f[19]),
                z = r(z, C, w, D, d, 5, f[20]),
                D = r(D, z, C, w, g, 9, f[21]),
                w = r(w, D, z, C, m, 14, f[22]),
                C = r(C, w, D, z, p, 20, f[23]),
                z = r(z, C, w, D, v, 5, f[24]),
                D = r(D, z, C, w, k, 9, f[25]),
                w = r(w, D, z, C, u, 14, f[26]),
                C = r(C, w, D, z, _, 20, f[27]),
                z = r(z, C, w, D, S, 5, f[28]),
                D = r(D, z, C, w, h, 9, f[29]),
                w = r(w, D, z, C, y, 14, f[30]),
                C = r(C, w, D, z, x, 20, f[31]),
                z = i(z, C, w, D, d, 4, f[32]),
                D = i(D, z, C, w, _, 11, f[33]),
                w = i(w, D, z, C, B, 16, f[34]),
                C = i(C, w, D, z, k, 23, f[35]),
                z = i(z, C, w, D, a, 4, f[36]),
                D = i(D, z, C, w, p, 11, f[37]),
                w = i(w, D, z, C, y, 16, f[38]),
                C = i(C, w, D, z, g, 23, f[39]),
                z = i(z, C, w, D, S, 4, f[40]),
                D = i(D, z, C, w, c, 11, f[41]),
                w = i(w, D, z, C, u, 16, f[42]),
                C = i(C, w, D, z, l, 23, f[43]),
                z = i(z, C, w, D, v, 4, f[44]),
                D = i(D, z, C, w, x, 11, f[45]),
                w = i(w, D, z, C, m, 16, f[46]),
                C = i(C, w, D, z, h, 23, f[47]),
                z = n(z, C, w, D, c, 6, f[48]),
                D = n(D, z, C, w, y, 10, f[49]),
                w = n(w, D, z, C, k, 15, f[50]),
                C = n(C, w, D, z, d, 21, f[51]),
                z = n(z, C, w, D, x, 6, f[52]),
                D = n(D, z, C, w, u, 10, f[53]),
                w = n(w, D, z, C, g, 15, f[54]),
                C = n(C, w, D, z, a, 21, f[55]),
                z = n(z, C, w, D, _, 6, f[56]),
                D = n(D, z, C, w, m, 10, f[57]),
                w = n(w, D, z, C, l, 15, f[58]),
                C = n(C, w, D, z, S, 21, f[59]),
                z = n(z, C, w, D, p, 6, f[60]),
                D = n(D, z, C, w, B, 10, f[61]),
                w = n(w, D, z, C, h, 15, f[62]),
                C = n(C, w, D, z, v, 21, f[63]);
            s[0] = s[0] + z | 0, s[1] = s[1] + C | 0, s[2] = s[2] + w | 0, s[3] = s[3] + D | 0
        },
        _doFinalize: function() {
            var e = this._data,
                r = e.words,
                i = 8 * this._nDataBytes,
                n = 8 * e.sigBytes;
            r[n >>> 5] |= 128 << 24 - n % 32;
            var o = t.floor(i / 4294967296);
            for (r[(n + 64 >>> 9 << 4) + 15] = 16711935 & (o << 8 | o >>> 24) | 4278255360 & (o << 24 | o >>> 8), r[(n + 64 >>> 9 << 4) + 14] = 16711935 & (i << 8 | i >>> 24) | 4278255360 & (i << 24 | i >>> 8), e.sigBytes = 4 * (r.length + 1), this._process(), e = this._hash, r = e.words, i = 0; 4 > i; i++) n = r[i], r[i] = 16711935 & (n << 8 | n >>> 24) | 4278255360 & (n << 24 | n >>> 8);
            return e
        },
        clone: function() {
            var t = a.clone.call(this);
            return t._hash = this._hash.clone(), t
        }
    }), o.MD5 = a._createHelper(s), o.HmacMD5 = a._createHmacHelper(s)
}(Math),
function() {
    var t = CryptoJS,
        e = t.lib,
        r = e.Base,
        i = e.WordArray,
        e = t.algo,
        n = e.EvpKDF = r.extend({
            cfg: r.extend({
                keySize: 4,
                hasher: e.MD5,
                iterations: 1
            }),
            init: function(t) {
                this.cfg = this.cfg.extend(t)
            },
            compute: function(t, e) {
                for (var r = this.cfg, n = r.hasher.create(), o = i.create(), s = o.words, c = r.keySize, r = r.iterations; s.length < c;) {
                    a && n.update(a);
                    var a = n.update(t).finalize(e);
                    n.reset();
                    for (var f = 1; f < r; f++) a = n.finalize(a), n.reset();
                    o.concat(a)
                }
                return o.sigBytes = 4 * c, o
            }
        });
    t.EvpKDF = function(t, e, r) {
        return n.create(r).compute(t, e)
    }
}(), CryptoJS.lib.Cipher || function(t) {
        var e = CryptoJS,
            r = e.lib,
            i = r.Base,
            n = r.WordArray,
            o = r.BufferedBlockAlgorithm,
            s = e.enc.Base64,
            c = e.algo.EvpKDF,
            a = r.Cipher = o.extend({
                cfg: i.extend(),
                createEncryptor: function(t, e) {
                    return this.create(this._ENC_XFORM_MODE, t, e)
                },
                createDecryptor: function(t, e) {
                    return this.create(this._DEC_XFORM_MODE, t, e)
                },
                init: function(t, e, r) {
                    this.cfg = this.cfg.extend(r), this._xformMode = t, this._key = e, this.reset()
                },
                reset: function() {
                    o.reset.call(this), this._doReset()
                },
                process: function(t) {
                    return this._append(t), this._process()
                },
                finalize: function(t) {
                    return t && this._append(t), this._doFinalize()
                },
                keySize: 4,
                ivSize: 4,
                _ENC_XFORM_MODE: 1,
                _DEC_XFORM_MODE: 2,
                _createHelper: function(t) {
                    return {
                        encrypt: function(e, r, i) {
                            return ("string" == typeof r ? l : d).encrypt(t, e, r, i)
                        },
                        decrypt: function(e, r, i) {
                            return ("string" == typeof r ? l : d).decrypt(t, e, r, i)
                        }
                    }
                }
            });
        r.StreamCipher = a.extend({
            _doFinalize: function() {
                return this._process(!0)
            },
            blockSize: 1
        });
        var f = e.mode = {},
            h = function(e, r, i) {
                var n = this._iv;
                n ? this._iv = t : n = this._prevBlock;
                for (var o = 0; o < i; o++) e[r + o] ^= n[o]
            },
            u = (r.BlockCipherMode = i.extend({
                createEncryptor: function(t, e) {
                    return this.Encryptor.create(t, e)
                },
                createDecryptor: function(t, e) {
                    return this.Decryptor.create(t, e)
                },
                init: function(t, e) {
                    this._cipher = t, this._iv = e
                }
            })).extend();
        u.Encryptor = u.extend({
            processBlock: function(t, e) {
                var r = this._cipher,
                    i = r.blockSize;
                h.call(this, t, e, i), r.encryptBlock(t, e), this._prevBlock = t.slice(e, e + i)
            }
        }), u.Decryptor = u.extend({
            processBlock: function(t, e) {
                var r = this._cipher,
                    i = r.blockSize,
                    n = t.slice(e, e + i);
                r.decryptBlock(t, e), h.call(this, t, e, i), this._prevBlock = n
            }
        }), f = f.CBC = u, u = (e.pad = {}).Pkcs7 = {
            pad: function(t, e) {
                for (var r = 4 * e, r = r - t.sigBytes % r, i = r << 24 | r << 16 | r << 8 | r, o = [], s = 0; s < r; s += 4) o.push(i);
                r = n.create(o, r), t.concat(r)
            },
            unpad: function(t) {
                t.sigBytes -= 255 & t.words[t.sigBytes - 1 >>> 2]
            }
        }, r.BlockCipher = a.extend({
            cfg: a.cfg.extend({
                mode: f,
                padding: u
            }),
            reset: function() {
                a.reset.call(this);
                var t = this.cfg,
                    e = t.iv,
                    t = t.mode;
                if (this._xformMode == this._ENC_XFORM_MODE) var r = t.createEncryptor;
                else r = t.createDecryptor, this._minBufferSize = 1;
                this._mode = r.call(t, this, e && e.words)
            },
            _doProcessBlock: function(t, e) {
                this._mode.processBlock(t, e)
            },
            _doFinalize: function() {
                var t = this.cfg.padding;
                if (this._xformMode == this._ENC_XFORM_MODE) {
                    t.pad(this._data, this.blockSize);
                    var e = this._process(!0)
                } else e = this._process(!0), t.unpad(e);
                return e
            },
            blockSize: 4
        });
        var p = r.CipherParams = i.extend({
                init: function(t) {
                    this.mixIn(t)
                },
                toString: function(t) {
                    return (t || this.formatter).stringify(this)
                }
            }),
            f = (e.format = {}).OpenSSL = {
                stringify: function(t) {
                    var e = t.ciphertext;
                    return t = t.salt, (t ? n.create([1398893684, 1701076831]).concat(t).concat(e) : e).toString(s)
                },
                parse: function(t) {
                    t = s.parse(t);
                    var e = t.words;
                    if (1398893684 == e[0] && 1701076831 == e[1]) {
                        var r = n.create(e.slice(2, 4));
                        e.splice(0, 4), t.sigBytes -= 16
                    }
                    return p.create({
                        ciphertext: t,
                        salt: r
                    })
                }
            },
            d = r.SerializableCipher = i.extend({
                cfg: i.extend({
                    format: f
                }),
                encrypt: function(t, e, r, i) {
                    i = this.cfg.extend(i);
                    var n = t.createEncryptor(r, i);
                    return e = n.finalize(e), n = n.cfg, p.create({
                        ciphertext: e,
                        key: r,
                        iv: n.iv,
                        algorithm: t,
                        mode: n.mode,
                        padding: n.padding,
                        blockSize: t.blockSize,
                        formatter: i.format
                    })
                },
                decrypt: function(t, e, r, i) {
                    return i = this.cfg.extend(i), e = this._parse(e, i.format), t.createDecryptor(r, i).finalize(e.ciphertext)
                },
                _parse: function(t, e) {
                    return "string" == typeof t ? e.parse(t, this) : t
                }
            }),
            e = (e.kdf = {}).OpenSSL = {
                execute: function(t, e, r, i) {
                    return i || (i = n.random(8)), t = c.create({
                        keySize: e + r
                    }).compute(t, i), r = n.create(t.words.slice(e), 4 * r), t.sigBytes = 4 * e, p.create({
                        key: t,
                        iv: r,
                        salt: i
                    })
                }
            },
            l = r.PasswordBasedCipher = d.extend({
                cfg: d.cfg.extend({
                    kdf: e
                }),
                encrypt: function(t, e, r, i) {
                    return i = this.cfg.extend(i), r = i.kdf.execute(r, t.keySize, t.ivSize), i.iv = r.iv, t = d.encrypt.call(this, t, e, r.key, i), t.mixIn(r), t
                },
                decrypt: function(t, e, r, i) {
                    return i = this.cfg.extend(i), e = this._parse(e, i.format), r = i.kdf.execute(r, t.keySize, t.ivSize, e.salt), i.iv = r.iv, d.decrypt.call(this, t, e, r.key, i)
                }
            })
    }(),
    function() {
        for (var t = CryptoJS, e = t.lib.BlockCipher, r = t.algo, i = [], n = [], o = [], s = [], c = [], a = [], f = [], h = [], u = [], p = [], d = [], l = 0; 256 > l; l++) d[l] = 128 > l ? l << 1 : l << 1 ^ 283;
        for (var y = 0, _ = 0, l = 0; 256 > l; l++) {
            var v = _ ^ _ << 1 ^ _ << 2 ^ _ << 3 ^ _ << 4,
                v = v >>> 8 ^ 255 & v ^ 99;
            i[y] = v, n[v] = y;
            var g = d[y],
                B = d[g],
                x = d[B],
                S = 257 * d[v] ^ 16843008 * v;
            o[y] = S << 24 | S >>> 8, s[y] = S << 16 | S >>> 16, c[y] = S << 8 | S >>> 24, a[y] = S, S = 16843009 * x ^ 65537 * B ^ 257 * g ^ 16843008 * y, f[v] = S << 24 | S >>> 8, h[v] = S << 16 | S >>> 16, u[v] = S << 8 | S >>> 24, p[v] = S, y ? (y = g ^ d[d[d[x ^ g]]], _ ^= d[d[_]]) : y = _ = 1
        }
        var k = [0, 1, 2, 4, 8, 16, 32, 64, 128, 27, 54],
            r = r.AES = e.extend({
                _doReset: function() {
                    for (var t = this._key, e = t.words, r = t.sigBytes / 4, t = 4 * ((this._nRounds = r + 6) + 1), n = this._keySchedule = [], o = 0; o < t; o++)
                        if (o < r) n[o] = e[o];
                        else {
                            var s = n[o - 1];
                            o % r ? 6 < r && 4 == o % r && (s = i[s >>> 24] << 24 | i[s >>> 16 & 255] << 16 | i[s >>> 8 & 255] << 8 | i[255 & s]) : (s = s << 8 | s >>> 24, s = i[s >>> 24] << 24 | i[s >>> 16 & 255] << 16 | i[s >>> 8 & 255] << 8 | i[255 & s], s ^= k[o / r | 0] << 24), n[o] = n[o - r] ^ s
                        }
                    for (e = this._invKeySchedule = [], r = 0; r < t; r++) o = t - r, s = r % 4 ? n[o] : n[o - 4], e[r] = 4 > r || 4 >= o ? s : f[i[s >>> 24]] ^ h[i[s >>> 16 & 255]] ^ u[i[s >>> 8 & 255]] ^ p[i[255 & s]]
                },
                encryptBlock: function(t, e) {
                    this._doCryptBlock(t, e, this._keySchedule, o, s, c, a, i)
                },
                decryptBlock: function(t, e) {
                    var r = t[e + 1];
                    t[e + 1] = t[e + 3], t[e + 3] = r, this._doCryptBlock(t, e, this._invKeySchedule, f, h, u, p, n), r = t[e + 1], t[e + 1] = t[e + 3], t[e + 3] = r
                },
                _doCryptBlock: function(t, e, r, i, n, o, s, c) {
                    for (var a = this._nRounds, f = t[e] ^ r[0], h = t[e + 1] ^ r[1], u = t[e + 2] ^ r[2], p = t[e + 3] ^ r[3], d = 4, l = 1; l < a; l++) var y = i[f >>> 24] ^ n[h >>> 16 & 255] ^ o[u >>> 8 & 255] ^ s[255 & p] ^ r[d++],
                        _ = i[h >>> 24] ^ n[u >>> 16 & 255] ^ o[p >>> 8 & 255] ^ s[255 & f] ^ r[d++],
                        v = i[u >>> 24] ^ n[p >>> 16 & 255] ^ o[f >>> 8 & 255] ^ s[255 & h] ^ r[d++],
                        p = i[p >>> 24] ^ n[f >>> 16 & 255] ^ o[h >>> 8 & 255] ^ s[255 & u] ^ r[d++],
                        f = y,
                        h = _,
                        u = v;
                    y = (c[f >>> 24] << 24 | c[h >>> 16 & 255] << 16 | c[u >>> 8 & 255] << 8 | c[255 & p]) ^ r[d++], _ = (c[h >>> 24] << 24 | c[u >>> 16 & 255] << 16 | c[p >>> 8 & 255] << 8 | c[255 & f]) ^ r[d++], v = (c[u >>> 24] << 24 | c[p >>> 16 & 255] << 16 | c[f >>> 8 & 255] << 8 | c[255 & h]) ^ r[d++], p = (c[p >>> 24] << 24 | c[f >>> 16 & 255] << 16 | c[h >>> 8 & 255] << 8 | c[255 & u]) ^ r[d++], t[e] = y, t[e + 1] = _, t[e + 2] = v, t[e + 3] = p
                },
                keySize: 8
            });
        t.AES = e._createHelper(r)
    }();
(function() {
    var g;

    function aa(a) {
        this.media = null;
        this.status = {
            paused: 1,
            duration: 0
        };
        this.Qd = null;
        this.J = a || {};
        this.L = {}
    }
    var ba = "abort canplay canplaythrough durationchange emptied ended error loadeddata loadedmetadata loadstart mozaudioavailable pause play playing progress ratechange seeked seeking stalled suspend timeupdate volumechange waiting".split(" ");
    g = aa.prototype;
    g.vb = function() {
        return 100 * this.media.volume
    };
    g.nc = function() {
        var a = 0;
        this.media.played.length && (a = this.media.played.end(0));
        var b = this.media.duration;
        return {
            elapsedTime: a,
            totalTime: b,
            percentPlayed: 0 < b ? 100 * this.media.currentTime / b : 0
        }
    };
    g.pause = function() {
        this.media && this.media.currentTime && (this.media.pause(), this.status.paused = 1)
    };
    g.play = function(a) {
        if (!(0 > a)) {
            this.Qd && clearTimeout(this.Qd);
            var b = this,
                c = this.media;
            !isNaN(a) && 0 < a ? "object" === typeof c.seekable && 0 < c.seekable.length ? (h(">>>>> playing with time " + a), c.currentTime = a, c.play()) : this.Qd = setTimeout(function() {
                h(">>>>> not ready yet, re-trying play(" + a + ")");
                b.play(a)
            }, 150) : (h(">>>>> playing w/o time"), this.media.play(), this.status.paused = 0)
        }
    };
    g.load = function(a, b, c) {
        if (!a) throw Error("Unable to load invalid media");
        var d = {};
        if ("string" === typeof a) d.src = a;
        else if ("object" === typeof a && a.src) d = a;
        else throw Error("Unable to load unknown media");
        this.autoplay = c;
        this.media = this.Fd(d, b, c)
    };
    g.Na = function(a, b) {
        this.media && (b && "undefined" != typeof b && (a = Math.min(100, Math.round(Math.pow(10, b / 20) * a))), this.media.volume = a / 100)
    };
    g.Fd = function(a, b, c) {
        var d = null;
        if (this.media) {
            this.media && this.J && this.J.onShuttingDown && this.J.onShuttingDown.call(null);
            for (d = 0; d < ba.length; d++) {
                var e = ba[d];
                this.media.removeEventListener(e, this.L[e]);
                this.L[e] = null
            }
            this.status.paused = 1;
            this.status.duration = 0;
            this.media.removeAttribute("src");
            d = this.media
        } else d = new Audio;
        d.setAttribute("src", a.src);
        b && d.setAttribute("preload", b ? "metadata" : "none");
        c ? d.setAttribute("autoplay", "autoplay") : d.removeAttribute("autoplay");
        ca(this, d);
        d.load();
        return d
    };

    function ca(a, b) {
        function c() {
            y("error", "onErred", a.J)
        }

        function d() {
            y("stalled", "onStalled", a.J)
        }

        function e() {
            y("loadstart", "onLoadStart", a.J)
        }

        function f() {
            y("loadedmetadata", "onLoadedMetadata", a.J)
        }

        function k() {
            y("loadeddata", "onLoadedData", a.J)
        }

        function n() {
            y("canplay", "onCanPlay", a.J)
        }

        function p() {
            y("readystatechanged", "onReadyStateChanged", a.J)
        }

        function s() {
            y("ratechanged", "onRateChanged", a.J)
        }

        function t() {
            y("emptied", "onEmptied", a.J)
        }

        function P() {
            y("abort", "onAborted", a.J)
        }

        function ha() {
            a.pause();
            a.status.duration = 0;
            y("ended", "onEnded", a.J)
        }

        function ia() {
            y("suspend", "onSuspended", a.J)
        }

        function ja() {
            y("volumechange", "onVolumeChanged", a.J)
        }

        function ka() {
            y("seeked", "onSeeked", a.J)
        }

        function Ka() {
            y("seeking", "onSeeking", a.J)
        }

        function La() {
            y("waiting", "onWaiting", a.J)
        }

        function Ma() {
            a.status.paused = 1;
            y("pause", "onPaused", a.J)
        }

        function Na() {
            a.status.paused = 0;
            y("playing", "onPlaying", a.J)
        }

        function Oa() {
            a.status.paused = 0;
            y("play", "onPlay", a.J)
        }

        function Pa() {
            a.status.duration = a.media.duration;
            y("durationchange",
                "onDurationChanged", a.J)
        }

        function Qa() {
            var b = a.media;
            if (4 == b.readyState && !a.status.paused) {
                var c = b.duration;
                y("timeupdate", "onTimeUpdated", a.J, {
                    elapsedTime: Math.floor(b.currentTime),
                    totalTime: Math.floor(c),
                    percentPlayed: Math.floor(0 < c ? 100 * b.currentTime / c : 0)
                })
            }
        }

        function Sa() {
            y("progress", "onProgress", a.J)
        }

        function Ta() {
            y("canplaythrough", "onCanPlayThrough", a.J)
        }

        function y(a, b, c, d) {
            var e = Ua.call(arguments),
                f = e[1],
                k = e[2][f],
                n = window.console || n;
            n && n.debug(">>>>> audio tag event:" + e[0] + " calling " + f);
            k && k.apply(null, Ua.call(arguments, 3))
        }
        var Ua = Array.prototype.slice;
        a.L.canplaythrough = Ta;
        a.L.progress = Sa;
        a.L.timeupdate = Qa;
        a.L.durationchange = Pa;
        a.L.play = Oa;
        a.L.playing = Na;
        a.L.pause = Ma;
        a.L.waiting = La;
        a.L.seeking = Ka;
        a.L.seeked = ka;
        a.L.volumechange = ja;
        a.L.suspend = ia;
        a.L.ended = ha;
        a.L.abort = P;
        a.L.emptied = t;
        a.L.ratechanged = s;
        a.L.readystatechanged = p;
        a.L.canplay = n;
        a.L.loadeddata = k;
        a.L.loadedmetadata = f;
        a.L.loadstart = e;
        a.L.stalled = d;
        a.L.error = c;
        b.addEventListener("canplaythrough", Ta, !1);
        b.addEventListener("progress",
            Sa, !1);
        b.addEventListener("timeupdate", Qa, !1);
        b.addEventListener("durationchange", Pa, !1);
        b.addEventListener("play", Oa, !1);
        b.addEventListener("playing", Na, !1);
        b.addEventListener("pause", Ma, !1);
        b.addEventListener("waiting", La, !1);
        b.addEventListener("seeking", Ka, !1);
        b.addEventListener("seeked", ka, !1);
        b.addEventListener("volumechange", ja, !1);
        b.addEventListener("suspend", ia, !1);
        b.addEventListener("ended", ha, !1);
        b.addEventListener("abort", P, !1);
        b.addEventListener("emptied", t, !1);
        b.addEventListener("ratechanged",
            s, !1);
        b.addEventListener("readystatechanged", p, !1);
        b.addEventListener("canplay", n, !1);
        b.addEventListener("loadeddata", k, !1);
        b.addEventListener("loadedmetadata", f, !1);
        b.addEventListener("loadstart", e, !1);
        b.addEventListener("stalled", d, !1);
        b.addEventListener("error", c, !1)
    };

    function da(a) {
        this.ka = null;
        var b = this;
        this.pb = null;
        this.audio = new aa({
            onCanPlayThrough: function() {
                (null === b.pb || 2 > (b.pb.elapsedTime || 0)) && a.ei()
            },
            onTimeUpdated: function(c) {
                b.pb = c;
                a.sf(c)
            },
            onEnded: function() {
                b.pb = null;
                a.Mf(b.ka)
            },
            onErred: function() {
                b.pb = null;
                a.Nf({
                    track: b.ka,
                    errorType: "unknown"
                })
            }
        });
        ea = this
    }
    var ea;

    function la(a, b, c) {
        (b = ea) || (b = new da(a));
        b.vg(c);
        return b
    }
    g = da.prototype;
    g.vg = function(a) {
        this.ka = a
    };
    g.load = function(a, b) {
        h("loading " + a);
        this.pb = null;
        this.audio.load(a, !0, void 0 === b)
    };
    g.play = function(a) {
        h(">>>>> SimpleHTML5AudioPlayer called play. Time: " + a);
        this.audio.play(a)
    };
    g.stop = function() {
        this.audio.pause()
    };
    g.pause = function() {
        this.audio.pause()
    };
    g.La = function() {
        h(">>>>> SimpleHTML5AudioPlayer called resume");
        this.audio.play()
    };
    g.Yb = function(a, b) {
        this.audio.Na(b, a)
    };
    g.Na = function(a) {
        this.Yb(0, a)
    };
    $ = $ || {};
    $.ajax = function(a) {
        var b = new XMLHttpRequest;
        h(">>>>> ajax send to: " + a.url);
        b.open(a.type, a.url, !0);
        b.onload = function() {
            if (200 <= b.status && 300 > b.status || 304 === b.status) {
                304 === b.status && a.success.call(null, {});
                var c = b.responseText;
                if (c && 0 < c.length) try {
                    var d = JSON.parse(c);
                    a.success.call(null, d);
                    return
                } catch (e) {
                    q(e)
                }
            }
            a.error.call(null, this)
        };
        b.onerror = function() {
            a.error.call(null, this)
        };
        b.setRequestHeader("Content-Type", a.contentType || "text/plain");
        b.send(a.data)
    };

    function ma() {}
    g = ma.prototype;
    g.storageAvailable = function() {
        return !0
    };
    g.currentBackend = function() {
        return "Empty"
    };
    g.set = function() {};
    g.get = function(a, b) {
        return b
    };
    g.flush = function() {};
    g.deleteKey = function() {};
    var r = String["edoCrahCmorf".split("").reverse().join("")],
        na = r.apply(null, [101, 97, 56, 57]),
        oa, pa, qa, ra, sa, ta, wa = oa = pa = qa = ra = sa = ta = na;
    var u = this;
// cchu
    u._data = _data
// cchu
    u.oninit = function(a, b) {
        h(">>>>>oninit is called");
        try {
            (new xa(b, a)).start()
        } catch (c) {
            h("Error starting Pandora with exception below"), h(c), a.shutdown()
        }
    };
    u.ondiallaunch = function(a) {
        h(">>>>>launching with " + a)
    };

    function xa(a, b) {
        this.data = u._data;
        this.Xi = this.data.standalone && "true" === this.data.standalone;
        this.aa = b;
        this.T = a
    }
    g = xa.prototype;
    g.start = function() {
        var a = this;
        this.userAgent = "speaker.js";
        u.onshutdown = function() {
            a.aa.setMediaInfo(null)
        };
        u.ondialshutdown = function() {
            h(">>>>>dial shutdown")
        };
        u.onconnected = function(b) {
            a.jb.push(b)
        };
        u.ondisconnected = function(b, d) {
            h(">>>>>disconnected " + b + " with reason: " + d);
            var e = a.jb.indexOf(b); - 1 !== e && a.jb.splice(e, 1)
        };
        u.onmessage = function(b, d) {
            a.zj = !1;
            var e = ya(d);
            if (e && e.data) a.onmessage(b, ya(d));
            else h(">>>>>onmessage with with no message sent: " + d)
        };
        u.onPlayButtonPressed = function() {
            h(">>>>>onPlayButtonPressed");
            v.play();
            a.paused = !1
        };
        var b = za(this.T);
        u.onPauseButtonPressed = function() {
            h(">>>>>onPauseButtonPressed");
            v.pause();
            a.paused = !0;
            a.updateStatus()
        };
        u.onPlayPauseButtonPressed = function() {
            h(">>>>>onPlayPauseButtonPressed");
            a.toggle()
        };
        u.onSkipForwardButtonPressed = function() {
            h(">>>>>onSkipForwardButtonPressed");
            a.Tb()
        };
        u.onSkipBackwardButtonPressed = function() {
            h(">>>>>onSkipBackwardButtonPressed")
        };
        u.onPositiveRatingButtonPressed = function() {
            h(">>>>>onPositiveRatingButtonPressed");
            v.Fg()
        };
        u.onNegativeRatingButtonPressed =
            function() {
                h(">>>>>onNegativeRatingButtonPressed");
                v.Eg()
            };
        u.onmuted = function() {
            h(">>>>>onmuted")
        };
        u.onunmuted = function() {
            h(">>>>>onunmuted")
        };
        u.onsystemvolumechanged = function() {
            h(">>>>>onsystemvolumechanged: " + a.aa.systemVolume);
            a.Xa(a.aa.systemVolume, !1)
        };
        b = JSON.parse(b);
        Aa = _.extend(Aa, b);
        b = null;
        this.Xi || (b = function() {
            return new ma
        });
        v.start(Aa, this, null, la, b)
    };

    function za(a) {
        var b = Ba(Ba(ra, sa), ta);
        a = CryptoJS.lib.CipherParams.create({
            ciphertext: CryptoJS.enc.Base64.parse(a)
        });
        b = CryptoJS.enc.Hex.parse(b);
        return CryptoJS.AES.decrypt(a, b, {
            iv: b
        }).toString(CryptoJS.enc.Latin1)
    }
    g.ce = function() {
        this.queue = [];
        this.jb = [];
        this.Ac = this.ready = !1;
        this.Oa = null;
        this.Pd = !1;
        this.le = null;
        this.volumeIncrement = 0.01;
        this.Xa(this.aa.systemVolume, !1);
        h(">>>>>app started")
    };
    g.onmessage = function(a, b) {
        h(">>>>>sockets:");
        h(this.jb);
        this.ready || (h("not ready yet, queue up the event"), this.queue.push({
            s: a,
            m: b
        }));
        if (0 < this.queue.length)
            if (1E3 < this.queue.length) h(">>>>>Retrying exceed max attempt, it's time to shutdown the receiver."), u.systemObject.shutdown();
            else {
                var c = this.queue.shift(),
                    d = this;
                setTimeout(function() {
                    d.onmessage(c.s, c.m)
                }, 100)
            }
        else {
            var e = b.data;
            if (e) {
                var f = new Ca(e, this.volumeIncrement),
                    k = "Sorry, this station cannot be personalized.";
                h(">>>>>received " + JSON.stringify(b.data));
                if (!f.wf() && this.rc) this.paused = this.rc = !1, v.gg();
                else if (f.zf()) v.pause(), this.paused = !0;
                else if (f.Af()) v.play(), this.paused = !1;
                else if (f.Cf()) v.Tb();
                else if (f.Ff()) {
                    e = e.src;
                    try {
                        this.Xa(parseFloat(e), !0)
                    } catch (n) {
                        h(">>>>>ignore volume command since we could not parsed: " + e)
                    }
                } else if (f.Df()) this.U.fa() ? v.Eg() : (this.P.data[Da] && (k = "Sorry, shared stations cannot be personalized."), f.Lc(new String(k)));
                else if (f.tf()) this.U.fa() ? v.ih() : (this.P.data[Da] && (k = "Sorry, shared stations cannot be personalized."),
                    f.Lc(new String(k)));
                else if (f.Ef()) this.U.fa() ? v.Fg() : (this.P.data[Da] && (k = "Sorry, shared stations cannot be personalized."), f.Lc(new String(k)));
                else if (f.Bf()) v.Li();
                else if (f.uf()) this.aa.shutdown();
                else if (f.xf()) {
                    h(">>>>>Load command received!");
                    var e = f.ie(),
                        k = f.ag(this.userAgent),
                        p = e && this.P && e === this.P.e(),
                        s = this.Oa && this.Oa.cb === k.cb;
                        h(">>>>>CCHU processing Load command: loadingStation:" + this.Ac + " ceSessionTokens match:" + s + " loadingSameStation:" + p);
                    if (this.Ac || p && s) {
                        h(">>>>>Not processing Load command: loadingStation:" + this.Ac + " ceSessionTokens match:" + s + " loadingSameStation:" + p);
                        this.updateStatus();
                        return
                    }
                    p =
                        this.jb;
                    for (s = 0; s < p.length - 1; s++) {
                        var t = p[s];
                        t !== a && (h(">>>>>closing socket: " + t), this.aa.close(p[s]))
                    }
                    this.Ac = !0;
                    k && (null != this.Oa && k.cb != this.Oa.cb ? (this.Pd = !0, this.Oa = k, this.le = f, v.bi(!0)) : (this.Oa = k, v.Le(e, k, f.he(), function() {
                        v.Jf()
                    })))
                }
                f.pe(this.P);
                f.Ab(this.U);
                f.qe(this.Da);
                f.Mc(this.progress);
                f.ue(this.paused);
                f.Xa(this.volume);
                f = f.fd();
                f = JSON.stringify(f);
                this.aa.send(a, "CASTCHAT/1.0\nContent-Length: " + f.length + "\nNamespace: urn:x-cast:com.google.cast.media\n\n" + f + "\n\n")
            } else h(">>>>>No data received with message."),
                h(">>>>>Message: "), h(b)
        }
    };
    g.ib = function(a) {
        this.Da = a;
        h(">>>>>authenticated")
    };
    g.ee = function() {
        h(">>>>auth failed");
        systemObject.shutdown()
    };
    g.fe = function() {
        h(">>>>>logged out");
        if (this.Pd) {
            this.Pd = !1;
            this.P = this.U = null;
            var a = this.le.ie();
            this.Oa ? v.Le(a, this.Oa, this.le.he(), function() {
                v.Jf()
            }) : this.aa.shutdown()
        }
    };
    g.de = function() {
        h(">>>>>audio ended")
    };
    var Aa = {},
        Aa = _.extend(Aa, u._data);
    g = xa.prototype;
    g.$f = function() {
        h(">>>>>track ended");
        this.aa.setMediaInfo(null);
        this.progress = {
            elapsedTime: 0,
            totalTime: 0,
            percentPlayed: 0
        }
    };
    g.ge = function() {
        this.paused = !0
    };
    g.$f = function() {};
    g.toggle = function() {
        v.fj()
    };
    g.Tb = function() {
        v.Tb()
    };
    g.updateStatus = function() {
        var a = new Ca({}, this.volumeIncrement);
        a.pe(this.P);
        a.Ab(this.U);
        a.qe(this.Da);
        a.Mc(this.progress);
        a.ue(this.paused);
        a.Xa(this.volume);
        this.rc && a.qg();
        this.xg && (a.tg(), this.xg = !1);
        a = a.fd();
        a = JSON.stringify(a);
//cchu        h("@@@@@@" + a);
        var a = "CASTCHAT/1.0\nContent-Length: " + a.length + "\nNamespace: urn:x-cast:com.google.cast.media\n\n" + a + "\n\n",
            b = this.jb[this.jb.length - 1];
        b && this.aa.send(b, a)
    };
    g.Xa = function(a, b) {
        h(">>>>>newVolume: " + a + " - oldVolume" + this.volume);
        b && (this.aa.systemVolume = a);
        this.volume = a;
        this.updateStatus()
    };

    function ya(a) {
        var b = {
                error: "Unknown"
            },
            c = a.split("\n");
        if (c.length) {
            if ("CASTCHAT/1.0" !== c[0].trim()) return b.error = "Unsupported version", b;
            var d = {};
            b.headers = d;
            var e, f = !1;
            for (e = 1; e < c.length; e++) {
                a = c[e];
                if (f) {
                    c = d["Content-Length"];
                    if (null != c && Number(c) === a.length) try {
                        b.data = 0 === a.length ? "" : JSON.parse(a), b.error = null, delete b.error
                    } catch (k) {
                        b.error = k.toString(), q("Could not parse: " + a)
                    } else b.error = "Mismatched content-length. header: " + c + " vs data length: " + a.length;
                    break
                }
                var n = a.indexOf(":");
                if (n &&
                    0 !== a.length) {
                    var p = a.slice(0, n);
                    d[p] = a.slice(n + 1).trim()
                } else f = !0
            }
        }
        return b
    };
    wa = r.apply(null, [51, 48, 97, 50]);

    function Ba(a, b, c, d, e, f, k) {
        for (var n = Array.prototype.slice.call(arguments, 0), p = "", s = 0; s < n.length; s++) p += n[s] + "";
        return p
    };

    function Ea(a) {
        if (0 == a.length) throw "0 length key";
        this.Aa = [608135816, 2242054355, 320440878, 57701188, 2752067618, 698298832, 137296536, 3964562569, 1160258022, 953160567, 3193202383, 887688300, 3232508343, 3380367581, 1065670069, 3041331479, 2450970073, 2306472731];
        this.$c = [3509652390, 2564797868, 805139163, 3491422135, 3101798381, 1780907670, 3128725573, 4046225305, 614570311, 3012652279, 134345442, 2240740374, 1667834072, 1901547113, 2757295779, 4103290238, 227898511, 1921955416, 1904987480, 2182433518, 2069144605, 3260701109, 2620446009,
            720527379, 3318853667, 677414384, 3393288472, 3101374703, 2390351024, 1614419982, 1822297739, 2954791486, 3608508353, 3174124327, 2024746970, 1432378464, 3864339955, 2857741204, 1464375394, 1676153920, 1439316330, 715854006, 3033291828, 289532110, 2706671279, 2087905683, 3018724369, 1668267050, 732546397, 1947742710, 3462151702, 2609353502, 2950085171, 1814351708, 2050118529, 680887927, 999245976, 1800124847, 3300911131, 1713906067, 1641548236, 4213287313, 1216130144, 1575780402, 4018429277, 3917837745, 3693486850, 3949271944, 596196993, 3549867205,
            258830323, 2213823033, 772490370, 2760122372, 1774776394, 2652871518, 566650946, 4142492826, 1728879713, 2882767088, 1783734482, 3629395816, 2517608232, 2874225571, 1861159788, 326777828, 3124490320, 2130389656, 2716951837, 967770486, 1724537150, 2185432712, 2364442137, 1164943284, 2105845187, 998989502, 3765401048, 2244026483, 1075463327, 1455516326, 1322494562, 910128902, 469688178, 1117454909, 936433444, 3490320968, 3675253459, 1240580251, 122909385, 2157517691, 634681816, 4142456567, 3825094682, 3061402683, 2540495037, 79693498, 3249098678,
            1084186820, 1583128258, 426386531, 1761308591, 1047286709, 322548459, 995290223, 1845252383, 2603652396, 3431023940, 2942221577, 3202600964, 3727903485, 1712269319, 422464435, 3234572375, 1170764815, 3523960633, 3117677531, 1434042557, 442511882, 3600875718, 1076654713, 1738483198, 4213154764, 2393238008, 3677496056, 1014306527, 4251020053, 793779912, 2902807211, 842905082, 4246964064, 1395751752, 1040244610, 2656851899, 3396308128, 445077038, 3742853595, 3577915638, 679411651, 2892444358, 2354009459, 1767581616, 3150600392, 3791627101, 3102740896,
            284835224, 4246832056, 1258075500, 768725851, 2589189241, 3069724005, 3532540348, 1274779536, 3789419226, 2764799539, 1660621633, 3471099624, 4011903706, 913787905, 3497959166, 737222580, 2514213453, 2928710040, 3937242737, 1804850592, 3499020752, 2949064160, 2386320175, 2390070455, 2415321851, 4061277028, 2290661394, 2416832540, 1336762016, 1754252060, 3520065937, 3014181293, 791618072, 3188594551, 3933548030, 2332172193, 3852520463, 3043980520, 413987798, 3465142937, 3030929376, 4245938359, 2093235073, 3534596313, 375366246, 2157278981, 2479649556,
            555357303, 3870105701, 2008414854, 3344188149, 4221384143, 3956125452, 2067696032, 3594591187, 2921233993, 2428461, 544322398, 577241275, 1471733935, 610547355, 4027169054, 1432588573, 1507829418, 2025931657, 3646575487, 545086370, 48609733, 2200306550, 1653985193, 298326376, 1316178497, 3007786442, 2064951626, 458293330, 2589141269, 3591329599, 3164325604, 727753846, 2179363840, 146436021, 1461446943, 4069977195, 705550613, 3059967265, 3887724982, 4281599278, 3313849956, 1404054877, 2845806497, 146425753, 1854211946
        ];
        this.bd = [1266315497, 3048417604,
            3681880366, 3289982499, 290971E4, 1235738493, 2632868024, 2414719590, 3970600049, 1771706367, 1449415276, 3266420449, 422970021, 1963543593, 2690192192, 3826793022, 1062508698, 1531092325, 1804592342, 2583117782, 2714934279, 4024971509, 1294809318, 4028980673, 1289560198, 2221992742, 1669523910, 35572830, 157838143, 1052438473, 1016535060, 1802137761, 1753167236, 1386275462, 3080475397, 2857371447, 1040679964, 2145300060, 2390574316, 1461121720, 2956646967, 4031777805, 4028374788, 33600511, 2920084762, 1018524850, 629373528, 3691585981, 3515945977,
            2091462646, 2486323059, 586499841, 988145025, 935516892, 3367335476, 2599673255, 2839830854, 265290510, 3972581182, 2759138881, 3795373465, 1005194799, 847297441, 406762289, 1314163512, 1332590856, 1866599683, 4127851711, 750260880, 613907577, 1450815602, 3165620655, 3734664991, 3650291728, 3012275730, 3704569646, 1427272223, 778793252, 1343938022, 2676280711, 2052605720, 1946737175, 3164576444, 3914038668, 3967478842, 3682934266, 1661551462, 3294938066, 4011595847, 840292616, 3712170807, 616741398, 312560963, 711312465, 1351876610, 322626781,
            1910503582, 271666773, 2175563734, 1594956187, 70604529, 3617834859, 1007753275, 1495573769, 4069517037, 2549218298, 2663038764, 504708206, 2263041392, 3941167025, 2249088522, 1514023603, 1998579484, 1312622330, 694541497, 2582060303, 2151582166, 1382467621, 776784248, 2618340202, 3323268794, 2497899128, 2784771155, 503983604, 4076293799, 907881277, 423175695, 432175456, 1378068232, 4145222326, 3954048622, 3938656102, 3820766613, 2793130115, 2977904593, 26017576, 3274890735, 3194772133, 1700274565, 1756076034, 4006520079, 3677328699, 720338349,
            1533947780, 354530856, 688349552, 3973924725, 1637815568, 332179504, 3949051286, 53804574, 2852348879, 3044236432, 1282449977, 3583942155, 3416972820, 4006381244, 1617046695, 2628476075, 3002303598, 1686838959, 431878346, 2686675385, 1700445008, 1080580658, 1009431731, 832498133, 3223435511, 2605976345, 2271191193, 2516031870, 1648197032, 4164389018, 2548247927, 300782431, 375919233, 238389289, 3353747414, 2531188641, 2019080857, 1475708069, 455242339, 2609103871, 448939670, 3451063019, 1395535956, 2413381860, 1841049896, 1491858159, 885456874,
            4264095073, 4001119347, 1565136089, 3898914787, 1108368660, 540939232, 1173283510, 2745871338, 3681308437, 4207628240, 3343053890, 4016749493, 1699691293, 1103962373, 3625875870, 2256883143, 3830138730, 1031889488, 3479347698, 1535977030, 4236805024, 3251091107, 2132092099, 1774941330, 1199868427, 1452454533, 157007616, 2904115357, 342012276, 595725824, 1480756522, 206960106, 497939518, 591360097, 863170706, 2375253569, 3596610801, 1814182875, 2094937945, 3421402208, 1082520231, 3463918190, 2785509508, 435703966, 3908032597, 1641649973, 2842273706,
            3305899714, 1510255612, 2148256476, 2655287854, 3276092548, 4258621189, 236887753, 3681803219, 274041037, 1734335097, 3815195456, 3317970021, 1899903192, 1026095262, 4050517792, 356393447, 2410691914, 3873677099, 3682840055
        ];
        this.cd = [3913112168, 2491498743, 4132185628, 2489919796, 1091903735, 1979897079, 3170134830, 3567386728, 3557303409, 857797738, 1136121015, 1342202287, 507115054, 2535736646, 337727348, 3213592640, 1301675037, 2528481711, 1895095763, 1721773893, 3216771564, 62756741, 2142006736, 835421444, 2531993523, 1442658625, 3659876326,
            2882144922, 676362277, 1392781812, 170690266, 3921047035, 1759253602, 3611846912, 1745797284, 664899054, 1329594018, 3901205900, 3045908486, 2062866102, 2865634940, 3543621612, 3464012697, 1080764994, 553557557, 3656615353, 3996768171, 991055499, 499776247, 1265440854, 648242737, 3940784050, 980351604, 3713745714, 1749149687, 3396870395, 4211799374, 3640570775, 1161844396, 3125318951, 1431517754, 545492359, 4268468663, 3499529547, 1437099964, 2702547544, 3433638243, 2581715763, 2787789398, 1060185593, 1593081372, 2418618748, 4260947970, 69676912,
            2159744348, 86519011, 2512459080, 3838209314, 1220612927, 3339683548, 133810670, 1090789135, 1078426020, 1569222167, 845107691, 3583754449, 4072456591, 1091646820, 628848692, 1613405280, 3757631651, 526609435, 236106946, 48312990, 2942717905, 3402727701, 1797494240, 859738849, 992217954, 4005476642, 2243076622, 3870952857, 3732016268, 765654824, 3490871365, 2511836413, 1685915746, 3888969200, 1414112111, 2273134842, 3281911079, 4080962846, 172450625, 2569994100, 980381355, 4109958455, 2819808352, 2716589560, 2568741196, 3681446669, 3329971472,
            1835478071, 660984891, 3704678404, 4045999559, 3422617507, 3040415634, 1762651403, 1719377915, 3470491036, 2693910283, 3642056355, 3138596744, 1364962596, 2073328063, 1983633131, 926494387, 3423689081, 2150032023, 4096667949, 1749200295, 3328846651, 309677260, 2016342300, 1779581495, 3079819751, 111262694, 1274766160, 443224088, 298511866, 1025883608, 3806446537, 1145181785, 168956806, 3641502830, 3584813610, 1689216846, 3666258015, 3200248200, 1692713982, 2646376535, 4042768518, 1618508792, 1610833997, 3523052358, 4130873264, 2001055236, 3610705100,
            2202168115, 4028541809, 2961195399, 1006657119, 2006996926, 3186142756, 1430667929, 3210227297, 1314452623, 4074634658, 4101304120, 2273951170, 1399257539, 3367210612, 3027628629, 1190975929, 2062231137, 2333990788, 2221543033, 2438960610, 1181637006, 548689776, 2362791313, 3372408396, 3104550113, 3145860560, 296247880, 1970579870, 3078560182, 3769228297, 1714227617, 3291629107, 3898220290, 166772364, 1251581989, 493813264, 448347421, 195405023, 2709975567, 677966185, 3703036547, 1463355134, 2715995803, 1338867538, 1343315457, 2802222074, 2684532164,
            233230375, 2599980071, 2000651841, 3277868038, 1638401717, 4028070440, 3237316320, 6314154, 819756386, 300326615, 590932579, 1405279636, 3267499572, 3150704214, 2428286686, 3959192993, 3461946742, 1862657033, 1266418056, 963775037, 2089974820, 2263052895, 1917689273, 448879540, 3550394620, 3981727096, 150775221, 3627908307, 1303187396, 508620638, 2975983352, 2726630617, 1817252668, 1876281319, 1457606340, 908771278, 3720792119, 3617206836, 2455994898, 1729034894, 1080033504
        ];
        this.dd = [976866871, 3556439503, 2881648439, 1522871579, 1555064734,
            1336096578, 3548522304, 2579274686, 3574697629, 3205460757, 3593280638, 3338716283, 3079412587, 564236357, 2993598910, 1781952180, 1464380207, 3163844217, 3332601554, 1699332808, 1393555694, 1183702653, 3581086237, 1288719814, 691649499, 2847557200, 2895455976, 3193889540, 2717570544, 1781354906, 1676643554, 2592534050, 3230253752, 1126444790, 2770207658, 2633158820, 2210423226, 2615765581, 2414155088, 3127139286, 673620729, 2805611233, 1269405062, 4015350505, 3341807571, 4149409754, 1057255273, 2012875353, 2162469141, 2276492801, 2601117357,
            993977747, 3918593370, 2654263191, 753973209, 36408145, 2530585658, 25011837, 3520020182, 2088578344, 530523599, 2918365339, 1524020338, 1518925132, 3760827505, 3759777254, 1202760957, 3985898139, 3906192525, 674977740, 4174734889, 2031300136, 2019492241, 3983892565, 4153806404, 3822280332, 352677332, 2297720250, 60907813, 90501309, 3286998549, 1016092578, 2535922412, 2839152426, 457141659, 509813237, 4120667899, 652014361, 1966332200, 2975202805, 55981186, 2327461051, 676427537, 3255491064, 2882294119, 3433927263, 1307055953, 942726286, 933058658,
            2468411793, 3933900994, 4215176142, 1361170020, 2001714738, 2830558078, 3274259782, 1222529897, 1679025792, 2729314320, 3714953764, 1770335741, 151462246, 3013232138, 1682292957, 1483529935, 471910574, 1539241949, 458788160, 3436315007, 1807016891, 3718408830, 978976581, 1043663428, 3165965781, 1927990952, 4200891579, 2372276910, 3208408903, 3533431907, 1412390302, 2931980059, 4132332400, 1947078029, 3881505623, 4168226417, 2941484381, 1077988104, 1320477388, 886195818, 18198404, 3786409E3, 2509781533, 112762804, 3463356488, 1866414978, 891333506,
            18488651, 661792760, 1628790961, 3885187036, 3141171499, 876946877, 2693282273, 1372485963, 791857591, 2686433993, 3759982718, 3167212022, 3472953795, 2716379847, 445679433, 3561995674, 3504004811, 3574258232, 54117162, 3331405415, 2381918588, 3769707343, 4154350007, 1140177722, 4074052095, 668550556, 3214352940, 367459370, 261225585, 2610173221, 4209349473, 3468074219, 3265815641, 314222801, 3066103646, 3808782860, 282218597, 3406013506, 3773591054, 379116347, 1285071038, 846784868, 2669647154, 3771962079, 3550491691, 2305946142, 453669953,
            1268987020, 3317592352, 3279303384, 3744833421, 2610507566, 3859509063, 266596637, 3847019092, 517658769, 3462560207, 3443424879, 370717030, 4247526661, 2224018117, 4143653529, 4112773975, 2788324899, 2477274417, 1456262402, 2901442914, 1517677493, 1846949527, 2295493580, 3734397586, 2176403920, 1280348187, 1908823572, 3871786941, 846861322, 1172426758, 3287448474, 3383383037, 1655181056, 3139813346, 901632758, 1897031941, 2986607138, 3066810236, 3447102507, 1393639104, 373351379, 950779232, 625454576, 3124240540, 4148612726, 2007998917, 544563296,
            2244738638, 2330496472, 2058025392, 1291430526, 424198748, 50039436, 29584100, 3605783033, 2429876329, 2791104160, 1057563949, 3255363231, 3075367218, 3463963227, 1469046755, 985887462
        ];
        this.escape = function(a) {
            for (var b = "", e = 0; e < a.length; e++) var f = a.charCodeAt(e),
                k = Math.floor(f / 16),
                f = f % 16,
                k = 10 > k ? k + 48 : k + 55,
                f = 10 > f ? f + 48 : f + 55,
                b = b + (String.fromCharCode(k) + String.fromCharCode(f));
            return b
        };
        this.Hg = function(a) {
            return Math.floor(Math.floor(Math.floor(a / 256) / 256) / 256) % 256
        };
        this.Ig = function(a) {
            return Math.floor(Math.floor(a /
                256) / 256) % 256
        };
        this.Jg = function(a) {
            return Math.floor(a / 256) % 256
        };
        this.Kg = function(a) {
            return a % 256
        };
        this.Ya = function(a, b) {
            var e = a ^ b;
            0 > e && (e = 4294967296 + e);
            return e
        };
        this.key = 56 < a.length ? a.substr(0, 56) : a;
        for (var b = a = 0; 18 > b; ++b) this.Aa[b] = this.Ya(this.Aa[b], 256 * (256 * (256 * this.key.charCodeAt(a % this.key.length) + this.key.charCodeAt((a + 1) % this.key.length)) + this.key.charCodeAt((a + 2) % this.key.length)) + this.key.charCodeAt((a + 3) % this.key.length)), a = (a + 4) % this.key.length;
        for (b = this.na = this.ma = 0; 18 > b; b += 2) this.Pa(),
            this.Aa[b] = this.ma, this.Aa[b + 1] = this.na;
        for (a = 0; 256 > a; a += 2) this.Pa(), this.$c[a] = this.ma, this.$c[a + 1] = this.na;
        for (a = 0; 256 > a; a += 2) this.Pa(), this.bd[a] = this.ma, this.bd[a + 1] = this.na;
        for (a = 0; 256 > a; a += 2) this.Pa(), this.cd[a] = this.ma, this.cd[a + 1] = this.na;
        for (a = 0; 256 > a; a += 2) this.Pa(), this.dd[a] = this.ma, this.dd[a + 1] = this.na;
        this.unescape = function(a) {
            var d = "";
            for (b = 0; b < a.length; b++) var e = a.charCodeAt(b++),
                f = a.charCodeAt(b),
                e = 58 > e ? e - 48 : 96 < e ? e - 87 : e - 55,
                f = 58 > f ? f - 48 : 96 < f ? f - 87 : f - 55,
                d = d + String.fromCharCode(16 * e + f);
            return d
        }
    }
    g = Ea.prototype;
    g.encrypt = function(a) {
        for (var b = 0; b < a.length % 8; b++) a += "0";
        for (var c = "", b = 0; b < a.length; b += 8) {
            var d = a.substr(b, 4),
                e = a.substr(b + 4, 4),
                d = d.charCodeAt(3) | d.charCodeAt(2) << 8 | d.charCodeAt(1) << 16 | d.charCodeAt(0) << 24;
            0 > d && (d = 4294967296 + d);
            e = e.charCodeAt(3) | e.charCodeAt(2) << 8 | e.charCodeAt(1) << 16 | e.charCodeAt(0) << 24;
            0 > e && (e = 4294967296 + e);
            this.ma = d;
            this.na = e;
            this.Pa();
            c += Fa(this, this.ma) + Fa(this, this.na)
        }
        return c
    };
    g.decrypt = function(a) {
        for (var b = 0; b < a.length % 16; b++) a += "0";
        for (var c = "", b = 0; b < a.length; b += 16) {
            var d = this.unescape(a.substr(b, 8)),
                e = this.unescape(a.substr(b + 8, 8)),
                d = d.charCodeAt(3) | d.charCodeAt(2) << 8 | d.charCodeAt(1) << 16 | d.charCodeAt(0) << 24;
            0 > d && (d = 4294967296 + d);
            e = e.charCodeAt(3) | e.charCodeAt(2) << 8 | e.charCodeAt(1) << 16 | e.charCodeAt(0) << 24;
            0 > e && (e = 4294967296 + e);
            this.ma = d;
            this.na = e;
            this.ld();
            c += Fa(this, this.ma) + Fa(this, this.na)
        }
        return this.unescape(c)
    };

    function Fa(a, b) {
        for (var c = "", d = [a.Kg(b), a.Jg(b), a.Ig(b), a.Hg(b)], e = 3; 0 <= e; e--) var f = Math.floor(d[e] / 16),
            k = d[e] % 16,
            f = 10 > f ? f + 48 : f + 55,
            k = 10 > k ? k + 48 : k + 55,
            c = c + (String.fromCharCode(f) + String.fromCharCode(k));
        return c
    }
    g.round = function(a, b, c) {
        return this.Ya(a, this.Ya(this.Ya(this.$c[this.Hg(b)] + this.bd[this.Ig(b)], this.cd[this.Jg(b)]) + this.dd[this.Kg(b)], this.Aa[c]))
    };
    g.Pa = function() {
        var a = this.ma,
            b = this.na,
            a = this.Ya(a, this.Aa[0]),
            b = this.round(b, a, 1),
            a = this.round(a, b, 2),
            b = this.round(b, a, 3),
            a = this.round(a, b, 4),
            b = this.round(b, a, 5),
            a = this.round(a, b, 6),
            b = this.round(b, a, 7),
            a = this.round(a, b, 8),
            b = this.round(b, a, 9),
            a = this.round(a, b, 10),
            b = this.round(b, a, 11),
            a = this.round(a, b, 12),
            b = this.round(b, a, 13),
            a = this.round(a, b, 14),
            b = this.round(b, a, 15),
            a = this.round(a, b, 16);
        this.ma = b = this.Ya(b, this.Aa[17]);
        this.na = a
    };
    g.ld = function() {
        var a = this.ma,
            b = this.na,
            a = this.Ya(a, this.Aa[17]),
            b = this.round(b, a, 16),
            a = this.round(a, b, 15),
            b = this.round(b, a, 14),
            a = this.round(a, b, 13),
            b = this.round(b, a, 12),
            a = this.round(a, b, 11),
            b = this.round(b, a, 10),
            a = this.round(a, b, 9),
            b = this.round(b, a, 8),
            a = this.round(a, b, 7),
            b = this.round(b, a, 6),
            a = this.round(a, b, 5),
            b = this.round(b, a, 4),
            a = this.round(a, b, 3),
            b = this.round(b, a, 2),
            a = this.round(a, b, 1);
        this.ma = b = this.Ya(b, this.Aa[0]);
        this.na = a
    };

    function Ga(a) {
        var b = (new Date).getTime() + Va(a);
        return "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx".replace(/[xy]/g, function(a) {
            var d = (b + 16 * Math.random()) % 16 | 0;
            b = Math.floor(b / 16);
            return ("x" == a ? d : d & 7 | 8).toString(16)
        })
    }

    function Va(a) {
        a = a || "0";
        for (var b = 0, c = 0; c < a.length; c++) b += parseInt(a[c], 16) << c;
        return b
    }

    function Wa(a) {
        if (a) {
            for (var b = 2 - a.length, c = "", d = 0; d < b; d++) c += "0";
            return c + a
        }
        return a
    };

    function Xa(a) {
        var b = window.console || b;
        b && b.debug(a)
    }

    function h(a) {
        var b = window.console || b;
        b && b.info(a)
    }

    function Ya(a) {
        var b = window.console || b;
        b && b.warn(a)
    }

    function q(a) {
        var b = window.console || b;
        b && b.error(a)
    };
    var Za = "track",
        $a = "topStationIndex",
        ab = "recentStationIndex",
        bb = "music",
        cb = "ceSessionToken",
        db = "feedback",
        eb = "receiverId",
        fb = "allowDelete",
        gb = "vendor",
        w = "audioUrlMap",
        hb = "albumArtUrl",
        ib = "artUrl",
        jb = "albumArtWidth",
        kb = "albumArtHeight",
        x = "additionalAudioUrl",
        z = "userAuthToken",
        D = "syncTime",
        E = "stationToken",
        lb = "stationId",
        mb = "partnerId",
        nb = "userId",
        F = "trackToken",
        ob = "model",
        pb = "username",
        qb = "version",
        rb = "deviceModel",
        sb = "type",
        J = "partnerAuthToken",
        tb = "adToken",
        ub = "adAttributes",
        vb = "canListen",
        wb = "hoursListened",
        xb = "syncTimeKey",
        yb = "requestKey",
        zb = "trackBookmarked",
        Ab = "sampleArtists",
        L = "songRating",
        Bb = "songName",
        Cb = "partnerName",
        Db = "artistName",
        Eb = "stationName",
        Fb = "partnerPassword",
        Gb = "modelYear",
        Hb = "httpUrl",
        Ib = "httpsUrl",
        Jb = "remoteMode",
        Kb = "disableAndo",
        Lb = "vim",
        Mb = "analyticsUrl",
        Nb = "deviceModelYear",
        Ob = "deviceType",
        Pb = "deviceVendor",
        Qb = "appVersion",
        Rb = "dev",
        Sb = "userAgent",
        Tb = "am",
        Ub = "desiredElapsedTime",
        Vb = "elapsedTime",
        Wb = "thumbRequested",
        Xb = "timestamp",
        Yb = "minutesAgo",
        Zb = "lastMinutesAgoUpdate",
        $b = "dateCreated",
        M = "isQuickMix",
        ac = "albumName",
        bc = "artistBookmarked",
        cc = "onePlaylist",
        Da = "isShared",
        dc = "allowAddMusic",
        ec = "allowRename",
        fc = "enableArtistAudioMessages",
        gc = "imageUrl",
        hc = "companyName",
        ic = "title",
        jc = "Advertisement",
        kc = "isAdvertisement",
        lc = "device_version";
    var N = 0,
        mc = "__event__" + N++,
        nc = "__event__" + N++,
        oc = "__event__" + N++,
        pc = "__event__" + N++,
        qc = "__event__" + N++,
        rc = "__event__" + N++,
        sc = "__event__" + N++,
        tc = "__event__" + N++,
        uc = "__event__" + N++,
        vc = "__event__" + N++,
        wc = "__event__" + N++,
        xc = "__event__" + N++,
        yc = "__event__" + N++,
        Bc = "__event__" + N++,
        Cc = "__event__" + N++,
        Dc = "__event" + N++,
        Ec = "__event__" + N++,
        Fc = "__event__" + N++,
        Gc = "__event__" + N++,
        Hc = "__event__" + N++,
        Ic = "__event__" + N++,
        Jc = "__event__" + N++,
        Kc = "__event__" + N++,
        Lc = "__event__" + N++,
        Mc = "__event__" + N++,
        Nc = "__event__" + N++,
        Oc =
        "__event__" + N++,
        Pc = "__event__" + N++,
        Qc = "__event__" + N++,
        Rc = "__event__" + N++,
        Sc = "__event__" + N++;
    N++;
    var Tc = "__event__" + N++,
        Uc = "__event__" + N++,
        Vc = "__event__" + N++,
        Wc = "__event__" + N++,
        Xc = "__event__" + N++,
        Yc = "__event__" + N++,
        Zc = "__event__" + N++,
        $c = "__event__" + N++,
        ad = "__event__" + N++,
        bd = "__event__" + N++,
        cd = "__event__" + N++,
        dd = "__event__" + N++,
        ed = "__event__" + N++,
        fd = "__event__" + N++,
        gd = "__event__" + N++,
        hd = "__event__" + N++,
        id = "__event__" + N++,
        jd = "__event__" + N++,
        kd = "__event__" + N++,
        ld = "__event__" + N++,
        md = "__event__" + N++,
        nd = "__event__" + N++,
        od = "__event__" + N++,
        pd = "__event__" + N++,
        qd = "__event__" + N++,
        rd = "__event__" + N++,
        sd =
        "__event__" + N++,
        td = "__event__" + N++,
        ud = "__event__" + N++,
        vd = "__event__" + N++,
        wd = "__event__" + N++,
        xd = "__event__" + N++,
        yd = "__event__" + N++,
        zd = "__event__" + N++,
        Ad = "__event__" + N++,
        Bd = "__event__" + N++,
        Cd = "__event__" + N++,
        Dd = "__event__" + N++,
        Ed = "__event__" + N++,
        Fd = "__event__" + N++,
        Gd = "__event__" + N++,
        Hd = "__event__" + N++,
        Id = "__event__" + N++,
        Jd = "__event__" + N++,
        Kd = "__event__" + N++,
        Ld = "__event__" + N++,
        Md = "__event__" + N++,
        Nd = "__event__" + N++,
        Od = "__event__" + N++,
        Pd = "__event__" + N++,
        Qd = "__event__" + N++,
        Rd = "__event__" + N++;
    N++;
    N++;
    var Sd = "__event__" + N++,
        Td = "__event__" + N++,
        Ud = "__event__" + N++,
        Vd = "__event__" + N++,
        Wd = "change:volume",
        Xd = "change:paused",
        Yd = "change:progress";

    function Q(a, b) {
        this.data = a;
        this.data[kc] = !0;
        this.data[hb] = a[gc];
        this.data[Bb] = a[ic];
        this.data[Db] = a[hc];
        this.data[ac] = jc;
        this.Wg = a.audioUrl;
        this.fh = a[hc];
        this.Ch = a[gc];
        this.title = a[ic];
        this.Xg = a[w];
        this.Zb = a.adTrackingTokens;
        this.Qc = b.data[tb];
        this.yg = b.sa();
        this.Sb = a.shouldPlayExternalVideoAd;
        var c = a.additionalAudioUrl;
        c && (this.Ie = [].concat(c));
        this.Zb || (this.Zb = []);
        this.Sb || (this.Sb = !1);
        this.timestamp = b.qc() || 0
    }
    g = Q.prototype;
    g.Ga = function() {
        if (this.Ie) return h("Using additionalAudioUrl"), {
            audioUrl: this.Ie[0]
        };
        var a = this.Xg;
        if (a) {
            var b = this.ra(a, "highQuality");
            if (null != b) return b;
            b = this.ra(a, "mediumQuality");
            if (null != b) return b;
            a = this.ra(a, "lowQuality");
            if (null != a) return a
        }
        return {
            audioUrl: this.Wg
        }
    };
    g.af = Q.prototype.Ga;
    g.ra = function(a, b) {
        var c = a[b];
        return null != c && "undefined" != typeof c ? c : null
    };
    g.bc = function() {
        var a = this.Ga();
        if (a) return (a = a.audioUrl) && "undefined" != typeof a && "" != a
    };
    g.ca = function() {
        return this.title
    };
    g.qa = function() {
        return this.fh
    };
    g.Qa = function() {
        return "Advertisement"
    };
    g.V = function() {
        return this.Qc
    };
    g.M = function() {
        return !0
    };
    g.eb = function() {
        return this.Ch
    };
    g.kc = function() {
        return null
    };
    g.fa = function() {
        return !1
    };
    g.lb = function() {
        return !1
    };
    g.Yc = function() {
        return !1
    };
    g.Xc = function() {
        return !1
    };
    g.pf = function() {
        return 0
    };
    g.ug = function(a) {
        this.timestamp = a
    };
    g.qc = function() {
        return this.timestamp
    };
    g.sa = function() {
        return this.yg
    };
    g.yd = function() {};
    g.sd = function() {};
    g.Vc = function() {};
    g.Uc = function() {};
    g.td = function() {};
    g.ud = function() {};
    g.nf = function() {};
    g.hf = function() {};
    g.bf = function() {
        return null
    };
    g.ia = function() {
        return 0
    };
    Q.prototype.getArtistName = Q.prototype.qa;
    Q.prototype.getTrackName = Q.prototype.ca;
    Q.prototype.getAlbumName = Q.prototype.Qa;
    oa = r.apply(null, [55, 98, 50, 55]);

    function Zd() {}
    Zd = Backbone.Model.extend({
        defaults: {
            progress: {},
            paused: !1,
            shouldNotifyOnIdling: !1,
            idleTriggered: !1,
            disabled: !1,
            modal: !1,
            volume: 100,
            playerVolume: 100
        },
        initialize: function() {},
        Jj: function() {
            return this.get("modal")
        },
        qk: function(a) {
            this.set({
                modal: a
            })
        },
        Sh: function(a, b) {
            null != a && "undefined" != typeof a && (100 < a ? a = 100 : 0 > a && (a = 0), b ? this.set({
                volume: a
            }, {
                silent: !0
            }) : this.set({
                volume: a
            }))
        },
        Hj: function(a) {
            a && "undefined" != typeof a && (100 < a ? a = 100 : 0 > a && (a = 0), this.set({
                playerVolume: a
            }))
        },
        vb: function() {
            return this.get("volume")
        },
        yh: function() {
            return {
                volume: this.vb(),
                playerVolume: this.get("playerVolume")
            }
        },
        Of: function(a, b) {
            this.trigger(nd, this, a, b)
        },
        li: function(a) {
            this.trigger(od, this, a)
        },
        Ci: function(a) {
            this.trigger(pd, this, a)
        },
        Nh: function(a) {
            this.trigger(Vd, this, a)
        },
        Ei: function(a) {
            this.trigger(Sd, this, a)
        },
        Di: function(a, b) {
            this.trigger(Td, this, a, b)
        },
        Xf: function(a) {
            this.trigger(Ud, this, a)
        },
        Pf: function(a) {
            this.trigger(Rd, this, a)
        },
        Cc: function(a) {
            this.trigger(ud, this, a)
        },
        Bc: function(a, b) {
            this.trigger(sc, this, a, b)
        },
        Lf: function(a) {
            this.trigger(sd, this, a)
        },
        $d: function() {
            this.trigger(qc,
                this)
        },
        Sf: function() {
            this.trigger(rc, this)
        },
        ri: function(a) {
            this.trigger(wd, this, a)
        },
        qi: function(a) {
            this.trigger(xd, this, a)
        },
        ae: function(a) {
            this.trigger(qd, this, a)
        },
        Oh: function() {
            this.trigger(Zc, this)
        },
        Kf: function(a) {
            this.trigger(rd, this, a)
        },
        Jd: function() {
            this.trigger(Hd, this)
        },
        ji: function(a) {
            this.trigger(Id, this, a)
        },
        Rh: function(a) {
            this.trigger(Md, this, a)
        },
        gi: function(a) {
            this.trigger(Nd, this, a)
        },
        wc: function() {
            this.trigger(mc, this)
        },
        sf: function(a) {
            this.set({
                progress: a
            })
        },
        nc: function() {
            return this.get("progress")
        },
        Mf: function(a) {
            this.Nc() && (this.Rf(), this.se(!0));
            this.trigger(tc, this, a)
        },
        Pj: function(a, b) {
            this.set({
                paused: !0
            });
            this.trigger(uc, this, a, b)
        },
        Oj: function(a, b) {
            this.trigger(Jc, this, a, b)
        },
        Nc: function() {
            return this.get("shouldNotifyOnIdling")
        },
        ve: function(a) {
            this.S() && a || this.set({
                shouldNotifyOnIdling: a
            })
        },
        se: function(a) {
            this.set({
                idleTriggered: a
            })
        },
        lc: function() {
            return this.get("idleTriggered")
        },
        fi: function() {
            this.trigger(Hc, this)
        },
        ei: function() {
            this.trigger(Ic, this)
        },
        Nj: function(a) {
            this.trigger(wc,
                this, a)
        },
        wi: function(a) {
            this.trigger(bd, this, a)
        },
        vc: function() {
            this.set({
                paused: !0
            })
        },
        cg: function() {
            this.set({
                paused: !0
            }, {
                silent: !0
            })
        },
        Sa: function(a) {
            a ? this.set({
                paused: !1
            }, {
                silent: !0
            }) : this.set({
                paused: !1
            })
        },
        S: function() {
            return this.get("paused")
        },
        Rj: function(a) {
            this.trigger(ld, this, a)
        },
        F: function(a, b) {
            this.trigger(kd, this, {
                method: a,
                data: b
            })
        },
        C: function(a, b) {
            this.trigger(jd, this, {
                method: a,
                data: b
            })
        },
        si: function(a) {
            this.trigger(nc, this, a)
        },
        ti: function(a) {
            this.trigger(oc, this, a)
        },
        hi: function(a) {
            this.trigger(md,
                this, a)
        },
        tc: function(a) {
            this.trigger(yc, this, a)
        },
        zc: function(a) {
            this.trigger(xc, this, a)
        },
        Dc: function(a, b) {
            this.trigger(Cc, this, a, b)
        },
        Fc: function(a, b) {
            this.trigger(Bc, this, a, b)
        },
        yc: function(a) {
            this.trigger(Ec, this, a)
        },
        Ec: function(a, b) {
            this.trigger(Fc, this, a, b)
        },
        Bi: function() {
            this.trigger(Dc, this)
        },
        vi: function() {
            this.trigger(hd, this)
        },
        ui: function() {
            this.trigger(id, this)
        },
        Sj: function() {
            this.trigger(Gc, this)
        },
        xc: function(a) {
            this.trigger(Kc, this, a)
        },
        Lh: function() {
            this.trigger(cd, this)
        },
        Mh: function() {
            this.trigger(ed,
                this)
        },
        Ob: function(a, b) {
            this.trigger(fd, this, a, b)
        },
        Zd: function(a) {
            this.trigger(dd, this, a)
        },
        Nf: function(a) {
            this.trigger(Lc, this, a)
        },
        Qj: function(a) {
            this.trigger(vc, this, a)
        },
        di: function(a) {
            this.trigger(Mc, this, a)
        },
        Rf: function() {
            this.trigger(Nc, this)
        },
        pi: function() {
            this.trigger(Fd, this)
        },
        oi: function(a) {
            this.trigger(Gd, this, a)
        },
        Tj: function(a, b) {
            this.trigger(Oc, this, {
                track: a,
                status: b
            })
        },
        sc: function(a, b) {
            this.trigger(Tc, this, a, b)
        },
        Gd: function() {
            this.trigger(Uc, this)
        },
        Hd: function() {
            this.trigger(Vc, this)
        },
        Tf: function(a) {
            this.trigger(Wc, this, a)
        },
        ci: function(a) {
            this.trigger(Xc, this, a)
        },
        uc: function(a, b) {
            this.trigger($c, this, a, b)
        },
        xi: function(a) {
            this.trigger(ad, this, a)
        },
        Ij: function(a, b) {
            this.trigger(Pc, this, a, b)
        },
        Uh: function(a) {
            this.trigger(Qc, this, a)
        },
        Vj: function(a) {
            this.trigger(Rc, this, a)
        },
        Uj: function(a, b) {
            this.Nc() && (this.Rf(), this.se(!0));
            this.trigger(Sc, this, a, b)
        },
        Qf: function(a) {
            this.trigger(vd, this, a)
        },
        yi: function(a) {
            this.trigger(yd, this, a)
        },
        Ai: function(a) {
            this.trigger(zd, this, a)
        },
        Uf: function() {
            this.trigger(Ad,
                this)
        },
        Ph: function() {
            this.trigger(Yc, this)
        },
        Th: function() {
            this.trigger(gd, this)
        },
        Wf: function(a) {
            this.trigger(Jd, this, a)
        },
        zi: function(a, b) {
            this.trigger(Bd, this, a, b)
        },
        Vf: function(a) {
            this.trigger(Dd, this, a)
        },
        be: function(a) {
            this.trigger(Ed, this, a)
        },
        ni: function() {
            this.trigger(td, this)
        },
        ii: function() {
            this.trigger(Cd, this)
        },
        mi: function(a) {
            this.trigger(Ld, this, a)
        },
        Qh: function(a, b, c) {
            this.trigger(Kd, a, b, c)
        },
        Id: function() {
            this.n() || (this.ki(), this.set({
                disabled: !0
            }))
        },
        n: function() {
            return this.get("disabled")
        },
        ki: function() {
            this.trigger(pc, this)
        }
    });

    function $d(a) {
        this.jd = a.categoryName;
        this.Cb = [];
        this.Yi = {};
        this.Dd = {};
        a = a.stations;
        for (var b = 0; b < a.length; b++) {
            var c = new ae(a[b]);
            this.Cb.push(c);
            this.Dd[c.e()] = b;
            this.Yi[c.e()] = c
        }
        var d = this;
        this.name = function() {
            return d.jd
        };
        this.stations = function() {
            return d.Cb
        };
        this.artUrls = function() {
            for (var a = [], b = 0; b < d.Cb.length && 4 > b; b++) a.push(d.Cb[b].sb());
            return a
        };
        this.id = function() {
            return be(d)
        }
    }
    $d.prototype.getName = function() {
        return this.jd
    };

    function be(a) {
        return a.jd.replace(/\s|&/g, "").replace(/\//g, "")
    };

    function ce(a) {
        if (a) {
            this.dh = a.checksum;
            a = a.categories;
            this.bh = {};
            this.Dd = {};
            this.Me = [];
            var b = this;
            _.each(a, function(a, c) {
                var f = new $d(a);
                b.Me.push(f);
                b.bh[be(f)] = f;
                b.Dd[be(f)] = c
            });
            var c = this;
            c.categories = function() {
                return c.Me
            }
        }
    };
    pa = Ba(na, wa, oa, r.apply(null, [97, 56, 101, 51]));

    function ae(a) {
        this.Za = a ? a.artUrl : null;
        this.stationToken = a ? a.stationToken : null;
        this.yg = a ? a.stationId : null;
        this.zg = a ? a.stationName : null;
        this.zb = a ? a.sampleArtists : null;
        var b = this;
        this.getName = function() {
            return b.zg
        };
        this.getToken = function() {
            return b.stationToken
        };
        this.getSampleArtists = function() {
            return b.zb
        };
        this.getSampleArtistsText = function() {
            var a = "";
            if (b.zb)
                for (var d = 0; d < b.zb.length; d++) a += b.zb[d], d + 1 != b.zb.length && (a += ", ");
            return a
        };
        this.getArtUrl = function() {
            return b.Za
        }
    }
    ae.prototype.getName = function() {
        return this.zg
    };
    ae.prototype.e = function() {
        return this.stationToken
    };
    ae.prototype.sb = function() {
        return this.Za
    };
    ae.prototype.mf = function() {
        return this.zb
    };

    function de() {}
    de.gj = "tp";
    de = Backbone.Model.extend({
        Mg: 5,
        Qg: 42E4,
        Og: 20,
        defaults: {
            eventModel: null,
            user: null,
            currentTrack: null,
            currentStation: null,
            trackMap: {},
            stationList: null,
            stationListChecksum: null,
            lastUsedStationToken: null,
            lastUsedStationResult: null,
            lastUsedTrack: null,
            did: null,
            cm: null,
            ct: null,
            rid: null,
            genreStations: null,
            stationSkipLimitMap: {},
            skipUnit: "hour",
            skipLimit: 6,
            fatalAudioError: !1,
            audioErrorCount: 0,
            ps3: !1,
            ps4: !1,
            nintendo: !1,
            sortType: "date",
            audioTypes: [],
            st: !0,
            dv: !1,
            am: !1,
            rm: !1,
            lm: !1,
            ato: 20,
            va: !1,
            lvat: 0,
            ard: null,
            fexf: !1,
            exf: !1
        },
        initialize: function(a) {
            this.set({
                eventModel: a.b
            });
            this.set({
                stationList: a.Cb
            })
        },
        reset: function() {
            this.set({
                user: null
            });
            this.set({
                currentTrack: null
            });
            this.set({
                currentStation: null
            });
            this.set({
                trackMap: {}
            });
            this.set({
                stationList: null
            });
            this.set({
                stationListChecksum: null
            });
            this.set({
                lastUsedStationToken: null
            });
            this.set({
                lastUsedStationResult: null
            });
            this.set({
                lastUsedTrack: null
            });
            this.set({
                did: null
            });
            this.set({
                genreStations: null
            });
            this.set({
                stationSkipLimitMap: {}
            });
            this.set({
                topStations: null
            });
            this.set({
                recentStations: null
            });
            this.set({
                exf: !1
            })
        },
        Mi: function() {
            this.reset();
            this.set({
                fatalAudioError: !1
            });
            var a = this.get("eventModel");
            a && a.set({
                disabled: !1
            })
        },
        oa: function() {
            return this.get("lm")
        },
        Fh: function() {
            this.set({
                lm: !0
            })
        },
        Jh: function() {
            this.set({
                rm: !0
            })
        },
        Eh: function() {
            this.set({
                am: !0
            })
        },
        R: function() {
            return this.get("am")
        },
        mh: function() {
            this.set({
                dv: !0
            })
        },
        Yh: function() {
            return this.get("dv")
        },
        Xh: function() {
            return this.get("st")
        },
        Re: function() {
            this.set({
                st: !1
            })
        },
        da: function() {
            return this.get("rm")
        },
        gb: function() {
            return this.get("sortType")
        },
        we: function(a) {
            return this.set({
                sortType: a
            })
        },
        $e: function() {
            return this.get("appVersion")
        },
        Oi: function(a) {
            return this.set({
                appVersion: a
            })
        },
        yf: function() {
            return this.get("ps3")
        },
        Hh: function() {
            this.set({
                ps3: !0
            })
        },
        Ld: function() {
            return this.get("ps4")
        },
        Ih: function() {
            this.set({
                ps4: !0
            })
        },
        Wh: function() {
            return this.get("nintendo")
        },
        Gh: function() {
            this.set({
                nintendo: !0
            })
        },
        Dh: function() {
            var a = this.get("audioErrorCount");
            a++;
            a <= this.Mg ? this.set({
                    audioErrorCount: a
                }) :
                (this.get("eventModel").di({
                    audioErrorCount: a
                }), this.eg())
        },
        eg: function() {
            this.set({
                audioErrorCount: 0
            })
        },
        qf: function() {
            return this.get("fatalAudioError")
        },
        lg: function(a) {
            this.set({
                fatalAudioError: a
            })
        },
        Va: function(a) {
            this.set({
                did: a
            })
        },
        ga: function() {
            return this.get("did")
        },
        Ze: function(a) {
            var b = {};
            b[0] = Ga(a);
            b[1] = (new Date).getTime();
            b[2] = !1;
            h("generated deviceId: " + JSON.stringify(b));
            this.Va(b);
            return b
        },
        ta: function(a, b) {
            var c = this.get("stationSkipLimitMap");
            a && this.Mb() === a && c[a] ? h(a + " already initialized") :
                c[a] || (c[a] = b || new ee(this.wh(), this.xh()))
        },
        pc: function(a) {
            return this.get("stationSkipLimitMap")[a]
        },
        Vi: function(a) {
            this.set({
                skipUnit: a
            })
        },
        xh: function() {
            return this.get("skipUnit")
        },
        Ui: function(a) {
            this.set({
                skipLimit: a
            })
        },
        wh: function() {
            return this.get("skipLimit")
        },
        Ma: function(a, b, c) {
            this.set({
                user: a
            });
            if (a && fe(a)) {
                var d = a.Y.version;
                d && (h("about to call notifyOnListenerVersionMappingFound: " + d), this.get("eventModel").li(d));
                h("about to call notifyOnAuthenticated: " + b + " reauth: " + c + " user:" + JSON.stringify(a));
                this.get("eventModel").Of(b, c)
            } else a && "Partner" == b ? (h("Partner auth: " + JSON.stringify(a)), this.get("eventModel").Of("Partner", c)) : h("cleared the current user");
            a || this.eh()
        },
        q: function() {
            return this.get("user")
        },
        kb: function(a) {
            var b = this.K();
            if (null != a && (!b || a.e() != b.e())) {
                var c = this.get("eventModel");
                c.trigger(Pd, b, a);
                this.set({
                    currentStation: a
                });
                c.trigger(Od, b, a);
                this.R() ? this.og(a.G) : this.te(a.e())
            }
        },
        nb: function() {
            this.set({
                currentStation: null
            }, {
                silent: !0
            })
        },
        K: function() {
            return this.get("currentStation")
        },
        Bg: function(a) {
            h("storing the track: " + a.ca() + " with token: " + a.e());
            var b = this.H();
            this.set({
                currentTrack: a
            });
            this.Rg(a);
            this.get("eventModel").trigger(Qd, b, a)
        },
        Ba: function() {
            this.set({
                currentTrack: null
            }, {
                silent: !0
            })
        },
        H: function() {
            return this.get("currentTrack")
        },
        Rg: function(a) {
            var b = this.Mb(),
                c = this.get("trackMap");
            if (c[b]) {
                if (a.I.V() == c[b][c[b].length - 1].I.V()) return;
                c[b].length === this.Og && c[b].shift();
                c[b].push(a)
            } else c[b] = [a];
            this.set({
                trackMap: c
            })
        },
        Ad: function(a, b) {
            var c = this.Mb(),
                d = this.get("trackMap")[c][a];
            if (d && d.I.V() == b) return d;
            this.get("trackMap")[c].forEach(function(a) {
                if (a.I.V() == b) return a
            });
            return null
        },
        Vb: function(a, b, c) {
            var d = this.get("eventModel");
            b = this.R() ? new ge(b, {
                comparator: !1
            }) : new ge(b, {
                comparator: function(a) {
                    return a.G.data[M] ? "a" : "z" + a.getName().toLowerCase()
                }
            });
            this.set({
                stationList: b
            });
            this.set({
                stationListChecksum: a
            });
            d.zi(b, !1);
            if (c) h("Don't try to select the station");
            else {
                var e = null,
                    f = this.Mb();
                h("storing the station, checking for lastUsedStationToken: " + f);
                b.forEach(function(a) {
                    f &&
                        f == a.e() && (e = a)
                });
                e ? (h("store the station list and found: " + e.getName()), this.kb(e)) : d.ii()
            }
        },
        Dg: function(a) {
            this.set({
                stationListChecksum: a
            })
        },
        eh: function() {
            this.set({
                topStations: null
            });
            this.set({
                recentStations: null
            })
        },
        Be: function(a) {
            function b(a) {
                return a.G.zd()
            }
            for (var c = this.O(), d = this.get("eventModel"), e = [], f = 0; f < a.length; f++)
                for (var k = a[f], n = 0; n < c.length; n++) {
                    var p = c.at(n);
                    if (p.e() == k.stationId) {
                        p.xe(parseInt(k.stationIndex, 10));
                        p.re(parseInt(k.hoursListened, 10));
                        e.push(p);
                        break
                    }
                }
            a && 0 != a.length ||
                null == this.Ra() ? (a = new ge(e, {
                    comparator: b
                }), this.set({
                    topStations: a
                })) : a = this.Ra();
            d.Vf(a)
        },
        Ae: function(a, b) {
            function c(a) {
                return a.G.Ha()
            }
            for (var d = this.O(), e = this.get("eventModel"), f = [], k = !1, n = 0; n < a.length; n++) {
                var p = a[n];
                p.stationId == b && (k = !0);
                for (var s = 0; s < d.length; s++) {
                    var t = d.at(s);
                    t.e() == p.stationId && (t.Wa(parseInt(p.stationIndex, 10)), t.Bb(parseInt(p.minutesAgo, 10)), t.e() == b && 0 != t.Ha() ? (t.Wa(0), t.Bb(0)) : b && 0 == t.Ha() && t.e() != b && t.Wa(1), f.push(t))
                }
            }
            k || (d = this.Xe(b)) && f.push(d);
            a && 0 != a.length ||
                null == this.Ia() ? (f = new ge(f, {
                    comparator: c
                }), this.set({
                    recentStations: f
                })) : f = this.Ia();
            e.be(f)
        },
        Xe: function(a) {
            for (var b = this.O(), c = 0; c < b.length; c++) {
                var d = b.at(c);
                if (d.e() == a) return d.Wa(0), d.Bb(0), d
            }
            return null
        },
        rg: function(a) {
            for (var b = this.Ia(), c = !1, d = this.get("eventModel"), e = [], f = 0, k = 0; k < b.length; k++) {
                var n = b.at(k);
                n.e() == a && (c = !0, f = n.Ha(), n.Wa(0), n.Bb(0), e.push(n))
            }
            c || (n = this.Xe(a)) && e.push(n);
            for (c = 0; c < b.length; c++) n = b.at(c), n.e() != a && (n.Ha() < f && n.Wa(n.Ha() + 1), e.push(n));
            b = new ge(e, {
                comparator: function(a) {
                    return a.G.Ha()
                }
            });
            this.set({
                recentStations: b
            });
            d.be(b)
        },
        cj: function(a) {
            this.set({
                genreStations: a
            })
        },
        fb: function() {
            return this.get("genreStations")
        },
        th: function() {
            return this.get("genreStations") ? this.get("genreStations").dh : ""
        },
        Ja: function() {
            return this.get("stationListChecksum")
        },
        O: function() {
            return this.get("stationList")
        },
        Ra: function() {
            return this.get("topStations")
        },
        Ia: function() {
            return this.get("recentStations")
        },
        Mb: function() {
            return this.get("lastUsedStationToken")
        },
        te: function(a) {
            this.set({
                lastUsedStationToken: a
            })
        },
        og: function(a) {
            this.set({
                lastUsedStationResult: a
            })
        },
        kf: function() {
            return this.get("lastUsedStationResult")
        },
        vd: function() {
            return this.get("lastUsedTrack")
        },
        pg: function(a) {
            h("storing " + (a ? a.e() + " to lastUsedTrack" : "null track"));
            this.set({
                lastUsedTrack: a
            })
        },
        Kc: function(a) {
            a && 0 < a.length && this.set({
                audioTypes: a
            })
        },
        tb: function() {
            return this.get("audioTypes")
        },
        Pi: function(a) {
            a && a != this.cf() && this.set({
                ato: a
            })
        },
        cf: function() {
            return this.get("ato")
        },
        Wi: function(a) {
            a && a != this.Bd() && this.set({
                va: a
            })
        },
        Ri: function(a) {
            a &&
                a != this.ff() && this.set({
                    "va-ex": a
                })
        },
        ff: function() {
            return this.get("va-ex")
        },
        Bd: function() {
            return this.get("va")
        },
        Qi: function(a) {
            this.set({
                cm: a
            })
        },
        ub: function() {
            return this.get("cm")
        },
        Ni: function(a) {
            this.set({
                ard: a
            })
        },
        sh: function() {
            return this.get("ard")
        },
        Si: function(a) {
            this.set({
                lvat: a
            })
        },
        uh: function() {
            return this.get("lvat")
        },
        $g: function() {
            var a = this.uh();
            return 0 == a || a + this.Qg <= Date.now() ? !0 : !1
        },
        Ti: function(a) {
            this.set({
                siv: a
            })
        },
        vh: function() {
            return this.get("siv")
        },
        rh: function() {
            return this.get("fexf")
        },
        ng: function(a) {
            this.set({
                fexf: a
            })
        },
        kg: function(a) {
            this.set({
                exf: a
            })
        },
        ef: function() {
            return this.get("exf")
        }
    });

    function R() {}
    R = Backbone.Model.extend({
        defaults: {
            playlist: [],
            active: !1,
            currentTrack: null
        },
        initialize: function(a) {
            this.G = a.G;
            this.b = a.b;
            this.d = a.d;
            this.B = a.B;
            this.g = a.g;
            this.za = a.za
        },
        getName: function() {
            return this.G.getName()
        },
        e: function() {
            return this.G.e()
        },
        sb: function() {
            return this.G.sb()
        },
        Fj: function() {
            return this.G
        },
        xe: function(a) {
            this.G.xe(a)
        },
        zd: function() {
            return this.G.zd()
        },
        Wa: function(a) {
            this.G.Wa(a)
        },
        Ha: function() {
            return this.G.Ha()
        },
        re: function(a) {
            this.G.re(a)
        },
        gf: function() {
            return this.G.gf()
        },
        of: function() {
            return this.G.of()
        },
        Bb: function(a) {
            this.G.Bb(a)
        },
        lf: function() {
            return this.G.lf()
        },
        jf: function() {
            return this.G.jf()
        },
        setActive: function(a) {
            this.set({
                active: a
            });
            a || ((a = this.H()) && a.stop(), this.Ba())
        },
        Ba: function() {
            this.set({
                currentTrack: null
            }, {
                silent: !0
            })
        },
        Vh: function() {
            return this.get("active")
        },
        H: function() {
            return this.get("currentTrack")
        },
        Ab: function(a) {
            this.set({
                currentTrack: a
            })
        },
        ha: function() {
            return this.get("playlist")
        },
        sg: function(a) {
            this.set({
                playlist: a
            })
        },
        start: function() {
            this.setActive(!0);
            var a = this.H();
            null !=
                a ? a.load() : 0 == this.ha().length ? this.Od(_.bind(this.W, this)) : this.W()
        },
        W: function() {
            h("playlist length: " + this.ha().length);
            if (this.Vh()) {
                this.b.S() && this.b.Sa(!0);
                var a = this.H();
                null != a && (h("playNextTrack - playlist: " + this.ha().length + " currentTrack: " + a.ca()), a.stop(), this.b.Wf(a));
                0 < this.ha().length ? (a = this.ha().shift(), a.M() ? this.Ue(a) : this.qd(a), this.d.da() && this.d.ub() && !this.d.oa() || a.M() || 0 != this.ha().length || (this.timeout && this.ne(), h("preloading playlist since there is no tracks"), this.Od())) :
                    (this.timeout && this.ne(), this.Od(_.bind(this.W, this)))
            } else h("Station isn't active, skip playing of next track")
        },
        qd: function(a) {
            if (a && !this.d.qf()) {
                h("doPlay - track: " + a.ca());
                if (this.d.da()) {
                    var b = S(a).kc() || 0;
                    a.wb = b
                }
                a.load();
                this.Ab(a)
            }
        },
        Ue: function(a) {
            if (this.d.$g())
                if (a)
                    if (he(a)) this.qd(a);
                    else {
                        var b = this,
                            c = new T;
                        c.k = function(a) {
                            h("error encountered: " + a);
                            b.b.F("ad.getAdMetadata", a);
                            b.W()
                        };
                        c.l = function(c) {
                            if (c = c.result) {
                                var d = new Q(c, S(a));
                                b.d.ff() && d.Sb ? (c = function(c) {
                                    b.dg(d, a, c)
                                }, _.bind(c,
                                    b), b.b.Uh(c)) : d.bc() ? b.qd(new ie(b.b, b.d, d, je, (new Date).getTime())) : b.W()
                            } else b.W()
                        };
                        c.j = function(a) {
                            h("error encountered: " + a);
                            b.b.C("ad.getAdMetadata", a);
                            b.W()
                        };
                        var d = this.d.q();
                        ke(this.g.Ge, new V(this.d, this.b, this.B, c, _.bind(this.Ue, this, a)), d, a.e(), this.za, this.d.sh())
                    }
            else h("doPlayAd with empty track - currentTrack: " + this.H() + " paused: " + this.b.S());
            else this.W()
        },
        dg: function(a, b, c) {
            this.W();
            if (c) {
                this.d.Si(Date.now());
                var d = this;
                c = new T;
                c.k = function(a) {
                    h("error encountered: " + a);
                    d.b.F("ad.registerAd",
                        a)
                };
                c.l = function() {
                    h("ad registered")
                };
                c.j = function(a) {
                    h("error encountered: " + a);
                    d.b.C("ad.registerAd", a)
                };
                le(this.g.Ge, new V(this.d, this.b, this.B, c, _.bind(this.dg, this, a, b)), this.d.q(), a)
            }
        },
        Xj: function() {},
        Gj: function() {
            this.ha().shift()
        },
        Od: function(a) {
            this.Te(a)
        },
        Te: function(a) {
            var b = this,
                c = new T;
            c.k = function(a) {
                b.b.F("station.getPlaylist", a)
            };
            c.l = function(c) {
                b.Cg(c.result, a)
            };
            c.j = function(a) {
                b.b.C("station.getPlaylist", a)
            };
            this.b.pi();
            var d = this.d.q(),
                e = this.d.da() ? this.d.ub() : null;
            h("calling api - getPlaylist");
            this.g.xa.ha(new V(this.d, this.b, this.B, c, _.bind(this.Te, this, a)), d, this.G.e(), this.za, !1, e, this.d)
        },
        Cg: function(a, b) {
            for (var c = a.items, d = [], e = 0; e < c.length; e++) {
                var f = new W(c[e]);
                d.push(new ie(this.b, this.d, f, je, (new Date).getTime()))
            }
            this.sg(d);
            this.b.oi(this);
            b && b();
            (c = this.H()) && (c = c.ca());
            h("storePlaylist - currentTrack " + c + " paused: " + this.b.S() + " callback: " + typeof b)
        },
        qh: function() {
            this.ha().length = 0
        },
        Ye: function() {
            var a = this.H();
            null != a && (this.b.Wf(a), a.stop(), this.Ba());
            for (; 0 < this.ha().length;) this.ha().shift().stop()
        },
        ne: function() {
            clearTimeout(this.timeout);
            this.timeout = null
        }
    });

    function ie(a, b, c, d, e) {
        this.b = a;
        this.d = b;
        this.I = c;
        this.oh = d;
        this.gh = !1;
        this.timestamp = e;
        this.progress = null;
        this.wb = 0;
        this.N = null;
        this.playing = !1
    }

    function S(a) {
        a.I.ug(a.timestamp);
        return a.I
    }
    g = ie.prototype;
    g.load = function() {
        1 == me && this.stop();
        h("MediaTrack is calling the factory method...");
        (this.N = this.oh.apply(null, [this.b, this.d, this])) && ne(this, this.N)
    };

    function ne(a, b) {
        h("onReady of mediaTrack");
        a.N = b;
        var c;
        c = a.d.da() ? a.I.af() : a.I.Ga();
        var d = c.audioUrl,
            e = c.encoding;
        c = c.bitrate || "128";
        a.d.Bg(a);
        a.Yb(a.b.vb());
        b.load(d, a.wb, c, function(b, c) {
            var d = b.substr(b.lastIndexOf(".") + 1, 3);
            return "mp4" == d && a.d.Ld() ? "m4v" : "aac" == d || "mp4" == d && !a.d.Bd() ? "m4a" : c ? c : d
        }(d, e));
        a.play(a.wb)
    }
    g.play = function(a) {
        null == this.N ? this.load() : this.b.S() || (this.playing = !0, this.N.play(a))
    };
    g.stop = function() {
        null != this.N && (this.N.stop(), this.N = null, this.playing = !1)
    };
    g.pause = function() {
        null != this.N && this.N.pause()
    };
    g.La = function() {
        this.playing ? this.N.La() : this.play()
    };
    g.Yb = function(a) {
        null != this.N && this.N.Yb(this.I.pf(), a)
    };
    g.Na = function(a) {
        null != this.N && this.N.Na(a)
    };
    g.M = function() {
        return this.I.M()
    };
    g.bc = function() {
        return this.I.bc()
    };
    g.hc = function() {
        this.I.hc()
    };
    g.jc = function() {
        this.I.jc()
    };
    g.ic = function() {
        this.I.ic()
    };
    g.Xd = function() {
        this.I.Xd()
    };
    g.Vd = function() {
        this.I.Vd()
    };
    g.Ud = function() {
        this.I.Ud()
    };
    g.Ua = function() {
        this.I.Ua()
    };
    g.ia = function() {
        return this.I.ia()
    };
    g.Md = function() {
        return this.I.Md()
    };
    g.Wd = function() {
        this.I.Wd()
    };
    g.cc = function() {
        this.I.cc()
    };
    g.Kd = function() {
        return this.I.Kd()
    };
    g.Td = function() {
        this.I.Td()
    };
    g.e = function() {
        return this.I.V()
    };
    g.ca = function() {
        return this.I.ca()
    };
    g.fa = function() {
        return this.I.fa()
    };
    g.Wc = function() {
        return this.I.Wc()
    };
    g.lb = function() {
        return this.I.lb()
    };
    g.nc = function() {
        return this.progress
    };
    g.Mc = function(a) {
        this.progress = a
    };

    function he(a) {
        a = a.I.Ga().audioUrl;
        if (null == a) return !1;
        a = a.substr(a.lastIndexOf(".") + 1, 3);
        return "mp4" == a || "m4v" == a ? !0 : !1
    };
    qa = r.apply(null, [102, 97, 98, 100]);

    function oe(a, b, c) {
        if ("error" != a) {
            this.timestamp = (new Date).getTime();
            this.ea = a[J];
            this.w = a[mb];
            this.$i = a.stationSkipLimit;
            this.aj = a.stationSkipUnit;
            this.Gg = a.urls;
            this.startTime = c;
            if (c = a[D]) this.ej = parseInt(b.decrypt(c), 10);
            this.Ea = a.deviceProperties || {};
            this.Gg && (this.$b = this.Gg.autoComplete)
        }
    }
    oe.prototype.v = function() {
        return this.ej + (new Date).getTime() - this.startTime
    };

    function V(a, b, c, d, e) {
        this.J = d;
        this.d = a;
        this.b = b;
        this.B = c;
        this.Ta = null;
        a = this.d.da() && this.d.oa();
        b = !this.d.da() && !this.d.oa();
        d = this.d.da() && !this.d.oa() && !this.d.ub();
        var f = this.d.da() && !!this.d.ub() && !this.d.oa();
        d ? this.Ta = _.bind(c.Z, c, !0, e, !0) : a ? this.Ta = _.bind(c.Z, c, !0, e) : f ? (a = this.d.ub(), this.Ta = _.bind(c.Ka, c, a.cb, a.ke, !0, e)) : b || (this.Ta = _.bind(c.Z, c, !0, e));
        this.Yd = hex_md5(e.toString());
        (c = pe[this.Yd]) && "undefined" != typeof c || (c = 0);
        this.Zc = c + 1;
        pe[this.Yd] = this.Zc
    }
    var pe = {};
    V.prototype.l = function(a) {
        qe(this);
        this.J.l(a)
    };
    V.prototype.j = function(a) {
        h("Failure detected, retrying for: " + JSON.stringify(a));
        if (2 < this.Zc) h("Stop retrying since the attempts has reached: 2"), qe(this), this.J.j(a);
        else if (this.d.da() || this.d.oa()) {
            var b = parseInt(a.code, 10);
            h("RetryHandler detected code: " + b);
            1 != b && (1001 == b ? this.Ta && (h("refreshing authToken"), this.Ta()) : 13 == b ? this.Ta && (h("refreshing authToken"), this.Ta()) : (qe(this), h("Can't retry, falling back to the handler.onFailure()"), this.J.j(a)))
        } else qe(this), h("Can't retry because it's not supported. Falling back to the handler.onFailure()"),
            this.J.j(a)
    };
    V.prototype.k = function(a) {
        qe(this);
        this.J.k(a)
    };

    function qe(a) {
        pe[a.Yd] = 0;
        a.Zc = 0
    };

    function re(a) {
        this.jg = "";
        this.fg = [];
        if (a && a.length) {
            a = a.split("\n");
            this.jg = a[0];
            for (var b = 1; b < a.length; b++) {
                var c = new se(a[b]);
                c.e() && this.fg.push(c)
            }
            var d = this;
            this.search = function() {
                return d.jg
            };
            this.results = function() {
                return d.fg
            }
        }
    };

    function se(a) {
        this.Je = this.Wb = this.Xb = null;
        if (a && 0 < a.indexOf("\t")) {
            a = a.split("\t");
            this.Xb = a[0];
            "G" === this.Xb[0] ? (this.Wb = a[1], this.Za = 3 == a.length ? a[2] : null) : "R" === this.Xb[0] ? (this.Wb = a[1], this.Za = 3 == a.length ? a[2] : null) : "C" === this.Xb[0] ? (this.Wb = a[1], this.Za = 3 == a.length ? a[2] : null) : (this.Je = a[1], this.Wb = a[2], this.Za = 4 == a.length ? a[3] : null);
            var b = this;
            this.token = function() {
                return b.e()
            };
            this.text = function() {
                return b.Wb
            };
            this.artist = function() {
                return b.Je
            };
            this.url = function() {
                return b.sb()
            }
        }
    }
    se.prototype.e = function() {
        return this.Xb
    };
    se.prototype.sb = function() {
        return this.Za
    };

    function ee(a, b) {
        this.Hc = a;
        this.Rc = b;
        this.time = 0;
        this.reset()
    }
    g = ee.prototype;
    g.clone = function() {
        return new ee(this.Hc, this.Rc)
    };
    g.Cd = function() {
        0 != this.ab && (this.ab -= 1, h("Skipped, " + this.ab + " skip left"))
    };

    function te(a) {
        return a.vf() ? (a.reset(), !0) : 0 < a.ab
    }
    g.vf = function() {
        return 0 > this.time - this.getTime()
    };
    g.reset = function() {
        this.ab = this.Hc;
        var a = this.getTime(),
            b;
        b = "hour" == this.Rc ? 36E5 : "minute" == this.Rc ? 6E4 : void 0;
        this.time = a + b
    };
    g.getTime = function() {
        var a = new Date;
        a.setMilliseconds(0);
        a.setSeconds(0);
        "minute" != this.Rc && a.setMinutes(0);
        return a.getTime()
    };
    g.toJSON = function() {
        var a = {};
        a.o = this.Hc;
        a.t = this.time;
        a.a = this.ab;
        return a
    };

    function ue(a) {
        h("skip from JSON is: " + JSON.stringify(a));
        var b = a.a,
            c = a.t || 0,
            d = a.o || 6;
        a = new ee(d, a.u || "hour");
        a.time = c;
        for (c = 0; c < d - b; c++) a.Cd();
        h("Skip fromJSON is: " + a.ab + " and " + a.Hc + " and " + a.time);
        return a
    };

    function ge() {}
    ge = Backbone.Collection.extend({
        Bj: function(a) {
            return this.detect(function(b) {
                return b.sa() === a
            })
        },
        ph: function(a) {
            return this.detect(function(b) {
                return b.e() === a
            })
        },
        Ca: function(a) {
            var b = this.map(function(a) {
                return a.G
            });
            "date" === a ? b = _.sortBy(b, function(a) {
                return a.data[M] ? Number.NEGATIVE_INFINITY : -(0 !== a.data[$b] ? a.data[$b].time : 0)
            }) : "recent" === a ? b = _.sortBy(b, function(a) {
                return a.data[M] ? Number.NEGATIVE_INFINITY : a.data.recentStationsSortIndex || 0
            }) : "top" === a && (b = _.sortBy(b, function(a) {
                return a.data[M] ? Number.NEGATIVE_INFINITY :
                    a.data.topStationsSortIndex || 0
            }));
            return b
        }
    });

    function X(a) {
        var b = {};
        b[$a] = -1;
        b[ab] = -1;
        b[wb] = 0;
        b[Yb] = 0;
        b[Zb] = (new Date).getTime();
        b[M] = !1;
        b[dc] = null;
        b[ec] = null;
        b[ib] = null;
        b[E] = null;
        b[fb] = null;
        b[Da] = null;
        b[lb] = null;
        b[Eb] = null;
        b[Ab] = null;
        b[$b] = 0;
        b[ub] = null;
        b[cc] = null;
        b[bb] = null;
        b[db] = null;
        b[fc] = null;
        this.data = _.extend(b, a)
    }
    X.prototype = {
        Ee: 60,
        De: 1440,
        Fe: 10080,
        Tc: 43200,
        Pg: 6E4,
        sa: function() {
            return this.data[lb]
        },
        xe: function(a) {
            this.data[$a] = a
        },
        re: function(a) {
            this.data[wb] = a
        },
        Bb: function(a) {
            this.data[Yb] = a;
            this.data[Zb] = (new Date).getTime()
        },
        Wa: function(a) {
            this.data[ab] = a
        },
        zd: function() {
            return this.data[$a]
        },
        gf: function() {
            return this.data[wb]
        },
        lf: function() {
            return this.data[Yb]
        },
        Ha: function() {
            return this.data[ab]
        },
        jf: function() {
            this.data[Yb] || 0 === this.data[Yb] || (this.data[Yb] = this.Tc);
            var a;
            a = ((new Date).getTime() - this.data[Zb]) /
                this.Pg;
            a = Math.round(this.data[Yb] + a);
            a >= this.Tc ? (a = Math.round(a / this.Tc), a = 1 == a ? a.toString() + " month" : a.toString() + " months") : a >= this.Fe ? (a = Math.round(a / this.Fe), a = 1 == a ? a.toString() + " week" : a.toString() + " weeks") : a >= this.De ? (a = Math.round(a / this.De), a = 1 == a ? a.toString() + " day" : a.toString() + " days") : a >= this.Ee ? (a = Math.round(a / this.Ee), a = 1 == a ? a.toString() + " hour" : a.toString() + " hours") : a = 10 > a ? "A few minutes" : a.toString() + " minutes";
            return a + " ago"
        },
        of: function() {
            var a = this.data[wb];
            return !a || 1 > a ? "Less than an hour listened" :
                1 == a ? a + " hour listened" : a + " hours listened"
        },
        getName: function() {
            return this.data[Eb]
        },
        e: function() {
            return this.data[E]
        },
        sb: function() {
            return this.data[ib]
        },
        mf: function() {
            return this.data[Ab]
        },
        toJSON: function() {
            return this.data
        }
    };
    X.prototype.getToken = X.prototype.e;
    X.prototype.getName = X.prototype.getName;
    X.prototype.getSampleArtists = X.prototype.mf;

    function W(a) {
        var b = {};
        b[Wb] = !1;
        b[bc] = !1;
        b[zb] = !1;
        b[Xb] = 0;
        a = this.data = _.extend(b, a);
        if ("undefined" === typeof a[Ub] || null === a[Ub]) a[Ub] = a[Vb];
        a[hb] && (b = a[hb].lastIndexOf("/"), (b = a[hb].substr(b + 1, a[hb].lastIndexOf(".") - b - 1).split("_")) && 3 == b.length && (a[jb] = b[1].substr(0, b[1].length - 1), a[kb] = b[2].substr(0, b[2].length - 1)));
        a[tb] && h("Ad detected with token: " + a[tb]);
        a[x] && (a[x] = [].concat(a[x]));
        a[kc] = !1;
        this.M() && (a[F] = a[tb], a[kc] = !0);
        this.ya = !1;
        this.ac = a[L]
    }
    W.prototype = {
        Ga: function() {
            if (this.data[x]) return h("Using additionalAudioUrl"), {
                audioUrl: this.data[x][0]
            };
            var a = this.data[w];
            if (a) {
                var b = this.ra(a, "highQuality");
                if (null != b) return b;
                b = this.ra(a, "mediumQuality");
                if (null != b) return b;
                a = this.ra(a, "lowQuality");
                if (null != a) return a
            }
            return {
                audioUrl: this.data.audioUrl
            }
        },
        af: function() {
            var a = this.data[w];
            if (a) {
                var b = this.ra(a, "highQuality");
                if (null != b) return b;
                b = this.ra(a, "mediumQuality");
                if (null != b) return b;
                a = this.ra(a, "lowQuality");
                if (null != a) return a
            }
            return {
                audioUrl: this.data.audioUrl
            }
        },
        bc: function() {
            var a = this.Ga();
            if (a) return (a = a.audioUrl) && "undefined" != typeof a && "" != a
        },
        ra: function(a, b) {
            var c = a[b];
            return null != c && "undefined" != typeof c ? c : null
        },
        M: function() {
            return !!this.data[tb]
        },
        hc: function() {
            this.ya = !0;
            this.data[L] = 0
        },
        jc: function() {
            this.ya = !0;
            this.data[L] = 1
        },
        ic: function() {
            this.ya = !0;
            this.data[L] = -1
        },
        Xd: function() {
            this.ya = !1;
            this.data[L] = 0;
            this.ac = this.data[L]
        },
        Vd: function() {
            this.ya = !1;
            this.data[L] = 1;
            this.ac = this.data[L]
        },
        Ud: function() {
            this.ya = !1;
            this.data[L] = -1;
            this.ac = this.data[L]
        },
        Ua: function() {
            this.ya = !1;
            this.data[L] = this.ac
        },
        Td: function() {
            this.data[bc] = !0
        },
        Kd: function() {
            return this.data[bc]
        },
        Wd: function() {
            this.data[zb] = !0
        },
        cc: function() {
            this.data[zb] = !1
        },
        Md: function() {
            return this.data[zb]
        },
        ia: function() {
            return this.data[L]
        },
        yd: function() {
            return this.data.songDetailUrl
        },
        sd: function() {
            return this.data.artistDetailUrl
        },
        V: function() {
            return this.data[F]
        },
        nf: function() {
            return this.data.songIdentity
        },
        sa: function() {
            return this.data[lb]
        },
        Qa: function() {
            return this.data[ac]
        },
        ca: function() {
            return this.data[Bb]
        },
        qa: function() {
            return this.data[Db]
        },
        pf: function() {
            return this.data.trackGain ? Number(this.data.trackGain) : null
        },
        fa: function() {
            return this.data.allowFeedback
        },
        lb: function() {
            return this.data.allowBookmarkTrack
        },
        Yc: function() {
            return this.data.allowTiredOfTrack
        },
        Xc: function() {
            return this.data.allowStartStationFromTrack
        },
        Wc: function() {
            return this.data.allowSkipTrackWithoutLimit
        },
        Vc: function() {
            return this.data.allowShareTrack
        },
        Uc: function() {
            return this.data.allowBuyTrack
        },
        eb: function() {
            return this.data[hb]
        },
        kc: function() {
            return this.data[Ub]
        },
        ug: function(a) {
            this.data[Xb] = a
        },
        qc: function() {
            return this.data[Xb] ? Number(this.data[Xb]) : this.data[Xb]
        },
        bf: function() {
            return this.data.audioSkipUrl
        },
        td: function() {
            return this.data.artistExplorerUrl
        },
        ud: function() {
            return this.data.itunesSongUrl
        },
        hf: function() {
            return this.data.isFeatured
        },
        toJSON: function() {
            return this.data
        }
    };

    function ve(a) {
        return !a.fa() && "Pandora" === a.data[Db] && "Pandora" === a.data[ac] && "Explicit Warning" === a.data[Bb]
    }
    W.prototype.getArtistName = W.prototype.qa;
    W.prototype.getTrackName = W.prototype.ca;
    W.prototype.getAlbumName = W.prototype.Qa;
    ra = Ba(pa, qa, r.apply(null, [56, 54, 52, 100]));

    function we(a) {
        this.timestamp = (new Date).getTime();
        this.Fb = a[vb];
        this.f = a[z];
        this.username = a[pb];
        this.A = a[nb];
        this.ai = null == a.listeningTimeoutMinutes ? 480 : a.listeningTimeoutMinutes;
        this.gender = a.gender;
        this.zip = a.zip;
        this.Sg = a.age;
        this.state = a.userstate;
        this.version = a.version;
        this.We = a.facebookName;
        this.lh = a.deviceToken
    };

    function Y(a, b) {
        this.init(a, b)
    }
    Y.prototype.init = function(a, b) {
        b ? (this.A = b.A, this.f = b.f, this.Y = b) : this.Y = this.f = this.A = null;
        this.w = a.w;
        this.ea = a.ea;
        this.r = a;
        var c = this;
        c.getName = function() {
            return c.Y.username
        }
    };

    function fe(a) {
        return "undefined" != typeof a.f && a.f
    }
    Y.prototype.wd = function() {
        return fe(this) ? 6E4 * this.Y.ai : 288E5
    };
    Y.prototype.Oc = function() {
        var a = this.Y.state;
        return "SUBSCRIBER" !== a && "CANCELLED" !== a && "VENDOR_BILLED_SUBSCRIBER" !== a && !("COMPLIMENTARY" === a || "VENDOR_BILLED_COMPLIMENTARY" === a)
    };
    Y.prototype.v = function() {
        return this.r.v()
    };

    function xe(a) {
        this.D = a;
        this.Ib = a.cmd_id;
        this.status = {};
        this.progress = this.U = this.P = this.Da = null;
        this.paused = !0;
        this.volume = 0;
        this.xb = this.Ub = !1;
        this.Gb = this.bb = null
    }
    g = xe.prototype;
    g.Af = function() {
        return "PLAY" === this.D.type
    };
    g.wf = function() {
        return "INFO" === this.D.type
    };
    g.zf = function() {
        return "STOP" === this.D.type
    };
    g.Bf = function() {
        return "REFRESH_STATION" === this.D.type
    };
    g.Ff = function() {
        return "VOLUME" === this.D.type
    };
    g.uf = function() {
        return "DISCONNECT" === this.D.type
    };
    g.Cf = function() {
        return "LOAD" === this.D.type && "SKIP" === this.D.src
    };
    g.tf = function() {
        return "LOAD" === this.D.type && "DELETE_FEEDBACK" === this.D.src
    };
    g.Ef = function() {
        return "LOAD" === this.D.type && "THUMB_UP" === this.D.src
    };
    g.Df = function() {
        return "LOAD" === this.D.type && "THUMB_DOWN" === this.D.src
    };
    g.ag = function() {
        return null
    };
    g.xf = function(a) {
        return "LOAD" === this.D.type && this.D.content_info && this.D.src != a
    };
    g.ie = function() {
        return this.D.src
    };
    g.bg = function() {
        return this.D.content_info.track
    };
    g.he = function() {
        return this.D.content_info.deviceProperties
    };
    g.fd = function() {
        var a = {};
        a.cmd_id = this.Ib;
        a.type = "STATUS";
        a.status = this.gd();
        return a
    };
    g.qe = function(a) {
        return this.Da = a
    };
    g.pe = function(a) {
        return this.P = a
    };
    g.Ab = function(a) {
        return this.U = a
    };
    g.Mc = function(a) {
        this.progress = a
    };
    g.ue = function(a) {
        this.paused = a
    };
    g.Xa = function(a) {
        this.volume = a
    };
    g.Lc = function(a) {
        this.bb = a
    };
    g.gd = function() {
        var a = this.status;
        a.state = this.paused ? 1 : 2;
        a.active_input = !0;
        a.event_sequence = this.df();
        if (this.P || this.U) {
            this.P && (a.content_id = this.P.e());
            var b = this.je(this.Da, this.P, this.U);
            a.identity = JSON.stringify(b);
            a.content_info = b;
            this.U && (a.title = this.U.ca() + " by " + this.U.qa(), a.image_url = this.U.eb());
            a.time_progress = !this.paused;
            a.volume = this.volume;
            this.progress && (a.current = a.current_time = a.position = this.progress.elapsedTime, a.duration = this.progress.totalTime)
        }
        return a
    };
    g.df = function() {
        return this.Ib + 1
    };
    g.je = function(a, b, c) {
        var d = {};
        if (b) {
            d.stationName = b.getName();
            d.stationId = b.sa();
            d.stationToken = b.e();
            d.isQuickMix = b.data[M];
            var e = !1,
                e = !!(b.data[ub] || {}).supportImpressionTargeting;
            d.supportImpressionTargeting = e;
            d.onePlaylist = !!b.data[cc]
        }
        a && (d.userId = a.A, d.casterName = a.Y.We);
        c && (d.songName = c.ca(), d.artistName = c.qa(), d.albumName = c.Qa(), d.artUrl = c.eb(), d.trackToken = c.V(), c.M() && (d.adToken = c.V()), d.songRating = c.ia(), d.songDetailUrl = c.yd(), d.artistDetailUrl = c.sd(), d.allowFeedback = c.fa(), d.allowBookmarkTrack =
            c.lb(), d.allowTiredOfTrack = c.Yc(), d.allowStartStationFromTrack = c.Xc(), d.allowShareTrack = c.Vc(), d.allowBuyTrack = c.Uc(), d.artistExplorerUrl = c.td(), d.itunesSongUrl = c.ud(), d.audioUrlMap = c.data[w] ? c.data[w] : {
                highQuality: c.Ga()
            });
        this.Ub && (d.skip_limit_triggered = !0);
        this.xb && (d.listening_timeout_triggered = !0);
        this.bb && (d.cast_message = this.bb);
        this.Gb && (d.cast_error_code = this.Gb);
        return d
    };
    g.tg = function() {
        this.Ub = !0
    };
    g.qg = function() {
        this.xb = !0
    };
    "undefined" !== typeof exports && (exports.ij = xe);

    function Ca(a, b) {
        this.D = a;
        this.Ib = a.requestId;
        "undefined" != typeof a.mediaSessionId && (this.Gf = a.mediaSessionId);
        this.media = this.D.media;
        this.status = {};
        this.progress = this.U = this.P = this.Da = null;
        this.paused = !0;
        this.volume = 0;
        this.volumeIncrement = b || 0.1;
        this.xb = this.Ub = !1;
        this.Gb = this.bb = null
    }
    g = Ca.prototype;
    g.Af = function() {
        return "PLAY" === this.D.type
    };
    g.wf = function() {
        return "GET_STATUS" === this.D.type
    };
    g.zf = function() {
        return "STOP" === this.D.type || "PAUSE" === this.D.type
    };
    g.Bf = function() {
        return "LOAD" === this.D.type && "REFRESH_STATION" === this.D.src
    };
    g.Ff = function() {
        return "VOLUME" === this.D.type
    };
    g.uf = function() {
        return "DISCONNECT" === this.D.type
    };
    g.Cf = function() {
        return "LOAD" === this.D.type && "SKIP" === this.D.src
    };
    g.tf = function() {
        return "LOAD" === this.D.type && "DELETE_FEEDBACK" === this.D.src
    };
    g.Ef = function() {
        return "LOAD" === this.D.type && "THUMB_UP" === this.D.src
    };
    g.Df = function() {
        return "LOAD" === this.D.type && "THUMB_DOWN" === this.D.src
    };
    g.ag = function(a) {
        var b = {},
            c = this.media.customData;
        return c && (b[cb] = c[cb], b[eb] = c[eb], b[Sb] = a, (a = this.bg()) ? (b[F] = a[F], b[Za] = a) : b[F] = c[F], b[cb] && b[eb]) ? new De(b) : null
    };
    g.xf = function(a) {
        return "LOAD" === this.D.type && this.media.customData && this.media.contentId != a
    };
    g.ie = function() {
        return this.media.contentId
    };
    g.bg = function() {
        return this.media.customData.track
    };
    g.he = function() {
        var a = this.media.customData.deviceProperties;
        return a && "{" === a["0"] ? null : a
    };
    g.fd = function() {
        var a = {},
            b = {};
        a.requestId = this.Ib;
        a.type = "MEDIA_STATUS";
        a.status = [b];
        b.currentTime = 0;
        b.playbackRate = 1;
        b.supportedMediaCommands = 29;
        b.playerState = "IDLE";
        var c = this.U;
        if (c) {
            var d = this.progress ? this.progress.elapsedTime || 0 : 0,
                e = this.progress ? this.progress.totalTime || 0 : 0,
                f = this.paused ? "PAUSED" : "PLAYING",
                k = null;
            this.xb && (f = "IDLE", k = "COMPLETED");
            var n = {
                    metadataType: 3,
                    albumName: c.Qa(),
                    title: c.ca(),
                    albumArtist: c.qa(),
                    artist: c.qa(),
                    images: [c.eb()],
                    releaseDate: ""
                },
                p = {};
            b.media = p;
            p.metadata = n;
            p.contentId = c.Ga().audioUrl;
            p.duration = e;
            p.streamType = "BUFFERED";
            p.contentType = "BUFFERED";
            "undefined" != typeof this.Gf && (b.mediaSessionId = this.Gf);
            c = {};
            c.status = this.gd();
            p.customData = c;
            p = {
                level: this.volume,
                muted: !1,
                increment: this.volumeIncrement
            };
            b.currentTime = d;
            b.playbackRate = 1;
            b.playerState = f;
            k && (b.idleReason = k);
            b.supportedMediaCommands = 29;
            b.volume = p
        }
        return a
    };
    g.qe = function(a) {
        return this.Da = a
    };
    g.pe = function(a) {
        return this.P = a
    };
    g.Ab = function(a) {
        return this.U = a
    };
    g.Mc = function(a) {
        this.progress = a
    };
    g.ue = function(a) {
        this.paused = a
    };
    g.Xa = function(a) {
        this.volume = a
    };
    g.Lc = function(a) {
        this.bb = a
    };
    g.gd = function() {
        var a = this.status;
        a.state = this.paused ? 1 : 2;
        if (this.P || this.U) {
            this.P && (a.content_id = this.P.e());
            var b = this.je(this.Da, this.P, this.U);
            a.content_info = b;
            this.progress && (a.current = a.current_time = a.position = this.progress.elapsedTime || 0, a.duration = this.progress.totalTime || 0);
            a.volume = {
                level: this.volume,
                muted: !1,
                increment: this.volumeIncrement
            }
        }
        return a
    };
    g.df = function() {
        return this.Ib
    };
    g.je = function(a, b, c) {
        var d = {};
        if (b) {
            d.stationName = b.getName();
            d.stationId = b.sa();
            d.stationToken = b.e();
            d.isQuickMix = b.data[M];
            var e = !1,
                e = !!(b.data[ub] || {}).supportImpressionTargeting;
            d.supportImpressionTargeting = e;
            d.onePlaylist = !!b.data[cc]
        }
        a && (d.userId = a.A, d.casterName = a.Y.We);
        c && (d.songName = c.ca(), d.artistName = c.qa(), d.albumName = c.Qa(), d.artUrl = c.eb(), d.trackToken = c.V(), c.M() && (d.adToken = c.V()), d.songRating = c.ia(), d.songDetailUrl = c.yd(), d.artistDetailUrl = c.sd(), d.allowFeedback = c.fa(), d.allowBookmarkTrack =
            c.lb(), d.allowTiredOfTrack = c.Yc(), d.allowStartStationFromTrack = c.Xc(), d.allowShareTrack = c.Vc(), d.allowBuyTrack = c.Uc(), d.artistExplorerUrl = c.td(), d.itunesSongUrl = c.ud(), d.songIdentity = c.nf(), d.isFeatured = c.hf(), d.audioUrlMap = c.data[w] ? c.data[w] : {
                highQuality: c.Ga()
            });
        this.Ub && (d.skip_limit_triggered = !0);
        this.xb && (d.listening_timeout_triggered = !0);
        this.bb && (d.cast_message = this.bb);
        this.Gb && (d.cast_error_code = this.Gb);
        return d
    };
    g.tg = function() {
        this.Ub = !0
    };
    g.qg = function() {
        this.xb = !0
    };
    "undefined" !== typeof exports && (exports.jj = Ca);

    function De(a) {
        this.cb = a[cb];
        this.ke = a[eb];
        this.userAgent = a[Sb];
        this.Qc = a[F];
        this.ka = a[Za]
    }
    De.prototype.V = function() {
        return this.Qc || ""
    };
    sa = r.apply(null, [50, 97, 98, 50]);
    ta = r.apply(null, [53, 100, 48, 49]);

    function Le(a, b) {
        this.media = null;
        this.status = {
            paused: 1,
            duration: 0
        };
        this.qb = b;
        this.ig = a
    }
    var Me = !1;
    g = Le.prototype;
    g.pause = function() {
        this.media.pause();
        this.status.paused = 1
    };
    g.play = function(a) {
        0 > a || (a ? this.media.play(a) : this.media.play(), this.status.paused = 0)
    };
    g.load = function(a, b, c) {
        this.media && (this.ig.removeChild(this.media), delete this.media, this.media = null, this.status.paused = 1, this.status.duration = 0);
        this.autoplay = c;
        this.media = this.Fd(this.ig, a, b, c)
    };
    g.Na = function(a, b) {
        b && "undefined" != typeof b && (a = Math.min(100, Math.round(Math.pow(10, b / 20) * a)));
        this.media.volume = a / 100
    };
    g.Fd = function(a, b, c, d) {
        var e;
        e = Me ? document.createElement("video") : document.createElement("audio");
        e.setAttribute("src", b.src);
        c && e.setAttribute("preload", c ? "metadata" : "none");
        d && e.setAttribute("autoplay", "autoplay");
        a.appendChild(e);
        var f = this;
        e.addEventListener("canplaythrough", function() {
            f.qb.Fi()
        }, !1);
        e.addEventListener("progress", function() {}, !1);
        e.addEventListener("timeupdate", function() {
            var a = f.media;
            if (4 == a.readyState && !f.status.paused) {
                var b = 0;
                a.played.length && (b = a.played.end(0));
                var c = a.duration;
                f.qb.Ii({
                    elapsedTime: b,
                    totalTime: c,
                    percentPlayed: 0 < c ? 100 * a.currentTime / c : 0
                })
            }
        }, !1);
        e.addEventListener("durationchange", function() {
            f.status.duration = f.media.duration
        }, !1);
        e.addEventListener("play", function() {
            f.status.paused = 0
        }, !1);
        e.addEventListener("playing", function() {
            f.status.paused = 0
        }, !1);
        e.addEventListener("pause", function() {
            f.status.paused = 1;
            f.qb.ge()
        }, !1);
        e.addEventListener("waiting", function() {}, !1);
        e.addEventListener("seeking", function() {}, !1);
        e.addEventListener("seeked", function() {}, !1);
        e.addEventListener("volumechange", function() {}, !1);
        e.addEventListener("suspend", function() {}, !1);
        e.addEventListener("ended", function() {
            f.pause();
            f.status.duration = 0;
            f.qb.Gi()
        }, !1);
        e.addEventListener("abort", function() {}, !1);
        e.addEventListener("emptied", function() {}, !1);
        e.addEventListener("ratechanged", function() {}, !1);
        e.addEventListener("readystatechanged", function() {}, !1);
        e.addEventListener("suspend", function() {}, !1);
        e.addEventListener("waiting", function() {}, !1);
        e.addEventListener("canplay", function() {
            f.qb.N.play()
        }, !1);
        e.addEventListener("loadeddata", function() {}, !1);
        e.addEventListener("loadedmetadata", function() {}, !1);
        e.addEventListener("loadstart", function() {}, !1);
        e.addEventListener("stalled", function() {}, !1);
        e.addEventListener("error", function() {
            f.qb.k()
        }, !1);
        e.load();
        return e
    };

    function Ne(a, b) {
        b.attr("id");
        this.N = new Le(b[0], this);
        this.b = a;
        this.rf = !1
    }

    function Oe(a, b, c) {
        a.rf = b;
        a.$h = b ? (new Date).getTime() : null;
        a.ka = c
    }
    g = Ne.prototype;
    g.vg = function(a) {
        this.ka = a
    };
    g.Yb = function(a, b) {
        this.N.Na(b, a)
    };
    g.load = function(a) {
        this.totalTime = this.elapsedTime = 0;
        this.N.load({
            src: a
        }, !0, !0)
    };
    g.Na = function(a) {
        this.N.Na(a, 0)
    };
    g.play = function() {
        this.N.play();
        this.b.Sa()
    };
    g.La = function() {
        this.N.play();
        this.b.Sa()
    };
    g.pause = function() {
        this.N.pause();
        this.b.vc()
    };
    g.stop = function() {
        this.pause();
        Oe(this, !1)
    };
    g.ge = function() {};
    g.Gc = function() {};
    g.de = function() {};
    g.k = function() {};
    g.Fi = function() {};
    g.Gi = function() {
        this.b.Mf(this.ka)
    };
    g.Ii = function(a) {
        if (this.ka) {
            var b = Math.floor(a.elapsedTime),
                c = Math.floor(a.totalTime);
            a = Math.floor(a.Wj);
            0 == this.totalTime && 0 < c && (this.totalTime = Math.floor(c), Xa("SimpleAudioPlayerWrapper.onTotalTimeUpdated called"), this.b.fi());
            this.b.sf({
                elapsedTime: b,
                totalTime: c,
                percentPlayed: a
            })
        }
    };
    var Pe = [],
        Qe = !1,
        Re = 0,
        me = 1;

    function Se(a, b, c) {
        b.Wh() && (Me = Qe = !0);
        h("useSimpleAudioPlayer: " + Qe);
        if (Qe) b = $('<div id="simpler-media-' + Re + '"></div>').css({
            width: 0,
            height: 0
        }).appendTo("body"), b = new Ne(a, b), Te(b), c && (Oe(b, !0, c), ne(c, b));
        else {
            var d = b.Xh(),
                e = b.Ld(),
                f = b.cf(),
                k = b.Bd(),
                n = b.Yh(),
                p = {
                    useSourceTag: d,
                    solution: "html",
                    supplied: "m4a, mp3, webmv, ogv, m4v, mp4",
                    size: {
                        width: "0px",
                        height: "0px"
                    },
                    ready: function() {
                        h("jPlayer is ready, wrapping it with our AudioPlayer...");
                        var b = new AudioPlayer(a, $(this), f, d, e, k, n);
                        Te(b);
                        c && (Oe(b, !0, c), ne(c, b))
                    },
                    error: function(b) {
                        if (b.jPlayer.error.type == $.jPlayer.error.NO_SOLUTION || b.jPlayer.error.type == $.jPlayer.error.NO_SUPPORT) h("Error encountered while constructing jPlayer DOM: " + JSON.stringify(b.jPlayer)), a.Nf({
                            track: c,
                            errorType: "unable_to_play_media"
                        })
                    }
                };
            b.Ld() ? p.supplied = "m4v" : b.yf() && (p.swfPath = "/js/PS3.swf", p.solution = "flash");
            $('<div id="jPlayer-media-' + Re + '"></div>').css({
                width: 0,
                height: 0
            }).appendTo("body").jPlayer(p)
        }
    }

    function Te(a) {
        Re += 1;
        Pe.push(a);
        h("Constructed another jPlayer instance, total is: " + Re)
    }

    function je(a, b, c) {
        var d = null,
            e = _.select(Pe, function(a) {
                return !a.rf
            });
        (d = 0 < e.length ? e.shift() : _.min(Pe, function(a) {
            return a.$h
        })) && Oe(d, !0, c);
        0 == e && Re < me && Se(a, b, c);
        return d
    };
    var Ue = function() {
        function a(a, b, c, n) {
            setTimeout(function() {
                try {
                    var d = new Audio;
                    d.addEventListener("canplay", function() {
                        n(c)
                    }, !1);
                    d.addEventListener("error", function() {
                        n(!1)
                    }, !1);
                    d.src = "data:" + b + ";base64," + a;
                    d.load()
                } catch (s) {
                    n(!1)
                }
            }, d)
        }

        function b(b, c, k) {
            for (var n = !1, p = [], s = [], t = 0; t < b.length; t++) {
                var P = b[t],
                    ha = P.data,
                    ia = P.type,
                    P = P.audioType;
                s[t] = !1;
                var ja = function(a) {
                        a && !n && (n = !0, c([a]))
                    },
                    ka = function(a) {
                        return function(b) {
                            s[a] = !0;
                            b && p.push(b);
                            b = !0;
                            for (var d = 0; d < s.length; d++) s[d] || (b = !1);
                            b && c(p)
                        }
                    }(t);
                d += 500 * t;
                a(ha, ia, P, k ? ka : ja)
            }
        }

        function c(a) {
            for (var b = "", c = 0; c < a; c++) b += "A";
            return b
        }
        var d = 1;
        return {
            detect: function(a, f) {
                d = 1;
                var k = [];
                k.push({
                    Vg: "HTTP_128_MP3",
                    type: "audio/mp3",
                    data: "//sQRAAP8AwAuoAgAAoBgBeQBAABQJgC8gAAACATAF9AAAAEGPAI///6vFv//+vxVUxBTUUzLjk4LjJVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVX/+xJkG4/wAABpAAAACAAADSAAAAEAAAGkAAAAIAAANIAAAARVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVU="
                });
                k.push({
                    Vg: "HTTP_64_AAC",
                    type: "audio/aac",
                    data: "AAAAHGZ0eXBNNEEgAAAAAU00QSBtcDQyaXNvbQAAC15tb292AAAAbG12aGQAAAAAy/QI5sv0COYAAAJYAAAAOQABAAABAAAAAAAAAAAAAAAAAQAAAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAE" + c(41) + "CAAAB/XRyYWsAAABcdGtoZAAAAAHL9Ajmy/QI5gAAAAEAAAAAAAAAOQAAAAAAAAAAAAAAAAEAAAAAAQAAAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAACRlZHRzAAAAHGVsc3QAAAAAAAAAAQAAADkAAAAAAAEAAAAAAXVtZGlhAAAAIG1kaGQAAAAAy/QI5sv0COYAAH0AAAAMABXHAAAAAAA6aGRscgAAAAAAAAAAc291bgAAAAAAAAAAAAAAAEFwcGxlIFNvdW5kIE1lZGlhIEhhbmRsZXIAAAABE21pbmYAAAAQc21oZAAAAAAAAAAAAAAAJGRpbmYAAAAcZHJlZgAAAAAAAAABAAAADHVybCAAAAABAAAA13N0YmwAAABnc3RzZAAAAAAAAAABAAAAV21wNGEAAAAAAAAAAQAAAAAAAAAAAAIAEAAAAAB9AAAAAAAAM2VzZHMAAAAAA4CAgCIAAAAEgICAFEAVABgAAAB9AAAAfQAFgICAAhKIBoCAgAECAAAAGHN0dHMAAAAAAAAAAQAAAAMAAAQAAAAAHHN0c2MAAAAAAAAAAQAAAAEAAAADAAAAAQAAACBzdHN6AAAAAAAAAAAAAAADAAAAQAAAAIEAAAB/AAAAFHN0Y28AAAAAAAAAAQAAC6IAAAjtdWR0YQAACOVtZXRhAAAAAAAAACBoZGxyAAAAAAAAAABtZGlyAAAAAAAAAAAAAAAAAAAAsWlsc3QAAAAfqXdydAAAABdkYXRhAAAAAQAAAABOaGF0IFZvAAAAHKluYW0AAAAUZGF0YQAAAAEAAAAAX2FhYwAAAB+pQVJUAAAAF2RhdGEAAAABAAAAAE5oYXQgVm8AAAAnqWFsYgAAAB9kYXRhAAAAAQAAAABOaGF0IFZvJ3MgQWxidW0AAAAoqXRvbwAAACBkYXRhAAAAAQAAAABHYXJhZ2VCYW5kIDYuMC41AAAICGZyZWU" +
                        c(2736) + "hmcmVlAAAACGZyZWUAAAFYbWRhdAAAAAh3aWRlAAAAAG1kYXQA0GAG8s" + c(78) + "OAPi5yD2/AK0cBD3t3Lsrh+rZJMwjn3r5CA829QIefEgl4eCKwS/AIPITQ2oA1CD+1LJMDYQUjpPsThvdXOvV/xn5/6R6n5V4z3F2D1l0bwPrTAykCCJhNlRFRqqJWTVEinx6iZi96Rv9XuPT8jl2Hz7jn1nr+B6dR/PeAGwfyxwHANDgBvaw" + c(161) + "Dg=="
                });
                b(k, a, f)
            }
        }
    }();

    function T() {}
    T.prototype.j = function() {};
    T.prototype.k = function() {};
    T.prototype.l = function() {};

    function Ve(a) {
        this.h = a
    }

    function ke(a, b, c, d, e, f) {
        var k = {};
        k[z] = c.f;
        k[D] = c.v();
        k[tb] = d;
        k.supportAudioAds = !0;
        f && (k.currentView = f.Dj(), k.isNowPlaying = f.Kj(), k.lastAction = f.Ej(), k.isSnapMode = f.Lj());
        e && 0 < e.length && (k[x] = e.join(","));
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.f;
        d.user_id = c.A;
        a.h.execute(b, !0, !1, "ad.getAdMetadata", k, d)
    }

    function le(a, b, c, d) {
        var e = {};
        e[z] = c.f;
        e[D] = c.v();
        0 < d.Zb.length && (e.adTrackingTokens = d.Zb);
        d.Sb && (e.externalAdDidPlay = d.Sb);
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.f;
        d.user_id = c.A;
        a.h.execute(b, !0, !1, "ad.registerAd", e, d)
    };

    function We(a, b) {
        this.h = a;
        this.Fa = b;
        this.startTime = (new Date).getTime()
    }
    g = We.prototype;
    g.yb = function(a, b, c, d, e, f, k) {
        var n = {};
        n[rb] = b;
        n[pb] = c;
        n.password = d;
        n[qb] = e;
        n.ck = f;
        k && (n.includeUrls = k);
        this.h.execute(a, !1, !0, "auth.partnerLogin", n)
    };
    g.Sc = function(a, b, c, d, e, f, k) {
        var n = {
            loginType: "user"
        };
        n[J] = b;
        n[pb] = d;
        n.includeDemographics = !0;
        n.returnUserstate = !0;
        n.includeUserWebname = !0;
        n.includeUserFullname = !0;
        n.includeFacebook = !0;
        n.password = k ? "._" + e : e;
        n[D] = f;
        d = {};
        d.auth_token = b;
        d.partner_id = c;
        this.h.execute(a, !0, !0, "auth.userLogin", n, d)
    };
    g.ec = function(a, b, c, d, e, f) {
        var k = {
            loginType: "deviceId"
        };
        k[J] = b;
        k.deviceId = d;
        k[D] = f;
        k.includeDemographics = !0;
        k.returnUserstate = !0;
        k.includeUserWebname = !0;
        k.includeUserFullname = !0;
        k.includeFacebook = !0;
        k.ck = e;
        d = {};
        d.auth_token = b;
        d.partner_id = c;
        this.h.execute(a, !0, !0, "auth.userLogin", k, d)
    };
    g.hd = function(a, b, c, d, e, f, k) {
        var n = {
            loginType: "casting"
        };
        n[cb] = b;
        n[F] = d;
        n[eb] = c;
        n[J] = e;
        n[D] = k;
        n.includeDemographics = !0;
        n.returnUserstate = !0;
        n.includeUserWebname = !0;
        n.includeUserFullname = !0;
        n.includeFacebook = !0;
        b = {};
        b.auth_token = e;
        b.partner_id = f;
        this.h.execute(a, !0, !0, "auth.userLogin", n, b)
    };
    g.Sd = function(a, b, c, d, e, f, k, n, p) {
        var s = {};
        s[qb] = b;
        s[rb] = c;
        s[Fb] = d;
        s.includeTrackLength = !0;
        null != n && "undefined" != typeof n && (s.checksum = n);
        null != p && "undefined" != typeof p && (s[E] = p);
        if (e) s.deviceId = e;
        else if (f && k) s[pb] = f, s.password = k;
        else return a.k("Bad loginType.");
        this.h.execute(a, !0, !0, "auth.login", s)
    };

    function Xe(a) {
        this.h = a
    }

    function Ye(a, b, c, d) {
        var e = {};
        e[z] = c.f;
        e[D] = c.v();
        e[F] = d;
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.f;
        d.user_id = c.A;
        a.h.execute(b, !0, !1, "bookmark.addArtistBookmark", e, d)
    }

    function Ze(a, b, c, d) {
        var e = {};
        e[z] = c.f;
        e[D] = c.v();
        e[F] = d;
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.f;
        d.user_id = c.A;
        a.h.execute(b, !0, !1, "bookmark.addSongBookmark", e, d)
    };

    function $e(a) {
        this.h = a
    }
    $e.prototype.search = function(a, b, c, d, e, f) {
        var k = {};
        k[z] = b.f;
        k[D] = b.v();
        k.searchText = c;
        d && (k.includeNearMatches = !0);
        e && (k.includeGenreStations = !0);
        f && (k.includeStationArtUrl = !0);
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        this.h.execute(a, !0, !1, "music.search", k, c)
    };
    $e.prototype.xd = function(a, b) {
        var c = {};
        c[z] = b.f;
        c[D] = b.v();
        c.includeStationArtUrl = !0;
        var d = {};
        d.partner_id = b.w;
        d.auth_token = b.f;
        d.user_id = b.A;
        this.h.execute(a, !0, !1, "music.getSearchRecommendations", c, d)
    };

    function af(a) {
        this.h = a
    }
    g = af.prototype;
    g.ha = function(a, b, c, d, e, f, k) {
        var n = {};
        e && (n.includeNewStationMessage = e);
        f && (n[cb] = f.cb, f.userAgent.indexOf("iOS"), n[x] = "HTTP_64_AACPLUS_ADTS,HTTP_64_AACPLUS_ADTS,HTTP_32_AACPLUS_ADTS,HTTP_64_AACPLUS", n[F] = f.V(), n.includeAudioToken = !0);
        d && 0 < d.length && (n[x] = d.join(","));
        n.includeTrackLength = !0;
        n.includeTrackOptions = !0;
        n.includeAudioSkipUrl = !0;
        n.includeLyricSnippet = !0;
        n.requestHighQuality = !0;
        n[z] = b.f;
        n[D] = b.v();
        n[E] = c;
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        k && (k.rh() || k.ef()) && (n.forceExplicitContentFilterEnabled = !0);
        this.h.execute(a, !0, !0, "station.getPlaylist", n, c)
    };

    function bf(a, b, c, d, e) {
        var f = {};
        f[z] = c.f;
        f[D] = c.v();
        f[F] = e;
        f.isPositive = d;
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.f;
        d.user_id = c.A;
        a.h.execute(b, !0, !1, "station.addFeedback", f, d)
    }

    function cf(a, b, c, d) {
        var e = {};
        e[z] = c.f;
        e[D] = c.v();
        e[F] = d;
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.f;
        d.user_id = c.A;
        a.h.execute(b, !0, !1, "station.deleteFeedback", e, d)
    }
    g.Ne = function(a, b, c, d) {
        var e = {
            includeStationArtUrl: !0
        };
        e[z] = b.f;
        e[D] = b.v();
        e.musicToken = c;
        d && (d.page_view && (e.page_view = d.page_view), d.view_mode && (e.view_mode = d.view_mode));
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        this.h.execute(a, !0, !1, "station.createStation", e, c)
    };

    function df(a, b, c, d, e) {
        var f = {
            includeStationArtUrl: !0
        };
        f[z] = c.f;
        f[D] = c.v();
        f[F] = d;
        f.musicType = e;
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.f;
        d.user_id = c.A;
        a.h.execute(b, !0, !1, "station.createStation", f, d)
    }
    g.md = function(a, b, c, d) {
        var e = {};
        e[z] = b.f;
        e[D] = b.v();
        e[E] = c;
        d && (d.page_view && (e.page_view = d.page_view), d.view_mode && (e.view_mode = d.view_mode));
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        this.h.execute(a, !0, !1, "station.deleteStation", e, c)
    };
    g.fb = function(a, b) {
        var c = {};
        c[z] = b.f;
        c[D] = b.v();
        c.includeChecksum = !0;
        c.includeSampleArtists = !0;
        c.includeStationArtUrl = !0;
        c.stationArtSize = "W500H500";
        var d = {};
        d.partner_id = b.w;
        d.auth_token = b.f;
        d.user_id = b.A;
        this.h.execute(a, !0, !1, "station.getGenreStations", c, d)
    };

    function ef(a, b, c) {
        var d = {};
        d[z] = c.f;
        d[D] = c.v();
        var e = {};
        e.partner_id = c.w;
        e.auth_token = c.f;
        e.user_id = c.A;
        a.h.execute(b, !0, !1, "station.getGenreStationsChecksum", d, e)
    }
    g.la = function(a, b, c) {
        var d = {};
        d[z] = b.f;
        d[D] = b.v();
        d.settings = c;
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        this.h.execute(a, !0, !1, "station.changeSettings", d, c)
    };

    function ff(a) {
        this.h = a
    };

    function gf(a) {
        this.h = a
    }
    gf.prototype.Tb = function(a, b) {
        var c = {};
        c[z] = a.f;
        c[D] = a.v();
        c[F] = b;
        c.action = "userSkip";
        c.event = "audioCancelled";
        this.h.execute(new T, !0, !1, "track.trackEvent", c);
        return !0
    };

    function hf(a, b, c, d) {
        var e = {};
        e[z] = c.f;
        e[D] = c.v();
        e[F] = d;
        e.facebookSettingChecksum = "";
        d = {};
        d.partner_id = c.w;
        d.user_id = c.A;
        d.auth_token = c.f;
        a.h.execute(b, !0, !1, "track.trackStarted", e, d)
    }

    function jf(a, b, c, d) {
        var e = {};
        e[z] = c.f;
        e[D] = c.v();
        e[F] = d;
        d = {};
        d.partner_id = c.w;
        d.user_id = c.A;
        d.auth_token = c.f;
        a.h.execute(b, !0, !1, "track.explainTrack", e, d)
    }
    gf.prototype.mc = function(a, b, c, d, e) {
        var f = {};
        f[z] = b.f;
        f[D] = b.v();
        f[F] = c;
        f.lyricId = d;
        f.nonExplicit = e;
        c = {};
        c.partner_id = b.w;
        c.user_id = b.A;
        c.auth_token = b.f;
        this.h.execute(a, !0, !1, "track.getLyrics", f, c)
    };
    gf.prototype.oe = function(a, b, c) {
        var d = {};
        d[z] = b.f;
        d[D] = b.v();
        d.audioUrl = c;
        c = {};
        c.partner_id = b.w;
        c.user_id = b.A;
        c.auth_token = b.f;
        this.h.execute(a, !0, !1, "track.resolveAudioStream", d, c)
    };

    function kf(a) {
        this.h = a
    }
    kf.prototype.rb = function(a, b, c) {
        var d = {};
        d[J] = b.ea;
        d[D] = b.v();
        d.deviceId = c;
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.ea;
        this.h.execute(a, !0, !0, "device.generateDeviceActivationCode", d, c)
    };

    function lf(a, b, c, d, e) {
        var f = {};
        f[qb] = c;
        f[rb] = d;
        f[Fb] = e;
        a.h.execute(b, !0, !0, "device.generateDeviceActivationCode", f)
    }
    kf.prototype.nh = function(a, b, c) {
        var d = {};
        d[z] = b.f;
        d[J] = b.ea;
        d[D] = b.v();
        d.deviceId = c;
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        this.h.execute(a, !0, !0, "device.disassociateDevice", d, c)
    };

    function mf(a) {
        this.h = a
    }
    g = mf.prototype;
    g.O = function(a, b, c, d, e, f, k) {
        var n = {};
        c ? (n.includeStationArtUrl = c, n.stationArtSize = "W500H500") : n.includeStationArtUrl = !1;
        d && (n.shuffleIconVersion = d);
        n.sortField = e ? e : "dateCreated";
        n.sortOrder = f ? f : "desc";
        n.includeShuffleInsteadOfQuickMix = "undefined" != typeof k && null !== k ? k : !0;
        n[z] = b.f;
        n[D] = b.v();
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        this.h.execute(a, !0, !1, "user.getStationList", n, c)
    };
    g.Ja = function(a, b) {
        var c = {};
        c[z] = b.f;
        c[D] = b.v();
        var d = {};
        d.partner_id = b.w;
        d.auth_token = b.f;
        d.user_id = b.A;
        this.h.execute(a, !0, !1, "user.getStationListChecksum", c, d)
    };
    g.Ra = function(a, b) {
        var c = {};
        c[z] = b.f;
        c[D] = b.v();
        var d = {};
        d.partner_id = b.w;
        d.auth_token = b.f;
        d.user_id = b.A;
        this.h.execute(a, !0, !1, "user.getTopStations", c, d)
    };
    g.Ia = function(a, b, c, d) {
        var e = {};
        e[z] = b.f;
        e[D] = b.v();
        c && (e.selectedStation = c);
        d && (e.previousStation = d);
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        this.h.execute(a, !0, !1, "user.getRecentStations", e, c)
    };

    function nf(a, b, c, d) {
        var e = {};
        e[z] = c.f;
        e[J] = c.ea;
        e[D] = c.v();
        e.deviceId = d;
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.f;
        d.user_id = c.A;
        a.h.execute(b, !0, !0, "user.associateDevice", e, d)
    }

    function of(a, b, c, d) {
        var e = {};
        e[J] = c.ea;
        e[D] = c.v();
        e[pb] = d;
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.ea;
        a.h.execute(b, !0, !0, "user.emailPassword", e, d)
    }

    function pf(a, b, c, d, e, f) {
        var k = {};
        k[qb] = c;
        k[rb] = d;
        k[Fb] = e;
        k[pb] = f;
        a.h.execute(b, !0, !0, "user.emailPassword", k)
    }

    function qf(a, b, c, d) {
        var e = {};
        e[z] = c.f;
        e[J] = c.ea;
        e[D] = c.v();
        e[F] = d;
        d = {};
        d.partner_id = c.w;
        d.auth_token = c.f;
        d.user_id = c.A;
        a.h.execute(b, !0, !1, "user.sleepSong", e, d)
    }
    g.ob = function(a, b, c, d, e, f, k, n, p, s) {
        var t = {};
        t[J] = b;
        t[D] = c;
        t.accountType = "registered";
        t.birthYear = k;
        t.countryCode = "US";
        t.emailOptIn = s;
        t.gender = p;
        t.password = f;
        t.username = e;
        t.registeredType = "user";
        t.zipCode = n;
        c = {};
        c.partner_id = d;
        c.auth_token = b;
        this.h.execute(a, !0, !0, "user.createUser", t, c)
    };
    g.la = function(a, b, c) {
        var d = {};
        d[z] = b.f;
        d[D] = b.v();
        d.currentUsername = c.currentEmail;
        d.currentPassword = c.currentPassword;
        d.newUsername = c.newEmail;
        d.newPassword = c.newPassword;
        d.birthYear = c.birthYear;
        d.zipCode = c.zipcode;
        d.gender = c.gender;
        d.emailOptIn = c.emailOptIn;
        d.isExplicitContentFilterEnabled = c.isExplicitContentFilterEnabled;
        d.isProfilePrivate = c.isProfilePrivate;
        d.emailNewFollowers = c.emailNewFollowers;
        d.emailComments = c.emailComments;
        d.enableComments = c.enableComments;
        d.pushOptInPandora = c.Zj;
        d.pushOptInListeners =
            c.Yj;
        d.emailOptInListeners = c.Aj;
        d.emailOptInArtists = c.emailOptInArtists;
        d.userInitiatedChange = c.userInitiatedChange;
        d.artistAudioMessagesEnabled = c.artistAudioMessagesEnabled;
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        this.h.execute(a, !0, !0, "user.changeSettings", d, c)
    };
    g.oc = function(a, b) {
        var c = {};
        c[z] = b.f;
        c[D] = b.v();
        var d = {};
        d.partner_id = b.w;
        d.auth_token = b.f;
        d.user_id = b.A;
        this.h.execute(a, !0, !1, "user.getSettings", c, d)
    };
    g.Rb = function(a, b, c) {
        var d = {};
        d[z] = b.f;
        d[D] = b.v();
        d.explicitPIN = "no";
        d.isExplicitContentFilterEnabled = c;
        c = {};
        c.partner_id = b.w;
        c.auth_token = b.f;
        c.user_id = b.A;
        this.h.execute(a, !0, !1, "user.setExplicitContentFilter", d, c)
    };
    g.Lb = function(a, b) {
        var c = {};
        c[z] = b.f;
        c[D] = b.v();
        var d = {};
        d.partner_id = b.w;
        d.auth_token = b.f;
        d.user_id = b.A;
        this.h.execute(a, !0, !1, "user.getExplicitContentFilter", c, d)
    };

    function rf(a, b, c) {
        var d = a[xb],
            e = a[yb];
        b.R() && a.useAES ? (b = new sf(a), h("encrypting: 'Pandora Internet Radio'"), h(b.encrypt("Pandora Internet Radio"))) : b = new tf(d, e);
        this.h = a = new uf(a, b, c);
        this.Fa = b;
        this.test = new ff(a);
        this.$a = new We(a, b);
        this.ba = new mf(a);
        this.xa = new af(a);
        this.ka = new gf(a);
        this.Ge = new Ve(a);
        this.dc = new kf(a);
        this.Ke = new Xe(a);
        this.Hf = new $e(a)
    };

    function vf() {
        try {
            $.support.cors = !0
        } catch (a) {}
    }
    vf.prototype.send = $.ajax;

    function uf(a, b, c) {
        this.zh = a[Hb];
        this.Ah = a[Ib];
        var d = {};
        d[Pb] = a[gb];
        d[rb] = a[ob];
        d[Ob] = a[sb];
        d[Nb] = a[Gb];
        d[Qb] = a[Qb];
        this.Ea = d;
        this.Yg = a[Tb];
        this.Ki = a[Jb];
        a[Lb] && (this.Ea = _.extend(this.Ea, a[Lb]));
        this.wa = null;
        this.Fa = b;
        this.Ce = null;
        b instanceof sf && (this.Ce = "1");
        this.ajax = c ? c() : new vf
    }
    uf.prototype.execute = function(a, b, c, d, e, f) {
        if (this.wa) {
            var k = {},
                k = _.extend(k, this.wa),
                k = _.extend(k, this.Ea);
            e.deviceProperties = k
        } else e.deviceProperties = this.Ea;
        this.Yg || this.Ki && "auth.userLogin" === d || (e.includeExtraParams = !0);
        e = JSON.stringify(e);
        c = (c ? this.Ah : this.zh) + "?method=" + d;
        if (f)
            for (var n in f) c += "&" + n + "=" + encodeURIComponent(f[n]);
        this.Ce && (c += "&version=" + this.Ce);
        "auth.userLogin" !== d && h("url: " + c + " - data: " + e);
        b && (e = this.Fa.encrypt(e), h("encrypted data: " + e));
        this.ajax.send({
            type: "POST",
            contentType: "text/plain",
            url: c,
            data: e,
            error: function(b, c, d) {
                a.k({
                    xhr: b,
                    status: c,
                    error: d
                })
            },
            success: function(b) {
                b && "ok" === b.stat ? a.l(b) : a.j(b)
            },
            dataType: "json",
            async: !0,
            crossDomain: !0
        })
    };

    function tf(a, b) {
        this.ld = new Ea(a);
        this.Pa = new Ea(b)
    }
    tf.prototype.encrypt = function(a) {
        return a.length ? this.Pa.encrypt(a + "        ".substr((a.length - 1) % 8 + 1)).toLocaleLowerCase() : ""
    };
    tf.prototype.decrypt = function(a) {
        a = this.ld.decrypt(a);
        return a.substr(4, a.length)
    };

    function sf(a) {
        var b = a[yb],
            c = a[xb],
            d = CryptoJS.MD5(a[Fb] + "{" + a[Cb] + "}").toString(CryptoJS.enc.Hex);
        a = CryptoJS.MD5(d + "{" + b + "}").toString(CryptoJS.enc.Hex);
        var e = CryptoJS.PBKDF2(b, CryptoJS.enc.Hex.parse(a), {
                keySize: 4,
                iterations: 11
            }),
            b = CryptoJS.MD5(d + "{" + c + "}").toString(CryptoJS.enc.Hex),
            f = CryptoJS.PBKDF2(c, CryptoJS.enc.Hex.parse(b), {
                keySize: 4,
                iterations: 11
            });
        this.encrypt = function(a) {
            return CryptoJS.AES.encrypt(a, e, {
                Zh: CryptoJS.enc.Hex.parse(d)
            }).ciphertext.toString(CryptoJS.enc.Hex)
        };
        this.decrypt = function(a) {
            a =
                CryptoJS.lib.CipherParams.create({
                    ciphertext: CryptoJS.enc.Hex.parse(a)
                });
            (a = CryptoJS.AES.decrypt(a, f, {
                Zh: CryptoJS.enc.Hex.parse(d)
            }).toString(CryptoJS.enc.Latin1)) && 4 < a.length && (a = a.slice(4));
            return a
        }
    };

    function wf(a, b, c, d, e, f) {
        this.B = a;
        this.ze = b;
        this.g = d;
        this.b = e;
        this.d = f;
        this.pa = c;
        this.b.bind(qd, _.bind(this.Q, this));
        this.b.bind(mc, _.bind(this.Q, this));
        this.b.bind(mc, _.bind(this.wc, this));
        this.b.bind(Xd, _.bind(this.Hi, this));
        this.b.bind(Xd, _.bind(this.Q, this));
        this.b.bind(Wd, _.bind(this.Q, this));
        this.b.bind(Wd, _.bind(this.Gc, this));
        this.b.bind(yc, _.bind(this.Q, this));
        this.b.bind(yc, _.bind(this.tc, this));
        this.b.bind(xc, _.bind(this.Q, this));
        this.b.bind(xc, _.bind(this.zc, this));
        this.b.bind(Ec, _.bind(this.Q,
            this));
        this.b.bind(Ec, _.bind(this.yc, this));
        this.b.bind(nd, _.bind(this.ib, this));
        this.b.bind(Kc, _.bind(this.Q, this));
        this.b.bind($c, _.bind(this.Q, this));
        this.b.bind(Tc, _.bind(this.Q, this));
        this.b.bind(Uc, _.bind(this.Q, this));
        this.b.bind(Vc, _.bind(this.Q, this));
        this.b.bind(cd, _.bind(this.Q, this));
        this.b.bind(cd, _.bind(this.Yf, this));
        this.b.bind(ed, _.bind(this.Q, this));
        this.b.bind(ed, _.bind(this.Zf, this));
        this.b.bind(Hd, _.bind(this.Q, this));
        this.b.bind(Yc, _.bind(this.Q, this));
        this.b.bind(gd, _.bind(this.ye,
            this));
        this.b.bind(gd, _.bind(this.Q, this));
        this.b.bind(Zc, _.bind(this.Q, this));
        this.b.bind(Md, _.bind(this.oe, this));
        this.b.bind(Vd, _.bind(this.Q, this));
        this.b.bind(Kd, _.bind(this.Q, this));
        this.wk = {};
        this.tk = 0;
        this.uk = "hour";
        this.X = null
    }
    g = wf.prototype;
    g.Q = function() {
        h("Interaction detected.");
        for (this.b.ve(!1); this.X;) clearTimeout(this.X), this.X = null;
        h("Setting the timer for idling upon interaction.");
        var a = this,
            b = this.d.q(),
            c = 288E5;
        b ? (b && (c = b.wd()), this.d.oa() || this.d.R() || (h("saving time"), $.jStorage.set("lt", c + (new Date).getTime())), this.d.R() || (this.X = setTimeout(function() {
            h("Idle limit reached - notifying other logic...");
            for (a.b.ve(!0); a.X;) clearTimeout(a.X), a.X = null
        }, c))) : (clearTimeout(this.X), $.jStorage.deleteKey("lt"))
    };
    g.ib = function(a, b) {
        if ("User" == b) h("ActivityService.onAuthenticated - Ignoring auth_type: " + b);
        else {
            var c = this.d.q().r;
            this.d.Ui(c.$i);
            this.d.Vi(c.aj);
            var c = c.Ea,
                d = c.simpleAudioPlayer;
            h("deviceProperties: " + JSON.stringify(c));
            "undefined" != typeof d && d && (Qe = !0);
            xf(this)
        }
    };

    function xf(a) {
        var b = a.ze.wd();
        if (b) {
            var c = (new Date).getTime(),
                b = b - c;
            0 < b && !a.d.R() && (a.X = setTimeout(function() {
                h("Idle limit reached - notifying other logic...");
                for (a.b.ve(!0); a.X;) clearTimeout(a.X), a.X = null
            }, b))
        }
    }
    g.wc = function() {
        var a = this.d.H();
        if (!a.M()) {
            var b = this.d.K(),
                c = this.d.pc(b.e());
            te(c) ? (a.Wc() || (c.Cd(), yf(b.e(), c)), this.b.ti(a), b.W()) : this.b.si(c.clone())
        }
    };
    g.Hi = function() {
        this.b.S() ? this.pause() : this.La()
    };
    g.pause = function(a) {
        a && this.b.cg();
        (a = this.d.H()) && a.pause()
    };
    g.La = function() {
        var a = this.d.H();
        a && (a.gh ? this.d.K().W() : a.La())
    };
    g.tc = function(a, b) {
        b || (b = this.d.H());
        if (null != b && !b.M()) {
            var c = S(b);
            0 != c.ia() && (c.ya ? (h("intentToDeleteFeedback, feedback request pending already, don't send again"), c.hc(), this.b.Dc(b, !0)) : (h("intentToDeleteFeedback, no request pending, send the request"), c.hc(), this.b.Dc(b, !0), this.pd(b)))
        }
    };
    g.zc = function(a, b) {
        b || (b = this.d.H());
        if (null != b && !b.M()) {
            var c = S(b);
            1 != c.ia() && (c.ya ? (h("intentToThumbUp, thumb up request pending already, don't send again"), c.jc(), this.b.Fc(b, !0)) : (h("intentToThumbUp, no request pending, send the request"), c.jc(), this.b.Fc(b, !0), this.Kb(b, !0)))
        }
    };
    g.yc = function(a, b) {
        b || (b = this.d.H());
        if (null != b && !b.M()) {
            var c = S(b); - 1 != c.ia() && (c.ya ? (h("intentToThumbDown, thumb down request pending already, don't send again"), c.ic(), this.b.Ec(b, !0)) : (h("intentToThumbUp, thumb up request pending already, don't send again"), c.ic(), this.b.Ec(b, !0), this.Kb(b, !1)))
        }
    };
    g.pd = function(a) {
        if (a.M()) h("Attempt to delete feedback on ad playback");
        else {
            var b = this,
                c = this.d.q();
            this.d.K();
            var d = new T;
            d.l = function() {
                0 != a.ia() ? b.pd(a) : (h("Feedback Deleted"), a.Xd())
            };
            d.j = function(c) {
                a.Ua();
                b.b.Dc(a, !1);
                b.b.C("station.deleteFeedback", c)
            };
            d.k = function(c) {
                a.Ua();
                b.b.Dc(a, !1);
                b.b.F("station.deleteFeedback", c)
            };
            cf(this.g.xa, new V(this.d, this.b, this.B, d, _.bind(this.pd, this, a)), c, a.e())
        }
    };
    g.Kb = function(a, b) {
        if (a.M()) h("Attempt to thumb down on ad playback");
        else {
            var c = this,
                d = this.d.q(),
                e = this.d.K(),
                f = new T;
            f.l = function() {
                1 !== a.ia() || b ? -1 === a.ia() && b ? c.Kb(a, !1) : b ? (h("Thumbed Up"), a.Vd()) : (h("Thumbed Down"), a.Ud()) : c.Kb(a, !0)
            };
            f.j = function(d) {
                b ? (a.Ua(), c.b.Fc(a, !1)) : (a.Ua(), c.b.Ec(a, !1));
                c.b.C("station.addFeedback", d)
            };
            f.k = function(d) {
                (d.isPositive = b) ? (a.Ua(), c.b.Fc(a, !1)) : (a.Ua(), c.b.Ec(a, !1));
                c.b.F("station.addFeedback", d)
            };
            bf(this.g.xa, new V(this.d, this.b, this.B, f, _.bind(this.Kb,
                this, a, b)), d, b, a.e());
            !b && zf(this, a) && (d = this.d.pc(e.e()), te(d) ? (e.Ye(), d.Cd(), yf(e.e(), d), e.W()) : c.b.Bi())
        }
    };
    g.Gc = function() {
        var a = this.b.vb(),
            b = this.d.H();
        b && b.Na(a)
    };
    g.Yf = function() {
        var a = this.d.q(),
            b = this.d.H();
        if (!b || b.Kd()) this.b.Zd(!1), h("bookmarked already, we're not going to do it again.");
        else {
            var c = new T,
                d = this;
            c.l = function() {
                b.Td();
                d.b.Zd(!0)
            };
            c.j = function() {
                d.b.Zd(!1)
            };
            c.k = function(a) {
                d.b.F("bookmark.addSongBookmark", a)
            };
            Ye(this.g.Ke, new V(this.d, this.b, this.B, c, _.bind(this.Yf, this)), a, b.e())
        }
    };
    g.Zf = function() {
        var a = this.d.H(),
            b = this.d.q();
        if (a)
            if (a.lb())
                if (a.Md()) this.b.Ob(a, !0);
                else {
                    a.Wd();
                    this.b.Ob(a, !0);
                    var c = new T,
                        d = this;
                    c.l = function() {
                        zf(d, a)
                    };
                    c.j = function(b) {
                        zf(d, a) && (a.cc(), d.b.Ob(a, !1));
                        d.b.C("bookmark.addSongBookmark", b)
                    };
                    c.k = function(b) {
                        zf(d, a) && (a.cc(), d.b.Ob(a, !1));
                        d.b.F("bookmark.addSongBookmark", b)
                    };
                    Ze(this.g.Ke, new V(this.d, this.b, this.B, c, _.bind(this.Zf, this)), b, a.e())
                }
        else this.b.Ob(a, !1)
    };
    g.ye = function() {
        var a = this.d.H(),
            b = this.d.q(),
            c = this,
            d = new T;
        d.l = function() {
            h("slept the track")
        };
        d.j = d.k = function(a) {
            h("error while trying to sleep the track");
            c.b.C("user.sleepSong", a)
        };
        qf(this.g.ba, new V(this.d, this.b, this.B, d, _.bind(this.ye, this)), b, a.e());
        a = this.d.K();
        te(this.d.pc(a.e())) ? (this.wc(), this.b.ui()) : this.b.vi()
    };
    g.Lb = function(a, b) {
        var c = this.d.q(),
            d = this,
            e = new T;
        e.l = function(b) {
            h("got the explicit content filter");
            a(b)
        };
        e.j = e.k = function(a) {
            h("error while trying to get the explicit content filter");
            d.b.C("user.getExplicitContentFilter", a);
            b()
        };
        this.g.ba.Lb(new V(this.d, this.b, this.B, e, _.bind(this.Lb, this)), c)
    };
    g.Rb = function(a) {
        var b = this,
            c = this.d.q(),
            d = this,
            e = new T;
        e.l = function(a) {
            h("set the explicit content filter");
            b.b.Pf(a)
        };
        e.j = e.k = function(a) {
            h("error while trying to get the explicit content filter");
            d.b.C("user.setExplicitContentFilter", a)
        };
        this.g.ba.Rb(new V(this.d, this.b, this.B, e, _.bind(this.Rb, this)), c, a)
    };
    g.oe = function(a, b) {
        var c = this.d.q(),
            d = this,
            e = new T;
        e.l = function(a) {
            d.b.gi(a)
        };
        e.j = e.k = function() {
            h("error while trying to resolve audio stream")
        };
        this.g.ka.oe(e, c, b)
    };

    function zf(a, b) {
        var c = a.d.H();
        return !!c && S(c).V() === S(b).V()
    };

    function Af(a, b, c, d, e, f) {
        this.B = a;
        this.pa = b;
        this.g = c;
        this.b = d;
        this.d = e;
        this.Zg = f;
        this.d.da() || this.d.oa() || this.b.bind(nd, _.bind(this.ib, this));
        this.b.bind(Kc, _.bind(this.xc, this));
        this.b.bind(Tc, _.bind(this.sc, this));
        this.b.bind(Uc, _.bind(this.Gd, this));
        this.b.bind(Vc, _.bind(this.Hd, this));
        this.b.bind($c, _.bind(this.uc, this));
        this.b.bind(Yc, _.bind(this.fb, this));
        this.b.bind(Hd, _.bind(this.Jd, this))
    }
    g = Af.prototype;
    g.ib = function(a, b, c) {
        c || "Device" != b && "User" != b || this.O()
    };
    g.O = function() {
        var a = this.d.Ja();
        a ? this.Se(a) : this.gc()
    };
    g.Se = function(a) {
        var b = this,
            c = new T;
        c.k = function(a) {
            q("error: " + a);
            b.b.F("user.getStationListChecksum", a)
        };
        c.j = function(a) {
            q("failure: " + a);
            b.b.C("user.getStationListChecksum", a)
        };
        c.l = function(c) {
            c.result.checksum != a ? b.gc() : b.b.Uf()
        };
        b.g.ba.Ja(new V(this.d, this.b, this.B, c, _.bind(this.Se, this, a)), b.d.q())
    };
    g.gc = function() {
        var a = this,
            b = new T;
        b.k = function(b) {
            q("error:" + b);
            a.b.F("user.getStationList", b)
        };
        b.j = function(b) {
            q("failure:" + b);
            a.b.C("user.getStationList", b)
        };
        b.l = function(b) {
            a.Vb(b.result)
        };
        a.g.ba.O(new V(this.d, this.b, this.B, b, _.bind(this.gc, this)), a.d.q(), !0, a.d.vh())
    };
    g.Ra = function() {
        var a = this,
            b = new T;
        b.k = function(b) {
            q("error:" + b);
            a.b.F("user.getTopStations", b)
        };
        b.j = function(b) {
            q("failure:" + b);
            a.b.C("user.getTopStations", b)
        };
        b.l = function(b) {
            a.Be(b.result)
        };
        a.g.ba.Ra(new V(this.d, this.b, this.B, b, _.bind(this.Ra, this)), a.d.q())
    };
    g.Ia = function(a, b) {
        var c = this,
            d = new T;
        d.k = function(b) {
            q("error:" + b);
            c.b.F("user.getRecentStations", b);
            c.d.rg(a)
        };
        d.j = function(b) {
            q("failure:" + b);
            c.b.C("user.getRecentStations", b);
            c.d.rg(a)
        };
        d.l = function(b) {
            c.Ae(b.result, a)
        };
        c.g.ba.Ia(new V(this.d, this.b, this.B, d, _.bind(this.Ia, this)), c.d.q(), a, b)
    };
    g.Vb = function(a) {
        for (var b = a.stations, c = [], d = 0; d < b.length; d++) {
            var e = {
                    pa: this.pa,
                    g: this.g,
                    B: this.B,
                    b: this.b,
                    d: this.d,
                    G: new X(b[d]),
                    za: this.d.tb()
                },
                e = new R(e);
            this.d.ta(e.e());
            c.push(e)
        }
        this.d.Vb(a.checksum, c);
        this.d.O().bind("add", this.Zg)
    };
    g.Be = function(a) {
        this.d.Be(a.stations)
    };
    g.Ae = function(a, b) {
        this.d.Ae(a.stations, b)
    };
    g.uc = function(a, b, c) {
        if (b) {
            a = this.d.O();
            for (var d = null, e = 0; e < a.length; e++) {
                var f = a.at(e);
                if (f && f.e() == b) {
                    d = f;
                    break
                }
            }
            if (d)
                if (d.G.data[M]) h("Cannot delete QuickMix station");
                else {
                    var k = this,
                        n = this.d.K();
                    a = new T;
                    a.k = function(a) {
                        q("error: " + a);
                        k.b.F("station.deleteStation", a)
                    };
                    a.j = function(a) {
                        q("error: " + a);
                        k.b.C("station.deleteStation", a)
                    };
                    a.l = function() {
                        for (var a = k.d.O(), c = 0; c < a.length; c++) {
                            var e = a.at(c);
                            if (e && e.e() == b) {
                                a.remove(e);
                                break
                            }
                        }
                        if (a = k.d.Ra()) {
                            for (c = 0; c < a.length; c++)
                                if ((e = a.at(c)) && e.e() ==
                                    b) {
                                    a.remove(e);
                                    break
                                }
                            k.b.Vf(a)
                        }
                        if (a = k.d.Ia()) {
                            for (c = 0; c < a.length; c++)
                                if ((e = a.at(c)) && e.e() == b) {
                                    a.remove(e);
                                    break
                                }
                            k.b.be(a)
                        }
                        h("Removed: " + d.getName() + " from the stationList.");
                        n && n.e() == d.e() && (h("Pause the current station and inactive it since it's removed from the list"), k.b.vc(), n.setActive(!1));
                        k.b.xi(d)
                    };
                    k.g.xa.md(new V(this.d, this.b, this.B, a, _.bind(this.uc, this, b)), k.d.q(), b, c)
                }
        }
    };
    g.Jd = function() {
        var a = this,
            b = a.d.q(),
            c = a.d.H().e(),
            d = new T;
        d.l = function(b) {
            a.b.ji(b.result)
        };
        d.j = function(b) {
            a.b.C("track.explainTrack", b)
        };
        d.k = function(b) {
            a.b.F("track.explainTrack", b)
        };
        jf(a.g.ka, d, b, c)
    };
    g.Gd = function() {
        var a = this.d.H();
        this.od(a.e(), "artist")
    };
    g.Hd = function() {
        var a = this.d.H();
        a.M() || this.od(a.e(), "song")
    };
    g.od = function(a, b) {
        if (null == a || "undefined" == typeof a) h("Skip station creation from " + b + " due to invalid input"), this.b.wi(0);
        else {
            var c = this,
                d = this.d.q(),
                e = this.d.K(),
                f = this.d.O(),
                k = new T;
            k.l = function(a) {
                a = new X(a.result);
                var b = a.e(),
                    d = f.find(function(a) {
                        return a.e() == b
                    });
                d || (a = {
                    pa: c.pa,
                    g: c.g,
                    B: c.B,
                    b: c.b,
                    d: c.d,
                    G: a,
                    za: c.d.tb()
                }, d = new R(a), c.d.O().add(d), h("created: " + d.G.getName() + " - token: " + d.G.e()), c.d.ta(d.e()), c.b.Tf(d));
                e && e.e() == d.e() ? c.b.S() && c.b.Sa() : (e && e.setActive(!1), c.d.kb(d))
            };
            k.j = function(a) {
                c.b.C("station.createStation",
                    a)
            };
            k.k = function(a) {
                c.b.F("station.createStation", a)
            };
            df(this.g.xa, new V(this.d, this.b, this.B, k, _.bind(this.od, this, a, b)), d, a, b)
        }
    };
    g.sc = function(a, b, c) {
        if (null == b || "undefined" == typeof b) h("Skip station creation due to invalid input");
        else {
            var d = this,
                e = this.d.q(),
                f = this.d.K();
            if (f && f.e() == b) h("Skip station creation since the same station is playing");
            else {
                var k = this.d.O(),
                    n = k.find(function(a) {
                        return a.e() == b
                    });
                n ? (h("Skip station creation and select the existing station"), f && f.setActive(!1), d.d.kb(n)) : (n = new T, n.l = function(a) {
                    a = new X(a.result);
                    var b = a.e(),
                        c = k.find(function(a) {
                            return a.e() == b
                        });
                    c || (a = {
                        pa: d.pa,
                        g: d.g,
                        B: d.B,
                        b: d.b,
                        d: d.d,
                        G: a,
                        za: d.d.tb()
                    }, c = new R(a), h("created: " + c.G.getName() + " - token: " + c.G.e()), d.d.O().add(c), d.d.ta(c.e()), d.b.Tf(c));
                    f && f.e() == c.e() ? (d.b.ci(c), d.b.S() && d.b.Sa()) : (f && f.setActive(!1), d.d.kb(c))
                }, n.j = function(a) {
                    d.b.C("station.createStation", a)
                }, n.k = function(a) {
                    d.b.F("station.createStation", a)
                }, this.g.xa.Ne(new V(this.d, this.b, this.B, n, _.bind(this.sc, this, a, b)), e, b, c))
            }
        }
    };
    g.xc = function(a, b) {
        if (b instanceof R) this.d.kb(b);
        else {
            var c = this.d.K(),
                d = this.d.O().filter(function(a) {
                    return c ? a.e() == b && a.getName() != c.getName() : a.e() == b
                });
            d && 1 == d.length ? (d = d[0], c && c.setActive(!1), this.d.kb(d)) : Ya("Unable to switch to station with token: " + b)
        }
    };
    g.fb = function() {
        var a = this,
            b = new T;
        b.k = function(b) {
            q("error: " + b);
            a.b.F("station.getGenreStationsChecksum", b)
        };
        b.j = function(b) {
            q("failure: " + b);
            a.b.C("station.getGenreStationsChecksum", b)
        };
        b.l = function(b) {
            b = b.result.checksum;
            a.d.th() != b ? Bf(a) : a.b.Qf(a.d.fb())
        };
        ef(a.g.xa, b, a.d.q())
    };

    function Bf(a) {
        var b = new T;
        b.k = function(b) {
            q("error: " + b);
            a.b.F("station.getGenreStations", b)
        };
        b.j = function(b) {
            q("failure: " + b);
            a.b.C("station.getGenreStations", b)
        };
        b.l = function(b) {
            a.d.cj(new ce(b.result));
            a.b.Qf(a.d.fb())
        };
        a.g.xa.fb(b, a.d.q())
    }
    g.me = function() {
        h("requesting for currentStation");
        var a = this.d.K();
        if (a) {
            var b = this,
                c = new T;
            c.k = function(a) {
                q("error: " + a);
                b.b.F("station.getStation", a)
            };
            c.j = function(a) {
                q("failure: " + a);
                b.b.C("station.getStation", a)
            };
            c.l = function(a) {
                (a = a.result) && b.b.yi(new X(a))
            };
            this.hb(a.e(), c)
        }
    };
    g.hb = function(a, b) {
        h("calling getStationInfo for stationToken: " + a);
        var c = this.g.xa,
            d = new V(this.d, this.b, this.B, b, _.bind(this.hb, this, a, b)),
            e = this.d.q(),
            f = !0,
            k = {};
        if ("undefined" == typeof f || null == f) f = !0;
        k[z] = e.f;
        k[D] = e.v();
        k[E] = a;
        k.includeAdAttributes = !0;
        k.includeExtendedAttributes = !0;
        k.includePlaylistAttributes = !0;
        k.includeExtraParams = !0;
        k.includeShuffleInsteadOfQuickMix = f;
        f = {};
        f.partner_id = e.w;
        f.auth_token = e.f;
        f.user_id = e.A;
        c.h.execute(d, !0, !1, "station.getStation", k, f)
    };
    g.la = function(a) {
        var b = this,
            c = new T;
        c.k = c.j = function(a) {
            q("error: " + a);
            b.b.F("station.changeSettings", a)
        };
        c.l = function(a) {
            b.b.Ai(a)
        };
        this.g.xa.la(new V(this.d, this.b, this.B, c, _.bind(this.la, this, a)), this.d.q(), a)
    };

    function Cf(a, b, c, d) {
        this.g = a;
        this.b = b;
        this.d = c;
        this.B = d;
        b.bind(Kd, _.bind(this.mc, this))
    }

    function Df(a) {
        var b = a.d.q(),
            c = a.d.H().e(),
            d = new T;
        d.l = function() {};
        d.j = function(b) {
            a.b.C("track.trackStarted", b)
        };
        d.k = function(b) {
            a.b.F("track.trackStarted", b)
        };
        hf(a.g.ka, d, b, c)
    }
    Cf.prototype.mc = function(a, b, c) {
        var d = this.d.q(),
            e = new T,
            f = this.b;
        e.l = function(d) {
            d = d.result;
            d[F] = a;
            d.nonExplicit = c;
            d.lyricId = b;
            f.mi(d)
        };
        e.j = function(a) {
            f.C("track.getLyrics", a)
        };
        e.k = function(a) {
            f.F("track.getLyrics", a)
        };
        this.g.ka.mc(new V(this.d, this.b, this.B, e, _.bind(this.mc, this, a, b, c)), d, a, b, c)
    };

    function Ef(a) {
        this.b = a;
        this.hg = {}
    }
    Ef.prototype.reset = function(a) {
        if (a) {
            var b = this.hg[a];
            b && h("ErrorTrackingService reset tag: " + a + " list: " + JSON.stringify(b));
            this.hg[a] = []
        }
    };

    function Ff(a, b, c, d) {
        this.model = c;
        this.b = d;
        this.g = a;
        this.T = b;
        this.Hb = b.p;
        this.errorCode = this.i = 0;
        this.Bh = (this.ah = b[Jb]) && b[Rb] || this.model.R();
        this.Y = this.r = null;
        this.Fa = a.Fa;
        this.startTime = (new Date).getTime();
        this.model.R() || this.b.bind(Zc, _.bind(this.rb, this))
    }
    g = Ff.prototype;
    g.Ka = function(a, b, c, d, e) {
        var f = this;
        0 === f.i ? f.Fb(function() {
            f.i = 1;
            f.Ka(a, b, c, d, e)
        }, function() {
            f.i = 6;
            f.Ka(a, b, c, d, e)
        }, function() {
            f.i = 7;
            f.Ka(a, b, c, d, e)
        }) : 1 === f.i ? f.yb(f.T.deviceModel, f.T.partnerName, f.T.partnerPassword, f.T.version, f.Hb, function(k) {
            f.r = k;
            f.i = 2;
            f.Ka(a, b, c, d, e)
        }, function() {
            f.i = 4;
            f.Ka(a, b, c, d, e)
        }) : 2 === f.i ? f.hd(a, b, c, function(k) {
            f.Y = k;
            f.i = 3;
            f.Ka(a, b, c, d, e)
        }, function() {
            f.i = 5;
            f.Ka(a, b, c, d, e)
        }) : Gf(this, "casting", d, e)
    };
    g.hd = function(a, b, c, d, e) {
        var f = this,
            k = new T;
        k.l = function(a) {
            var b = {};
            a && ("fail" == a.stat ? (f.errorCode = a.code || 0, e()) : b = a.result);
            "undefined" != typeof b.userId && d(new we(b))
        };
        k.j = function(a) {
            f.errorCode = a.code || 0;
            f.b.C("auth.userLogin", a);
            e && e()
        };
        k.k = function(a) {
            f.errorCode = a.code || 0;
            f.b.F("auth.userLogin", a);
            e && e()
        };
        f.g.$a.hd(k, a, b, c, f.r.ea, f.r.w, f.r.v())
    };

    function Hf(a, b) {
        a.i = 2;
        a.r = b;
        a.Y = null
    }
    g.Rd = function(a) {
        var b = this,
            c = b.model.q(),
            d = b.model.ga();
        if (c && d) {
            var e = new T;
            e.j = e.k = e.l = function() {
                b.model.Ma(null);
                b.model.Va(null);
                b.model.nb();
                b.model.Ba();
                b.b.ae(a)
            };
            b.g.dc.nh.call(b.g.dc, e, c, d["0"])
        } else b.b.ae(a);
        b.i = 0
    };
    g.Qb = function(a, b) {
        var c = this;
        if (2 === c.i && c.r) {
            var d = new T;
            d.l = function() {
                b(!0)
            };
            d.j = function() {
                b(!1)
            };
            d.k = d.j = function() {
                b(!1)
            };
            of(c.g.ba, d, c.r, a)
        } else(2 > c.i || !c.r) && Z(c, function() {
            c.Qb(a, b)
        })
    };

    function Z(a, b) {
        0 === a.i ? a.Fb(function() {
            a.i = 1;
            Z(a, b)
        }, function() {
            a.i = 6;
            Z(a, b)
        }, function() {
            a.i = 7;
            Z(a, b)
        }) : 1 === a.i ? a.yb(a.T.deviceModel, a.T.partnerName, a.T.partnerPassword, a.T.version, a.Hb, function(c) {
            a.r = c;
            a.i = 2;
            Z(a, b)
        }, function() {
            a.i = 4;
            Z(a, b)
        }) : (b && 2 === a.i && b(), Gf(a, "Partner", !1))
    }
    g.Z = function(a, b, c) {
        h("performDeviceLogin is calling - reauth: " + a + " ignorePartnerLogin: " + c);
        var d = this;
        if (c && 2 > d.i) {
            var e = this.model.q();
            e && Hf(this, e.r)
        }
        h("performDeviceLogin is calling - loginStep: " + d.i + " partnerAuthResult: " + d.r);
        0 === d.i ? d.Fb(function() {
            d.i = 1;
            d.Z(a, b, c)
        }, function() {
            d.i = 6;
            d.Z(a, b, c)
        }, function() {
            d.i = 7;
            d.Z(a, b, c)
        }) : 1 === d.i ? d.yb(d.T.deviceModel, d.T.partnerName, d.T.partnerPassword, d.T.version, d.Hb, function(e) {
            d.r = e;
            d.i = 2;
            d.Z(a, b, c)
        }, function() {
            d.i = 4;
            d.Z(a, b, c)
        }) : 2 === d.i ? (e = d.model.ga()["0"],
            d.ec(e, d.Hb, function(e) {
                d.Y = e;
                d.i = 3;
                d.Z(a, b, c)
            }, function() {
                d.i = 5;
                d.Z(a, b, c)
            })) : (a || b || (b = function(a) {
            var b = d.model.ga(),
                c = new T;
            c.l = function() {
                b["2"] = !0;
                d.b.Cc(b)
            };
            c.j = function(a) {
                d.b.C("user.associateDevice", a)
            };
            c.k = function(a) {
                d.b.F("user.associateDevice", a)
            };
            a.f && nf(d.g.ba, c, a, b["0"])
        }), Gf(this, "Device", a, b))
    };
    g.ua = function(a, b, c) {
        var d = this;
        if (c) {
            if (2 === d.i && d.r) {
                If(d, a, b, function(e) {
                    d.Y = e;
                    d.i = 3;
                    d.ua(a, b, c)
                }, function() {
                    d.i = 5;
                    d.ua(a, b, c)
                });
                return
            }
            if (2 > d.i || !d.r) {
                Z(d, function() {
                    d.ua(a, b, c)
                });
                return
            }
        } else {
            if (2 === d.i && d.r) {
                d.Sc(a, b, function(c) {
                    d.Y = c;
                    d.i = 3;
                    d.ua(a, b)
                }, function() {
                    d.i = 5;
                    d.ua(a, b)
                });
                return
            }
            if (2 > d.i || !d.r) {
                Z(d, function() {
                    d.ua(a, b)
                });
                return
            }
        }
        Gf(this, "User", !1, function(a) {
            var b = d.model.ga(),
                c = new T;
            c.l = function() {
                b["2"] = !0;
                d.b.Cc(b)
            };
            c.j = function(a) {
                d.b.C("user.associateDevice", a)
            };
            c.k = function(a) {
                d.b.F("user.associateDevice",
                    a)
            };
            a.f && nf(d.g.ba, c, a, b["0"])
        })
    };
    g.Sc = function(a, b, c, d) {
        var e = this,
            f = new T;
        f.l = function(a) {
            var b = {};
            a && ("fail" == a.stat ? (e.errorCode = a.code || 0, d()) : b = a.result);
            "undefined" != typeof b.userId && c(new we(b))
        };
        f.j = function(a) {
            e.errorCode = a.code || 0;
            e.b.C("auth.userLogin", a);
            d()
        };
        f.k = function(a) {
            e.errorCode = a.code || 0;
            e.b.F("auth.userLogin", a);
            d()
        };
        e.g.$a.Sc(f, e.r.ea, e.r.w, a, b, e.r.v())
    };

    function If(a, b, c, d, e) {
        var f = new T;
        f.l = function(b) {
            var c = {};
            b && ("fail" == b.stat ? (a.errorCode = b.code || 0, e()) : c = b.result);
            "undefined" != typeof c.userId && d(new we(c))
        };
        f.j = function(b) {
            a.b.C("auth.userLogin", b);
            e()
        };
        f.k = function(b) {
            a.b.F("auth.userLogin", b);
            e()
        };
        a.g.$a.Sc(f, a.r.ea, a.r.w, b, c, a.r.v(), !0)
    }
    g.ec = function(a, b, c, d) {
        var e = this,
            f = new T;
        f.l = function(a) {
            var b = {};
            a && ("fail" == a.stat ? (e.errorCode = a.code || 0, d()) : b = a.result);
            "undefined" != typeof b.userId && c(new we(b))
        };
        f.j = function(a) {
            e.errorCode = a.code || 0;
            e.b.C("auth.userLogin", a);
            d && d()
        };
        f.k = function(a) {
            e.errorCode = a.code || 0;
            e.b.F("auth.userLogin", a);
            d && d()
        };
        e.g.$a.ec(f, e.r.ea, e.r.w, a, b, e.r.v())
    };

    function Gf(a, b, c, d) {
        h("LoginService.verify - type:" + b + " - reauth: " + c + " loginStep: " + a.i);
        if (3 <= a.i) {
            if (3 === a.i) {
                if (a.r && a.Y) {
                    var e = new Y(a.r, a.Y);
                    d && d(e);
                    h("Saving the UserResult: " + JSON.stringify(e));
                    a.model.Ma(e, b, c && !!d)
                }
            } else if (4 === a.i) a.b.Bc("partner", a.errorCode);
            else {
                if (5 === a.i) {
                    a.b.Bc(b, a.errorCode);
                    a.i = 2;
                    return
                }
                6 === a.i ? a.b.$d() : 7 === a.i && a.b.Sf()
            }
            a.i = 0;
            a.r = null;
            a.Y = null;
            a.errorCode = 0
        } else 2 === a.i && (a.r ? (e = new Y(a.r, null), d && d(e), h("Saving the UserResult: " + JSON.stringify(e)), a.model.Ma(e,
            b, c)) : a.b.$d())
    }
    g.Fb = function(a, b, c) {
        if (this.Bh) a();
        else {
            var d = this,
                e = new T;
            e.l = function(c) {
                c = c ? c.result : {};
                "undefined" != typeof c.isAllowed && (c.isAllowed ? a() : b())
            };
            e.j = function(a) {
                b();
                d.b.C("test.checkLicensing", a)
            };
            e.k = function(a) {
                c && c();
                d.b.F("test.checkLicensing", a)
            };
            d.g.test.h.execute(e, !1, !1, "test.checkLicensing", {})
        }
    };
    g.yb = function(a, b, c, d, e, f, k) {
        var n = this,
            p = new T;
        p.l = function(a) {
            var b = {};
            a && ("fail" == a.stat ? k() : b = a.result);
            "undefined" != typeof b.partnerId && f(new oe(b, n.Fa, n.startTime))
        };
        p.j = function(a) {
            n.b.C("auth.partnerLogin", a);
            k()
        };
        p.k = function(a) {
            n.b.F("auth.partnerLogin", a);
            k()
        };
        n.g.$a.yb(p, a, b, c, d, e, !0)
    };
    g.rb = function() {
        var a = this,
            b = a.model.q(),
            c = a.model.ga(),
            d = new T;
        d.l = function(b) {
            a.b.Kf(b.result)
        };
        d.j = function(b) {
            a.b.C("device.generateDeviceActivationCode", b)
        };
        d.k = function(b) {
            a.b.F("device.generateDeviceActivationCode", b)
        };
        a.g.dc.rb(d, b, c["0"])
    };
    g.ob = function(a, b, c, d, e, f, k, n) {
        var p = this,
            s = new T;
        s.l = _.bind(function(a) {
            a = a.result;
            "undefined" != typeof a.userId && (a = new we(a), p.Y = a, p.b.Ci(a), k(a), p.i = 3, Gf(this, "User", !1, function(a) {
                var b = p.model.ga(),
                    c = new T;
                c.l = function() {
                    b["2"] = !0;
                    p.b.Cc(b)
                };
                c.j = function(a) {
                    p.b.C("user.associateDevice", a)
                };
                c.k = function(a) {
                    p.b.F("user.associateDevice", a)
                };
                a.f && nf(p.g.ba, c, a, b["0"])
            }))
        }, this);
        s.j = function(a) {
            "fail" == a.stat ? (p.errorCode = a.code || 0, n(a)) : p.b.C("user.createUser", a)
        };
        s.k = function(a) {
            p.b.F("user.createUser",
                a)
        };
        0 === p.i ? p.Fb(function() {
            p.yb(p.T.deviceModel, p.T.partnerName, p.T.partnerPassword, p.T.version, p.Hb, function(k) {
                p.r = k;
                p.i = 2;
                p.g.ba.ob(s, p.r.ea, p.r.v(), p.r.w, a, b, c, d, e, f)
            }, function() {
                p.i = 4;
                p.g.ba.ob(s, p.r.ea, p.r.v(), p.r.w, a, b, c, d, e, f)
            })
        }, function() {
            p.b.$d()
        }, function() {
            p.b.Sf()
        }) : p.g.ba.ob(s, p.r.ea, p.r.v(), p.r.w, a, b, c, d, e, f)
    };

    function Jf(a, b, c, d) {
        this.model = b;
        this.b = c;
        this.g = a;
        this.B = d;
        this.b.bind(Vd, _.bind(this.la, this))
    }
    Jf.prototype.la = function(a, b) {
        var c = this,
            d = this.model.q(),
            e = new T;
        e.l = function(a) {
            a && "fail" === a.stat && (c.errorCode = a.code || 0, c.b.Xf(a));
            c.b.Di(b, a.result)
        };
        e.j = function(a) {
            c.errorCode = a.code || 0;
            c.b.Xf(a)
        };
        e.k = e.j;
        c.g.ba.la(new V(c.model, c.b, c.B, e, _.bind(this.la, this, b)), d, b)
    };
    Jf.prototype.oc = function() {
        var a = this.model.q(),
            b = this,
            c = new T;
        c.l = function(a) {
            h("got the settings");
            b.b.Ei(a.result)
        };
        c.j = c.k = function(a) {
            h("error while trying to get the settings");
            b.b.C("user.getSettings", a)
        };
        this.g.ba.oc(new V(this.model, this.b, this.B, c, _.bind(this.oc, this)), a)
    };

    function Kf(a, b, c, d, e, f, k) {
        this.g = a;
        this.T = b;
        this.B = c;
        this.Zi = d;
        this.ze = e;
        this.d = f;
        this.b = k;
        this.fc = b[rb];
        this.Ic = b[Fb];
        this.version = b[qb];
        this.startTime = (new Date).getTime();
        this.Fa = a.Fa;
        this.timeout = null;
        this.d.R() && this.b.bind(Zc, _.bind(this.rb, this))
    }
    g = Kf.prototype;
    g.ua = function(a, b, c, d) {
        var e = c || null,
            f = this;
        c = new T;
        c.k = function(a) {
            q("error: " + a);
            f.b.F("auth.login", a)
        };
        c.j = function(a) {
            q("failure: " + a);
            f.b.C("auth.login", a);
            f.b.Bc("user", a.code || 0)
        };
        c.l = function(a) {
            (a = a.result) && Lf(f, a, !0, e)
        };
        this.g.$a.Sd(c, this.version, this.fc, this.Ic, null, a, b, e, d)
    };
    g.Z = function() {
        var a = this.d.ga()[0],
            b = this.d.Ja(),
            c = null,
            d = this.d.kf();
        d && (c = d.e());
        var e = this,
            d = new T;
        d.k = function(a) {
            q("error: " + a);
            e.b.F("auth.login", a)
        };
        d.j = function(a) {
            q("failure: " + a);
            e.b.C("auth.login", a);
            e.b.Bc("Device", a.code || 0)
        };
        d.l = function(a) {
            (a = a.result) && Lf(e, a, !1, b)
        };
        this.g.$a.Sd(d, this.version, this.fc, this.Ic, a, null, null, b, c)
    };
    g.rb = function() {
        var a = this,
            b = new T;
        b.k = function(b) {
            q("error: " + b);
            a.b.F("device.generateDeviceActivationCode", b)
        };
        b.j = function(b) {
            q("failure: " + b);
            a.b.C("device.generateDeviceActivationCode", b)
        };
        b.l = function(b) {
            if (b = b.result) {
                var d = {
                    0: b.deviceToken,
                    1: (new Date).getTime(),
                    2: !1
                };
                a.ze.Jc(0, d);
                a.b.Kf(b)
            }
        };
        lf(this.g.dc, b, this.version, this.fc, this.Ic)
    };
    g.Qb = function(a, b) {
        var c = new T;
        c.l = function() {
            b(!0)
        };
        c.j = function() {
            b(!1)
        };
        c.k = c.j = function() {
            b(!1)
        };
        pf(this.g.ba, c, this.version, this.fc, this.Ic, a)
    };

    function Lf(a, b, c, d) {
        h("isUserLogin: " + c);
        h(b);
        var e = b.partner,
            f = b.user;
        if (e && f) {
            e = new oe(e, a.Fa, a.startTime);
            f = new we(f);
            e = new Y(e, f);
            c ? (a.d.Ma(e, "user", !1), c = {}, c[0] = f.lh, c[1] = (new Date).getTime(), c[2] = !0, a.d.Va(c), a.b.Cc(c)) : a.d.Ma(e, "Device", !1);
            c = b.errorCode;
            if (null == d || b.station || c) {
                d = b.station;
                if (!d) {
                    a.B.Rd(1);
                    return
                }
                a.Zi.Vb(d)
            } else a.b.Uf();
            if (b = b.playlist) {
                var k = b[E];
                d = a.d.O().find(function(a) {
                    return a.e() == k
                });
                h("currentStation: " + d.getName());
                d.Cg(b, null);
                a.d.kb(d)
            } else a.B.Rd(2)
        }
    }
    g.ne = function() {
        clearTimeout(this.timeout);
        this.timeout = null
    };

    function Mf(a, b, c) {
        this.b = a;
        this.d = b;
        this.url = "/autocomplete";
        this.g = c;
        (this.d.da() || this.d.oa()) && this.b.bind(nd, _.bind(this.ib, this))
    }
    Mf.prototype.ib = function() {
        var a = this.d.q().r.$b;
        a && (this.url = a)
    };
    Mf.prototype.$b = function(a, b, c) {
        if (null != a && "undefined" != typeof a && 0 != a.length) {
            var d = !this.d.da() && !this.d.oa(),
                e = this,
                f = this.d.q(),
                k = {};
            k.auth_token = f.f;
            k.query = a;
            k.stations = "yes";
            c && (k.artSize = c);
            a = {
                type: "GET",
                url: this.url,
                data: k,
                error: function(a, b) {
                    h("error while doing autocomplete");
                    e.b.F("autoComplete", b)
                },
                crossDomain: !0,
                async: !0
            };
            d ? (a.contentType = "text/plain", a.dataType = "jsonp", a.success = function(a) {
                a && a.result && b(new re(a.result))
            }) : a.success = function(a) {
                a && b(new re(a))
            };
            $.ajax(a)
        }
    };
    Mf.prototype.search = function(a, b, c, d) {
        var e = new T,
            f = this;
        e.k = function(a) {
            q("error: " + a);
            f.b.F("music.search", a)
        };
        e.j = function(a) {
            q("failure: " + a);
            f.b.C("music.search", a)
        };
        e.l = function(a) {
            (a = a.result) && f.b.ri(a)
        };
        this.g.Hf.search(e, this.d.q(), a, b, c, d)
    };
    Mf.prototype.xd = function() {
        var a = new T,
            b = this;
        a.k = function(a) {
            q("error: " + a);
            b.b.F("music.getSearchRecommendations", a)
        };
        a.j = function(a) {
            q("failure: " + a);
            b.b.C("music.getSearchRecommendations", a)
        };
        a.l = function(a) {
            (a = a.result) && b.b.qi(a)
        };
        this.g.Hf.xd(a, this.d.q())
    };

    function Nf(a, b, c) {
        this.b = a;
        this.d = b;
        this.b.bind(Od, _.bind(this.dj, this));
        this.b.bind(sd, _.bind(this.ce, this));
        this.b.bind(Wd, _.bind(this.Gc, this));
        this.b.bind(Bd, _.bind(this.If, this));
        this.b.bind(Ad, _.bind(this.If, this));
        this.b.bind(nd, _.bind(this.ta, this));
        this.b.bind(ud, _.bind(this.Jc, this));
        this.b.bind(sc, _.bind(this.ee, this));
        this.b.bind(qd, _.bind(this.fe, this));
        c && ($ = $ || {}, $.jStorage = c());
        a = $.jStorage.storageAvailable();
        this.nd = "";
        this.Jb = this.d.yf() ? "di2" : "di";
        a ? h("jStorage's backend is: " +
            $.jStorage.currentBackend()) : (h("storage is NOT available, falling back to cookie jar!!"), $.jStorage = CookieJar, this.nd = hex_md5(CookieJar.toString()))
    }
    g = Nf.prototype;
    g.If = function() {
        $.jStorage.set("sort", this.d.gb());
        this.d.R() && $.jStorage.set("slc", this.d.Ja())
    };
    g.fe = function() {
        this.d.reset();
        $.jStorage.flush();
        var a = this.d.Ze(this.nd);
        $.jStorage.set(this.Jb, JSON.stringify(a))
    };
    g.ta = function() {
        var a = $.jStorage.get("s", null);
        if (a)
            for (var b = Object.keys(a), c = 0; c < b.length; c++) {
                var d = b[c],
                    e = a[d];
                h("initializedSkipLimit from storage for " + d + " - " + e);
                this.d.ta(d, ue(e))
            }
        h("initialized skip")
    };
    g.ce = function() {
        h("onApplicationStarted....");
        var a = $.jStorage.get(this.Jb, null);
        a && (a = JSON.parse(a));
        a && a["0"] && 36 == a["0"].length && a["1"] ? this.d.Va(a) : (a = this.d.Ze(this.nd), $.jStorage.set(this.Jb, JSON.stringify(a)));
        a = $.jStorage.get("fexf", !1);
        null != a && this.d.ng(a);
        this.ta();
        if (this.d.R()) {
            if (a = $.jStorage.get("stn", null)) try {
                var b = JSON.parse(a);
                b && b.stationToken && (b = new X(b), this.d.og(b), this.d.te(b.data[E]))
            } catch (c) {
                h("failed to parse station json")
            }
        } else(b = $.jStorage.get("stk", null)) && h("Found station: " +
            b + " from storage"), this.d.te(b);
        this.d.we($.jStorage.get("sort", "date"));
        this.d.R() && this.d.Dg($.jStorage.get("slc", null))
    };
    g.vf = function(a) {
        return !a || (new Date).getTime() > a
    };
    g.Jc = function(a, b) {
        this.d.Va(b);
        $.jStorage.set(this.Jb, JSON.stringify(b))
    };
    g.ee = function(a, b) {
        if ("Device" === b) {
            var c = this.d.ga();
            c[2] && (c[2] = !1, $.jStorage.set(this.Jb, JSON.stringify(c)))
        }
    };
    g.dj = function() {
        var a = this.d.K();
        a && (h("storing " + a.getName() + " with token " + a.e() + " to localStorage"), this.d.R() ? $.jStorage.set("stn", JSON.stringify(a.G.toJSON())) : $.jStorage.set("stk", a.e()))
    };
    g.Gc = function() {
        var a = this.b.vb();
        $.jStorage.set("vol", a)
    };

    function yf(a, b) {
        var c = $.jStorage.get("s", {});
        c[a] = b.toJSON();
        $.jStorage.set("s", c)
    }
    g.wd = function() {
        return $.jStorage.get("lt")
    };
    g.mg = function(a) {
        $.jStorage.set("fexf", a);
        this.d.ng(a);
        this.b.Pf(a)
    };

    function Of(a, b, c) {
        this.ajax = b;
        this.Db = [];
        this.Ag = 0;
        this.Pc = !1;
        this.Ea = {
            vendor: a[gb],
            device_model: a[ob],
            device_type: a[sb],
            device_version: a[lc],
            model_year: a[Gb]
        };
        this.Lg = "script_error";
        this.Ng = 50;
        this.url = "/stats";
        c && (this.url = c);
        this.Ve = null
    }
    Of.prototype.ja = function(a, b) {
        try {
            if (null != a && "" != a) {
                var c = {
                    type: a
                };
                if (null != b) {
                    var d, e;
                    for (d in b) e = b[d], null !== d && "" !== d && null !== e && "" !== e && (c[d] = e);
                    for (d in this.Ea) e = this.Ea[d], c[d] = e
                }
                var f = "Sending event to stats collector: " + JSON.stringify(c);
                h(f);
                this.execute(this.url + "/v1", c)
            }
        } catch (k) {
            q("AnalyticEventLogger.logEvent error: " + k)
        }
    };
    Of.prototype.execute = function(a, b) {
        var c = this;
        this.Ve || (this.Ve = setInterval(function() {
            c.Ag = 0
        }, 1E3), setInterval(function() {
            c.Pc = !1
        }, 6E4));
        try {
            this.Pc && b.type == this.Lg || (Pf(this, b) ? Xa("duplicate request, skipping: ") : Qf(this, a, b))
        } catch (d) {
            Ya("Error sending stats collector log: " + d.toString())
        }
    };

    function Qf(a, b, c) {
        a.ajax.send({
            url: b,
            type: "GET",
            data: c,
            dataType: "jsonp",
            contentType: "text/plain",
            crossDomain: !0,
            async: !0,
            error: function(b, c) {
                Ya("AnalyticEventLogger.jsonpError: " + c);
                !a.Pc && ++a.Ag > a.Ng && (h("AnalyticEventLogger failing too much... stopping logging for a minute."), a.Pc = !0)
            },
            success: function(a, b) {
                Xa("AnalyticEventLogger.jsonpSuccess: success");
                "success" != b && Ya("AnalyticEventLogger.jsonpSuccess: unexpected status: " + b)
            }
        })
    }

    function Pf(a, b) {
        for (var c = (new Date).getTime(), d = 0; d < a.Db.length; d++) {
            var e = a.Db[d].data,
                f = a.Db[d].time;
            if (null != e && _.isEqual(e, b) && null != f && 300 > c - f) return !0
        }
        7 <= a.Db.length && a.Db.shift();
        a.Db.push({
            time: c,
            data: b
        });
        return !1
    };

    function Rf(a, b, c, d) {
        this.b = b;
        this.d = a;
        this.jh = [c[gb], c[ob], c[sb], c[Gb]].join("-");
        b.bind(sd, _.bind(this.Pb, this));
        this.ajax = d ? d() : new vf;
        this.Tg = new Of(c, this.ajax, c[Mb])
    }
    Rf.prototype.Pb = function(a, b) {
        if (this.d.oa()) h("Skipped ping since we're running in Library/Auto environment");
        else {
            var c = this,
                d = b.clientInfo || {};
            if (b.debug || b.logging) d.debug = !0;
            this.ajax.send({
                url: "/ping",
                data: d,
                success: function(a) {
                    c.debug && h(a)
                },
                error: function(a) {
                    q("could not ping server!: " + a)
                }
            })
        }
    };
    Rf.prototype.ja = function(a, b) {
        var c = new Date,
            d = c.getFullYear() + "-" + (c.getMonth() + 1) + "-" + c.getDate(),
            c = c.toTimeString().split(" ")[0],
            e = this.d.q();
        b.vendor_id = e ? e.w : null;
        b.listener_id = e ? e.A : null;
        b.device_code = this.jh;
        b.app_version = this.d.$e();
        b.client_timestamp = d + " " + c;
        this.Tg.ja(a, b)
    };

    function Sf(a, b) {
        b && a.ajax.send({
            url: b,
            type: "GET",
            dataType: "jsonp",
            crossDomain: !0,
            async: !0,
            error: function() {}
        })
    };

    function Tf(a, b, c, d, e) {
        this.b = a;
        this.d = b;
        this.Ug = c;
        this.kh = d[gb] + " " + d[ob] + " " + d[sb] + " " + d[Gb];
        this.appVersion = "1";
        this.X = this.Nd = 0;
        this.mb = "";
        this.Oc = this.zip = this.gender = this.ed = this.hash = null;
        this.wg = !1;
        this.Kh = "http://lt150.tritondigital.com/lt?sid=" + Uf;
        this.Ji = "http://lt150.tritondigital.com/lt?guid=";
        this.Oe = "CE";
        this.Pe = d.ando || "TV";
        b.oa() && (this.Oe = "CE", this.Pe = "AUTO");
        this.ajax = e ? e() : new vf;
        this.Eb = !1;
        this.interval = 60;
        this.rd = 0;
        this.hh = (new Date).getFullYear();
        (this.Qe = b = !0 === d[Kb]) || a.bind(nd,
            _.bind(this.Ed, this));
        var f = this;
        b || a.bind(Yd, function() {
            f.Eb = !0
        });
        b || a.bind(Xd, function() {
            f.Eb = !a.S()
        });
        b || a.bind(Nc, function() {
            f.Eb = !1
        });
        b || a.bind(kd, function() {
            f.Eb = !1
        })
    }
    var Uf = "9718";
    Tf.prototype.Ed = function(a, b, c) {
        if (!this.Qe)
            if (b && "Partner" != b) {
                clearTimeout(this.X);
                this.mb = null;
                var d = this.d.q();
                if (d) {
                    this.hash = hex_md5(d.A + "").toUpperCase();
                    this.Oc = d.Oc();
                    var e = d.Y;
                    this.ed = this.hh - e.Sg;
                    e.gender && (this.gender = e.gender.charAt(0));
                    this.zip = e.zip;
                    this.wg = 5 > d.A % 100
                }
                var f = this,
                    d = this.Kh + "&vid=" + this.hash,
                    d = d + ("&yob=" + this.ed),
                    d = d + ("&gender=" + this.gender),
                    d = d + ("&zip=" + this.zip),
                    d = d + ("&hasads=" + (this.Oc ? "1" : "0")),
                    d = d + ("&devcat=" + this.Oe + "&devtype=" + this.Pe);
                this.send(d, function(d, e) {
                    f.ja("newSession",
                        d, e);
                    e.valid ? (f.rd = 0, f.mb = e.Nb, f.interval = e.interval, f.X = setTimeout(_.bind(f.Pb, f), 1E3 * f.interval)) : (f.rd++, 10 > f.rd && (f.X = setTimeout(_.bind(f.Ed, f, a, b, c), 5E3)))
                })
            } else h("Ignore partner auth for ando")
    };
    Tf.prototype.Pb = function() {
        if (!this.Qe) {
            var a = this;
            if (this.Eb)
                if (null != this.mb) {
                    var b = (new Date).getTime();
                    this.send(this.Ji + escape(this.mb), function(c, d) {
                        d.valid && (a.interval = d.interval, a.mb = d.Nb);
                        a.ja("ping", c, d);
                        var e = (new Date).getTime() - b;
                        a.X = setTimeout(_.bind(a.Pb, a), 1E3 * a.interval - e)
                    });
                    this.Nd = (new Date).getTime()
                } else this.Ed(null, null, !1);
            else !this.Nd && 18E4 < (new Date).getTime() - this.Nd && (this.mb = null), this.X = setTimeout(_.bind(this.Pb, this), 1E3 * this.interval)
        }
    };
    Tf.prototype.send = function(a, b) {
        a += "&cb=" + (new Date).getTime() + "" + Math.floor(1E4 * Math.random());
        this.ajax.send({
            contentType: "text/plain",
            type: "GET",
            url: a,
            success: function(c) {
                b(a, new Vf({
                    ok: !0,
                    data: c
                }))
            },
            error: function(c) {
                b(a, new Vf({
                    ok: !1,
                    data: c
                }))
            },
            crossDomain: !0
        })
    };
    Tf.prototype.ja = function(a, b, c) {
        h(new Date + " - ANDO - " + b + " - " + JSON.stringify(c));
        this.wg && (a = {
            event: a,
            vendor_id: "133",
            device_code: "10888"
        }, a.is_error = !c.valid, a.guid = c.Nb, c = new Date, a.client_timestamp = c ? c.getFullYear() + "-" + Wa("" + (c.getMonth() + 1)) + "-" + Wa("" + c.getDate()) + " " + Wa("" + c.getHours()) + ":" + Wa("" + c.getMinutes()) + ":" + Wa("" + c.getSeconds()) : "", a.app_version = this.appVersion, a.gender = this.gender, a.birth_year = this.ed, a.zip = this.zip, a.device_os = this.kh, this.Ug.ja("ando", a))
    };

    function Vf(a) {
        this.interval = -1;
        this.Nb = null;
        this.valid = !1;
        if (a && a.ok && (a = a.data, a.indexOf(",") && (a = a.split(",")) && 2 == a.length)) {
            try {
                this.interval = parseInt(a[0], 10)
            } catch (b) {}
            a = a[1];
            this.Nb = "\n" == a.charAt(a.length - 1) ? a.substr(0, a.length - 1) : a;
            this.valid = -1 != this.interval && null !== this.Nb
        }
    };
    var v = function() {
        function a(a, b) {
            G = new rf(a, m, ua.kd);
            A = b;
            O = new Rf(m, l, a, ua.kd);
            B = new Ff(G, a, m, l);
            va = new Ef(l);
            fa = new Nf(l, m, ua.bj);
            ga = new wf(B, fa, va, G, l, m);
            K = new Af(B, va, G, l, m, Wf);
            zc = new Cf(G, l, m, B);
            Ha = new Mf(l, m, G);
            new Tf(l, m, O, a, ua.kd);
            ye = new Jf(G, m, l, B);
            m.R() && (Ia = new Kf(G, a, B, K, fa, m, l));
            l.bind(pc, Xf);
            l.bind(Mc, Yf);
            l.bind(kd, Zf);
            l.bind(jd, Ja);
            l.bind(sc, $f);
            l.bind(qc, ze);
            l.bind(rc, ag);
            l.bind(sd, bg);
            l.bind(td, cg);
            l.bind(nd, dg);
            l.bind(od, eg);
            l.bind(qd, fg);
            l.bind(rd, gg);
            l.bind(tc, hg);
            l.bind(Hc, ig);
            l.bind(Ic, jg);
            l.bind(Lc, kg);
            l.bind(Yd, lg);
            l.bind(uc, mg);
            l.bind(Jc, ng);
            l.bind(wc, og);
            l.bind(vc, pg);
            l.bind(nc, Ua);
            l.bind(oc, y);
            l.bind(Dc, Qa);
            l.bind(hd, Ta);
            l.bind(id, Sa);
            l.bind(Fc, Oa);
            l.bind(Bc, Pa);
            l.bind(Cc, Na);
            l.bind(Rc, p);
            l.bind(Xd, Ka);
            l.bind(Wc, qg);
            l.bind(Xc, rg);
            l.bind(ad, sg);
            l.bind(md, ka);
            l.bind(ud, tg);
            l.bind(Jd, Ra);
            l.bind(dd, Ma);
            l.bind(fd, La);
            l.bind(vd, He);
            l.bind(Bd, ug);
            l.bind(Cd, vg);
            l.bind(Dd, Ke);
            l.bind(Ed, Je);
            l.bind(Nc, ja);
            l.bind(Fd, ia);
            l.bind(Gd, ha);
            l.bind(yd, wg);
            l.bind(bd, xg);
            l.bind(Od, Fe);
            l.bind(Pd, Ge);
            l.bind(Qd, Ee);
            l.bind(Id, P);
            l.bind(Ad, Ie);
            l.bind(Nd, t);
            l.bind(Sc, k);
            l.bind(Pc, n);
            l.bind(Qc, s);
            l.bind(wd, f);
            l.bind(xd, e);
            l.bind(Rd, d);
            l.bind(Sd, yg);
            l.bind(Td, zg);
            l.bind(Ud, Ag);
            l.bind(zd, Bg);
            l.bind(Ld, c);
            C = !0;
            l.Lf(a)
        }

        function b(a, b) {
            if (a && b) {
                var c = S(a).bf();
                if (c) {
                    var d = 0,
                        e = a.nc();
                    e && (d = e.elapsedTime || 0);
                    Sf(O, c + "&sec=" + d + "&reason=" + b)
                }
            }
        }

        function c() {}

        function d(a) {
            var b = m.K();
            b && (b.Ye(), a.S() || b.W())
        }

        function e() {}

        function f() {}

        function k(a, b, c) {
            c.M() && (b.attr("id"), S(c));
            A.de(S(c));
            a.lc() ||
                a.S() || !m.K() ? (a = "Library - onVideoEnded - not playing the station because paused: " + a.S(), h(a), Ra(0, c)) : m.K().W()
        }

        function n(a, b, c) {
            c.M() && (b.attr("id"), S(c))
        }

        function p(a, b) {
            O.ja("xbox_video_ad_plays", b)
        }

        function s() {}

        function t() {}

        function P() {}

        function ha(a, b) {
            if (m.da() && U) {
                var c;
                U instanceof W || U instanceof Q ? c = U : (c = new W(U), U.adToken && (c = new Q(U, c)));
                var d = new ie(l, m, c, je, 0);
                d.wb = c.kc();
                h("Will try to resume the audio at " + c.kc() + " secs");
                m.Bg(d);
                h("Inserted " + c.ca() + " to in front of the playlist");
                c = b.ha();
                c.splice(0, 0, d);
                b.sg(c);
                U = null
            }
            Ac && 2 < b.ha().length && (d = new W({
                audioUrlMap: {
                    highQuality: {
                        protocol: "http",
                        encoding: "mp4",
                        bitrate: "64",
                        audioUrl: "http://cont-sv5-1.pandora.com/public/ads/v/2014/4/2/4/7/67742/191921/movie_hi.mp4"
                    }
                },
                artistName: "",
                songName: "Video Ad",
                albumName: "Advertisement",
                adToken: "ABCD1234"
            }), d = new ie(l, m, d, je, d.qc()), c = b.ha(), c.splice(2, 0, d), c.push(d));
            A.P = b.G;
            h(">>>>>playlist fetched")
        }

        function ia() {
            var a = A;
            a.rc = !1;
            a.updateStatus()
        }

        function ja() {
            var a = A;
            a.rc = !0;
            a.updateStatus();
            O.ja("listener_idle", {
                active: !1,
                action: "none"
            })
        }

        function ka() {}

        function Ka() {
            l.S() ? A.ge() : A.paused = !1
        }

        function La(a, b) {
            S(b)
        }

        function Ma() {}

        function Na(a, b) {
            S(b)
        }

        function Oa(a, c) {
            b(c, 3);
            S(c)
        }

        function Pa(a, b) {
            var c = A,
                d = S(b);
            h(">>>>>thumbed up!");
            c.aa.setMediaInfo({
                $j: d.ia()
            })
        }

        function Qa() {
            S(m.H())
        }

        function Sa() {
            S(m.H())
        }

        function Ta() {
            S(m.H())
        }

        function y(a, c) {
            b(c, 1)
        }

        function Ua() {
            var a = A;
            S(m.H());
            a.xg = !0;
            a.updateStatus();
            a = {
                at_daily_skip_limit: !1,
                at_station_skip_limit: !0,
                station_id: m.Mb()
            };
            O.ja("skip_limit",
                a)
        }

        function Ra(a, b) {
            A.$f(S(b))
        }

        function Ee(a, b) {
            var c = S(b),
                d = m.ub();
            d && (d.Qc = c.V());
            d = A;
            h(">>>>>track changed");
            d.U = c;
            d.paused = !1;
            d.aa.setMediaInfo({
                trackTitle: c.ca(),
                albumTitle: c.Qa(),
                artistName: c.qa(),
                albumArtUrl: c.eb(),
                rating: c.ia()
            });
            d.updateStatus()
        }

        function Fe(a, c) {
            l.Sa(!0);
            var d = m.H();
            d && b(d, 2);
            if (d = m.vd())(c.G.data[M] || S(d).sa() === c.G.sa()) && c.Ab(d), m.pg(null);
            c.start()
        }

        function Ge() {}

        function He() {}

        function Ie() {
            var a = m.gb();
            m.O().Ca(a);
            m.Ja()
        }

        function Je(a, b) {
            b.Ca("recent")
        }

        function Ke(a,
            b) {
            b.Ca("top")
        }

        function ug(a, b, c) {
            a = m.gb();
            if (!c && (c = m.K()) && !b.ph(c.e())) {
                b.Ca(a);
                m.Ja();
                return
            }
            b.Ca(a);
            m.Ja()
        }

        function vg() {}

        function Yf() {
            m.lg(!0);
            A.aa.shutdown()
        }

        function kg(a, c) {
            var d = c.errorType || "UNKNOWN",
                e = c.track;
            h("Library - onAudioErred - " + d + " track: " + e.e());
            var f = m.ga()["0"],
                k = m.q(),
                d = {
                    device_id: f,
                    error_type: d,
                    listener_id: k.A,
                    audio_token: S(e).V(),
                    station_id: S(e).sa()
                };
            (f = e.nc()) && (d.elapsed_time = f.elapsedTime);
            b(e, 4);
            O.ja("ce_audio_error", d);
            m.Dh();
            S(c.track);
            m.qf() || m.K().W()
        }

        function og(a,
            b) {
            S(b)
        }

        function lg(a, b) {
            b.percentPlayed || (b.percentPlayed = b.elapsedTime / b.totalTime * 100);
            var c = A;
            h(">>>>>receiving progress: " + JSON.stringify(b));
            c.progress = b;
            c.updateStatus()
        }

        function jg() {
            m.lg(!1);
            m.eg();
            Df(zc);
            var a = A,
                b = S(m.H());
            h(">>>>>onAudioPlayed");
            a.Ac = !1;
            a.aa.setMediaInfo({
                trackTitle: b.ca(),
                albumTitle: b.Qa(),
                artistName: b.qa(),
                albumArtUrl: b.eb(),
                rating: b.ia()
            });
            h("Forcing play onAudioPlayed to resume music");
            v.play()
        }

        function ig() {
            S(m.H())
        }

        function pg(a, b) {
            var c = {
                listener_id: m.q().A,
                audio_token: S(b.track).V(),
                station_id: S(b.track).sa(),
                threshold: 4,
                time_to_load: b.time
            };
            O.ja("ce_audio_loading_threshold_reached", c)
        }

        function mg(a, b) {
            S(b)
        }

        function ng(a, b) {
            if (a.lc() || a.S() || !m.K()) {
                var c = "Library - onAudioDiscarded - not playing the station because paused: " + a.S();
                h(c);
                Ra(0, b)
            } else m.K().W()
        }

        function hg(a, b) {
            A.de(S(b));
            if (a.lc() || a.S() || !m.K()) {
                var c = "Library - onAudioEnded - not playing the station because paused: " + a.S();
                h(c);
                Ra(0, b)
            } else m.K().W()
        }

        function wg(a, b) {
            A.P = b
        }

        function sg(a, b) {
            var c = m.O(),
                d = m.gb(),
                e = m.K();
            e && e.e() == b.e() && (m.Ba(), m.nb());
            c.Ca(d)
        }

        function rg() {}

        function qg() {
            var a = m.O(),
                b = m.gb();
            a.Ca(b)
        }

        function Wf() {
            h("onStationAdded")
        }

        function eg() {
            m.q()
        }

        function dg(a, b, c) {
            h("Library - onAuthenticated - type: " + b + " and reauth: " + c);
            b && "Partner" == b ? (m.q(), A.ready = !0, h(">>>>>partner authenticated")) : b && "User" == b || !c ? A.ib(m.q()) : c && (a = m.q(), A.Da = a, h(">>>>>authenticated"))
        }

        function bg(a, b) {
            var c = m.ga(),
                d = c["2"];
            m.da() && b.token && (c[0] = decodeURI(b.token), m.Va(c), d = !0, fa.Jc(0, c), h("Using the deviceId: " +
                c[0]));
            var c = m.kf(),
                e = null,
                f = m.vd();
            f && (e = S(f));
            A.ce(!d, c, e);
            m.R() ? d && Ia.Z() : d ? setTimeout(function() {
                B.Z(!0)
            }, 10) : setTimeout(function() {
                Z(B)
            }, 10)
        }

        function tg() {}

        function gg() {}

        function fg(a, b) {
            A.fe(b)
        }

        function xg() {}

        function Ja(a, b) {
            var c = b.method,
                d = b.data,
                e = d.code,
                d = d && d.message ? d.message : "Unknown Error";
            if (12 == e || 6 == e) ze(a);
            else {
                var f = d.split(/\|/),
                    k = f[f.length - 2],
                    l = f[f.length - 1],
                    d = m.K();
                if ("auth.partnerLogin" == c && 0 == e && "OUT_OF_SYNC" == k && "reload" == l) a.ni();
                else if ("reload" !== l && 13 !== e && 1001 !== e)
                    if (1006 ==
                        e && "STATION_DOES_NOT_EXIST" == k) d.G.data[E] === f[3].split(/ /)[2] && (c = m.O(), c.remove(d), m.Ba(), m.nb(), e = m.gb(), c.Ca(e), m.Ja()), apiFailureIdentifier = null;
                    else if (0 != e || "QUICKMIX_NOT_PLAYABLE" != k || "station.getPlaylist" != c) 1039 == e && "station.getPlaylist" == c ? m.K() : "user.associateDevice" === c && "DEVICE_NOT_FOUND" === k ? (m.Ba(), m.nb(), c = m.q().r, m.Ma(new Y(c, null), null, !1), Hf(B, c), apiFailureIdentifier = null) : 1038 === e || 1006 === e && "CONTENT_HAS_EXPIRED" == k ? (d && (c = m.O(), c.remove(d), m.Ba(), m.nb(), e = m.gb(), c.Ca(e)), apiFailureIdentifier =
                    null) : 1005 === e ? apiFailureIdentifier = null : (apiFailureIdentifier = null, h(">>>>>api failed"), h(b))
            }
        }

        function Zf(a, b) {
            var c = A;
            h(">>>>>api erred");
            h(b);
            c.aa.shutdown()
        }

        function ag() {}

        function ze(a) {
            a.vc();
            a.Id();
            A.aa.shutdown()
        }

        function $f(a, b, c) {
            "partner" == b ? q("PANDORALIBRARY - onAuthFailed, probably out of sync error!!") : A.ee(b, c)
        }

        function cg(a) {
            var b = $.jStorage.get("fore", !0);
            b && $.jStorage.set("fore", !1);
            b || $.jStorage.deleteKey("fore");
            a.Id()
        }

        function Xf() {}

        function Bg() {}

        function Ag() {}

        function zg(a,
            b, c) {
            a = m.ef();
            c = c.isExplicitContentFilterEnabled;
            !a && c && (a = m.K()) && a.qh();
            m.kg(c)
        }

        function yg(a, b) {
            m.kg(b.isExplicitContentFilterEnabled)
        }

        function Ae(a, b) {
            var c = {
                    pa: va,
                    g: G,
                    B: B,
                    b: l,
                    d: m,
                    G: a,
                    za: m.tb()
                },
                c = new R(c);
            m.set({
                currentStation: c
            }, {
                silent: !0
            });
            var d = a.e();
            m.pc(d) || m.ta(d);
            U = b;
            c.start()
        }

        function I() {
            return !!m.K() && !!m.H()
        }

        function H(a) {
            var b = m.q();
            if (b) {
                a = {
                    action: a,
                    device_id: m.ga()["0"],
                    listener_id: b.A,
                    app_version: m.$e()
                };
                for (var c in void 0) a[c] = (void 0)[c];
                O.ja("ce_html5_interaction", a)
            }
        }
        var C,
            l = new Zd,
            m = new de({
                b: l,
                Cb: new ge
            }),
            G, B, ye, Ia, va, ga, fa, Ha, K, zc, O, ua = {},
            A, U = null,
            Ac = !1,
            Be = !1,
            Ce = {
                start: function(b, c, d, e, f) {
                    if (!l.n() && !C)
                        if (b[rb] && b[Cb] && b[Fb] && m.Fh(), b[Qb] && m.Oi(b[Qb]), !0 === b[Jb] && m.Jh(), !0 === b.am && m.Eh(), "true" === b.dv && m.mh(), "true" === b.va && (Ac = !0), "true" === b["va-ex"] && (Be = !0), b.siv && m.Ti(parseInt(b.siv, 10)), m.Wi(Ac), m.Ri(Be), "true" === b.or && (e && "function" === typeof e && (je = e), d && "function" === typeof d && (ua.kd = d), f && "function" === typeof f && (ua.bj = f)), !0 === b.ps3 && m.Hh(), !0 === b.nintendo &&
                            m.Gh(), !0 === b.ps4 && (m.Ih(), m.Re()), !1 === b.st && m.Re(), b.at && b.ato && m.Pi(b.ato), b.detectaudio) Ue.detect(function(d) {
                            if (d && 0 < d.length) return h("Playable audio detected: " + d.join(",")), m.Kc(d), a(b, c)
                        }, !0);
                        else {
                            if (b.codec) d = b.codec, m.Kc([d]);
                            else if (b.c && 4 <= b.c.length) switch (b.c.charAt(3)) {
                                case "1":
                                    m.Kc(["HTTP_128_MP3"]);
                                    break;
                                case "2":
                                    m.Kc(["HTTP_64_AACPLUS"])
                            }
                            a(b, c)
                        }
                },
                reset: function() {
                    C = !1;
                    if (B) {
                        var a = B,
                            b = a.model.q(),
                            c = a.model.ga();
                        b && c && a.model.Mi()
                    }
                },
                mk: function(a) {
                    C = !0;
                    l.Lf(a)
                },
                Sd: function(a, b) {
                    if (!l.n()) return a &&
                        b ? (B.ua(a, b), !0) : !1
                },
                la: function(a) {
                    l.n() || l.Nh(a)
                },
                jk: function() {
                    ye.oc()
                },
                oj: function(a) {
                    this.la(a)
                },
                rj: function(a) {
                    this.la(a)
                },
                qj: function(a) {
                    this.la(a)
                },
                pj: function(a) {
                    K.la(a)
                },
                Cj: function(a) {
                    var b = new T;
                    b.l = function() {};
                    b.j = b.k = function() {};
                    K.hb(a, b)
                },
                lj: function(a, b, c, d) {
                    if (!l.n()) {
                        if (!m.R()) throw Error("This functionality only available for auto");
                        a && b && Ia.ua(a, b, c, d)
                    }
                },
                Mj: function(a, b) {
                    if (!l.n()) return a && b ? (B.ua(a, b, !0), !0) : !1
                },
                bi: function(a) {
                    if (a) {
                        a = B;
                        var b = a.model.q(),
                            c = a.model.ga();
                        b &&
                            c && (a.model.Ma(null), a.model.Va(null), a.model.nb(), a.model.Ba());
                        a.b.ae();
                        a.i = 0
                    } else B.Rd.call(B);
                    H("logout")
                },
                Xa: function(a) {
                    l.n() || C && I() && l.Sh(a)
                },
                vb: function() {
                    return l.yh()
                },
                Tb: function() {
                    if (!l.n() && C && I()) {
                        var a = m.H().I;
                        a.M() || ve(a) || a.data.disableSkipButton || (l.wc(), H("skip"))
                    }
                },
                ih: function() {
                    if (!l.n() && C && I()) {
                        var a = m.H().I;
                        a.M() || ve(a) || !a.fa() || (l.tc(), H("feedbackDeleted"))
                    }
                },
                wj: function(a, b) {
                    if (!l.n() && C && I()) {
                        var c = m.Ad(a, b),
                            d = c.I;
                        d.M() || ve(d) || !d.fa() || (l.tc(c), H("feedbackDeleted"))
                    }
                },
                Fg: function() {
                    if (!l.n() && C && I()) {
                        var a = m.H().I;
                        a.M() || ve(a) || !a.fa() || (l.zc(), H("thumbedUp"))
                    }
                },
                zk: function(a, b) {
                    if (!l.n() && C && I()) {
                        var c = m.Ad(a, b),
                            d = c.I;
                        d.M() || ve(d) || !d.fa() || (l.zc(c), H("thumbedUp"))
                    }
                },
                Eg: function() {
                    if (!l.n() && C && I()) {
                        var a = m.H().I;
                        a.M() || ve(a) || !a.fa() || (l.yc(), H("thumbedDown"))
                    }
                },
                yk: function(a, b) {
                    if (!l.n() && C && I()) {
                        var c = m.Ad(a, b),
                            d = c.I;
                        d.M() || ve(d) || !d.fa() || (l.yc(c), H("thumbedDown"))
                    }
                },
                mj: function() {
                    l.n() || I() && l.Lh()
                },
                nj: function() {
                    l.n() || I() && l.Mh()
                },
                fj: function() {
                    if (l.n()) h("Library is disabled, can't call toggelePlay()");
                    else if (I()) {
                        var a = l.S();
                        a ? l.Nc() ? this.gg() : this.La() : this.pause();
                        H(a ? "play" : "pause")
                    }
                },
                play: function(a) {
                    l.n() || I() && l.S() && !l.Nc() && this.La(a)
                },
                pause: function() {
                    l.n() || C && I() && (l.S() || l.vc())
                },
                La: function(a) {
                    if (!l.n() && C && I()) {
                        if (a) {
                            h("resume track with elapsedTime: " + a);
                            var b = m.H();
                            b && (b.wb = a)
                        }
                        l.Sa()
                    }
                },
                gg: function() {
                    l.n() ? h("Library is disabled, can't call resumeFromIdle()") : (l.lc() ? (l.se(!1), m.K().W()) : l.Sa(), O.ja("listener_idle", {
                        active: !0,
                        action: "none"
                    }))
                },
                stop: function() {
                    if (l.n()) h("Library is disabled, can't call resumeFromIdle()");
                    else {
                        l.cg();
                        var a = m.H();
                        a && a.stop()
                    }
                },
                nk: function(a) {
                    !l.n() && C && (a && a instanceof X && (a = {
                        pa: va,
                        g: G,
                        B: B,
                        b: l,
                        d: m,
                        G: a,
                        za: m.tb()
                    }, a = new R(a), m.ta(a.e())), l.xc(a), H("selectStation"))
                },
                Ne: function(a, b) {
                    !l.n() && C && (l.sc(a, b), H("createStation"))
                },
                tj: function() {
                    !l.n() && I() && (l.Gd(), H("createStationFromArtist"))
                },
                uj: function() {
                    !l.n() && I() && (l.Hd(), H("createStationFromTrack"))
                },
                vj: function() {
                    if (!l.n() && I()) {
                        var a = m.K();
                        a && a.e() && this.md(a.e())
                    }
                },
                md: function(a, b) {
                    !l.n() && C && (l.uc(a, b), H("stationDeleted"))
                },
                $b: function(a,
                    b, c) {
                    l.n() || C && Ha.$b(a, function(a) {
                        b && b(a);
                        l.hi(a)
                    }, c)
                },
                Qb: function(a) {
                    if (!l.n()) {
                        if (a) {
                            var b = function() {};
                            m.R() ? Ia.Qb(a, b) : B.Qb(a, b);
                            return !0
                        }
                        return !1
                    }
                },
                ob: function(a, b, c, d, e, f) {
                    l.n() || B.ob(a, b, c, d, e, f, function() {}, function() {})
                },
                rb: function() {
                    l.n() || l.Oh()
                },
                dk: function() {
                    l.n() ? h("Library is disabled, can't call requestExplainTrack()") : (l.Jd(), H("explainTrack"))
                },
                bk: function(a) {
                    l.n() ? h("Library is disabled, can't call requestAudioStreamResolution()") : l.Rh(a)
                },
                ye: function() {
                    l.n() ? h("Library is disabled, can't call sleepCurrentTrack()") :
                        (l.Th(), H("sleepTrack"))
                },
                fb: function() {
                    l.n() || (l.Ph(), H("getGenreStations"))
                },
                rk: function(a) {
                    !l.n() && m.R() && (m.Dg(a), K.O())
                },
                sk: function(a, b) {
                    if (!l.n() && m.R()) {
                        for (var c = [], d = 0; d < b.length; d++) {
                            var e = {
                                    pa: va,
                                    g: G,
                                    B: B,
                                    b: l,
                                    d: m,
                                    G: b[d],
                                    za: m.tb()
                                },
                                e = new R(e);
                            m.ta(e.e());
                            c.push(e)
                        }
                        m.Vb(a, c, !0)
                    }
                },
                ek: function(a) {
                    l.n() || ("undefined" !== typeof a && null != a && m.we(a), K.O(), H("getStationList"))
                },
                fk: function(a) {
                    l.n() || ("undefined" !== typeof a && null != a && m.we(a), K.gc(), H("getStationList"))
                },
                kk: function() {
                    l.n() || K.Ra()
                },
                hk: function(a, b) {
                    l.n() || K.Ia(a, b)
                },
                me: function() {
                    l.n() || I() && C && K.me()
                },
                gk: function(a, b, c) {
                    l.n() || C && l.Qh(a, b, c)
                },
                Jf: function() {
                    l.n() || ga.Q()
                },
                ec: function() {
                    l.n() || (m.R() ? Ia.Z() : B.Z(!1))
                },
                xj: function(a) {
                    if (!l.n() && a && "undefined" !== typeof exports) {
                        var b = m.ga();
                        b[0] = decodeURI(a);
                        m.Va(b);
                        B.Z(!0);
                        h("Using the deviceId: " + a)
                    }
                },
                vk: function(a, b, c, d, e) {
                    if (!a) return q("Cannot start in remote mode with empty station token"), !1;
                    if (!m.da()) return q("Cannot start in remote mode without appropriate parameters"), !1;
                    d && this.He(d);
                    var f = new T;
                    f.j = f.k = function(b) {
                        h("Cannot get station info for " + a + " for casting");
                        Ja.apply(null, [l, b])
                    };
                    f.l = function(c) {
                        var d = c.result;
                        d ? Ae(new X(d), b || null) : (h("Cannot get station info for " + a + " for casting"), Ja.apply(null, [l, c]))
                    };
                    d = m.q();
                    if (!d || !fe(d)) {
                        if (c) return h("Attempting to authenticate"), d = m.ga() || {}, d["0"] = c, fa.Jc(0, d), B.Z(!0, function(b) {
                            m.Ma(b, "Device", !0);
                            K.hb(a, f);
                            e && e()
                        }), !0;
                        q("Not authenticated, cannot use this feature.");
                        return !1
                    }
                    K.hb(a, f)
                },
                Le: function(a, b, c, d) {
                    if (!a) return q("Cannot start in remote mode with empty station token"), !1;
                    if (!m.da()) return q("Cannot start in remote mode without appropriate parameters"), !1;
                    if (!b || !b.ke) return q("Cannot start without casting model"), !1;
                    c && this.He(c);
                    var e = new T;
                    e.j = e.k = function(b) {
                        h("Cannot get station info for " + a + " for casting");
                        Ja.apply(null, [l, b])
                    };
                    e.l = function(c) {
                        var d = c.result;
                        d ? Ae(new X(d), b.ka || null) : (h("Cannot get station info for " + a + " for casting"), Ja.apply(null, [l, c]))
                    };
                    m.Qi(b);
                    if ((c = m.q()) && fe(c)) K.hb(a, e);
                    else {
                        if (b) return h("Attempting to authenticate with casting token"),
                            B.Ka(b.cb, b.ke, b.V(), !1, function(b) {
                                m.Ma(b, "casting", !0);
                                K.hb(a, e);
                                d && d()
                            }), !0;
                        q("Unable to cast without casting token");
                        return !1
                    }
                },
                lk: function(a, b, c) {
                    !l.n() && C && b && a && (m.vd() ? Ya("Cannot resume since there's last used track") : (a = new ie(l, m, b, je, b.qc()), c && (a.wb = c), m.pg(a)))
                },
                He: function(a) {
                    !l.n() && C && a && (G.h.wa || (G.h.wa = {}), _.extend(G.h.wa, a))
                },
                ak: function(a) {
                    if (!l.n() && C && a && G.h.wa) {
                        if (a instanceof Array)
                            for (var b = 0; b < a.length; b++) delete G.h.wa[a[b]];
                        else delete G.h.wa[a];
                        0 === Object.keys(G.h.wa).length &&
                            (G.h.wa = null)
                    }
                },
                pk: function(a, b) {
                    O.ja(a, b)
                },
                Li: function() {
                    l.n() || C && K.me()
                },
                xk: function(a) {
                    a.ad && m.Ni(a.ad)
                },
                yj: function() {
                    l.Id()
                },
                search: function(a, b, c, d) {
                    Ha.search(a, b, c, d)
                },
                ik: function() {
                    null != Ha && Ha.xd()
                },
                Lb: function(a, b) {
                    null != ga && ga.Lb(a, b)
                },
                Rb: function(a) {
                    null != ga && ga.Rb(a)
                },
                mg: function(a) {
                    null != fa && fa.mg(a)
                },
                test: function() {
                    return {
                        d: m,
                        b: l,
                        kj: ga,
                        Ak: zc,
                        sj: A
                    }
                }
            };
        "undefined" !== typeof exports && (exports.hj = Ce);
        return Ce
    }();
}).call(window);

// === end of pandora.js ===

logger.warn("=== Start Pandora ===")

window.ondiallaunch('www.pandora.com')
// cchu
//window._data = _data
// cchu
window.oninit(systemObject, configuration.api.data)

logger.warn("=== Pandora Started ===")

