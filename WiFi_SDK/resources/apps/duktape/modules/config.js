//global config
var uuid = require("uuid");
var defaultPort = 8002;
var loggerLevel = {trace:0, debug:1, info:2, warn:3, error:4, fatal:5}

var configuration = {
    logging: {
        enabled: true,
        console: {
            enabled: true,
            level: loggerLevel.info
        },
        dial: {
            enabled: true,
            level: loggerLevel.info
        },
        library: {
            enabled: true,
            level: loggerLevel.info
        }
    },
    dial: {
        enabled: true,
        path: "/apps/",
        port: defaultPort,
        uuid: uuid.v4(),
        "device": {
            "name": "speaker.js prototype",
            "model": "speaker.js",
            "description": "Prototype for speaker.js"
        },
        "vendor": {
            "url": "pandora.com",
            "name": "Pandora Media Inc"
        },
        castchat: {
            enabled: true,
            version: 1,
            path: "/castchat"
        }
    },
    api: {
        key: "tl3?=fr2&+$(9vd>",
        jsUrl: "http://ce-beta.savagebeast.com/audio/?model=test&vendor=cypress&type=STEREO_CONNECTED&modelYear=2016&badge=2qtfnbgfbjkzidt45thoqlazj6lw2o5uhp4bfd6ctmlyuzpmeurjkmdoqlknasdjwfytg77x4e4mk",
        data: "8kuvG7z2GMdqzMJcBz3hK0I3MKNjsrqhG5nZ2PnbQQQ4C/7/ARRYk2ZXX4SqWQVNZACrvknC1Q9CDaSUzSvvpPKzN3BTFzpDIZdBGiYQqm7YSAJK8psDHigsPNfDOwCu7DFAzmc9U5Af6vAcyRiksQD/Rry3xEi621N0f6Hgk+tNfiYylKas6BzWVTC5OO7MUCVM46uIPxQFcU+eRYJBMRvWviRTGghPJqrviIbjcCCn1gCpwjHshDfDYzBK9CwmfU4KP8aNfovgEaAvkWpGFqjyAReFVHqrf3W2wOEhsEBmXJQahTxAvCvA+qhvhAW8xjHsma9XSvPUleGFbh08lCEkQfehi638jrD7dEhlzsxHPNwhqzYcjZOEe+Yss8MiQCFkUA/dSN1yvHNz/q0MNfUkumi0yu/9a7IpiTFLc5ekvr8MKJauNRVzcxOJzQI44J9TezBftzQ/pWWwVrJp1fEJqYHt8ACaQik/JrFna+j/deROxw8yzuZaEIGBRt1w"
    }
};

module.exports = configuration;
