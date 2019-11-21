print("Start of script");

if (typeof ssid === 'undefined')
    ssid = "YOUR_AP_SSID";

if (typeof password === 'undefined')
    password = "YOUR_AP_PASSPHRASE";

if (typeof audio_src == 'undefined')
    audio_src = "http://www.mfiles.co.uk/mp3-downloads/edvard-grieg-peer-gynt1-morning-mood.mp3"

var timer = new time;

print("Creating new 'Audio' object variable audio_object");
var audio_object = new Audio(audio_src);

function durationchangeCallback()
{
    print("'durationchange' event callback called; 'duration'="+audio_object.duration);
}

function endedCallback()
{
    print("'ended' event callback called");

    wifi.disconnect();

    if (this.failed == 0)
    {
        print("Finished playing audio");
        print("*** AUDIO TEST PASSED ***");
    }
}

function errorCallback()
{
    print("'error' event callback called; stopping timers");

    timer.clearTimer();
    wifi.disconnect();

    this.failed = 1;

    print("*** AUDIO TEST FAILED ***");
}

function pauseCallback()
{
    print("'pause' event callback called");
}

function playCallback()
{
    print("'play' event callback called");
}

function playingCallback()
{
    print("'playing' event callback called");
}

function timeupdateCallback()
{
    print("'timeupdate' event callback called; currentTime="+this.currentTime);
}

function timerCallback1()
{
    print("timerCallback1 called");

    print("Setting volume to 0.2");
    audio_object.setAttribute("volume", 0.2);

    print("Seeking to last 10 s mark");
    audio_object.currentTime = audio_object.duration - 10;

    print("Setting timer");
    this.setTimer(5000, timerCallback2);
}

function timerCallback2()
{
    print("timerCallback2 called");

    print("Pausing audio playback");
    audio_object.pause();

    print("Setting timer");
    this.setTimer(5000, timerCallback3);
}

function timerCallback3()
{
    print("timerCallback3 called");

    print("Resuming audio playback");
    audio_object.play();
}

function wifiConnectCallback()
{
    print("Setting volume to 0.5\n");
    audio_object.volume = 0.5;

    audio_object.addEventListener("durationchange", durationchangeCallback, 0);
    audio_object.addEventListener("ended", endedCallback, 0);
    audio_object.addEventListener("error", errorCallback, 0);
    audio_object.addEventListener("pause", pauseCallback, 0);
    audio_object.addEventListener("play", playCallback, 0);
    audio_object.addEventListener("playing", playingCallback, 0);
    audio_object.addEventListener("timeupdate", timeupdateCallback, 0);

    audio_object.failed = 0;

    print("Playing audio");
    audio_object.play();

    print("Setting timer");
    timer.setTimer(10000, timerCallback1);
}

try {
    var wifi = require("wifi");

    print("Connecting to AP via 'wifi.connect(" + ssid + ",{password:" +
          password + "},wifiConnectCallback)");
    wifi.connect(ssid, {password:password}, wifiConnectCallback);
} catch (e) {
    print("*** AUDIO TEST FAILED ***");
}

print("End of script");
