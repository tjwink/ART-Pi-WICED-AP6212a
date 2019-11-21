print("Start of script");

function timerCallback1()
{
    print("timerCallback1 called");
    print("Getting UTC time via 'this.getUtcTime()'");
    print("\tval=" + this.getUtcTime());

    print("*** TIME TEST PASSED ***");
}

try {
    print("Creating new 'time' module variable time_module");
    var time_module = require("time");

    print("Delaying for 1s via 'time_module.mDelay(1000)'");
    time_module.mDelay(1000);

    print("Getting UTC time via 'time_module.getUtcTime()'");
    print("\tval=" + time_module.getUtcTime());

    print("Setting UTC time to 5s via 'time_module.setUtcTime(5)'");
    time_module.setUtcTime(5);

    print("Getting UTC time via 'time_module.getUtcTime()'");
    print("\tval=" + time_module.getUtcTime());

    print("Creating new 'time' object variable time_object");
    var time_object = new time;

    print("Delaying for 4s via 'time_object.mDelay(4000)'");
    time_object.mDelay(4000);

    print("Getting UTC time via 'time_object.getUtcTime()'");
    print("\tval=" + time_object.getUtcTime());

    print("Setting UTC time to 10s via 'time_object.setUtcTime(5)'");
    time_object.setUtcTime(10);

    print("Getting UTC time via 'time_object.getUtcTime()'");
    print("\tval=" + time_object.getUtcTime());

    print("Setting a timer via 'time_object.setTimer(5000, timerCallback1)'");
    time_object.setTimer(5000, timerCallback1);
} catch (e) {
    print("*** TIME TEST FAILED ***");
}

print("End of script");

