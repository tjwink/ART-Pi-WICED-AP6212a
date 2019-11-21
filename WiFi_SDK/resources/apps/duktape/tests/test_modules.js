print("Start of script");

try {
    var greetings_module = require('modules/greetings.js');

    var Greetings = new greetings_module();
    print("Greetings.hello() : "+Greetings.hello());
    print("Greetings.goodbye() : "+Greetings.goodbye());

    print("*** MODULES TEST PASSED ***");
} catch (e) {
    print("*** MODULES TEST FAILED ***");
}
