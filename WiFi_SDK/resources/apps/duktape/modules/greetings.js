/* Include a node.js-style module
 * Note: Only the module directory is specified, even though the actual
 *       javascript is located at /modules/nodejs_style/index.js
 */
var nodejs_module = require('nodejs_style/nodejs_style');

function greetings()
{
    this.hello = function() {
        return nodejs_module.hello;
    }

    this.goodbye = function() {
        return nodejs_module.goodbye;
    }
}

module.exports = greetings;
