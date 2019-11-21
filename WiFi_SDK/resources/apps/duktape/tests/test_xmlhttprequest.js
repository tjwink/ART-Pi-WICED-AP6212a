print("Start of script");

if (typeof ssid === 'undefined')
    ssid = "YOUR_AP_SSID";

if (typeof password === 'undefined')
    password = "YOUR_AP_PASSPHRASE";

print("Creating new 'XMLHttpRequest' object variable xhr");
var xhr = new XMLHttpRequest;

function httpGetOnerrorCallback()
{
    print("HTTP GET error!");

    wifi.disconnect();

    print("*** XMLHTTPREQUEST TEST FAILED ***");
}

function httpGetOnloadCallback()
{
    print("HTTP GET response:");
    print("  status: "+this.status);
    print("  responseText: ");
    print(this.responseText);

    print("Sending HTTP PUT request");
    xhr.open("PUT", "http://www.httpbin.org/put");
    xhr.onerror = httpPutOnerrorCallback;
    xhr.onload = httpPutOnloadCallback;
    xhr.setRequestHeader("Some-Header", "LALALA");
    xhr.send("Hello World!!!");
};

function httpPutOnerrorCallback()
{
    print("HTTP PUT error!");

    wifi.disconnect();

    print("*** XMLHTTPREQUEST TEST FAILED ***");
}

function httpPutOnloadCallback()
{
    print("HTTP PUT response:");
    print("  status: "+this.status);
    print("  responseText: ");
    print(this.responseText);

    print("Sending HTTPS POST request");
    xhr.open("POST", "https://www.httpbin.org/post");
    xhr.onerror = httpPostOnerrorCallback;
    xhr.onload = httpPostOnloadCallback;
    xhr.send("POST data 1234567890");
};

function httpPostOnerrorCallback()
{
    print("HTTPS POST error!");

    wifi.disconnect();

    print("*** XMLHTTPREQUEST TEST FAILED ***");
}

function httpPostOnloadCallback()
{
    print("HTTPS POST response:");
    print("  status: "+this.status);
    print("  responseText: ");
    print(this.responseText);

    wifi.disconnect();

    print("*** XMLHTTPREQUEST TEST PASSED ***");
};

function wifiConnectCallback()
{
    print("Connected to AP '"+ssid+"'");

    print("Sending HTTP GET request" );
    xhr.open("GET", "http://www.httpbin.org/get");
    xhr.onerror = httpGetOnerrorCallback;
    xhr.onload = httpGetOnloadCallback;
    xhr.send();
};

try {
    var wifi = require("wifi");

    print("Connecting to AP via 'wifi.connect(" + ssid + ",{password:" +
          password + "},wifiConnectCallback)");
    wifi.connect(ssid,{password:password},wifiConnectCallback);
} catch (e) {
    print("*** XMLHTTPREQUEST TEST FAILED ***");
}

print("End of script");
