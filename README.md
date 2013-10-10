README
======

A node.js library to communicate with multiwii based flight controllers using the MSP (MultiwiiSerialProtocol). 


Requirements
------------

You need node.js and npm properly installed and configured. I use node 0.10.* but I think it works fine on earlier versions


Status
------

### msp.js
 
The protocol implementation. Note that not all messages are fully parsed. It's a work in progress, feel free to follow the // TODO: markers and jump in :)

### example.js

A simple example. Sends a bunch of commands to the serial port and prints response messages to the standard output as json objects

    cd node-msp
    npm link .
    node example.js


What is it for
--------------

First use I can think of is.. you can put it on any cheap and light hardware running node.js, connect it to the flight controller, and collect in-flight statistics on a SD card. Statistics are useful for PID tuning, development, offline analisis.. Or maybe use the lib to implement a cross-browser node-webkit based Multiwii configuration interface. Or maybe send waypoints to the FC programmatically. Or.. Impress me.