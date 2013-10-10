var serialport = require('serialport');
var msp = require('msp');

var argv = require('optimist')
  .boolean('help')
  .argv;

if (argv.help) {
  console.log("Example: $0 --device /dev/tty.SLAB_USBtoUART --baudrate 115200");
  process.exit();
}

var device = argv.device || '/dev/tty.SLAB_USBtoUART';

var options = {
  baudrate: argv.baudrate || 115200
};

var port = new serialport.SerialPort(device, options, true);
var protocol = new msp.Protocol();

var port_message_send = function (port, code, data) {
  // we use msp.Protocol helper function to encode a message, ready
  // to be sent on the serial port
  var buffer = protocol.message_encode(code, data);

  port.write(buffer, function (err, res) {
    // res = num of chars written
    if (err) {
      console.error("Error writing to port");
      console.error(err);
      return;
    }
  });
};

protocol.on('*', function (name, data) {
  // print out all messages coming from the port
  data.code = msp.codes[name];
  data.name = name;
  data.ts = Date.now();
  data = JSON.stringify(data, null, '  ');
  console.log(data);
});

port.on('error', function () { console.error(arguments); });

port.on('open', function () {
  // feed the protocol with new data coming from the port
  port.on('data', function (data) { protocol.message_decode(data); });

  var later = function () {
    port_message_send(port, msp.codes.MSP_UID, msp.codes.MSP_UID);
    port_message_send(port, msp.codes.MSP_ACC_TRIM, msp.codes.MSP_ACC_TRIM);
    port_message_send(port, msp.codes.MSP_IDENT, msp.codes.MSP_IDENT);
    port_message_send(port, msp.codes.MSP_STATUS, msp.codes.MSP_STATUS);
    port_message_send(port, msp.codes.MSP_PID, msp.codes.MSP_PID);
    port_message_send(port, msp.codes.MSP_RC_TUNING, msp.codes.MSP_RC_TUNING);
    port_message_send(port, msp.codes.MSP_BOXNAMES, msp.codes.MSP_BOXNAMES);
    port_message_send(port, msp.codes.MSP_BOX, msp.codes.MSP_BOX);
    port_message_send(port, msp.codes.MSP_PID, msp.codes.MSP_PID);
    port_message_send(port, msp.codes.MSP_DEBUG, msp.codes.MSP_DEBUG);
  };

  setTimeout(later, 100);
});

port.on('close', function () {
  console.log('port closed');
});

