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

port.on('error', function () { console.error(arguments); });

port.on('open', function () {
  port.on('data', function (data) { protocol.message_decode(data); });

  var before = Date.now();

  var frame_create = function (name, data) {
    var frame = {};

    var frame_complete = function () {
      frame.ts = Date.now();
      var elapsed = Date.now() - before;
      frame.elapsed = elapsed;
      console.log('frame completed', elapsed);
      before = Date.now()
      frame_create();
    };

    var messages = [
      'MSP_UID',
      'MSP_STATUS',
      'MSP_RAW_IMU',
      'MSP_SERVO',
      'MSP_MOTOR',
      'MSP_RC',
      'MSP_RAW_GPS',
      'MSP_COMP_GPS',
      'MSP_ATTITUDE',
      'MSP_ALTITUDE',
      'MSP_BAT',
      'MSP_RC_TUNING',
      'MSP_PID',
      'MSP_BOX',
      'MSP_MISC',
      'MSP_MOTOR_PINS',
      'MSP_BOXNAMES',
      'MSP_PIDNAMES',
    ];

    var iterate = function () {
      var current = messages.shift();
      if (!current) return frame_complete();
      protocol.once(current, function (data) {
        for (var key in data) { frame[key] = data[key]; }
        iterate();
      });
      port_message_send(port, msp.codes[current], msp.codes[current]);
    };

    iterate();
  };

  frame_create()
});

port.on('close', function () {
  console.log('port closed');
});

