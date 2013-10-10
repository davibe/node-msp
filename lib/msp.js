var codes = {
  MSP_IDENT:        100,
  MSP_STATUS:       101,
  MSP_RAW_IMU:      102,
  MSP_SERVO:        103,
  MSP_MOTOR:        104,
  MSP_RC:         105,
  MSP_RAW_GPS:      106,
  MSP_COMP_GPS:       107,
  MSP_ATTITUDE:       108,
  MSP_ALTITUDE:       109,
  MSP_BAT:        110,
  MSP_RC_TUNING:      111,
  MSP_PID:        112,
  MSP_BOX:        113,
  MSP_MISC:         114,
  MSP_MOTOR_PINS:     115,
  MSP_BOXNAMES:       116,
  MSP_PIDNAMES:       117,
  
  MSP_SET_RAW_RC:     200,
  MSP_SET_RAW_GPS:    201,
  MSP_SET_PID:      202,
  MSP_SET_BOX:      203,
  MSP_SET_RC_TUNING:    204,
  MSP_ACC_CALIBRATION:  205,
  MSP_MAG_CALIBRATION:  206,
  MSP_SET_MISC:       207,
  MSP_RESET_CONF:     208,
  MSP_SELECT_SETTING:   210,
  
  MSP_BIND:         240,
  
  MSP_EEPROM_WRITE:     250,
  
  MSP_DEBUGMSG:       253,
  MSP_DEBUG:        254,
  
  // Additional baseflight commands that are not compatible with MultiWii
  MSP_UID:        160,
  MSP_ACC_TRIM:       240,
  MSP_SET_ACC_TRIM:     239,
  MSP_GPSSVINFO:      164 // get Signal Strength (only U-Blox)
};


code_to_name = {};

for (var key in codes) {
  var value = codes[key];
  code_to_name[value] = key + "";
}



var events = require('events');

var Protocol = function () {
  events.EventEmitter.call(this);
};


Protocol.prototype.__proto__ = events.EventEmitter.prototype;


Protocol.prototype.message_encode = function (code, data) {    
  if (typeof data === 'object') {
    var size = 6 + data.length; // 6 bytes for protocol overhead
    var checksum = 0;
    
    var buffer_out = new ArrayBuffer(size);
    var buf_view = new Uint8Array(buffer_out);    
    
    buf_view[0] = 36; // $
    buf_view[1] = 77; // M
    buf_view[2] = 60; // <
    buf_view[3] = data.length; // data length
    buf_view[4] = code; // code
    
    checksum = buf_view[3] ^ buf_view[4];
    
    for (var i = 0; i < data.length; i++) {
      buf_view[i + 5] = data[i];
      checksum ^= buf_view[i + 5];
    }

    buf_view[5 + data.length] = checksum;
  } else {
    var buffer_out = new ArrayBuffer(7);
    var buf_view = new Uint8Array(buffer_out);
    
    buf_view[0] = 36; // $
    buf_view[1] = 77; // M
    buf_view[2] = 60; // <
    buf_view[3] = 0; // data length
    buf_view[4] = code; // code
    buf_view[5] = data; // data
    buf_view[6] = buf_view[3] ^ buf_view[4] ^ buf_view[5]; // checksum
  }

  var buffer_out = new Buffer(new Uint8Array(buffer_out));

  return buffer_out;
};


Protocol.prototype.message_decode = function (data) {
  var data = new Uint8Array(data);
  var message_state = 0, message_status, message_checksum, message_code;
  var message_length_received = 0;
  var message_buffer_uint8_view = [];

  for (var i = 0; i < data.length; i++) {
    switch (message_state) {
      case 0: // sync char 1
        if (data[i] == 36) { // $
          message_state++;
        }
        break;
      case 1: // sync char 2
        if (data[i] == 77) { // M
          message_state++;
        } else { // restart and try again
          message_state = 0;
        }
        break;
      case 2: // direction (should be >)
        if (data[i] == 62) { // >
          message_status = 1;
        } else { // unknown
          message_status = 0;
        }
        message_state++;
        break;
      case 3:
        message_length_expected = data[i]; // data length
        message_checksum = data[i];
        message_buffer = new ArrayBuffer(message_length_expected);
        message_buffer_uint8_view = new Uint8Array(message_buffer);
        message_state++;
        break;
      case 4:
        message_code = data[i]; // code
        message_checksum ^= data[i];
        
        if (message_length_expected != 0) { // standard message
          message_state++;
        } else { // MSP_ACC_CALIBRATION, etc...
          message_state += 2;
        }
        break;
      case 5: // data / payload
        message_buffer_uint8_view[message_length_received] = data[i];
        message_checksum ^= data[i];
        message_length_received++;
        
        if (message_length_received >= message_length_expected) {
          message_state++;
        }
        break;
      case 6: // CRC
        if (message_checksum == data[i]) {
          // process data
          var name = code_to_name[message_code];
          this.message_decode_payload(message_code, message_buffer);
        }
        // Reset variables
        message_length_received = 0;
        message_state = 0;           
        break;
    }
  }
};


Protocol.prototype.emit_message = function (name, data) {
  this.emit(name, data);
  this.emit('*', name, data);
};

Protocol.prototype.message_decode_payload = function (msp_code, data) {
  var view = new DataView(data, 0);

  var msp_code_name = code_to_name[msp_code];
  var payload = {};


  var data_array_unit_parse = function (data, max, size) {
    size = size || 2;

    var size_to_getter = {
      1: 'getUint8',
      2: 'getUint16',
      4: 'getUint32'
    };
    var getter = size_to_getter[size];
    max = max || -1;

    var res = {};
    var view = new DataView(data, 0);

    for (var i = 0; i < data.byteLength; i += size) {
      var index = i / size;
      res[index] = view[getter](i, 1);

      if (max == index)
        return res;
    }

    return res;
  };
  

  switch (msp_code) {
    case codes.MSP_IDENT:
      var version = parseFloat((view.getUint8(0) / 100).toFixed(2));
      var multi_type = view.getUint8(1);
      
      var multi_type_name_to_code = {
        'TRI': 1,
        'QUAD+': 2,
        'QUAD X': 3,
        'BI': 4,
        'GIMBAL': 5,
        'Y6': 6,
        'HEX 6': 7,
        'FLYING_WING': 8,
        'Y4': 9,
        'HEX6 X': 10,
        'OCTO X8': 11,
        'AIRPLANE': 12,
        'Heli 120': 13,
        'Heli 90': 14,
        'Vtail': 15,
        'HEX6 H': 16
      };

      var multi_type_code_to_name = {};
      for (var key in multi_type_name_to_code) {
        var value = multi_type_name_to_code[key];
        multi_type_code_to_name[value] = key;
      }

      payload.version = version;
      payload.multi_type_code = multi_type;
      payload.multi_type_name = multi_type_code_to_name[multi_type];
      
      this.emit_message(msp_code_name, payload);
      break;
    case codes.MSP_STATUS:
      payload.cycleTime = view.getUint16(0, 1);
      payload.i2cError = view.getUint16(2, 1);
      payload.activeSensors = view.getUint16(4, 1);
      payload.mode = view.getUint32(6, 1);
      payload.profile = view.getUint8(10);
      
      this.emit_message(msp_code_name, payload);
      break;
    case codes.MSP_RAW_IMU:
      var readings = data_array_unit_parse(data);
      payload.accelerometer = readings.slice(0, 3);
      payload.gyroscope = readings.slice(3, 6);
      payload.magnetometer = readings.slice(6, 9);

      this.emit_message(msp_code_name, payload);
      break;
    case codes.MSP_SERVO:
      payload.servos = data_array_unit_parse(data);

      this.emit_message(msp_code_name, payload);
      break;
    case codes.MSP_MOTOR:
      payload.motors = data_array_unit_parse(data);

      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_RC:
      var readings = data_array_unit_parse(data);
      var names = [
        'roll',
        'pitch',
        'yaw',
        'throttle',
        'aux1',
        'aux2',
        'aux3',
        'aux4'
      ];
      
      for (var index in names) {
        var name = names[index];
        var value = readings[index];
        payload[name] = value;
      }

      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_RAW_GPS:
      payload.fix = view.getUint8(0);
      payload.numSat = view.getUint8(1);
      payload.lat = view.getUint32(2, 1);
      payload.lon = view.getUint32(6, 1);
      payload.alt = view.getUint16(10, 1);
      payload.speed = view.getUint16(12, 1);

      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_COMP_GPS:
      payload.distanceToHome = view.getUint16(0, 1);
      payload.directionToHome = view.getUint16(2, 1);
      payload.update = view.getUint8(4);

      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_ATTITUDE:
      payload.kinematicsX = view.getInt16(0, 1) / 10.0;
      payload.kinematicsY = view.getInt16(2, 1) / 10.0;
      payload.kinematicsZ = view.getInt16(4, 1);

      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_ALTITUDE:
      payload.altitude = parseFloat((view.getInt32(0, 1) / 100.0).toFixed(2)); // correct scale factor

      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_BAT:
      payload.voltage = view.getUint8(0) / 10.0;
      payload.power = view.getUint16(1, 1);

      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_RC_TUNING:
      payload.rc_rate = parseFloat((view.getUint8(0) / 100).toFixed(2));
      payload.rc_expo = parseFloat((view.getUint8(1) / 100).toFixed(2));
      payload.roll_pitch_rate = parseFloat((view.getUint8(2) / 100).toFixed(2));
      payload.yaw_rate = parseFloat((view.getUint8(3) / 100).toFixed(2));
      payload.dynamic_thr_pid = parseFloat((view.getUint8(4) / 100).toFixed(2));
      payload.throttle_mid = parseFloat((view.getUint8(5) / 100).toFixed(2));
      payload.throttle_expo = parseFloat((view.getUint8(6) / 100).toFixed(2));

      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_PID:
      var readings = data_array_unit_parse(data, null, 1)
      payload.pid = readings;

      // TODO: still need to decide how to payload it
      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_BOX:
      payload.aux_config_values = data_array_unit_parse(data);

      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_MISC:
      break; 
    case codes.MSP_MOTOR_PINS:
      break; 
    case codes.MSP_BOXNAMES:
      var aux_config = [];
      var buff = [];
      var delimiter = 0x3B // ASCII(';')

      for (var i = 0; i < data.byteLength; i++) {
        if (view.getUint8(i) == delimiter) {
          aux_config.push(String.fromCharCode.apply(null, buff));
          buff = [];
        } else {
          buff.push(view.getUint8(i));
        }
      }

      payload.aux_config = aux_config;

      this.emit_message(msp_code_name, payload);

      break; 
    case codes.MSP_PIDNAMES:
      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_SET_RAW_RC:
      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_SET_RAW_GPS:
      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_SET_PID:
      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_SET_BOX:
      this.emit_message(msp_code_name, payload);
      break; 
    case codes.MSP_SET_RC_TUNING:
      this.emit_message(msp_code_name, payload);
      break;  
    case codes.MSP_ACC_CALIBRATION:
      this.emit_message(msp_code_name, payload);
      break;  
    case codes.MSP_MAG_CALIBRATION:
      this.emit_message(msp_code_name, payload);
      break;  
    case codes.MSP_SET_MISC:
      this.emit_message(msp_code_name, payload);
      break;  
    case codes.MSP_RESET_CONF:
      this.emit_message(msp_code_name, payload);
      break;  
    case codes.MSP_SELECT_SETTING:
      this.emit_message(msp_code_name, payload);
      break;
    case codes.MSP_EEPROM_WRITE:
      this.emit_message(msp_code_name, payload);
      break;  
    case codes.MSP_DEBUGMSG:
      this.emit_message(msp_code_name, payload);
      break;  
    case codes.MSP_DEBUG:
      // we assume all variables are 16 bit and we read all of them
      var readings = data_array_unit_parse(data);
      payload.readings = readings;

      this.emit_message(msp_code_name, payload);
      break;

    // The following are Baseflight specific
    case codes.MSP_UID:
      if (data.byteLength > 0) {
        payload.uid = data_array_unit_parse(data, null, 4);

        this.emit_message(msp_code_name, payload);
      }
      break;
    case codes.MSP_ACC_TRIM:
      if (data.byteLength > 0) {
        payload.accelerometer_trims = data_array_unit_parse(data);

        this.emit_message(msp_code_name, payload);
      }
      break;
    case codes.MSP_SET_ACC_TRIM:
      break;
    case codes.MSP_GPSSVINFO:
      var readings = data_array_unit_parse(data, null, 1);
      var channels_count = readings.shift();
      payload.chn = [];
      payload.svid = [];
      payload.quality = [];
      payload.cno = [];

      for(var index = 0; index < channels_count; index++) {
        payload.chn.push(readings.shift());
        payload.svid.push(readings.shift());
        payload.quality.push(readings.shift());
        payload.cno.push(readings.shift());
      }

      this.emit_message(msp_code_name, payload);
      break;
    default:
      this.emit_message('error', 'Unknown code detected');
  }
};


module.exports = {
  Protocol: Protocol,
  codes: codes
};
