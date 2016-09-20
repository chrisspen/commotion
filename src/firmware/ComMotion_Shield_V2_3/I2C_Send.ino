
// https://www.arduino.cc/en/Tutorial/MasterReader
// http://arduino.stackexchange.com/a/3388/4478
void send_wire_int(uint16_t num){

  // Send low byte
  Wire.write((uint8_t)num);

  // Send high byte
  num >>= 8;
  Wire.write((uint8_t)num);
}

// This is called from the I2C host.
void I2C_Send()
{
  // Encoder counts (acount and bcount) are 2-byte ints
  // but I2C can only send one byte at a time,
  // so we break each up.

//  send_wire_int(acount_abs);// Send encoder A count.
//  send_wire_int(bcount_abs);// Send encoder B count.

  send_wire_int(0);
  send_wire_int(0);

}

