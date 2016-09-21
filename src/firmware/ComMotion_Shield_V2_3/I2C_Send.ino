
// This is called from the I2C host.
void I2C_Send()
{
  
  byte myArray[4];
  
  myArray[0] = lowByte(acount_abs);
  myArray[1] = highByte(acount_abs);
  myArray[2] = lowByte(bcount_abs);
  myArray[3] = highByte(bcount_abs);
  
  Wire.write(myArray, 4);
  
}

