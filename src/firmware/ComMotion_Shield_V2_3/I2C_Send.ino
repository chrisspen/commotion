
// This is called from the I2C host.
// Wire supports sending at most 32 bytes at a time.
void I2C_Send()
{
  
  byte myArray[4];
  
  myArray[0] = lowByte(acount);
  myArray[1] = highByte(acount);
  myArray[2] = lowByte(bcount);
  myArray[3] = highByte(bcount);
  
  Wire.write(myArray, 4);
  
}
