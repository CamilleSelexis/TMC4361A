

#include "TMC4361A.h"
//#include "TMC2660_REG.h"
#include "Arduino.h"

TMC4361A::TMC4361A(uint8_t cs, uint8_t rst_pin) {
	_cs = cs;
	_tgt_pin = 0;
	_rst_pin = rst_pin;
	_axis = 0;
	_vmax = VMAX_DEFAULT;
	_amax = AMAX_DEFAULT;
	_usteps = 256;
	_stepPerRev = 200;
	_rampmode = 0b101;//Trapezoidal ramp with position mode
	_encoderResolution = 4096;
}

void TMC4361A::begin() {
	//resetController();

	SPI.begin();
	_spiSettings = SPISettings(1000000,MSBFIRST,SPI_MODE3);
	//SPI.beginTransaction(_spiSettings);

	writeRegister(TMC4361A_SPIOUT_CONF,0x8440010B);//844->SPI timing 10B-> 1us between poll TMC26x S/D output
	writeRegister(TMC4361A_STEP_CONF, 0x00FB0C80);// 200 steps/rev 256 usteps
	writeRegister(TMC4361A_CLK_FREQ,CLK_FREQ); //16.7MHz external clock

	//init_EncoderSPI();//Init the encoder
	init_TMC2660();//Init the driver
	powerOffMOSFET();

	//Movement parameters
	uint32_t DIR_SETUP_TIME = 2;//#clock cycle step pulse wait after dir change
	uint32_t STP_LENGTH_ADD = 1;//#clock cycle step pulse is held
	writeRegister(TMC4361A_STP_LENGTH_ADD,((DIR_SETUP_TIME << 16) | STP_LENGTH_ADD));
	writeRegister(TMC4361A_RAMPMODE,0b101);
	//max value for 16 MHz : 4.194 Mpps = 82 rps at 256 usteps
	setVMAX(_vmax); //1 rps
	setAMAX(_amax); //Set both AMAX and DMAX
	writeRegister(TMC4361A_XACTUAL,0); //reset position
	writeRegister(TMC4361A_X_TARGET,0);//reset target
	delay(500);
	powerOnMOSFET();//Disable the MOSFET to prevent the motor from over heating

}

void TMC4361A::init_TMC2660() {

//TMC4361A_COVER_LOW_WR
	CHOPCONF_REG = 0x000901B4;
	writeRegister(TMC4361A_COVER_LOW_WR,CHOPCONF_REG); //CHOPCONF
	SGSCONF_REG = 0x000D4107;
	writeRegister(TMC4361A_COVER_LOW_WR,SGSCONF_REG); //SGSCONF Current Scale
	DRVCONF_REG = 0x000E0010;
	writeRegister(TMC4361A_COVER_LOW_WR,DRVCONF_REG); //DRVCONF SG & SPI interface
	DRVCTRL_REG = 0x00000000;
	writeRegister(TMC4361A_COVER_LOW_WR,DRVCTRL_REG); //DRVCTRL 256 usteps
	SMARTEN_REG = 0x000A8202;
	writeRegister(TMC4361A_COVER_LOW_WR,SMARTEN_REG); //coolStep Control Register
	//min current at 1/4
	//32 val out of threshold to trigger decrease
	//1 val below threshold 
}

void TMC4361A::init_EncoderSPI() {
	writeRegister(TMC4361A_ENC_IN_CONF,0x00010400); //internal multiturn
	writeRegister(TMC4361A_ENC_IN_RES_WR, 0x00001000); //resolution 4096
	writeRegister(TMC4361A_ENC_IN_DATA,0x0008000F); //21B Angle data bits + 1 2 Status bits + 1+ 1 multiturn bits
	writeRegister(TMC4361A_ADDR_TO_ENC, ENCODER_ANGLE_ADDR); //For angle data (0x2C for multiturn data)
	uint32_t SER_CLK_IN_HIGH = 0x0004;
  uint32_t SER_CLK_IN_LOW = 0x0004;
	writeRegister(TMC4361A_SER_CLK_IN_HIGH_WR,(SER_CLK_IN_LOW<<16)|SER_CLK_IN_HIGH);
	writeRegister(TMC4361A_SSI_IN_CLK_DELAY_WR, 0x00F00080); //8*16 clock between cs low & start of data transfer
	writeRegister(TMC4361A_SER_PTIME_WR, 0x13880);// 5ms between call

	writeRegister(TMC4361A_GENERAL_CONF,0x00006020|0x00300C00);//Encoder in SPI mode
}

void TMC4361A::resetController() {
	pinMode(_rst_pin,OUTPUT);
	digitalWrite(_rst_pin,LOW);
	delay(100);
	digitalWrite(_rst_pin,HIGH);
	pinMode(_rst_pin,INPUT);
}
void TMC4361A::powerOffMOSFET() {
	writeRegister(TMC4361A_COVER_LOW_WR,0x000901B0); //CHOPCONF Disable the MOSFET
	delay(2);
}
void TMC4361A::powerOnMOSFET() {
	writeRegister(TMC4361A_COVER_LOW_WR,0x000901B4);//CHOPCONF Enable the MOSFET
	delay(2);
}
//Set target absolute
void TMC4361A::setTarget(uint32_t xtarget) { 
	powerOnMOSFET();
	writeRegister(TMC4361A_X_TARGET, xtarget);
}
//Set target relative to current position
void TMC4361A::setTargetRelative(uint32_t xrelative) {
	uint32_t xtarget = getCurrentPos() + xrelative;
	powerOnMOSFET();
	writeRegister(TMC4361A_X_TARGET,xtarget);
}

uint32_t TMC4361A::getCurrentPos(){
	return readRegister(TMC4361A_XACTUAL);
}
void TMC4361A::setCurrentPos(long pos){
	writeRegister(TMC4361A_XACTUAL,pos);
}

uint32_t TMC4361A::getCurrentTarget(){
	return readRegister(TMC4361A_X_TARGET);
}

void TMC4361A::setVMAX(uint32_t vmax, byte format){
	switch (format){
		case 0: //no calculation
			_vmax = vmax;
			break;
		case 1: //vmax in turn/s
			_vmax = ((vmax*_stepPerRev*_usteps)<<8);//eight first bit are after the coma
			break;
		case 2: //vmax in rpm
			_vmax = ((vmax*_stepPerRev*_usteps/60)<<8);
			break;
		default: 
			_vmax = VMAX_DEFAULT;
			break;
	}
	writeRegister(TMC4361A_VMAX,_vmax);
}

void TMC4361A::setVMAX(uint32_t vmax){
	setVMAX(vmax,0);
}

uint32_t TMC4361A::getVMAX(){
	return readRegister(TMC4361A_VMAX);
}
//AMAX is only 24bit long so 8 MSB are useless
void TMC4361A::setAMAX(uint32_t amax){
	_amax = amax;
	writeRegister(TMC4361A_AMAX,_amax);
	writeRegister(TMC4361A_DMAX,_amax);
}

uint32_t TMC4361A::getAMAX(){
	return readRegister(TMC4361A_AMAX);
}

//Return the encoder angle as given by the encoder
float TMC4361A::getEncoderAngle(){
	uint32_t angle_data = readRegister(TMC4361A_ADDR_FROM_ENC);
	float angle = float(angle_data&0x0fff)*360/_encoderResolution;

	return angle;
}

//Return the Encoder multiturn position given by the encoder
float TMC4361A::getEncoderTurn(){
	writeRegister(TMC4361A_ADDR_TO_ENC,ENCODER_TURN_ADDR);
	delay(10);
	uint32_t turn_data = readRegister(TMC4361A_ADDR_FROM_ENC);
	float turn = float(turn_data&0x0fff)/8;

	writeRegister(TMC4361A_ADDR_TO_ENC,ENCODER_ANGLE_ADDR);
	return turn;
}

//Return the encoder pos from the TMC4361A
uint32_t TMC4361A::getEncoderPos(){
	return readRegister(TMC4361A_ENC_POS);
}

void TMC4361A::setClock(uint32_t clockFreq){
	writeRegister(TMC4361A_CLK_FREQ,clockFreq);
}

uint32_t TMC4361A::getClock(){
	return readRegister(TMC4361A_CLK_FREQ);
}

void TMC4361A::setUsteps(int usteps){
	byte code;
	switch (usteps){
		case 1:
			code = 0b1000;
			break;
		case 2:
			code = 0b0111;
			break;
		case 4:
			code = 0b0110;
			break;
		case 8:
			code = 0b0101;
			break;
		case 16:
			code = 0b0100;
			break;
		case 32:
			code = 0b0011;
			break;
		case 64:
			code = 0b0010;
			break;
		case 128:
			code = 0b0001;
			break;
		case 256:
			code = 0b0000;
			break;
		default:
			Serial.println("Incorrect ustepping value, try using 1,2,4,8,16,32,64,128,256");
			code = 0b0000;
			break;
	}
	_usteps = usteps;
	DRVCTRL_REG = (DRVCTRL_REG&0xfffffff0)|code;
	writeRegister(TMC4361A_COVER_LOW_WR,DRVCTRL_REG);//clear the last 4 bits and reset them accordingly
}

int TMC4361A::getUsteps(){
	return _usteps;
}

void TMC4361A::setCurrentScale(uint8_t currentScale){
	if(currentScale>32 || currentScale<0){
		Serial.println("CurrentScale is a 5 bit value (0 -> 32)");
		return;
	}
	SGSCONF_REG = (SGSCONF_REG&0xffffffe0)|currentScale;
	writeRegister(TMC4361A_COVER_LOW_WR, SGSCONF_REG);
}

uint8_t TMC4361A::getCurrentScale(){
	return SGSCONF_REG&0x0000001f;
}

bool TMC4361A::checkFlag(TMC4361A::FlagType flag) {
	return readRegisterBit(TMC4361A_STATUS, flag);
}

bool TMC4361A::isTargetReached()
{
  return checkFlag(TARGET_REACHED_F);
}

void TMC4361A::writeRegister(const byte address, const long data)
{
  spiTransfer(address | 0x80, data);
}

long TMC4361A::readRegister(const byte address)
{
  spiTransfer(address & 0x7F, 0); //Dummy call to load the read address
  return spiTransfer(address & 0x7F, 0);
}

void TMC4361A::setRegisterBit(const byte address, const byte bit)
{
  uint32_t value = readRegister(address);
  bitSet(value, bit);
  writeRegister(address, value);
}

void TMC4361A::clearRegisterBit(const byte address, const byte bit)
{
  uint32_t value = readRegister(address);
  bitClear(value, bit);
  writeRegister(address, value);
}

bool TMC4361A::readRegisterBit(const byte address, const byte bit)
{
  return bitRead(readRegister(address), bit);
}

long TMC4361A::spiTransfer(const byte address, const long data)
{
  long returnBuffer = 0;

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_cs, LOW);
  //digitalWrite(D7,LOW);
  delay(5);

  _spiStatus = SPI.transfer(address);
  //Send data MSB first
  for (int i = 3; i >= 0; i--)
    returnBuffer |= (SPI.transfer((data >> (i*8)) & 0xFF) << (i*8));

  digitalWrite(_cs, HIGH);
  //digitalWrite(D7,HIGH);
  SPI.endTransaction();

  return returnBuffer;
}
