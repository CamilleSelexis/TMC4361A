//Library develloped for the ACCNT encoders

#include "TMC4361A.h"
//#include "TMC2660_REG.h"
#include <Arduino.h>
//#include "RPC.h"

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
	_encoderResolution = 131072;
}

void TMC4361A::begin() {
//resetController();
	SPI.begin();
	_spiSettings = SPISettings(1000000,MSBFIRST,SPI_MODE3);
	//SPI.beginTransaction(_spiSettings);

	//Reset the controller
	writeRegister(TMC4361A_RESET_REG,0x52535400); // reset code
	delay(200);
	writeRegister(TMC4361A_SPIOUT_CONF,0x8440010B);//844->SPI timing 10B-> 1us between poll TMC26x S/D output
	writeRegister(TMC4361A_STEP_CONF, 0x00FB0C80);// 200 steps/rev 256 usteps
	writeRegister(TMC4361A_CLK_FREQ,CLK_FREQ); //20MHz external clock

	//init_EncoderSPI();//Init the encoder
	//init_EncoderSSI();
	delay(200);
	init_TMC2660();//Init the driver
	powerOffMOSFET();

	//Movement parameters
	uint32_t DIR_SETUP_TIME = 4;//#clock cycle step pulse wait after dir change
	uint32_t STP_LENGTH_ADD = 1;//#clock cycle step pulse is held -> 250 ns?
	writeRegister(TMC4361A_STP_LENGTH_ADD,((DIR_SETUP_TIME << 16) | STP_LENGTH_ADD));
	writeRegister(TMC4361A_RAMPMODE,0b101); //Trapezoidal motion profile in positioning mode
	//max value for 16 MHz : 4.194 Mpps = 82 rps at 256 usteps
	setVMAX(_vmax); //1 rps
	setAMAX(_amax); //Set both AMAX and DMAX
	writeRegister(TMC4361A_XACTUAL,0); //reset position
	writeRegister(TMC4361A_X_TARGET,0);//reset target
	delay(500);
	powerOnMOSFET();//Enable the MOSFET

}

void TMC4361A::init_TMC2660() {

	//add control that coverdone is properly set
	uint32_t events;

	//TMC4361A_COVER_LOW_WR
	CHOPCONF_REG = 0x000901B4;
	writeRegister(TMC4361A_COVER_LOW_WR,CHOPCONF_REG); //CHOPCONF
	events = clearEvent();
	//Check that cover done was set

	SGSCONF_REG = 0x000D4107;
	//SGSCONF_REG = 0x000D410A;
	writeRegister(TMC4361A_COVER_LOW_WR,SGSCONF_REG); //SGSCONF Current Scale
	events = clearEvent();

	DRVCONF_REG = 0x000E0000; //Msteps readback
	writeRegister(TMC4361A_COVER_LOW_WR,DRVCONF_REG); //DRVCONF SG & SPI interface
	events = clearEvent();

	DRVCTRL_REG = 0x00000000;
	writeRegister(TMC4361A_COVER_LOW_WR,DRVCTRL_REG); //DRVCTRL 256 usteps
	events = clearEvent();

	SMARTEN_REG = 0x000A8202;
	writeRegister(TMC4361A_COVER_LOW_WR,SMARTEN_REG); //coolStep Control Register
	events = clearEvent();
	//min current at 1/4
	//32 val out of threshold to trigger decrease
	//1 val below threshold 
}

void TMC4361A::init_CLPosital(uint32_t ZeroPos) {
		 // Closed Loop calibration of TMC4361 Motion Controller with SPI encoder

	//Standard SSI encoder configuration
	uint32_t ENC_IN_CONF = 0x00001000; //Multiturn data EN
	//ENC_IN_CONF = ENC_IN_CONF | 0x10000; //Internal multiturn enable -> this way it can properly behave whith overflows
	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF);
	writeRegister(TMC4361A_ENC_IN_RES_WR, 0x00020000); //resolution = 131072
	uint8_t SINGLETURN_RES = 0x10; // 17-1 = 16
	uint8_t MULTITURN_RES = 0x0F; //16 multi + 8 blank = 24 24-1 = 23 //17
	uint8_t STATUS_BIT_CNT = 0x00; //Status bits set as multiturn bits, but unused nonetheless
	uint8_t SERIAL_ADDR_BITS = 0x00; //8 bits for the address
	uint8_t SERIAL_DATA_BITS = 0x00; //0 DAta bits for encoder config
	uint32_t ENC_IN_DATA = SINGLETURN_RES | MULTITURN_RES<<5 | STATUS_BIT_CNT <<10 |SERIAL_ADDR_BITS <<16 |SERIAL_DATA_BITS << 24;
	writeRegister(TMC4361A_ENC_IN_DATA,ENC_IN_DATA);//0x00003010

	//Encoder at 1 MHz -> so 20 native clock cycle for 1 encoder clock cycle
	uint32_t SER_CLK_IN_HIGH = 0x000A;
  uint32_t SER_CLK_IN_LOW = 0x000A;
	writeRegister(TMC4361A_SER_CLK_IN_HIGH_WR,(SER_CLK_IN_LOW<<16)|SER_CLK_IN_HIGH);
	uint32_t SSI_IN_CLK_DELAY = readRegister(TMC4361A_SSI_IN_CLK_DELAY_WR);
	SSI_IN_CLK_DELAY = SSI_IN_CLK_DELAY & 0xFFFF0000;
	SSI_IN_CLK_DELAY = SSI_IN_CLK_DELAY | 0x00AA; //8*20 + 10 = 170
	writeRegister(TMC4361A_SSI_IN_CLK_DELAY_WR,SSI_IN_CLK_DELAY);
	writeRegister(TMC4361A_SER_PTIME_WR,0x007D0); //100us between call

	writeRegister(TMC4361A_GENERAL_CONF,0x00006020|0x00000400);//Encoder in SSI mode
	//---------------------------------------------------------------------------
	delay(500);
	clearEvent();
	//Set current position
	writeRegister(TMC4361A_VMAX, 0);
	delay(5);
	uint32_t encoderPos =getEncoderPos();
	long currentPos = 0;
	uint32_t overflow = 3355437030;
	if(abs(long(encoderPos-ZeroPos)) >20000000){
		if(encoderPos < ZeroPos) currentPos = overflow-ZeroPos+encoderPos; // Overflow was passed positively
		if(encoderPos > ZeroPos) currentPos = encoderPos-overflow -ZeroPos; //Overflow was passed negatively
		//currentPos = 0;
	}
	else {
		currentPos = encoderPos-ZeroPos;
	}
	setCurrentPos(currentPos);
	setTarget(currentPos);
	delay(5);
	writeRegister(TMC4361A_VMAX, 0);
	//Closed loop configuration -> basic calibration
	writeRegister(TMC4361A_CL_BETA,0x000000FF); //CL_BETA = 255
	writeRegister(TMC4361A_CL_DELTA_P_WR,0x00010000); //CL_DELTA_P = 1
	uint16_t CL_CYCLE = 0x07D0; // >= encoder request rate = 100 us
	uint16_t SER_ENC_VAR = 0xFF; // Automatically at 1/8*ENC_IN_RES -> 16384 step -> Max speed = 16384*10000 = 163840000 steps/s = 1000 tour/sec
	uint32_t CYCLE_VAR = CL_CYCLE<<16 | SER_ENC_VAR;
	writeRegister(TMC4361A_CL_CYCLE_WR,CYCLE_VAR); //Closed loop cycle = 100us
	//Calibration Procedure--------------------------------------------------------------
 	//Move the motor to a full step
 	writeRegister(TMC4361A_RAMPMODE,0b100); //Change rampmode for calibration
	writeRegister(TMC4361A_VMAX,0x00100000); //Slow speed 4096 Us/s for calibration
	delay(50);
	//Put TMC2660 read response in uSteps format
	uint32_t MSCNT = readRegister(TMC4361A_COVER_LOW_WR);

	SGSCONF_REG = SGSCONF_REG |0x0000001F; //Full current during calibration 0x000D4107
 	writeRegister(TMC4361A_COVER_LOW_WR,SGSCONF_REG);

 	setTargetRelative(384-(MSCNT>>10)%256); //Move to a full step
 	delay(50); //Move the motor on a full step
 	ENC_IN_CONF = ENC_IN_CONF | 0x00400000; //regulation modus = cl 0x00411000
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF);

 	ENC_IN_CONF = ENC_IN_CONF | 0x01000000; //CL_calibration enable 0x01411000
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF); //Start cl calibration
 	delay(5);
 	//writeRegister(TMC4361A_CL_OFFSET,ZeroPos);
 	delay(50);
 	ENC_IN_CONF = ENC_IN_CONF & 0xFEFFFFFF; //Clear cl_calibration
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF); //Turn off cl calibration 0x00411000
 	SGSCONF_REG = 0x000D4107; //Reset current scale
 	writeRegister(TMC4361A_COVER_LOW_WR,SGSCONF_REG);

 	writeRegister(TMC4361A_VMAX,0x00000000); //VMAX to 0
 	//-----------------------------------------------------------------------------------
 	ENC_IN_CONF = ENC_IN_CONF | 0x00400000;
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF); //Turn on closed loop operation
 	
 	//Setup max value between 2 encoder calls
 	SER_ENC_VAR = 0xFF; //maximum value permitted
 	ENC_IN_CONF = ENC_IN_CONF | 0x80000000; //Serial encoder variation limit enable
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF);

 	//Setup motion profile
 	writeRegister(TMC4361A_RAMPMODE,0b110); //S-shaped ramp
 	writeRegister(TMC4361A_BOW1,AMAX_DEFAULT/2);
 	writeRegister(TMC4361A_BOW2,AMAX_DEFAULT);
 	writeRegister(TMC4361A_BOW3,AMAX_DEFAULT);
 	writeRegister(TMC4361A_BOW4,AMAX_DEFAULT/2);
 	writeRegister(TMC4361A_AMAX,AMAX_DEFAULT);
 	writeRegister(TMC4361A_DMAX,AMAX_DEFAULT);
 	writeRegister(TMC4361A_VMAX,VMAX_DEFAULT);
 	//Set max encoder variation
 	writeRegister(TMC4361A_CL_TR_TOLERANCE_WR,0x00000100); //Tolerance for target reached = 256 -> 1FS
 	writeRegister(TMC4361A_ENC_POS_DEV_TOL_WR,0x00000A00); //Max ENC_POS_DEV before considered as error -> 10 FS 2560
 	
 	//PI
 	writeRegister(TMC4361A_CL_TOLERANCE_WR,0x00000080); //cl_tolerance = 128 -> CL_delta maxed at 128 uS error
 	writeRegister(TMC4361A_CL_DELTA_P_WR,0x00010000); //cl_p = 1

 	//Catch up velocity param
 	writeRegister(TMC4361A_CL_VMAX_CALC_P_WR,0x00000400); //16*16 prop term for vel limit
 	writeRegister(TMC4361A_CL_VMAX_CALC_I_WR,0x00000032); //50 int term for vel limit
 	writeRegister(TMC4361A_PID_I_CLIP_WR,0x00500000); //DV_CLIP/PID_I
 	writeRegister(TMC4361A_PID_DV_CLIP_WR,0x01000000); //cl_vlimit = 1/10 of max vel
 	ENC_IN_CONF = ENC_IN_CONF | 0x08000000; //Enable catch up velocity limitation
 	ENC_IN_CONF = ENC_IN_CONF |0x10000; //Internal multiturn -- Disable to find proper 0 position
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF);
 	
 	clearEvent();

}

void TMC4361A::init_EncoderSSI() {
	//Posital encoder need 8 blank clock cycle to generate data, 16 muliturn bits and 17 single turn -> transmission length = 41 bits
	//Max clock for the encoder before changing the setup time is 1MHz
	//Max cycle time at 50 us
	uint32_t ENC_IN_CONF = 0x00001000; //default 0x00010400 -> Encoder gives multiturn data
	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF); //0x00015400  -- 0x00010400internal multiturn calc with enc pos latched on N event ??
	writeRegister(TMC4361A_ENC_IN_RES_WR, 0x00020000); //resolution 131072 ENC_Const calculated automatically = 0.39
	uint8_t SINGLETURN_RES = 0x10; // 17-1 = 16
	uint8_t MULTITURN_RES = 0x17; // 16 multiturn + 8 blank = 24 24-1 = 0x17
	uint8_t STATUS_BIT_CNT = 0x00; //Status bits set as multiturn bits, but unused nonetheless
	uint8_t SERIAL_ADDR_BITS = 0x00; //0 bits for the address SPI only
	uint8_t SERIAL_DATA_BITS = 0x00; //0 DAta bits for encoder config SPI only
	uint32_t ENC_IN_DATA = SINGLETURN_RES | MULTITURN_RES<<5 | STATUS_BIT_CNT <<10 |SERIAL_ADDR_BITS <<16 |SERIAL_DATA_BITS << 24;
	writeRegister(TMC4361A_ENC_IN_DATA,ENC_IN_DATA);//0x00003010
	//20MHz clock -> Period = 50ns
	//Posital encoder goes at max 1MHz for comm -> 20 MHz clock
	uint32_t SER_CLK_IN_HIGH = 0x000A; //1MHz from 20MHz clock -> 20/20 = 1 -> 20 CS for total comm
  uint32_t SER_CLK_IN_LOW = 0x000A; // then 10 clock for high and 10 for low
	writeRegister(TMC4361A_SER_CLK_IN_HIGH_WR,(SER_CLK_IN_LOW<<16)|SER_CLK_IN_HIGH);
	//This should work but it doesn't seem to work with the multiturn data
	//uint32_t SSI_IN_CLK_DELAY = 0x00B4; //9*20 clock cycle before start - 8 blank clock cycle + 1 clock for setup ?
	//writeRegister(TMC4361A_SSI_IN_CLK_DELAY_WR, SSI_IN_CLK_DELAY); //9*20 clock between cs low & start of data transfer
	writeRegister(TMC4361A_SER_PTIME_WR,0x007D0); //100us between call
	//writeRegister(TMC4361A_GENERAL_CONF,0x00006020|0x00300400);//Encoder in SSI mode, DCsetp & full step something
	writeRegister(TMC4361A_GENERAL_CONF,0x00006020|0x00000400); //Encoder in SSI mode
}

void TMC4361A::init_EncoderSPI() {
	uint32_t ENC_IN_CONF = 0x00014400; //default 0x00010400 -> 4 = ENC_POS is latched to enc_latch / 1 = multiturn data / 1 = internal multiturn /
	writeRegister(TMC4361A_ENC_IN_CONF,0x00014400); //0x00015400  -- 0x00010400internal multiturn calc with enc pos latched on N event ??
	writeRegister(TMC4361A_ENC_IN_RES_WR, 0x00001000); //resolution 4096 ENC_Const calculated automatically = 12.5
	uint8_t SINGLETURN_RES = 0x0B; // 12-1 = 11
	uint8_t MULTITURN_RES = 0x03; //4-1 = 3
	uint8_t STATUS_BIT_CNT = 0x00; //Status bits set as multiturn bits, but unused nonetheless
	uint8_t SERIAL_ADDR_BITS = 0x08; //8 bits for the address
	uint8_t SERIAL_DATA_BITS = 0x00; //0 DAta bits for encoder config
	uint32_t ENC_IN_DATA = SINGLETURN_RES | MULTITURN_RES<<5 | STATUS_BIT_CNT <<10 |SERIAL_ADDR_BITS <<16 |SERIAL_DATA_BITS << 24;
	writeRegister(TMC4361A_ENC_IN_DATA,ENC_IN_DATA);//0x0008000F
	writeRegister(TMC4361A_ADDR_TO_ENC, ENCODER_ANGLE_ADDR); //For angle data (0x2C for multiturn data)
	uint32_t SER_CLK_IN_HIGH = 0x0004;
  uint32_t SER_CLK_IN_LOW = 0x0004;
	writeRegister(TMC4361A_SER_CLK_IN_HIGH_WR,(SER_CLK_IN_LOW<<16)|SER_CLK_IN_HIGH);
	writeRegister(TMC4361A_SSI_IN_CLK_DELAY_WR, 0x00F00080); //8*16 clock between cs low & start of data transfer
	//writeRegister(TMC4361A_SER_PTIME_WR, 0x13880);// 5ms between call
	writeRegister(TMC4361A_SER_PTIME_WR,0x01900); //400us between call

	writeRegister(TMC4361A_GENERAL_CONF,0x00006020|0x00300C00);//Encoder in SPI mode
}

void TMC4361A::init_closedLoop() {
	 // Closed Loop calibration of TMC4361 Motion Controller with SPI encoder

	//Standard SPI encoder configuration
	uint32_t ENC_IN_CONF = 0x00011000; //default 0x00010400 -> 4 = ENC_POS is latched to enc_latch / 1 = multiturn data / 1 = internal multiturn /
	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF); //0x00015400  -- 0x00010400internal multiturn calc with enc pos latched on N event ??
	writeRegister(TMC4361A_ENC_IN_RES_WR, 0x00001000); //resolution 4096 ENC_Const calculated automatically = 12.5
	uint8_t SINGLETURN_RES = 0x0B; // 12-1 = 11
	uint8_t MULTITURN_RES = 0x03; //4-1 = 3
	uint8_t STATUS_BIT_CNT = 0x00; //Status bits set as multiturn bits, but unused nonetheless
	uint8_t SERIAL_ADDR_BITS = 0x08; //8 bits for the address
	uint8_t SERIAL_DATA_BITS = 0x00; //0 DAta bits for encoder config
	uint32_t ENC_IN_DATA = SINGLETURN_RES | MULTITURN_RES<<5 | STATUS_BIT_CNT <<10 |SERIAL_ADDR_BITS <<16 |SERIAL_DATA_BITS << 24;
	writeRegister(TMC4361A_ENC_IN_DATA,ENC_IN_DATA);//0x0008000F
	writeRegister(TMC4361A_ADDR_TO_ENC, ENCODER_ANGLE_ADDR); //For angle data (0x2C for multiturn data)
	uint32_t SER_CLK_IN_HIGH = 0x0002;
  uint32_t SER_CLK_IN_LOW = 0x0002;
	writeRegister(TMC4361A_SER_CLK_IN_HIGH_WR,(SER_CLK_IN_LOW<<16)|SER_CLK_IN_HIGH);
	writeRegister(TMC4361A_SSI_IN_CLK_DELAY_WR, 0x00F00004); //4 clock between cs low & start of data transfer
	//writeRegister(TMC4361A_SER_PTIME_WR, 0x13880);// 5ms between call
	writeRegister(TMC4361A_SER_PTIME_WR,0x003E8); //50us between call

	writeRegister(TMC4361A_GENERAL_CONF,0x00006020|0x00300C00);//Encoder in SPI mode
	delay(500);
	clearEvent();
	//Closed loop configuration -> basic calibration
	writeRegister(TMC4361A_CL_BETA,0x000000FF); //CL_BETA = 255
	writeRegister(TMC4361A_CL_DELTA_P_WR,0x00010000); //CL_DELTA_P = 1
	uint32_t CL_CYCLE = 0x03E80000;
	writeRegister(TMC4361A_CL_CYCLE_WR,CL_CYCLE); //Update encoder value every 50 us
	writeRegister(TMC4361A_RAMPMODE,0b100); //Change rampmode for calibration
	writeRegister(TMC4361A_VMAX,0x00100000); //Slow speed 4096 Us/s for calibration

	//Put TMC2660 read response in uSteps format
	/*uint32_t MSCNT = readRegister(TMC4361A_COVER_LOW_WR);
	writeRegister(TMC4361A_X_TARGET,384-(MSCNT>>10)%256); // the 10 MSB are the msteps
 	delay(50);*/
 	ENC_IN_CONF = ENC_IN_CONF | 0x00400000; //regulation modus = cl
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF);
 	ENC_IN_CONF = ENC_IN_CONF | 0x01000000; //CL_calibration enable
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF); //Start cl calibration
 	delay(50);
 	ENC_IN_CONF = ENC_IN_CONF & 0xFEFFFFFF; //Clear cl_calibration
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF); //Turn off cl calibration*/
 	writeRegister(TMC4361A_VMAX,0x00000000); //VMAX to 0

 	//Setup max value between 2 encoder calls
 	//SER_ENC_VARIATION = 10 -> 255*1/8*4096/256 = 512
 	writeRegister(TMC4361A_CL_CYCLE_WR,CL_CYCLE | 0x000000FF); //defines the multiplier for max tolerated value shouldn't be greater than 180° = 524288 uSTeps
 	ENC_IN_CONF = ENC_IN_CONF | 0x80000000;
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF);

 	//Setup closed loop operation
 	writeRegister(TMC4361A_RAMPMODE,0b101); //Set trapez ramp with POS mode
 	writeRegister(TMC4361A_VMAX,VMAX_DEFAULT); //2 turn/s
 	writeRegister(TMC4361A_AMAX,AMAX_DEFAULT);
 	writeRegister(TMC4361A_DMAX,AMAX_DEFAULT);

 	//Set max encoder variation
 	writeRegister(TMC4361A_CL_TR_TOLERANCE_WR,0x00000010); //CL tolerance for target reached event = 256 target reached if within 1 FS
 	writeRegister(TMC4361A_ENC_POS_DEV_TOL_WR,0x00002000); //Max tolerated dev is 8192 before ENC_FAIL_Flag is set
 	writeRegister(TMC4361A_CL_TOLERANCE_WR,0x00000020); //cl_tolerance = 128 max dev before CL_delta is maxed
 	writeRegister(TMC4361A_CL_DELTA_P_WR,0x00010000); //cl_p = 1
 	writeRegister(TMC4361A_CL_VMAX_CALC_P_WR,0x00000032); //50 prop term for vel limit
 	writeRegister(TMC4361A_CL_VMAX_CALC_I_WR,0x00000032); //50 int term for vel limit
 	writeRegister(TMC4361A_PID_I_CLIP_WR,0x000000FF); //255 clipping value for int term vel limit
 	writeRegister(TMC4361A_PID_DV_CLIP_WR,0x00002710); //cl_vlimit = 10000 pps

 	ENC_IN_CONF = ENC_IN_CONF | 0x08000000; //Enable catch up velocity
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF);
 	clearEvent();
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
void TMC4361A::setTarget(long xtarget) { 
	powerOnMOSFET();
	writeRegister(TMC4361A_X_TARGET, xtarget);
}
//Set target relative to current position
void TMC4361A::setTargetRelative(long xrelative) {
	long xtarget = getCurrentPos() + xrelative;
	powerOnMOSFET();
	writeRegister(TMC4361A_X_TARGET,xtarget);
}

long TMC4361A::getCurrentPos(){
	return readRegister(TMC4361A_XACTUAL);
}
void TMC4361A::setCurrentPos(long pos){
	writeRegister(TMC4361A_XACTUAL,pos);
}

long TMC4361A::getCurrentTarget(){
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
	uint32_t angle_data = getEncoderPos();
	float angle = float(angle_data&0x1FFFF)*360/_encoderResolution;

	return angle;
}

float TMC4361A::getEncoderAngleSSI(){
	uint32_t angle_data = readRegister(TMC4361A_ADDR_FROM_ENC);
	float angle = float(angle_data & 0x1ffff)*360/131072;
	return angle;
}
//Return the Encoder multiturn position given by the encoder
float TMC4361A::getEncoderTurn(){
	writeRegister(TMC4361A_ADDR_TO_ENC,ENCODER_TURN_ADDR);
	delay(10);
	uint32_t turn_data = readRegister(TMC4361A_ADDR_FROM_ENC);
	float turn = float(turn_data&0x0fff)/8;

	writeRegister(TMC4361A_ADDR_TO_ENC,ENCODER_ANGLE_ADDR);//Rewrite the encoder angle address
	return turn;
}

//Return the encoder pos from the TMC4361A
uint32_t TMC4361A::getEncoderPos(){
	return readRegister(TMC4361A_ENC_POS);
}

uint32_t TMC4361A::getEncoderData(){
	return readRegister(TMC4361A_DATA_FROM_ENC);
}

uint32_t TMC4361A::getEncoderRaw(){
	return readRegister(TMC4361A_ADDR_FROM_ENC);
}

void TMC4361A::alignEncoder(){
	uint32_t Xactual = readRegister(TMC4361A_XACTUAL);
	writeRegister(TMC4361A_ENC_POS,Xactual);
}

long TMC4361A::getEncoderDev(){
	return readRegister(TMC4361A_ENC_POS_DEV_RD);
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

uint32_t TMC4361A::clearEvent(){
	uint32_t events = readRegister(TMC4361A_EVENTS);
}

bool TMC4361A::checkFlag(TMC4361A::FlagType flag) {
	return readRegisterBit(TMC4361A_STATUS, flag);
}

bool TMC4361A::isEncoderFail(){
	return checkFlag(ENC_FAIL_F);
}


bool TMC4361A::isSerialEncoderVar(){
	return checkFlag(SER_ENC_VAR_F);
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

void TMC4361A::begin_closedLoop(){
	SPI.begin();
	_spiSettings = SPISettings(1000000,MSBFIRST,SPI_MODE3);
	//SPI.beginTransaction(_spiSettings);
	uint32_t event;
	//Reset the controller
	writeRegister(TMC4361A_RESET_REG,0x52535400); // reset code
	delay(200);
	writeRegister(TMC4361A_SPIOUT_CONF,0x4220000A);//844->SPI timing A -> SPI mode with current 
	writeRegister(TMC4361A_STEP_CONF, 0x00FB0C80);// 200 steps/rev 256 usteps
	writeRegister(TMC4361A_CLK_FREQ,CLK_FREQ); //20MHz external clock
	writeRegister(TMC4361A_INTR_CONF,0x02000000); //Coverdone as intr
	event = readRegister(TMC4361A_EVENTS); //Clear events

	//param TMC2660
	DRVCTRL_REG = 0x00000000;
	writeRegister(TMC4361A_COVER_LOW_WR,DRVCTRL_REG); //DRVCTRL 256 usteps
	delay(50);
	event = readRegister(TMC4361A_EVENTS); //Clear events

	CHOPCONF_REG = 0x00090585;
	writeRegister(TMC4361A_COVER_LOW_WR,CHOPCONF_REG); //CHOPCONF
	delay(50);
	event = readRegister(TMC4361A_EVENTS); //Clear events

	SMARTEN_REG = 0x000A0000;
	writeRegister(TMC4361A_COVER_LOW_WR,SMARTEN_REG); //coolStep Control Register
	delay(50);
	readRegister(TMC4361A_EVENTS); //Clear events

	SGSCONF_REG = 0x000C0018;
	writeRegister(TMC4361A_COVER_LOW_WR,SGSCONF_REG); //SGSCONF Current Scale
	delay(50);
	event = readRegister(TMC4361A_EVENTS); //Clear events	
	DRVCONF_REG = 0x000EF080;
	writeRegister(TMC4361A_COVER_LOW_WR,DRVCONF_REG); //DRVCONF SG & SPI interface
	delay(50);
	event = readRegister(TMC4361A_EVENTS); //Clear events	
	
	//Param abs encoder in SPI mode
	uint32_t ENC_IN_CONF = 0x00014400; //default 0x00010400 -> 4 = ENC_POS is latched to enc_latch / 1 = multiturn data / 1 = internal multiturn /
	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF); //0x00015400  -- 0x00010400internal multiturn calc with enc pos latched on N event ??
	writeRegister(TMC4361A_ENC_IN_RES_WR, 0x00001000); //resolution 4096 ENC_Const calculated automatically = 12.5
	uint8_t SINGLETURN_RES = 0x0B; // 12-1 = 11
	uint8_t MULTITURN_RES = 0x03; //4-1 = 3
	uint8_t STATUS_BIT_CNT = 0x00; //Status bits set as multiturn bits, but unused nonetheless
	uint8_t SERIAL_ADDR_BITS = 0x08; //8 bits for the address
	uint8_t SERIAL_DATA_BITS = 0x00; //0 DAta bits for encoder config
	uint32_t ENC_IN_DATA = SINGLETURN_RES | MULTITURN_RES<<5 | STATUS_BIT_CNT <<10 |SERIAL_ADDR_BITS <<16 |SERIAL_DATA_BITS << 24;
	writeRegister(TMC4361A_ENC_IN_DATA,ENC_IN_DATA);//0x0008000F
	writeRegister(TMC4361A_ADDR_TO_ENC, ENCODER_ANGLE_ADDR); //For angle data (0x2C for multiturn data)
	uint32_t SER_CLK_IN_HIGH = 0x0004;
  uint32_t SER_CLK_IN_LOW = 0x0004;
	writeRegister(TMC4361A_SER_CLK_IN_HIGH_WR,(SER_CLK_IN_LOW<<16)|SER_CLK_IN_HIGH);
	writeRegister(TMC4361A_SSI_IN_CLK_DELAY_WR, 0x00F00010); //8*16 clock between cs low & start of data transfer
	//writeRegister(TMC4361A_SER_PTIME_WR, 0x01900);// 400us between call
	writeRegister(TMC4361A_SER_PTIME_WR,0x003E8); //50us between call
	writeRegister(TMC4361A_GENERAL_CONF,0x00006020|0x00300C00);//Encoder in SPI mode
	
	//Start Closed loop setup
	writeRegister(TMC4361A_CL_BETA,0x000000FF); //CL_BETA = 255
	writeRegister(TMC4361A_CL_DELTA_P_WR,0x00014000); //CL_DELTA_P = 1.25
	writeRegister(TMC4361A_CL_TOLERANCE_WR,0x00000020); //cl_tolerance = 32 > ENC_CONST = 

	writeRegister(TMC4361A_CL_VMAX_CALC_P_WR,0x000003E8); //P for vel limit = 1000;
	writeRegister(TMC4361A_CL_VMAX_CALC_I_WR,0x00000032);//I for vel limit = 50
	writeRegister(TMC4361A_PID_DV_CLIP_WR,0x0000C350);//Max vel +50kpps
	writeRegister(TMC4361A_PID_I_CLIP_WR,0x000003E8); //Clipping = 1000

	writeRegister(TMC4361A_CURRENT_CONF,0x00000000); //disable current scaling
	writeRegister(TMC4361A_RAMPMODE,0b100); //Change rampmode for init
	writeRegister(TMC4361A_VMAX,0x00100000); //Slow speed 4096 Us/s
	uint32_t MSCNT = readRegister(TMC4361A_MSCNT_RD);
	uint32_t xactual = readRegister(TMC4361A_XACTUAL);
 	writeRegister(TMC4361A_X_TARGET,xactual + 384 - (MSCNT%256)); //Move to full step
 	delay(5000); //Wait for full step done
 	writeRegister(TMC4361A_VMAX,0x00000000); //VMAX to 0
 	MSCNT = readRegister(TMC4361A_MSCNT_RD);
 	writeRegister(TMC4361A_XACTUAL,MSCNT);
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF || 0x00400000); //Turn on cl operation
 	writeRegister(TMC4361A_ENC_IN_CONF,ENC_IN_CONF || 0x08000000); //Turn on velocity limit

 	writeRegister(TMC4361A_SCALE_VALUES,0x0064FF64);//Set current scaling values
 	writeRegister(TMC4361A_CL_UPSCALE_DELAY,0x000003E8); //cl_upscale
 	writeRegister(TMC4361A_CL_DOWNSCALE_DELAY,0x000186A0); //cl_downscale
 	writeRegister(TMC4361A_CURRENT_CONF,0x00000080); //Enable current scaling
 	
 	writeRegister(TMC4361A_CL_TR_TOLERANCE_WR,0x0000003C); //CL_TOLERANCE = 60 3 encoder transitions
 	//Set Ramp values
 	writeRegister(TMC4361A_RAMPMODE,0b101); //Set back S ramp with POS mode
 	writeRegister(TMC4361A_VMAX,VMAX_DEFAULT); //2 turn/s
 	writeRegister(TMC4361A_AMAX,AMAX_DEFAULT);
 	writeRegister(TMC4361A_DMAX,AMAX_DEFAULT);

}

void TMC4361A::beginCL() {
	//resetController();

	SPI.begin();
	_spiSettings = SPISettings(1000000,MSBFIRST,SPI_MODE3);
	//SPI.beginTransaction(_spiSettings);

	//Reset the controller
	writeRegister(TMC4361A_RESET_REG,0x52535400); // reset code
	delay(200);
	clearEvent();
	//writeRegister(TMC4361A_EVENT_CLEAR_CONF,0b1<<14 | 1 << 17); //ENC_FAIL & SER_ENC_VAR is not cleared when reading event

	writeRegister(TMC4361A_SPIOUT_CONF,0x8440010B);//844->SPI timing 10B-> 1us between poll TMC26x S/D output
	writeRegister(TMC4361A_STEP_CONF, 0x00FB0C80);// 200 steps/rev 256 usteps
	writeRegister(TMC4361A_CLK_FREQ,CLK_FREQ); //20MHz external clock

	//init_EncoderSPI();//Init the encoder
	
	init_TMC2660();//Init the driver
	//Movement parameters
	uint32_t DIR_SETUP_TIME = 2;//#clock cycle step pulse wait after dir change
	uint32_t STP_LENGTH_ADD = 1;//#clock cycle step pulse is held
	writeRegister(TMC4361A_STP_LENGTH_ADD,((DIR_SETUP_TIME << 16) | STP_LENGTH_ADD));
	clearEvent();
	init_closedLoop();
	delay(500);
	//powerOnMOSFET();//Disable the MOSFET to prevent the motor from over heating

}