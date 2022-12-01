#ifndef _TMC4361A_H
#define _TMC4361A_H

#include "Arduino.h"
#include "SPI.h"
#include "TMC4361A_Register.h"
#include "TMC2660_REG.h"


#define VMAX_DEFAULT 0x01000000 // 2 turn/s 23 digits / 8 decimal max value for 16 MHz : 4.194 Mpps = 82 rps at 256 usteps
//#define VMAX_DEFAULT 0x00C80000
//#define AMAX_DEFAULT 0x0000FFFF
#define AMAX_DEFAULT 0x0000FFFC //22 digits / 2 decimalmax value for 16 MHz : 2.097 Mpps2
#define AMAX_SLOW 	 0x0000FFFF

#define CLK_FREQ 	 20000000
#define ENCODER_ANGLE_ADDR 0x20
#define ENCODER_TURN_ADDR 0x2C

class TMC4361A
{
	private:
		uint8_t _axis;
		uint8_t _cs;
		uint8_t _tgt_pin;
		uint8_t _rst_pin;
		long _clk_freq;
		SPISettings _spiSettings;
		byte _spiStatus;
		uint32_t _rampmode;
		uint32_t _vmax;
		uint32_t _amax; //max acceleration use same value for deceleration
		int _usteps;
		int _stepPerRev;
		int _encoderResolution;

		uint32_t GENERAL_CONF;
		uint32_t SPIOUT_CONF;
		uint32_t STEP_CONF;
		uint32_t ENC_IN_CONF;
		uint32_t ENC_IN_RES_WR;
		uint32_t ENC_IN_DATA;
		uint32_t SER_PTIME_WR;

	 	long spiTransfer(const byte address, const long data);

	public:

		enum FlagType {
		    TARGET_REACHED_F = 0,
		    POS_COMP_REACHED_F,
		    VEL_REACHED_F,
		    VEL_STATE_F0,
		    VEL_STATE_F1,
		    RAMP_STATE_F0,
		    RAMP_STATE_F1,
		    STOPL_ACTIVE_F,
		    STOPR_ACTIVE_F,
		    VSTOPL_ACTIVE_F,
		    VSTOPR_ACTIVE_F,
		    ACTIVE_STALL_F,
		    HOME_ERROR_F,
		    FS_ACTIVE_F,
		    ENC_FAIL_F = 14,
		    N_ACTIVE_F,
		    ENC_LATCH_F,
		    SER_ENC_VAR_F = 17
	  	};

	  	enum EventType {
		    TARGET_REACHED = 0,
		    POS_COMP_REACHED,
		    VEL_REACHED,
		    VEL_STATE_ZERO,
		    VEL_STATE_POS,
		    VEL_STATE_NEG,
		    RAMP_STATE_ACCEL_ZERO,
		    RAMP_STATE_ACCEL_POS,
		    RAMP_STATE_ACCEL_NEG,
		    MAX_PHASE_TRAP,
		    FROZEN,
		    STOPL,
		    STOPR,
		    VSTOPL_ACTIVE,
		    VSTOPR_ACTIVE,
		    HOME_ERROR,
		    XLATCH_DONE,
		    FS_ACTIVE,
		    ENC_FAIL,
		    N_ACTIVE,
		    ENC_DONE,
		    SER_ENC_DATA_FAIL,
		    SER_DATA_DONE = 23,
		    SERIAL_ENC_FLAG,
		    COVER_DONE,
		    ENC_VEL_ZERO,
		    CL_MAX,
		    CL_FIT,
		    STOP_ON_STALL_EV,
		    MOTOR_EV,
		    RST_EV
		};

		TMC4361A(uint8_t cs, uint8_t tgt_pin);
		//call this function to init the controller
		void begin();
		void beginCL();
		void begin_closedLoop();
		//call this function to init the driver
		
		void init_TMC2660();
		void init_EncoderSPI();
		void init_EncoderSSI();
		void init_closedLoop();
		void resetController();
		void powerOffMOSFET();
		void powerOnMOSFET();
		void setTarget(long xtarget);
		void setTargetRelative(long xrelative);
		long getCurrentPos();
		void setCurrentPos(long pos);
		long getCurrentTarget();
		void setVMAX(uint32_t vmax, byte format);
		void setVMAX(uint32_t vmax);
		uint32_t getVMAX();
		void setAMAX(uint32_t amax);
		uint32_t getAMAX();
		void setClock(uint32_t clockFreq);
		uint32_t getClock();
		void setCurrentScale(uint8_t currentScale);
		uint8_t getCurrentScale();
		void setUsteps(int usteps);
		int getUsteps();
		float getEncoderAngle();
		float getEncoderTurn();
		uint32_t getEncoderPos();
		uint32_t getEncoderData();
		uint32_t getEncoderRaw();
		void alignEncoder();
		uint32_t getEncoderDev();
		bool checkFlag(TMC4361A::FlagType flag);
		bool isTargetReached();
		bool isEncoderFail();
		bool isSerialEncoderVar();
		void clearEvent();
		//
		void writeRegister(const byte address, const long data);
  		long readRegister(const byte address);

		void setRegisterBit(const byte address, const byte bit);
		void clearRegisterBit(const byte address, const byte bit);
		bool readRegisterBit(const byte address, const byte bit);
};

#endif//_TMC4361A_H