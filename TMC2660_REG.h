#ifndef _TMC2660_REG_H
#define _TMC2660_REG_H
	static uint32_t DRVCTRL_REG; //uSteps, Step interpolation // 2 mode for STEP/DIR & in SPI
	static uint32_t CHOPCONF_REG; //MOSFET control for energy saving/decrase heat generation, 
	static uint32_t SMARTEN_REG; //Current optimisation, 
	static uint32_t SGSCONF_REG; //Current setting, stallguard options for stall reading
	static uint32_t DRVCONF_REG; //Step/Dir interface mode, readout msg, slope control

#endif//TMC2660_REG_H