#ifndef _TOF_H_
#define _TOF_H_


struct tofSensor{
    int address;
    int i2c_file;
};

int tofInit(int iChan, int iAddr, int bLongRange);
// static unsigned short readReg16(unsigned char ucAddr);
// static unsigned char readReg(unsigned char ucAddr);
// static void readMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount);
// static void writeMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount);
// static void writeReg16(unsigned char ucAddr, unsigned short usValue);
// static void writeReg(unsigned char ucAddr, unsigned char ucValue);
// static void writeRegList(unsigned char *ucList);
// static int getSpadInfo(unsigned char *pCount, unsigned char *pTypeIsAperture);
// static uint16_t decodeTimeout(uint16_t reg_val);
// static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
// static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
// static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
// static uint16_t encodeTimeout(uint16_t timeout_mclks)
// static void getSequenceStepTimeouts(uint8_t enables, SequenceStepTimeouts * timeouts)
// static int setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
// static int setMeasurementTimingBudget(uint32_t budget_us)
// static uint32_t getMeasurementTimingBudget(void)
// static int performSingleRefCalibration(uint8_t vhv_init_byte)
// static int initSensor(int bLongRangeMode);
// uint16_t readRangeContinuousMillimeters(void)
int tofReadDistance(void);
int tofGetModel(int *model, int *revision);


#endif






