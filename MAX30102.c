/***************************************************
  This is a library written for the Maxim MAX30105 Optical Smoke Detector
  Adapted for STM32 HAL from SparkFun Arduino library

  Original Authors: Peter Jansen and Nathan Seidle (SparkFun)
  STM32 Adaptation: 2024
  BSD license, all text above must be included in any redistribution.
 *****************************************************/

#include "max30102.h"

// Timeout for I2C operations (milliseconds)
#define I2C_TIMEOUT 100

// Initialize the MAX30105 sensor
bool MAX30105_begin(MAX30105_t *dev, I2C_HandleTypeDef *hi2c, uint8_t i2caddr) {
    dev->hi2c = hi2c;
    dev->i2caddr = i2caddr;

    // Check that a MAX30105 is connected
    if (MAX30105_readPartID(dev) != MAX_30105_EXPECTEDPARTID) {
        return false;
    }

    // Read revision ID
    MAX30105_readRevisionID(dev);

    return true;
}

// Read a single byte from a register
uint8_t MAX30105_readRegister8(MAX30105_t *dev, uint8_t reg) {
    uint8_t data = 0;

    if (HAL_I2C_Mem_Read(dev->hi2c, dev->i2caddr, reg, I2C_MEMADD_SIZE_8BIT,
                         &data, 1, I2C_TIMEOUT) != HAL_OK) {
        return 0;
    }

    return data;
}

// Write a single byte to a register
void MAX30105_writeRegister8(MAX30105_t *dev, uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(dev->hi2c, dev->i2caddr, reg, I2C_MEMADD_SIZE_8BIT,
                      &value, 1, I2C_TIMEOUT);
}

// Read, mask, and write a register
void MAX30105_bitMask(MAX30105_t *dev, uint8_t reg, uint8_t mask, uint8_t thing) {
    uint8_t originalContents = MAX30105_readRegister8(dev, reg);
    originalContents = originalContents & mask;
    MAX30105_writeRegister8(dev, reg, originalContents | thing);
}

// Interrupt Configuration
uint8_t MAX30105_getINT1(MAX30105_t *dev) {
    return MAX30105_readRegister8(dev, MAX30105_INTSTAT1);
}

uint8_t MAX30105_getINT2(MAX30105_t *dev) {
    return MAX30105_readRegister8(dev, MAX30105_INTSTAT2);
}

void MAX30105_enableAFULL(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_ENABLE);
}

void MAX30105_disableAFULL(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_DISABLE);
}

void MAX30105_enableDATARDY(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_ENABLE);
}

void MAX30105_disableDATARDY(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_DISABLE);
}

void MAX30105_enableALCOVF(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_ENABLE);
}

void MAX30105_disableALCOVF(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_DISABLE);
}

void MAX30105_enablePROXINT(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_ENABLE);
}

void MAX30105_disablePROXINT(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_DISABLE);
}

void MAX30105_enableDIETEMPRDY(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_ENABLE);
}

void MAX30105_disableDIETEMPRDY(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_DISABLE);
}

// Soft reset
void MAX30105_softReset(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);

    // Wait for reset to complete (timeout after 100ms)
    uint32_t startTime = HAL_GetTick();
    while ((HAL_GetTick() - startTime) < 100) {
        uint8_t response = MAX30105_readRegister8(dev, MAX30105_MODECONFIG);
        if ((response & MAX30105_RESET) == 0) break;
        HAL_Delay(1);
    }
}

// Power management
void MAX30105_shutDown(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN);
}

void MAX30105_wakeUp(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

// LED Mode Configuration
void MAX30105_setLEDMode(MAX30105_t *dev, uint8_t mode) {
    MAX30105_bitMask(dev, MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

// ADC Range Configuration
void MAX30105_setADCRange(MAX30105_t *dev, uint8_t adcRange) {
    MAX30105_bitMask(dev, MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

// Sample Rate Configuration
void MAX30105_setSampleRate(MAX30105_t *dev, uint8_t sampleRate) {
    MAX30105_bitMask(dev, MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

// Pulse Width Configuration
void MAX30105_setPulseWidth(MAX30105_t *dev, uint8_t pulseWidth) {
    MAX30105_bitMask(dev, MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

// LED Pulse Amplitude Configuration
void MAX30105_setPulseAmplitudeRed(MAX30105_t *dev, uint8_t amplitude) {
    MAX30105_writeRegister8(dev, MAX30105_LED1_PULSEAMP, amplitude);
}

void MAX30105_setPulseAmplitudeIR(MAX30105_t *dev, uint8_t amplitude) {
    MAX30105_writeRegister8(dev, MAX30105_LED2_PULSEAMP, amplitude);
}

void MAX30105_setPulseAmplitudeGreen(MAX30105_t *dev, uint8_t amplitude) {
    MAX30105_writeRegister8(dev, MAX30105_LED3_PULSEAMP, amplitude);
}

void MAX30105_setPulseAmplitudeProximity(MAX30105_t *dev, uint8_t amplitude) {
    MAX30105_writeRegister8(dev, MAX30105_LED_PROX_AMP, amplitude);
}

// Proximity Threshold
void MAX30105_setProximityThreshold(MAX30105_t *dev, uint8_t threshMSB) {
    MAX30105_writeRegister8(dev, MAX30105_PROXINTTHRESH, threshMSB);
}

// Multi-LED Slot Configuration
void MAX30105_enableSlot(MAX30105_t *dev, uint8_t slotNumber, uint8_t device) {
    switch (slotNumber) {
        case 1:
            MAX30105_bitMask(dev, MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
            break;
        case 2:
            MAX30105_bitMask(dev, MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
            break;
        case 3:
            MAX30105_bitMask(dev, MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
            break;
        case 4:
            MAX30105_bitMask(dev, MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
            break;
        default:
            break;
    }
}

void MAX30105_disableSlots(MAX30105_t *dev) {
    MAX30105_writeRegister8(dev, MAX30105_MULTILEDCONFIG1, 0);
    MAX30105_writeRegister8(dev, MAX30105_MULTILEDCONFIG2, 0);
}

// FIFO Configuration
void MAX30105_setFIFOAverage(MAX30105_t *dev, uint8_t numberOfSamples) {
    MAX30105_bitMask(dev, MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

void MAX30105_clearFIFO(MAX30105_t *dev) {
    MAX30105_writeRegister8(dev, MAX30105_FIFOWRITEPTR, 0);
    MAX30105_writeRegister8(dev, MAX30105_FIFOOVERFLOW, 0);
    MAX30105_writeRegister8(dev, MAX30105_FIFOREADPTR, 0);
}

void MAX30105_enableFIFORollover(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

void MAX30105_disableFIFORollover(MAX30105_t *dev) {
    MAX30105_bitMask(dev, MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE);
}

void MAX30105_setFIFOAlmostFull(MAX30105_t *dev, uint8_t numberOfSamples) {
    MAX30105_bitMask(dev, MAX30105_FIFOCONFIG, MAX30105_A_FULL_MASK, numberOfSamples);
}

// FIFO Pointers
uint8_t MAX30105_getWritePointer(MAX30105_t *dev) {
    return MAX30105_readRegister8(dev, MAX30105_FIFOWRITEPTR);
}

uint8_t MAX30105_getReadPointer(MAX30105_t *dev) {
    return MAX30105_readRegister8(dev, MAX30105_FIFOREADPTR);
}

// Temperature Reading
float MAX30105_readTemperature(MAX30105_t *dev) {
    // Trigger temperature conversion
    MAX30105_writeRegister8(dev, MAX30105_DIETEMPCONFIG, 0x01);

    // Wait for conversion to complete (timeout after 100ms)
    uint32_t startTime = HAL_GetTick();
    while ((HAL_GetTick() - startTime) < 100) {
        uint8_t response = MAX30105_readRegister8(dev, MAX30105_INTSTAT2);
        if ((response & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0) break;
        HAL_Delay(1);
    }

    // Read temperature registers
    int8_t tempInt = (int8_t)MAX30105_readRegister8(dev, MAX30105_DIETEMPINT);
    uint8_t tempFrac = MAX30105_readRegister8(dev, MAX30105_DIETEMPFRAC);

    // Calculate temperature
    return (float)tempInt + ((float)tempFrac * 0.0625f);
}

float MAX30105_readTemperatureF(MAX30105_t *dev) {
    float temp = MAX30105_readTemperature(dev);
    if (temp != -999.0f) {
        temp = temp * 1.8f + 32.0f;
    }
    return temp;
}

// Proximity Threshold
void MAX30105_setPROXINTTHRESH(MAX30105_t *dev, uint8_t val) {
    MAX30105_writeRegister8(dev, MAX30105_PROXINTTHRESH, val);
}

// Part ID
uint8_t MAX30105_readPartID(MAX30105_t *dev) {
    return MAX30105_readRegister8(dev, MAX30105_PARTID);
}

void MAX30105_readRevisionID(MAX30105_t *dev) {
    dev->revisionID = MAX30105_readRegister8(dev, MAX30105_REVISIONID);
}

uint8_t MAX30105_getRevisionID(MAX30105_t *dev) {
    return dev->revisionID;
}

// Setup function with default parameters
void MAX30105_setup(MAX30105_t *dev, uint8_t powerLevel, uint8_t sampleAverage,
                    uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange) {

    MAX30105_softReset(dev);

    // FIFO Configuration
    if (sampleAverage == 1) MAX30105_setFIFOAverage(dev, MAX30105_SAMPLEAVG_1);
    else if (sampleAverage == 2) MAX30105_setFIFOAverage(dev, MAX30105_SAMPLEAVG_2);
    else if (sampleAverage == 4) MAX30105_setFIFOAverage(dev, MAX30105_SAMPLEAVG_4);
    else if (sampleAverage == 8) MAX30105_setFIFOAverage(dev, MAX30105_SAMPLEAVG_8);
    else if (sampleAverage == 16) MAX30105_setFIFOAverage(dev, MAX30105_SAMPLEAVG_16);
    else if (sampleAverage == 32) MAX30105_setFIFOAverage(dev, MAX30105_SAMPLEAVG_32);
    else MAX30105_setFIFOAverage(dev, MAX30105_SAMPLEAVG_4);

    MAX30105_enableFIFORollover(dev);

    // Mode Configuration
    if (ledMode == 3) MAX30105_setLEDMode(dev, MAX30105_MODE_MULTILED);
    else if (ledMode == 2) MAX30105_setLEDMode(dev, MAX30105_MODE_REDIRONLY);
    else MAX30105_setLEDMode(dev, MAX30105_MODE_REDONLY);
    dev->activeLEDs = ledMode;

    // Particle Sensing Configuration
    if (adcRange < 4096) MAX30105_setADCRange(dev, MAX30105_ADCRANGE_2048);
    else if (adcRange < 8192) MAX30105_setADCRange(dev, MAX30105_ADCRANGE_4096);
    else if (adcRange < 16384) MAX30105_setADCRange(dev, MAX30105_ADCRANGE_8192);
    else if (adcRange == 16384) MAX30105_setADCRange(dev, MAX30105_ADCRANGE_16384);
    else MAX30105_setADCRange(dev, MAX30105_ADCRANGE_2048);

    if (sampleRate < 100) MAX30105_setSampleRate(dev, MAX30105_SAMPLERATE_50);
    else if (sampleRate < 200) MAX30105_setSampleRate(dev, MAX30105_SAMPLERATE_100);
    else if (sampleRate < 400) MAX30105_setSampleRate(dev, MAX30105_SAMPLERATE_200);
    else if (sampleRate < 800) MAX30105_setSampleRate(dev, MAX30105_SAMPLERATE_400);
    else if (sampleRate < 1000) MAX30105_setSampleRate(dev, MAX30105_SAMPLERATE_800);
    else if (sampleRate < 1600) MAX30105_setSampleRate(dev, MAX30105_SAMPLERATE_1000);
    else if (sampleRate < 3200) MAX30105_setSampleRate(dev, MAX30105_SAMPLERATE_1600);
    else if (sampleRate == 3200) MAX30105_setSampleRate(dev, MAX30105_SAMPLERATE_3200);
    else MAX30105_setSampleRate(dev, MAX30105_SAMPLERATE_50);

    if (pulseWidth < 118) MAX30105_setPulseWidth(dev, MAX30105_PULSEWIDTH_69);
    else if (pulseWidth < 215) MAX30105_setPulseWidth(dev, MAX30105_PULSEWIDTH_118);
    else if (pulseWidth < 411) MAX30105_setPulseWidth(dev, MAX30105_PULSEWIDTH_215);
    else if (pulseWidth == 411) MAX30105_setPulseWidth(dev, MAX30105_PULSEWIDTH_411);
    else MAX30105_setPulseWidth(dev, MAX30105_PULSEWIDTH_69);

    // LED Pulse Amplitude Configuration
    MAX30105_setPulseAmplitudeRed(dev, powerLevel);
    MAX30105_setPulseAmplitudeIR(dev, powerLevel);
    MAX30105_setPulseAmplitudeGreen(dev, powerLevel);
    MAX30105_setPulseAmplitudeProximity(dev, powerLevel);

    // Multi-LED Mode Configuration
    MAX30105_enableSlot(dev, 1, SLOT_RED_LED);
    if (ledMode > 1) MAX30105_enableSlot(dev, 2, SLOT_IR_LED);
    if (ledMode > 2) MAX30105_enableSlot(dev, 3, SLOT_GREEN_LED);

    MAX30105_clearFIFO(dev);
}

// Data Collection Functions
uint8_t MAX30105_available(MAX30105_t *dev) {
    int8_t numberOfSamples = dev->sense.head - dev->sense.tail;
    if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;
    return (uint8_t)numberOfSamples;
}

uint32_t MAX30105_getRed(MAX30105_t *dev) {
    if (MAX30105_safeCheck(dev, 250)) {
        return dev->sense.red[dev->sense.head];
    }
    return 0;
}

uint32_t MAX30105_getIR(MAX30105_t *dev) {
    if (MAX30105_safeCheck(dev, 250)) {
        return dev->sense.IR[dev->sense.head];
    }
    return 0;
}

uint32_t MAX30105_getGreen(MAX30105_t *dev) {
    if (MAX30105_safeCheck(dev, 250)) {
        return dev->sense.green[dev->sense.head];
    }
    return 0;
}

uint32_t MAX30105_getFIFORed(MAX30105_t *dev) {
    return dev->sense.red[dev->sense.tail];
}

uint32_t MAX30105_getFIFOIR(MAX30105_t *dev) {
    return dev->sense.IR[dev->sense.tail];
}

uint32_t MAX30105_getFIFOGreen(MAX30105_t *dev) {
    return dev->sense.green[dev->sense.tail];
}

void MAX30105_nextSample(MAX30105_t *dev) {
    if (MAX30105_available(dev)) {
        dev->sense.tail++;
        dev->sense.tail %= STORAGE_SIZE;
    }
}

// Check for new data in FIFO
uint16_t MAX30105_check(MAX30105_t *dev) {
    uint8_t readPointer = MAX30105_getReadPointer(dev);
    uint8_t writePointer = MAX30105_getWritePointer(dev);

    int numberOfSamples = 0;

    if (readPointer != writePointer) {
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0) numberOfSamples += 32;

        int bytesLeftToRead = numberOfSamples * dev->activeLEDs * 3;

        while (bytesLeftToRead > 0) {
            int toGet = bytesLeftToRead;
            if (toGet > I2C_BUFFER_LENGTH) {
                toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (dev->activeLEDs * 3));
            }

            bytesLeftToRead -= toGet;

            uint8_t buffer[I2C_BUFFER_LENGTH];

            // Read FIFO data
            if (HAL_I2C_Mem_Read(dev->hi2c, dev->i2caddr, MAX30105_FIFODATA,
                                I2C_MEMADD_SIZE_8BIT, buffer, toGet, I2C_TIMEOUT) == HAL_OK) {

                int bufferIndex = 0;
                while (toGet > 0) {
                    dev->sense.head++;
                    dev->sense.head %= STORAGE_SIZE;

                    uint8_t temp[4];
                    uint32_t tempLong;

                    // Read RED
                    temp[3] = 0;
                    temp[2] = buffer[bufferIndex++];
                    temp[1] = buffer[bufferIndex++];
                    temp[0] = buffer[bufferIndex++];
                    memcpy(&tempLong, temp, sizeof(tempLong));
                    tempLong &= 0x3FFFF;
                    dev->sense.red[dev->sense.head] = tempLong;

                    if (dev->activeLEDs > 1) {
                        // Read IR
                        temp[3] = 0;
                        temp[2] = buffer[bufferIndex++];
                        temp[1] = buffer[bufferIndex++];
                        temp[0] = buffer[bufferIndex++];
                        memcpy(&tempLong, temp, sizeof(tempLong));
                        tempLong &= 0x3FFFF;
                        dev->sense.IR[dev->sense.head] = tempLong;
                    }

                    if (dev->activeLEDs > 2) {
                        // Read GREEN
                        temp[3] = 0;
                        temp[2] = buffer[bufferIndex++];
                        temp[1] = buffer[bufferIndex++];
                        temp[0] = buffer[bufferIndex++];
                        memcpy(&tempLong, temp, sizeof(tempLong));
                        tempLong &= 0x3FFFF;
                        dev->sense.green[dev->sense.head] = tempLong;
                    }

                    toGet -= dev->activeLEDs * 3;
                }
            }
        }
    }

    return (uint16_t)numberOfSamples;
}

// Safe check with timeout
bool MAX30105_safeCheck(MAX30105_t *dev, uint8_t maxTimeToCheck) {
    uint32_t markTime = HAL_GetTick();

    while (1) {
        if ((HAL_GetTick() - markTime) > maxTimeToCheck) return false;

        if (MAX30105_check(dev) > 0) return true;

        HAL_Delay(1);
    }
}
