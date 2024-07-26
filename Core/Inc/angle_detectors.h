#ifndef ANGLE_DETECTORS_H
#define ANGLE_DETECTORS_H

#include "allocator.h"
#include <utility>
#include <vector>

#include "stm32f1xx.h"

class Potentiometer
{
    public:
    enum MinMax {Min, Max};

    friend class PotentiometersHandler;

private:
    Potentiometer(uint8_t id);
    bool readCalibratedFromFlash();
    bool writeCalibratedToFlash() const;

    uint16_t * getFlashAddress() const;

    const uint8_t id_;
    
    std::pair<uint16_t, uint16_t> min_max_;
};

class PotentiometersHandler
{
public:
    PotentiometersHandler(uint8_t potentiometers_num);
    void startConversion();
    void stopConversion();
    uint16_t getConversionResult(uint8_t potenciometer_id);
    
    void updateMinMax(uint8_t id, Potentiometer::MinMax min_max_flag);
    bool saveCalibrationToFlash();

    bool flashIsValid() const;

    static const uint32_t FLASH_CALIB_BASE_ADDR = FLASH_BASE + 26UL * 1024UL;

private:
    static const uint8_t BUFFER_LENGTH = 100;
    const uint16_t *conv_buffer_;

    std::vector<Potentiometer, Heap4Alocator<Potentiometer>> potentiometers_;

    bool flash_is_valid_;
};

#endif //ANGLE_DETECTORS_H