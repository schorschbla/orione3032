#include "Mlx90614TemperatureSensor.h"

const uint8_t Mlx90614RegisterTobj1 = 0x07;
const uint8_t Mlx90614RegisterTobj2 = 0x08;

const double Mlx90614FactorDegree = 0.02;
const double Mlx90614OffsetKelvin = -273.15;

Mlx90614TemperatureSensor::Mlx90614TemperatureSensor(TwoWire &wire, uint8_t address) :
    wire(wire), address(address)
{
}

int Mlx90614TemperatureSensor::readValue(Register reg, double &value)
{
    uint16_t sample;
    uint8_t pec;
    
    wire.beginTransmission(address);
    wire.write(reg == Mlx90614TemperatureSensor::Register::Obj1 ? Mlx90614RegisterTobj1 : Mlx90614RegisterTobj2);
    wire.endTransmission(false);
    if (wire.requestFrom(address, 3) != 3)
    {
         wire.endTransmission();
         return -1;
    }
    sample = wire.read() | (wire.read() << 8);
    pec = wire.read();
    wire.endTransmission();

    value = sample * Mlx90614FactorDegree + Mlx90614OffsetKelvin;

    return 0;
}
