#include "ADXL355.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
#include <cstring>

using namespace std;

ADXL355::ADXL355(uint8_t deviceAddress, gpio_num_t sdaPin, gpio_num_t sclPin) 
	: _deviceAddress(deviceAddress), _sdaPin(sdaPin), _sclPin(sclPin)
{
	esp_err_t espRc = ESP_OK;
	
	_conf.mode = I2C_MODE_MASTER;
	_conf.sda_io_num = _sdaPin;
	_conf.scl_io_num = _sclPin;
	_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	_conf.master.clk_speed = 400000;
	
	CheckEspRc("Failed to configure i2c: ", (espRc = i2c_param_config(I2C_NUM_1, &_conf)));
	
	if (espRc == 0)
		CheckEspRc("Failed to install driver: ", (espRc = i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0)));
}

uint8_t ADXL355::GetAnalogDevicesID()
{
	uint8_t data = 0;
	esp_err_t espRc = ReadByte(DEVID_AD, &data);

	return (espRc == 0)? data : 0;
}

uint8_t ADXL355::GetAnalogDevicesMEMSID()
{
	uint8_t data = 0;
	esp_err_t espRc = ReadByte(DEVID_MST, &data);

	return (espRc == 0)? data : 0;
}
	
uint8_t ADXL355::GetDeviceId()
{
	uint8_t data = 0;
	esp_err_t espRc = ReadByte(PARTID, &data);

	return (espRc == 0)? data : 0;
}

uint8_t ADXL355::GetRevision()
{
	uint8_t data = 0;
	esp_err_t espRc = ReadByte(REVID, &data);

	return (espRc == 0)? data : 0;
}

int ADXL355::GetFifoCount()
{
    uint8_t data = 0;
	esp_err_t espRc = ReadByte(FIFO_ENTRIES, &data);

    return (espRc == 0)? data : 0;
}

ADXL355::STATUS_VALUES ADXL355::GetStatus()
{
    uint8_t data = 0;
	esp_err_t espRc = ReadByte(STATUS, &data);

    return (espRc == 0)? (STATUS_VALUES)data : (STATUS_VALUES)0;
}

bool ADXL355::IsFifoFull()
{
    STATUS_VALUES work = GetStatus();

    return (work & STATUS_VALUES::FIFO_FULL)? true : false;
}

bool ADXL355::IsFifoOverrun()
{
    STATUS_VALUES work = GetStatus();

    return (work & STATUS_VALUES::FIFO_OVERRUN)? true : false;
}

bool ADXL355::IsDataReady()
{
    STATUS_VALUES work = GetStatus();

    return (work & STATUS_VALUES::DATA_READY)? true : false;
}

ADXL355::RANGE_VALUES ADXL355::GetRange()
{
    uint8_t data = 0;
	esp_err_t espRc = ReadByte(I2CSPEED_INTPOLARITY_RANGE, &data);

    return (espRc == 0)? (RANGE_VALUES)(data & RANGE_VALUES::RANGE_MASK) : (RANGE_VALUES)0;
}

int ADXL355::SetRange(RANGE_VALUES value)
{
	if (IsRunning())
		return -1;
		
	uint8_t data = 0;
	esp_err_t espRc = ReadByte(I2CSPEED_INTPOLARITY_RANGE, &data);

	if (espRc == ESP_OK)
	{
		data &= ~(RANGE_VALUES::RANGE_MASK);
		data |= (int)value;
		
		espRc = WriteByte(I2CSPEED_INTPOLARITY_RANGE, data);
	}
	
	return espRc;
}

ADXL355::HPF_CORNER ADXL355::GetHpfCorner()
{
	uint8_t data = 0;
	esp_err_t espRc = ReadByte(FILTER, &data);

    return (espRc == 0)? (HPF_CORNER)((data & HPF_CORNER::HPF_CORNER_MASK) >> 4) : (HPF_CORNER)0;
}

int ADXL355::SetHpfCorner(HPF_CORNER value)
{
	if (IsRunning())
		return -1;
		
	uint8_t data = 0;
	esp_err_t espRc = ReadByte(FILTER, &data);
	
	if (espRc == ESP_OK)
	{
		data = (data & ~(HPF_CORNER::HPF_CORNER_MASK << 4)) | ((int)value) << 4;
		espRc = WriteByte(FILTER, data);
	}
	
	return espRc;
}

ADXL355::ODR_LPF ADXL355::GetOdrLpf()
{
	uint8_t data = 0;
	esp_err_t espRc = ReadByte(FILTER, &data);

    return (espRc == 0)? (ODR_LPF)(data & ODR_LPF::ODR_LPF_MASK) : (ODR_LPF)0;
}

int ADXL355::SetOdrLpf(ODR_LPF value)
{
	if (IsRunning())
		return (ODR_LPF)-1;
		
	uint8_t data = 0;
	esp_err_t espRc = ReadByte(FILTER, &data);

	if (espRc == ESP_OK)
	{
		data = (data & ~(ODR_LPF::ODR_LPF_MASK)) | ((int)value);
		espRc = WriteByte(FILTER, data);
	}
	
	return espRc;
}

int ADXL355::GetTrim(int32_t *x, int32_t *y, int32_t *z)
{
    uint8_t output[6];

    memset(output, 0xff, sizeof(output));

    esp_err_t espRc = ReadBytes(OFFSET_X_H, output, sizeof(output));

    if (espRc == ESP_OK)
    {
        *x = TwosCompliment((output[0] << 8 | output[1]) << 4);
        *y = TwosCompliment((output[2] << 8 | output[3]) << 4);
        *z = TwosCompliment((output[4] << 8 | output[5]) << 4);
    }

    return espRc;
}

int ADXL355::SetTrim(int32_t x, int32_t y, int32_t z)
{
	if (IsRunning())
		return -1;
		
    int16_t workx = (x >> 4);
    int16_t worky = (y >> 4);
    int16_t workz = (z >> 4);
    uint8_t hix = (workx & 0xff00) >> 8;
    uint8_t lox = workx & 0x00ff;
    uint8_t hiy = (worky & 0xff00) >> 8;
    uint8_t loy = worky & 0x00ff;
    uint8_t hiz = (workz & 0xff00) >> 8;
    uint8_t loz = workz & 0x00ff;

	esp_err_t espRc;
	
    espRc = WriteByte(OFFSET_X_H, hix);

	if (espRc == ESP_OK)
		WriteByte(OFFSET_X_L, lox);
    
    if (espRc == ESP_OK)
		WriteByte(OFFSET_Y_H, hiy);

    if (espRc == ESP_OK)
		WriteByte(OFFSET_Y_L, loy);

    if (espRc == ESP_OK)
		WriteByte(OFFSET_Z_H, hiz);

    if (espRc == ESP_OK)
		WriteByte(OFFSET_Z_L, loz);
	
	return espRc;
}

bool ADXL355::IsRunning()
{
	uint8_t data = 0;
	bool result = false;
	
	esp_err_t espRc = ReadByte(POWER_CTL, &data);
	
	if (espRc != 0)
		result = false;
	else
		result = (data & POWER_CTL_VALUES::POWER_CTL_OFF)
			? false
			: true;

    return result;
}

int ADXL355::Start()
{
	uint8_t data = 0;
	esp_err_t espRc;
	
	espRc = ReadByte(POWER_CTL, &data);

	if (espRc == 0 && (data & POWER_CTL_VALUES::POWER_CTL_OFF))
	{
		data = data & (int)POWER_CTL_VALUES::POWER_CTL_ON;
		espRc = WriteByte(POWER_CTL, data);
	}
	
	return espRc;
}

int ADXL355::Stop()
{
	uint8_t data = 0;
	esp_err_t espRc;
	
	espRc = ReadByte(POWER_CTL, &data);

	if (espRc == 0 && !(data & POWER_CTL_VALUES::POWER_CTL_OFF))
	{
		data = data | (int)POWER_CTL_VALUES::POWER_CTL_OFF;
		
		espRc = WriteByte(POWER_CTL, data);
	}
	
	return espRc;
}

bool ADXL355::IsTempSensorOn()
{
    bool result = false;
	uint8_t data = 0;
	
	esp_err_t espRc = ReadByte(POWER_CTL, &data);
	
	if (espRc != ESP_OK)
		return false;

	result = ((data & POWER_CTL_VALUES::POWER_CTL_OFF) || (data & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF))
		? false
		: true;

    return result;
}

int ADXL355::StartTempSensor()
{
	uint8_t data = 0;
	
	esp_err_t espRc = ReadByte(POWER_CTL, &data);

	if (espRc == 0 && data & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF)
	{
		data = data & (int)POWER_CTL_VALUES::POWER_CTL_TEMP_ON;
		espRc = WriteByte(POWER_CTL, data);
	}
	
	return espRc;
}

int ADXL355::StopTempSensor()
{
	uint8_t data = 0;
	
	esp_err_t espRc = ReadByte(POWER_CTL, &data);

	if (!(data & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF))
	{
		data = data | (int)POWER_CTL_VALUES::POWER_CTL_TEMP_OFF;
		espRc = WriteByte(POWER_CTL, data);
	}
	
	return espRc;
}

double ADXL355::GetTemperatureC()
{
	uint8_t data[2];

    ReadBytes(TEMP2, data, sizeof(data));
    int itemp = (data[0] << 8) | data[1];

    double dtemp = ((double)(1852 - itemp)) / 9.05 + 19.21;

	return dtemp;
}

double ADXL355::GetTemperatureF()
{
    double result = GetTemperatureC();

    return result * 9 / 5 + 32;
}

esp_err_t ADXL355::GetRawAxes(int32_t *x, int32_t *y, int32_t *z)
{
	uint8_t data[9];
	esp_err_t espRc;

	espRc = ReadBytes(XDATA3, data, sizeof(data));
	
    uint32_t workx = 0;
    uint32_t worky = 0;
    uint32_t workz = 0;

    if (espRc == ESP_OK)
    {
        workx = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        worky = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
        workz = (data[6] << 12) | (data[7] << 4) | (data[8] >> 4);
		*x = TwosCompliment(workx);
		*y = TwosCompliment(worky);
		*z = TwosCompliment(workz);
    }
	
	return espRc;
}

int ADXL355::ReadFifoEntries(long *output)
{
    int fifoCount = GetFifoCount();
    uint8_t data[9];
    memset(data, 0, sizeof(data));

    unsigned long work[3];

    for (int i = 0; i < fifoCount / 3; i++)
    {
        int result = ReadBytes(FIFO_DATA, data, sizeof(data));
		
        if (result == 0)
        {
            for (int j = 0; j < 9; j+= 3)
            {
                work[j / 3] = (data[0 + j] << 12) | (data[1 + j] << 4) | (data[2 + j] >> 4);
                output[i * 3 + j / 3] = TwosCompliment(work[j / 3]); 
            }
        }
        else
        {
            return -1;
        }
    }

    return fifoCount / 3;
}

esp_err_t ADXL355::ReadByte(uint8_t reg, uint8_t *data)
{
	esp_err_t espRc = ESP_OK;

	{
		I2CCmdHandle cmd;

		CheckEspRc("Failed to start 1: ", (espRc = i2c_master_start(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to write byte 1: ", (espRc = i2c_master_write_byte(cmd.GetCmd(), (_deviceAddress << 1) | I2C_MASTER_WRITE, 1)));
	
		if (espRc == 0)
			CheckEspRc("Failed to write byte 2: ", (espRc = i2c_master_write_byte(cmd.GetCmd(), reg, 0)));

		if (espRc == 0)
			CheckEspRc("Failed to stop 1: ", (espRc = i2c_master_stop(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to begin 1: ", (espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd.GetCmd(), 100 / portTICK_PERIOD_MS)));
	}

    vTaskDelay(30 / portTICK_RATE_MS);

	if (espRc == 0)
	{
		I2CCmdHandle cmd;
		
		CheckEspRc("Failed to start 2: ", (espRc = i2c_master_start(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to write read request: ", (espRc = i2c_master_write_byte(cmd.GetCmd(), (_deviceAddress << 1) | I2C_MASTER_READ, 1)));

		if (espRc == 0)
			CheckEspRc("Failed to read: ", (espRc = i2c_master_read_byte(cmd.GetCmd(), data, I2C_MASTER_NACK)));

		if (espRc == 0)
			CheckEspRc("Failed to stop 2: ", (espRc = i2c_master_stop(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to begin 2: ", (espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd.GetCmd(), 100 / portTICK_PERIOD_MS)));
	}
	
	return espRc;
}

esp_err_t ADXL355::WriteByte(uint8_t reg, uint8_t data)
{
	esp_err_t espRc = ESP_OK;

	{
		I2CCmdHandle cmd;

		CheckEspRc("Failed to start 1: ", (espRc = i2c_master_start(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to write byte 1: ", (espRc = i2c_master_write_byte(cmd.GetCmd(), (_deviceAddress << 1) | I2C_MASTER_WRITE, 1)));
	
		if (espRc == 0)
			CheckEspRc("Failed to write byte 2: ", (espRc = i2c_master_write_byte(cmd.GetCmd(), reg, 0)));
	
		if (espRc == 0)
			CheckEspRc("Failed to write byte 3: ", (espRc = i2c_master_write_byte(cmd.GetCmd(), data, 0)));

		if (espRc == 0)
			CheckEspRc("Failed to stop 1: ", (espRc = i2c_master_stop(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to begin 1: ", (espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd.GetCmd(), 100 / portTICK_PERIOD_MS)));
	}
	
	return espRc;
}

esp_err_t ADXL355::ReadBytes(uint8_t reg, uint8_t *data, size_t length)
{
	esp_err_t espRc = ESP_OK;

	{
		I2CCmdHandle cmd;

		CheckEspRc("Failed to start 1: ", (espRc = i2c_master_start(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to write byte 1: ", (espRc = i2c_master_write_byte(cmd.GetCmd(), (_deviceAddress << 1) | I2C_MASTER_WRITE, 1)));
	
		if (espRc == 0)
			CheckEspRc("Failed to write byte 2: ", (espRc = i2c_master_write_byte(cmd.GetCmd(), reg, 0)));

		if (espRc == 0)
			CheckEspRc("Failed to stop 1: ", (espRc = i2c_master_stop(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to begin 1: ", (espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd.GetCmd(), 100 / portTICK_PERIOD_MS)));
	}

    vTaskDelay(30 / portTICK_RATE_MS);

	if (espRc == ESP_OK)
	{
		I2CCmdHandle cmd;
		
		CheckEspRc("Failed to start 2: ", (espRc = i2c_master_start(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to write read request: ", (espRc = i2c_master_write_byte(cmd.GetCmd(), (_deviceAddress << 1) | I2C_MASTER_READ, 1)));

		if (espRc == 0)
			CheckEspRc("Failed to multibyte read: ", (espRc = i2c_master_read(cmd.GetCmd(), data, length - 1, I2C_MASTER_ACK)));
		
		if (espRc == 0)
			CheckEspRc("Failed to read: ", (espRc = i2c_master_read_byte(cmd.GetCmd(), data + length - 1, I2C_MASTER_NACK)));

		if (espRc == 0)
			CheckEspRc("Failed to stop 2: ", (espRc = i2c_master_stop(cmd.GetCmd())));

		if (espRc == 0)
			CheckEspRc("Failed to begin 2: ", (espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd.GetCmd(), 100 / portTICK_PERIOD_MS)));
	}
	
	return espRc;
}

esp_err_t ADXL355::WriteBytes(uint8_t reg, uint8_t *data, size_t length)
{
	esp_err_t espRc = ESP_OK;

	return espRc;
}

esp_err_t ADXL355::CheckEspRc(const string &message, esp_err_t rc)
{
	if (rc != 0)
	{
		char buf[100];
		cout << message << "0x" << hex << rc << dec << " = " << esp_err_to_name_r(rc, buf, sizeof(buf)) << endl;
	}
	
	return rc;
}

int32_t ADXL355::TwosCompliment(uint32_t value)
{
	return ((int32_t)value << 12) >> 12;
}

double ADXL355::ValueToGals(int32_t rawValue, int decimals)
{
    double slider = (decimals > 1)? pow(10.0, (double)decimals) : 1.0;

    double result = (double)rawValue / 260000.0 * 980.665;

    result = round(result * slider) / slider;

    return result;
}

