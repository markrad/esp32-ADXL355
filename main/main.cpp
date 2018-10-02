#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "ADXL355.h"

#define SDA_PIN 23
#define SCL_PIN 22

// static char tag[] = "i2cscanner";

using namespace std;

void calibrateSensor(ADXL355 &sensor, int fifoReadCount);

void task_i2cscanner(void *ignore) {

	ADXL355 sensor(0x1d, (gpio_num_t)SDA_PIN, (gpio_num_t)SCL_PIN);
	
	cout << "Analog Devices ID = "  << (int)sensor.GetAnalogDevicesID() << endl;             // Always 173
	cout << "Analog Devices MEMS ID = "  << (int)sensor.GetAnalogDevicesMEMSID() << endl;    // Always 29
	cout << "Device ID = "  << (int)sensor.GetDeviceId() << endl;                            // Always 237 (355 octal)
	cout << "Revision = "  << (int)sensor.GetRevision() << endl;

	sensor.Stop();

	sensor.SetRange(ADXL355::RANGE_VALUES::RANGE_8G);

	ADXL355::RANGE_VALUES rangeValue = sensor.GetRange();
	
	switch (rangeValue)
	{
	case ADXL355::RANGE_VALUES::RANGE_2G:
		cout << "Range 2g" << endl;
		break;
	case ADXL355::RANGE_VALUES::RANGE_4G:
		cout << "Range 4g" << endl;
		break;
	case ADXL355::RANGE_VALUES::RANGE_8G:
		cout << "Range 8g" << endl;
		break;
	default:
		cout << "Unknown range" << endl;
		break;
	}

	sensor.SetOdrLpf(ADXL355::ODR_LPF::ODR_31_25_AND_7_813);
	cout << "Low pass filter = " << sensor.GetOdrLpf() << endl;

	ADXL355::HPF_CORNER hpfCorner = sensor.GetHpfCorner();

	cout << "hpfCorner = " << hpfCorner << endl;

	ADXL355::ODR_LPF OdrLpf = sensor.GetOdrLpf();

	cout << "OdrLpf = " << OdrLpf << endl;
	
	calibrateSensor(sensor, 5);

	sensor.StartTempSensor();
	
	cout << "Temperature is " << sensor.GetTemperatureF() << "F" << endl;
				
	int32_t x = 0;
	int32_t y = 0;
	int32_t z = 0;
	
//	for (int i = 0; i < 10; i++)
	while (true)
	{
		if (0 == sensor.GetRawAxes(&x, &y, &z))
		{
			cout << "Axes received: x="
				<< ADXL355::ValueToGals(x) 
				<< ";y=" 
				<< ADXL355::ValueToGals(y) 
				<< ";z=" 
				<< ADXL355::ValueToGals(z) 
				<< endl;
		}
		
		vTaskDelay(500 / portTICK_RATE_MS);
	}
	cout << "FIFO entries = " << sensor.GetFifoCount() << endl;
	
	if (sensor.IsFifoFull())
		cout << "FIFO is full" << endl;
	
	if (sensor.IsFifoOverrun())
		cout << "FIFO is overrun" << endl;
	
	if (sensor.Stop())
	{
		cout << "Device is probably not running now" << endl;
		
		if (!sensor.IsRunning())
		{
			cout << "Device is not running" << endl;
		}
	}
	
	vTaskDelete(NULL);
}

void calibrateSensor(ADXL355 &sensor, int fifoReadCount)
{
	long fifoOut[32][3];
	int result;
	int readings = 0;
	long totalx = 0;
	long totaly = 0;
	long totalz = 0;

	memset(fifoOut, 0, sizeof(fifoOut));

	cout << endl << "Calibrating device with " << fifoReadCount << " fifo reads" << endl << endl;

	sensor.Stop();
	sensor.SetTrim(0, 0, 0);
	sensor.Start();
	this_thread::sleep_for(chrono::milliseconds(2000));

	for (int j = 0; j < fifoReadCount; j++)
	{
		cout << "Fifo read number " << j + 1 << endl;

		while (!sensor.IsFifoFull())
		{
			this_thread::sleep_for(chrono::milliseconds(10));
		}

		if (-1 != (result = sensor.ReadFifoEntries((long *)fifoOut)))
		{
			cout << "Retrieved " << result << " entries" << endl;
			readings += result;

			for (int i = 0; i < result; i++)
			{
				totalx += fifoOut[i][0];
				totaly += fifoOut[i][1];
				totalz += fifoOut[i][2];
			}
		}
		else
		{
			cerr << "Fifo read failed" << endl;
		}
	}

	long avgx = totalx / readings;
	long avgy = totaly / readings;
	long avgz = totalz / readings;

	cout
		<< endl
		<< "Total/Average X=" << totalx << "/" << avgx
		<< "; Y=" << totaly << "/" << avgy
		<< "; Z=" << totalz << "/" << avgz
		<< endl << endl;

	sensor.Stop();
	sensor.SetTrim(avgx, avgy, avgz);

	int32_t xTrim;
	int32_t yTrim;
	int32_t zTrim;

	result = sensor.GetTrim(&xTrim, &yTrim, &zTrim);

	if (result == 0)
	{
		cout << "xTrim=" << xTrim << ";yTrim=" << yTrim << ";zTrim=" << zTrim << endl;
	}

	sensor.Start();
	this_thread::sleep_for(chrono::milliseconds(2000));
}

extern "C" void app_main()
{
    nvs_flash_init();
    //initialise_wifi();

    if ( xTaskCreate(&task_i2cscanner, "i2cscanner_task", 1024 * 10, NULL, 5, NULL) != pdPASS ) {
        printf("Creation of i2c scanner task failed\r\n");
    }
}
