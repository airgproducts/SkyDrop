#include "raw.h"
#include "logger.h"
#include "../../drivers/storage/storage.h"

uint8_t raw_start(char * path)
{
	char filename[128];

	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t wday;
	uint8_t month;
	uint16_t year;

	datetime_from_epoch(time_get_utc(), &sec, &min, &hour, &day, &wday, &month, &year);

	sprintf_P(filename, PSTR("%sRAW"), path);
	DEBUG("RAW filename %s\n", filename);

	uint8_t res = f_open(&log_file, filename, FA_WRITE | FA_CREATE_ALWAYS);
	assert(res == FR_OK);

	//cannot create file
	if (res != FR_OK)
		return false;

	return LOGGER_ACTIVE;
}

uint32_t gps_time = 0;
uint16_t gps_time_ms=0;

void raw_step()
{
	if (!fc.gps_data.valid) {
		return;
	}

	if(fc.gps_data.utc_time==gps_time && fc.gps_data.utc_ms==gps_time_ms) {
		return;
	}

	gps_time = fc.gps_data.utc_time;
	gps_time_ms = fc.gps_data.utc_ms;

	#define LINELEN 62
	uint8_t line[LINELEN];
	uint16_t wl;
	uint8_t l = LINELEN;

	line[0] = 0xAA;

	uint32_t time = task_get_ms_tick();
	memcpy(line + 1, (void *)&fc.gps_data.utc_time, 4);
	memcpy(line + 5, (void *)&fc.gps_data.utc_ms, 2);
	memcpy(line + 7, (void *)&fc.gps_data.fix, 1);

	memcpy(line + 8, (void *)&fc.acc.raw, 6);
	memcpy(line + 14, (void *)&fc.gyro.raw, 6);
	memcpy(line + 20, (void *)&fc.mag.raw, 6);

	memcpy(line + 26, (void *)&fc.altitude1, 4);

	memcpy(line + 30, (void *)&fc.gps_data.latitude, 4);
	memcpy(line + 34, (void *)&fc.gps_data.longtitude, 4);

    memcpy(line + 38, (void *)&fc.vario.vario, 4);
	memcpy(line + 42, (void *)&fc.vario.pressure, 4);

	memcpy(line + 46, (void *)&fc.imu.quat[0], 4);
	memcpy(line + 50, (void *)&fc.imu.quat[1], 4);
	memcpy(line + 54, (void *)&fc.imu.quat[2], 4);
	memcpy(line + 58, (void *)&fc.imu.quat[3], 4);

	assert(f_write(&log_file, line, l, &wl) == FR_OK);
	assert(wl == l);
	assert(f_sync(&log_file) == FR_OK);
}

void raw_stop()
{
	assert(f_close(&log_file) == FR_OK);
}