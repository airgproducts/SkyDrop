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

void raw_step()
{
	uint8_t line[39];
	uint16_t wl;
	uint8_t l = 39;

	line[0] = 0xAA;

	uint32_t time = task_get_ms_tick();
	memcpy(line + 1, (void *)&fc.gps_data.utc_time, 4);

	memcpy(line + 5, (void *)&fc.acc.raw, 6);
	memcpy(line + 11, (void *)&fc.gyro.raw, 6);
	memcpy(line + 17, (void *)&fc.mag.raw, 6);

	memcpy(line + 23, (void *)&fc.altitude1, 4);

	if (fc.gps_data.valid)
	{
		memcpy(line + 27, (void *)&fc.gps_data.latitude, 4);
		memcpy(line + 31, (void *)&fc.gps_data.longtitude, 4);
	}
	else
	{
		memset(line + 27, 0xFF, 8);
	}

    memcpy(line + 35, (void *)&fc.vario.vario, 4);

	assert(f_write(&log_file, line, l, &wl) == FR_OK);
	assert(wl == l);
	assert(f_sync(&log_file) == FR_OK);
}

void raw_stop()
{
	assert(f_close(&log_file) == FR_OK);
}