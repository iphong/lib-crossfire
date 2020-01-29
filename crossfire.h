#include <cinttypes>
#include <functional>
#include <math.h>

#ifndef LIB_CROSSFIRE_H
#define LIB_CROSSFIRE_H

// Device address
#define BROADCAST_ADDRESS              0x00
#define RADIO_ADDRESS                  0xEA
#define MODULE_ADDRESS                 0xEE

// Frame id
#define GPS_ID                         0x02
#define CF_VARIO_ID                    0x07
#define BATTERY_ID                     0x08
#define LINK_ID                        0x14
#define CHANNELS_ID                    0x16
#define ATTITUDE_ID                    0x1E
#define FLIGHT_MODE_ID                 0x21
#define PING_DEVICES_ID                0x28
#define DEVICE_INFO_ID                 0x29
#define REQUEST_SETTINGS_ID            0x2A

#define TELEMETRY_RX_PACKET_SIZE       128

#if !defined(DIM)
#define DIM(__arr) (sizeof((__arr)) / sizeof((__arr)[0]))
#endif

enum crossfire_sensor_num_t {
	RX_RSSI1,
	RX_RSSI2,
	RX_QUALITY,
	RX_SNR,
	RX_ANTENNA,
	RF_MODE,
	TX_POWER,
	TX_RSSI,
	TX_QUALITY,
	TX_SNR,
	BATT_VOLTAGE,
	BATT_CURRENT,
	BATT_CAPACITY,
	GPS_LATITUDE,
	GPS_LONGITUDE,
	GPS_GROUND_SPEED,
	GPS_HEADING,
	GPS_ALTITUDE,
	GPS_SATELLITES,
	ATTITUDE_PITCH,
	ATTITUDE_ROLL,
	ATTITUDE_YAW,
	FLIGHT_MODE,
	VERTICAL_SPEED,
	UNKNOWN,
};

enum telemetry_unit_num_t {
	UNIT_RAW,
	UNIT_VOLTS,
	UNIT_AMPS,
	UNIT_MILLIAMPS,
	UNIT_KTS,
	UNIT_METERS_PER_SECOND,
	UNIT_FEET_PER_SECOND,
	UNIT_KMH,
	UNIT_SPEED = UNIT_KMH,
	UNIT_MPH,
	UNIT_METERS,
	UNIT_DIST = UNIT_METERS,
	UNIT_FEET,
	UNIT_CELSIUS,
	UNIT_TEMPERATURE = UNIT_CELSIUS,
	UNIT_FAHRENHEIT,
	UNIT_PERCENT,
	UNIT_MAH,
	UNIT_WATTS,
	UNIT_MILLIWATTS,
	UNIT_DB,
	UNIT_RPMS,
	UNIT_G,
	UNIT_DEGREE,
	UNIT_RADIANS,
	UNIT_MILLILITERS,
	UNIT_FLOZ,
	UNIT_MILLILITERS_PER_MINUTE,
	UNIT_MAX = UNIT_MILLILITERS_PER_MINUTE,
	UNIT_SPARE1,
	UNIT_SPARE2,
	UNIT_SPARE3,
	UNIT_SPARE4,
	UNIT_SPARE5,
	UNIT_SPARE6,
	UNIT_SPARE7,
	UNIT_SPARE8,
	UNIT_SPARE9,
	UNIT_SPARE10,
	UNIT_HOURS,
	UNIT_MINUTES,
	UNIT_SECONDS,
	UNIT_FIRST_VIRTUAL,
	UNIT_CELLS = UNIT_FIRST_VIRTUAL,
	UNIT_DATETIME,
	UNIT_GPS,
	UNIT_BITFIELD,
	UNIT_TEXT,
	UNIT_GPS_LONGITUDE,
	UNIT_GPS_LATITUDE,
	UNIT_DATETIME_YEAR,
	UNIT_DATETIME_DAY_MONTH,
	UNIT_DATETIME_HOUR_MIN,
	UNIT_DATETIME_SEC
};

static const uint8_t CRC8_LOOKUP_TABLE[256] = {
		0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
		0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
		0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
		0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
		0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
		0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
		0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
		0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
		0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
		0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
		0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
		0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
		0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
		0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
		0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
		0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
		0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
		0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
		0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
		0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
		0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
		0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
		0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
		0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
		0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
		0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
		0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
		0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
		0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
		0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
		0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
		0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

static uint8_t crc8(const uint8_t *ptr, size_t len) {
	uint8_t crc = 0;
	for (size_t i = 0; i < len; i++) {
		crc = CRC8_LOOKUP_TABLE[crc ^ *ptr++];
	}
	return crc;
}

struct crossfire_sensor_t {
	const uint8_t id;
	const uint8_t subId;
	const char *name;
	const telemetry_unit_num_t unit;
	const uint8_t precision;
	double value;
};

static crossfire_sensor_t sensors[] = {
		{LINK_ID,        0, "RX_RSSI1",    UNIT_DB,                0},
		{LINK_ID,        1, "RX_RSSI2",    UNIT_DB,                0},
		{LINK_ID,        2, "RX_QUALITY",  UNIT_PERCENT,           0},
		{LINK_ID,        3, "RX_SNR",      UNIT_DB,                0},
		{LINK_ID,        4, "ANTENNA",     UNIT_RAW,               0},
		{LINK_ID,        5, "RF_MODE",     UNIT_RAW,               0},
		{LINK_ID,        6, "TX_POWER",    UNIT_MILLIWATTS,        0},
		{LINK_ID,        7, "TX_RSSI",     UNIT_DB,                0},
		{LINK_ID,        8, "TX_QUALITY",  UNIT_PERCENT,           0},
		{LINK_ID,        9, "TX_SNR",      UNIT_DB,                0},
		{BATTERY_ID,     0, "BATT",        UNIT_VOLTS,             1},
		{BATTERY_ID,     1, "CURR",        UNIT_AMPS,              1},
		{BATTERY_ID,     2, "CAPACITY",    UNIT_MAH,               0},
		{GPS_ID,         0, "GPS",         UNIT_GPS_LATITUDE,      0},
		{GPS_ID,         0, "GPS",         UNIT_GPS_LONGITUDE,     0},
		{GPS_ID,         2, "GSPD",        UNIT_KMH,               1},
		{GPS_ID,         3, "HDG",         UNIT_DEGREE,            3},
		{GPS_ID,         4, "ALT",         UNIT_METERS,            0},
		{GPS_ID,         5, "SATELLITES",  UNIT_RAW,               0},
		{ATTITUDE_ID,    0, "PITCH",       UNIT_RADIANS,           3},
		{ATTITUDE_ID,    1, "ROLL",        UNIT_RADIANS,           3},
		{ATTITUDE_ID,    2, "YAW",         UNIT_RADIANS,           3},
		{FLIGHT_MODE_ID, 0, "FLIGHT_MODE", UNIT_TEXT,              0},
		{CF_VARIO_ID,    0, "VSPD",        UNIT_METERS_PER_SECOND, 2},
		{0,              0, "UNKNOWN",     UNIT_RAW,               0},
};


struct crossfire_telemetry_t {
public:
	static double value(crossfire_sensor_num_t index) {
		return sensors[index].value;
	}

	void push(uint8_t data) {
		if (_offset == 0 && (data != RADIO_ADDRESS)) {
			return;
		}
		if ((_offset == 1 && (data < 2 || data > TELEMETRY_RX_PACKET_SIZE - 2)) ||
		    _offset >= TELEMETRY_RX_PACKET_SIZE) {
			_offset = 0;
			return;
		}
		_buffer[_offset++] = data;
		if (_offset > 4) {
			uint8_t size = _buffer[1];
			if (size + 2 == _offset && checksum()) {
				processData();
				_offset = 0;
			}
		}
	}

	void push(uint8_t *data, size_t len) {
		for (size_t i = 0; len > i; i++) push(data[i]);
	}

private:
	uint8_t _offset;
	uint8_t _buffer[TELEMETRY_RX_PACKET_SIZE];

	template<int N>
	bool readValue(size_t index, int &value) {
		bool result = false;
		uint8_t *byte = &_buffer[index];
		value = (int) (*byte & 0x80 ? -1 : 0);
		for (uint8_t i = 0; N > i; i++) {
			value <<= 8;
			if (*byte != 0xff) {
				result = true;
			}
			value += *byte++;
		}
		return result;
	}

	static void saveValue(uint8_t index, uint8_t addr, int value) {
		sensors[index].value = (sensors[index].precision ? (double)value / pow(10, sensors[index].precision)
		                                                 : (double)value);
	}

	void processData() {
		uint8_t addr = _buffer[0];
		uint8_t type = _buffer[2];
		int value = 0;

		switch (type) {
			case CF_VARIO_ID:
				if (readValue<2>(3, value))
					saveValue(VERTICAL_SPEED, addr, value);
				break;

			case GPS_ID:
				if (readValue<4>(3, value))
					saveValue(GPS_LATITUDE, addr, value);
				if (readValue<4>(7, value))
					saveValue(GPS_LONGITUDE, addr, value);
				if (readValue<2>(11, value))
					saveValue(GPS_GROUND_SPEED, addr, value);
				if (readValue<2>(13, value))
					saveValue(GPS_HEADING, addr, value);
				if (readValue<2>(15, value))
					saveValue(GPS_ALTITUDE, addr, value / 1000);
				if (readValue<1>(17, value))
					saveValue(GPS_SATELLITES, addr, value);
				break;

			case LINK_ID:
				for (unsigned int i = 0; i <= TX_SNR; i++) {
					if (readValue<1>(3 + i, value)) {
						if (i == TX_POWER) {
							static const int power_values[] = {0, 10, 25, 100, 500, 1000, 2000, 250};
							value = ((unsigned) value < DIM(power_values) ? power_values[value] : 0);
						}
						saveValue(i, addr, value);
					}
				}
				break;

			case BATTERY_ID:
				if (readValue<2>(3, value))
					saveValue(BATT_VOLTAGE, addr, value);
				if (readValue<2>(5, value))
					saveValue(BATT_CURRENT, addr, value);
				if (readValue<3>(7, value))
					saveValue(BATT_CAPACITY, addr, value);
				break;

			case ATTITUDE_ID:
				if (readValue<2>(3, value))
					saveValue(ATTITUDE_PITCH, addr, value / 10);
				if (readValue<2>(5, value))
					saveValue(ATTITUDE_ROLL, addr, value / 10);
				if (readValue<2>(7, value))
					saveValue(ATTITUDE_YAW, addr, value / 10);
				break;
		}
	}

	bool checksum() {
		uint8_t len = _buffer[1];
		uint8_t crc = crc8(&_buffer[2], len - 1);
		return (crc == _buffer[len + 1]);
	}
};

crossfire_telemetry_t crossfire;


#endif //LIB_CROSSFIRE_H
