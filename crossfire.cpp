#include "crossfire.h"

uint8_t crsf_telemetry_offset;
uint8_t crsf_telemetry_buffer[CRSF_FRAME_SIZE_MAX];

uint8_t CRC8_LOOKUP_TABLE[256] = {
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

crossfire_sensor_t sensors[] = {
		{CRSF_FRAMETYPE_LINK,        0, UNIT_DB,                0, "RX_RSSI1",    0},
		{CRSF_FRAMETYPE_LINK,        1, UNIT_DB,                0, "RX_RSSI2",    0},
		{CRSF_FRAMETYPE_LINK,        2, UNIT_PERCENT,           0, "RX_QUALITY",  0},
		{CRSF_FRAMETYPE_LINK,        3, UNIT_DB,                0, "RX_SNR",      0},
		{CRSF_FRAMETYPE_LINK,        4, UNIT_RAW,               0, "ANTENNA",     0},
		{CRSF_FRAMETYPE_LINK,        5, UNIT_RAW,               0, "RF_MODE",     0},
		{CRSF_FRAMETYPE_LINK,        6, UNIT_MILLIWATTS,        0, "TX_POWER",    0},
		{CRSF_FRAMETYPE_LINK,        7, UNIT_DB,                0, "TX_RSSI",     0},
		{CRSF_FRAMETYPE_LINK,        8, UNIT_PERCENT,           0, "TX_QUALITY",  0},
		{CRSF_FRAMETYPE_LINK,        9, UNIT_DB,                0, "TX_SNR",      0},
		{CRSF_FRAMETYPE_BATTERY,     0, UNIT_VOLTS,             1, "BATT",        0},
		{CRSF_FRAMETYPE_BATTERY,     1, UNIT_AMPS,              1, "CURR",        0},
		{CRSF_FRAMETYPE_BATTERY,     2, UNIT_MAH,               0, "CAPACITY",    0},
		{CRSF_FRAMETYPE_GPS,         0, UNIT_GPS_LATITUDE,      0, "GPS",         0},
		{CRSF_FRAMETYPE_GPS,         0, UNIT_GPS_LONGITUDE,     0, "GPS",         0},
		{CRSF_FRAMETYPE_GPS,         2, UNIT_KMH,               1, "GSPD",        0},
		{CRSF_FRAMETYPE_GPS,         3, UNIT_DEGREE,            3, "HDG",         0},
		{CRSF_FRAMETYPE_GPS,         4, UNIT_METERS,            0, "ALT",         0},
		{CRSF_FRAMETYPE_GPS,         5, UNIT_RAW,               0, "SATELLITES",  0},
		{CRSF_FRAMETYPE_ATTITUDE,    0, UNIT_RADIANS,           3, "PITCH",       0},
		{CRSF_FRAMETYPE_ATTITUDE,    1, UNIT_RADIANS,           3, "ROLL",        0},
		{CRSF_FRAMETYPE_ATTITUDE,    2, UNIT_RADIANS,           3, "YAW",         0},
		{CRSF_FRAMETYPE_FLIGHT_MODE, 0, UNIT_TEXT,              0, "FLIGHT_MODE", 0},
		{CRSF_FRAMETYPE_VARIO,       0, UNIT_METERS_PER_SECOND, 2, "VSPD",        0},
		{0,                          0, UNIT_RAW,               0, "UNKNOWN",     0},
};


uint8_t crc8(uint8_t *ptr, size_t len) {
	uint8_t crc = 0;
	for (size_t i = 0; i < len; i++) {
		crc = CRC8_LOOKUP_TABLE[crc ^ *ptr++];
	}
	return crc;
}

bool crsf_telemetry_checksum() {
	int len = crsf_telemetry_buffer[1];
	int crc = crc8(&crsf_telemetry_buffer[2], len - 1);
	return (crc == crsf_telemetry_buffer[len + 1]);
}

bool crsf_telemetry_get_value(int N, size_t index, int &value) {
	bool result = false;
	uint8_t *byte = &crsf_telemetry_buffer[index];
	value = (int) *byte & 0x80 ? -1 : 0;
	for (size_t i = 0; N > i; i++) {
		value <<= 8;
		if (*byte != 0xff) {
			result = true;
		}
		value += *byte++;
	}
	return result;
}

void crsf_telemetry_set_value(size_t index, int value) {
	sensors[index].value = (sensors[index].precision ? (double) value / pow(10, sensors[index].precision)
	                                                 : (double) value);
	if (index == CRSF_GPS_ALTITUDE) sensors[index].value /= 1000;
}

void crsf_telemetry_process_data() {
//	crossfire_address_t address = (crossfire_address_t)crsf_telemetry_buffer[0];
	auto frametype = (crossfire_frame_type_t) crsf_telemetry_buffer[2];
	int value = 0;

	switch (frametype) {
		case CRSF_FRAMETYPE_VARIO:
			if (crsf_telemetry_get_value(2, 3, value))
				crsf_telemetry_set_value(CRSF_VERTICAL_SPEED, value);
			break;

		case CRSF_FRAMETYPE_GPS:
			if (crsf_telemetry_get_value(4, 3, value))
				crsf_telemetry_set_value(CRSF_GPS_LATITUDE, value);
			if (crsf_telemetry_get_value(4, 7, value))
				crsf_telemetry_set_value(CRSF_GPS_LONGITUDE, value);
			if (crsf_telemetry_get_value(2, 11, value))
				crsf_telemetry_set_value(CRSF_GPS_GROUND_SPEED, value);
			if (crsf_telemetry_get_value(2, 13, value))
				crsf_telemetry_set_value(CRSF_GPS_HEADING, value);
			if (crsf_telemetry_get_value(2, 15, value))
				crsf_telemetry_set_value(CRSF_GPS_ALTITUDE, value);
			if (crsf_telemetry_get_value(1, 17, value))
				crsf_telemetry_set_value(CRSF_GPS_SATELLITES, value);
			break;

		case CRSF_FRAMETYPE_LINK:
			for (unsigned int i = 0; i <= CRSF_TX_SNR; i++) {
				if (crsf_telemetry_get_value(1, 3 + i, value)) {
					if (i == CRSF_TX_POWER) {
						static const int power_values[] = {0, 10, 25, 100, 500, 1000, 2000, 250};
						value = ((unsigned) value < DIM(power_values) ? power_values[value] : 0);
					}
					crsf_telemetry_set_value(i, value);
				}
			}
			break;

		case CRSF_FRAMETYPE_BATTERY:
			if (crsf_telemetry_get_value(2, 3, value))
				crsf_telemetry_set_value(CRSF_BATT_VOLTAGE, value);
			if (crsf_telemetry_get_value(2, 5, value))
				crsf_telemetry_set_value(CRSF_BATT_CURRENT, value);
			if (crsf_telemetry_get_value(3, 7, value))
				crsf_telemetry_set_value(CRSF_BATT_CAPACITY, value);
			break;

		case CRSF_FRAMETYPE_ATTITUDE:
			if (crsf_telemetry_get_value(2, 3, value))
				crsf_telemetry_set_value(CRSF_ATTITUDE_PITCH, value / 10);
			if (crsf_telemetry_get_value(2, 5, value))
				crsf_telemetry_set_value(CRSF_ATTITUDE_ROLL, value / 10);
			if (crsf_telemetry_get_value(2, 7, value))
				crsf_telemetry_set_value(CRSF_ATTITUDE_YAW, value / 10);
			break;

		default:
			break;
	}
}

void crsf_telemetry_push_byte(uint8_t data) {
	if (crsf_telemetry_offset == 0 && (data != CRSF_ADDRESS_RADIO_RADIO)) {
		return;
	}
	if ((crsf_telemetry_offset == 1 && (data < 2 || data > CRSF_FRAME_SIZE_MAX - 2)) ||
	    crsf_telemetry_offset >= CRSF_FRAME_SIZE_MAX) {
		crsf_telemetry_offset = 0;
		return;
	}
	crsf_telemetry_buffer[crsf_telemetry_offset++] = data;
	if (crsf_telemetry_offset > 4) {
		uint8_t size = crsf_telemetry_buffer[1];
		if (size + 2 == crsf_telemetry_offset && crsf_telemetry_checksum()) {
			crsf_telemetry_process_data();
			crsf_telemetry_offset = 0;
		}
	}
}

void crsf_telemetry_push_byte(uint8_t *data, int len) {
	for (int i = 0; len > i; i++) crsf_telemetry_push_byte(data[i]);
}

crossfire_sensor_t crsf_telemetry_get_sensor(crossfire_sensor_idx_t sensor_index) {
	return sensors[sensor_index];
}
