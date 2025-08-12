#include "common.h"

#include <ranges>

namespace libmodbus_static{

namespace checksum {
int calculate_lrc(std::span<uint8_t> puc_frame) {
	uint8_t uc_lrc = 0;

	for (uint8_t b: puc_frame)
		uc_lrc += b;

	return static_cast<uint8_t>(-static_cast<int8_t>(uc_lrc));
}

uint16_t calculate_crc16(std::span<uint8_t> buffer) {
	uint16_t cur_crc = 0xFFFF;

	for (uint8_t b: buffer) {
		cur_crc ^= b;
		for (int i: std::ranges::iota_view{0, 8}) {
			bool odd = cur_crc & 1;
			cur_crc >>= 1;
			if (odd)
				cur_crc ^= 0xA001;
		}
	}

	return cur_crc;
}
}

}

