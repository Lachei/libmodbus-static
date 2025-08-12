#pragma once

#include <span>
#include <array>
#include <string_view>
#include <inttypes.h>

// Modbus frames
//
// Modbus RTU
// --------------------------------------------------------
// | Slave address (1 byte) | Function code (1 byte) | Data (n bytes) | CRC check (2 bytes) |
// --------------------------------------------------------
// or
// --------------------------------------------------------
// | Slave address (1 byte) | Function code (1 byte) | Exception code (1 byte) | CRC check (2 bytes) |
// --------------------------------------------------------
//
// Modbus Ascci
// --------------------------------------------------------
// | Start ':' | Address (2 hex chars) | Function (2 hex chars) | Data (hex chars) | LRC (2 hex chars) | End CR LF (2 chars) |
// --------------------------------------------------------
//
// Modbus Tcp 
// --------------------------------------------------------
//| Transaction ID (2 bytes) | Protocol ID (2 bytes always 0x0000) | Length (2 bytes) | Address (1 byte) | Functioon Code | Data (n bytes) |
// --------------------------------------------------------

#define RESULT_ASSERT(cond, result) if (!(cond)) return result

namespace libmodbus_static {

using result = std::string_view;
constexpr std::string_view OK{"ok"};
constexpr inline uint8_t h_byte(uint16_t h) { return (h >> 8) & 0xff; }
constexpr inline uint8_t l_byte(uint16_t h) { return h & 0xff; }

enum struct register_t: uint8_t {
	NONE = 0,
	BITS = 1,
	BITS_WRITE = 2,
	HALFS = 3,
	HALFS_WRITE = 4,
};
/**
 * @brief Modbus function codes as defined in the Modbus specification
 *
 * This enumeration contains all the standard Modbus function codes supported
 * by this library. Each function code corresponds to a specific operation
 * that can be performed on Modbus data.
 */
enum struct function_code : uint8_t {
	NONE = 0,                        ///< No function code (invalid/uninitialized)
	READ_COILS = 1,                  ///< Read coils (discrete outputs) - FC 01
	READ_DISCRETE_INPUTS = 2,        ///< Read discrete inputs - FC 02
	READ_HOLDING_REGISTERS = 3,      ///< Read holding registers - FC 03
	READ_INPUT_REGISTERS = 4,        ///< Read input registers - FC 04
	WRITE_SINGLE_COIL = 5,           ///< Write single coil - FC 05
	WRITE_SINGLE_REGISTER = 6,       ///< Write single register - FC 06
	READ_EXCEPTION_STATUS = 7,       ///< Read exception status - FC 07
	DIAGNOSTICS = 8,                 ///< Diagnostics - FC 08
	WRITE_MULTIPLE_COILS = 15,       ///< Write multiple coils - FC 15 (0x0F)
	WRITE_MULTIPLE_REGISTERS = 16,   ///< Write multiple registers - FC 16 (0x10)
};

/**
 * @brief Modbus exception codes returned in error responses
 *
 * When a Modbus request cannot be processed successfully, the server
 * responds with an exception frame containing one of these exception codes
 * to indicate the specific error condition.
 */
enum struct exception_code : uint8_t {
	NONE = 0x00,                                        ///< No exception (success)
	ILLEGAL_FUNCTION = 0x01,                            ///< Function code not supported
	ILLEGAL_DATA_ADDRESS = 0x02,                        ///< Data address not valid
	ILLEGAL_DATA_VALUE = 0x03,                          ///< Data value not valid
	SLAVE_DEVICE_FAILURE = 0x04,                        ///< Unrecoverable error in slave device
	ACKNOWLEDGE = 0x05,                                 ///< Request accepted, processing
	SLAVE_DEVICE_BUSY = 0x06,                           ///< Slave device busy
	NEGATIVE_ACKNOWLEDGMENT = 0x07,                     ///< Request cannot be performed
	MEMORY_PARITY_ERROR = 0x08,                         ///< Memory parity error
	GATEWAY_PATH_UNAVAILABLE = 0x0A,                    ///< Gateway path unavailable
	GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B,    ///< Gateway target device failed to respond
};

/**
 * @brief Diagnostic sub-function codes for function code 08 (Diagnostics)
 *
 * The diagnostics function (FC 08) supports various sub-functions for
 * testing communication and retrieving diagnostic information from
 * Modbus devices. This enumeration defines the standard sub-function codes.
 */
enum struct diagnostic_code : uint16_t {
	RETURN_QUERY_DATA = 0x0000,                         ///< Echo back query data
	RESTART_COMMUNICATIONS_OPTION = 0x0001,             ///< Restart communications option
	RETURN_DIAGNOSTIC_REGISTER = 0x0002,                ///< Return diagnostic register
	CHANGE_ASCII_INPUT_DELIMITER = 0x0003,              ///< Change ASCII input delimiter
	FORCE_LISTEN_ONLY_MODE = 0x0004,                    ///< Force listen only mode
	CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER = 0x000A,    ///< Clear counters and diagnostic register
	RETURN_BUS_MESSAGE_COUNT = 0x000B,                  ///< Return bus message count
	RETURN_BUS_COMMUNICATION_ERROR_COUNT = 0x000C,      ///< Return bus communication error count
	RETURN_BUS_EXCEPTION_ERROR_COUNT = 0x000D,          ///< Return bus exception error count
	RETURN_SLAVE_MESSAGE_COUNT = 0x000E,                ///< Return slave message count
	RETURN_SLAVE_NO_RESPONSE_COUNT = 0x000F,            ///< Return slave no response count
	RETURN_SLAVE_NAK_COUNT = 0x0010,                    ///< Return slave NAK count
	RETURN_SLAVE_BUSY_COUNT = 0x0011,                   ///< Return slave busy count
	RETURN_BUS_CHARACTER_OVERRUN_COUNT = 0x0012,        ///< Return bus character overrun count
};

namespace checksum {
int calculate_lrc(std::span<uint8_t> puc_frame);
uint16_t calculate_crc16(std::span<uint8_t> buffer);
}

template<int N>
struct static_byte_vector {
	using T = uint8_t;
	std::array<T, N> storage{};
	int cur_size{};
	constexpr uint8_t& operator[](int i) { return storage[std::min(i, cur_size)]; }
	constexpr const uint8_t& operator[](int i) const { return storage[std::min(i, cur_size)]; }
	constexpr T* begin() { return storage.begin(); }
	constexpr T* end() { return storage.begin() + cur_size; }
	constexpr const T* begin() const { return storage.begin(); }
	constexpr const T* end() const { return storage.begin() + cur_size; }
	constexpr T* push() { if (cur_size >= N) return {}; return storage.data() + cur_size++; }
	constexpr bool push(const T& e) { if (cur_size == N) return false; storage[cur_size++] = e; return true; }
	constexpr bool push(T&& e) { if (cur_size == N) return false; storage[cur_size++] = std::move(e); return true; }
	constexpr void clear() { cur_size = 0; }
	constexpr bool empty() const { return cur_size == 0; }
	constexpr int size() const { return cur_size; }
	constexpr std::span<uint8_t> span() const { return {storage.data(), cur_size}; }
};

template<int MAX_SIZE = 256>
struct modbus_frame {
	enum struct state {
		WRITE_ADDR_START_MBAP,
		WRITE_ADDR,
		WRITE_FC,
		WRITE_DATA_EC,
		WRITE_DATA,
		WRITE_CRC,
		FINAL,
	};
	enum struct type {
		NORMAL,
		EXCEPTION,
	};
	struct mbap_header {
		uint16_t transaction_id{}; ///< Transaction identifier for matching requests/responses
		uint16_t protocol_id{};    ///< Protocol identifier (always 0 for Modbus)
		uint16_t length{};        ///< Number of following bytes (PDU length + 1)
	};

	state cur_state{state::WRITE_ADDR_START_MBAP};
	static_byte_vector<MAX_SIZE> frame_data{};
	mbap_header *tcp_header{};
	uint8_t *addr{};
	uint8_t *fc{};
	uint8_t *ec{};
	std::span<uint8_t> data{};
	constexpr void clear() {
		cur_state = state::WRITE_ADDR_START_MBAP;
		frame_data.clear();
		tcp_header = {};
		addr = {};
		fc = {};
		data = {};
	}
	// ---------------------------------------------------------------------------------------
	// Write functions
	// ---------------------------------------------------------------------------------------
	constexpr result write_ascii_start() {
		RESULT_ASSERT(cur_state == state::WRITE_ADDR_START_MBAP, "STATE_NOT_WRITE_START");
		RESULT_ASSERT(frame_data.push(':'), "WRITE_ASCII_START_FAILED");
		cur_state = state::WRITE_ADDR;
		return OK;
	}
	constexpr result write_mbap(const mbap_header &header) {
		RESULT_ASSERT(cur_state == state::WRITE_ADDR_START_MBAP, "STATE_NOT_WRITE_MBAP");
		this->tcp_header = reinterpret_cast<mbap_header*>(frame_data.end());
		RESULT_ASSERT(frame_data.push(h_byte(header.transaction_id)), "WRITE_TRANS_ID_FAILED");
		RESULT_ASSERT(frame_data.push(l_byte(header.transaction_id)), "WRITE_TRANS_ID_FAILED");
		RESULT_ASSERT(frame_data.push(h_byte(header.protocol_id)), "WRITE_PROTOCOL_ID_FAILED");
		RESULT_ASSERT(frame_data.push(l_byte(header.protocol_id)), "WRITE_PROTOCOL_ID_FAILED");
		cur_state = state::WRITE_ADDR;
		return OK;
	}
	constexpr result write_addr(uint8_t addr) {
		RESULT_ASSERT(cur_state == state::WRITE_ADDR_START_MBAP || cur_state == state::WRITE_ADDR, 
				"STATE_NOT_WRITE_ADDR");
		this->addr = frame_data.end();
		RESULT_ASSERT(frame_data.push(addr), "WRITE_ADDR_FAILED");
		cur_state = state::WRITE_FC;
		return OK;
	}
	constexpr result write_fc(function_code fc, type t = type::NORMAL) {
		RESULT_ASSERT(cur_state == state::WRITE_FC, "STATE_NOT_WRITE_FC");
		RESULT_ASSERT(fc >= function_code::NONE && fc <= function_code::WRITE_MULTIPLE_REGISTERS,
				"INVALID_FUNCTION_CODE");
		if (t == type::EXCEPTION)
			reinterpret_cast<uint8_t&>(fc) |= 0x80;
		this->fc = frame_data.end();
		RESULT_ASSERT(frame_data.push(fc), "WRITE_FC_FAILED");
		cur_state = state::WRITE_DATA_EC;
		return OK;
	}
	constexpr result write_data(std::span<uint8_t> data) {
		RESULT_ASSERT(cur_state == state::WRITE_DATA_EC || cur_state == state::WRITE_DATA, 
				"STATE_NOT_WRITE_DATA");
		for (uint8_t b: data)
			RESULT_ASSERT(frame_data.push(b), "WRITE_DATA_FAILED");
		cur_state = state::WRITE_DATA;
		return OK;
	}
	constexpr result write_ec(exception_code ec) {
		RESULT_ASSERT(cur_state == state::WRITE_DATA_EC, "STATE_NOT_WRITE_EC");
		this->ec = frame_data.end();
		RESULT_ASSERT(frame_data.push(ec), "WRITE_EC_FAILED");
		cur_state = state::WRITE_CRC;
		return OK;
	}
	constexpr result write_checksum(uint16_t crc) {
		RESULT_ASSERT(cur_state == state::WRITE_CRC, "STATE_NOT_WRITE_CRC");
		RESULT_ASSERT(frame_data.push(h_byte(crc)), "FAILED_CRC_WRITE_0");
		RESULT_ASSERT(frame_data.push(l_byte(crc)), "FAILED_CRC_WRITE_1");
		cur_state = state::FINAL;
		return OK;
	}
	// ---------------------------------------------------------------------------------------
	// Receive functions
	// ---------------------------------------------------------------------------------------
	constexpr result receive_byte(uint8_t b) {
		RESULT_ASSERT(frame_data.push(b), "RECEIVE_BYTE_FAILED");
		return OK;
	}
	constexpr result parse_rtu_frame() {
		RESULT_ASSERT(frame_data.size() < 4, "INVALID_RTU_FRAME");
		RESULT_ASSERT(checksum::calculate_crc16(frame_data.span()) == 0, "INVALID_RTU_CRC");

		addr = &frame_data[0];
		fc = &frame_data[1];
		bool is_exception = uint8_t(fc) & 0x80;
		if (is_exception) {
			*fc &= 0x7f;
			ec = &frame_data[2];
		} else {
			data = {&frame_data[2], &frame_data[frame_data.size() - 2]};
		}

		return OK;
	}
	constexpr result parse_ascii_frame(std::span<uint8_t> frame) {
		RESULT_ASSERT(false, "Not yet implemented");
		RESULT_ASSERT(frame_data.size() < 5, "INVALID_ASCII_FRAME");
		RESULT_ASSERT(frame_data[0] == ':', "INVALID_ASCII_FRAME_START");
		RESULT_ASSERT(checksum::calculate_crc16(frame_data.span()) == 0, "INVALID_ASCII_CRC");


		return OK;
	}
	constexpr result parse_tcp_frame(std::span<uint8_t> frame) {
		RESULT_ASSERT(frame_data.size() < 9, "INVALID_TCP_FRAME");

		tcp_header = reinterpret_cast<mbap_header*>(&frame[0]);
		fc = &tcp_header->fc;
		bool is_exception = *fc & 0x80;
		if (is_exception) {
			*fc &= 0x7f;
			ec = &frame_data[8];
		} else {
			data = {&frame_data[8], &frame_data.back()};
		}

		return OK;
	}
};

}

#undef RESULT_ASSERT

