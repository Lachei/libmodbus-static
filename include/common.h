#pragma once

#include <span>
#include <array>
#include <string_view>
#include <inttypes.h>
#include <ranges>

//TODO: remove
#include <iostream>

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
constexpr std::string_view OK{"OK"};
constexpr std::string_view INVALID_CRC{"CRC_CHECK_FAILED"};
constexpr inline uint8_t h_byte(uint16_t h) { return (h >> 8) & 0xff; }
constexpr inline uint8_t l_byte(uint16_t h) { return h & 0xff; }

enum struct transport_t: uint8_t {
	NONE = 0,
	RTU = 1,
	TCP = 2,
	ASCII = 3
};

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

struct type {
	bool REQUEST: 1{};
	bool RESPONSE: 1{};
	bool EXCEPTION: 1{};
};
inline bool fc_requires_length(function_code fc, type t)  {
	return (t.REQUEST && (fc == function_code::WRITE_MULTIPLE_COILS || fc == function_code::WRITE_MULTIPLE_REGISTERS))
			|| (t.RESPONSE && (fc >= function_code::READ_COILS && fc <= function_code::READ_INPUT_REGISTERS));
}

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
	std::array<uint8_t, N> storage{};
	int cur_size{};
	constexpr uint8_t& operator[](int i) { return storage[std::min(i, cur_size)]; }
	constexpr const uint8_t& operator[](int i) const { return storage[std::min(i, cur_size)]; }
	constexpr uint8_t* begin() { return storage.begin(); }
	constexpr uint8_t* end() { return storage.begin() + cur_size; }
	constexpr const uint8_t* begin() const { return storage.begin(); }
	constexpr const uint8_t* end() const { return storage.begin() + cur_size; }
	constexpr uint8_t* push() { if (cur_size >= N) return {}; return storage.data() + cur_size++; }
	constexpr bool push(uint8_t e) { if (cur_size == N) return false; storage[cur_size++] = e; return true; }
	constexpr void clear() { cur_size = 0; }
	constexpr bool empty() const { return cur_size == 0; }
	constexpr int size() const { return cur_size; }
	constexpr std::span<uint8_t> span() { return {begin(), end()}; }
	constexpr std::span<const uint8_t> span() const { return {begin(), end()}; }
};

template<int MAX_SIZE = 256>
struct modbus_frame {
	enum struct state {
		WRITE_ADDR_START_MBAP = 0,
		WRITE_ADDR = 1,
		WRITE_FC = 2,
		WRITE_LENGTH = 3,
		WRITE_DATA_EC = 4,
		WRITE_DATA = 5,
		WRITE_CRC_0 = 6,
		WRITE_CRC_1 = 7,
		FINAL = 8,
	};
	struct mbap_header {
		uint16_t transaction_id{}; ///< Transaction identifier for matching requests/responses
		uint16_t protocol_id{};    ///< Protocol identifier (always 0 for Modbus)
		uint16_t length{};        ///< Number of following bytes (PDU length + 1)
	};

	state cur_state{state::WRITE_ADDR_START_MBAP};
	transport_t transport{transport_t::NONE};
	static_byte_vector<MAX_SIZE> frame_data{};
	mbap_header *tcp_header{};
	uint8_t *addr{};
	uint8_t *fc{};
	uint8_t *byte_count{};
	uint8_t *ec{};
	uint8_t *data{};
	type t{.REQUEST = true};
	constexpr bool empty() {return cur_state == state::WRITE_ADDR_START_MBAP || frame_data.empty() || !fc;}
	constexpr void clear() {
		cur_state = state::WRITE_ADDR_START_MBAP;
		transport = transport_t::NONE;
		frame_data.clear();
		tcp_header = {};
		addr = {};
		fc = {};
		byte_count = {};
		data = {};
		t = {.REQUEST = true};
	}
	constexpr bool is_ascii() const { return frame_data[0] == ':'; }
	constexpr bool is_tcp() const { return tcp_header; }
	constexpr bool is_rtu() const { return addr && !is_ascii() && !is_tcp(); }
	constexpr void set_type(type t) { this->t = t; }
	// ---------------------------------------------------------------------------------------
	// Write functions
	// ---------------------------------------------------------------------------------------
	constexpr int missing_data_bytes() {
		if (!fc)
			return -1;
		bool requires_length = fc_requires_length(function_code(*fc), t);
		if (requires_length && !byte_count)
			return -1;
		if (requires_length)
			return (*byte_count - (frame_data.end() - byte_count) + 1);
		return 5 - (frame_data.end() - fc);
	}
	constexpr result write_ascii_start() {
		RESULT_ASSERT(cur_state == state::WRITE_ADDR_START_MBAP, "STATE_NOT_WRITE_START");
		RESULT_ASSERT(frame_data.push(':'), "WRITE_ASCII_START_FAILED");
		transport = transport_t::ASCII;
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
		RESULT_ASSERT(frame_data.push(0), "WRITE_TCP_LENGTH_FAILED");
		RESULT_ASSERT(frame_data.push(0), "WRITE_TCP_LENGTH_FAILED");
		transport = transport_t::TCP;
		cur_state = state::WRITE_ADDR;
		return OK;
	}
	constexpr result write_addr(uint8_t addr) {
		RESULT_ASSERT(cur_state == state::WRITE_ADDR_START_MBAP || cur_state == state::WRITE_ADDR, 
				"STATE_NOT_WRITE_ADDR");
		this->addr = frame_data.end();
		RESULT_ASSERT(frame_data.push(addr), "WRITE_ADDR_FAILED");
		if (transport == transport_t::NONE)
			transport = transport_t::RTU;
		cur_state = state::WRITE_FC;
		return OK;
	}
	constexpr result write_fc(function_code fc) {
		RESULT_ASSERT(cur_state == state::WRITE_FC, "STATE_NOT_WRITE_FC");
		RESULT_ASSERT(fc >= function_code::NONE && fc <= function_code::WRITE_MULTIPLE_REGISTERS,
				"INVALID_FUNCTION_CODE");
		if (t.EXCEPTION)
			reinterpret_cast<uint8_t&>(fc) |= 0x80;
		this->fc = frame_data.end();
		RESULT_ASSERT(frame_data.push(uint8_t(fc)), "WRITE_FC_FAILED");
		if (fc_requires_length(fc, t))
			cur_state = state::WRITE_LENGTH;
		else
			cur_state = state::WRITE_DATA_EC;
		return OK;
	}
	constexpr result write_length(uint8_t l) {
		RESULT_ASSERT(cur_state == state::WRITE_LENGTH, "STATE_NOT_WRITE_LENGTH");
		byte_count = frame_data.end();
		RESULT_ASSERT(frame_data.push(l), "WRITE_LENGTH_FAILED");
		cur_state = state::WRITE_DATA;
		return OK;
	}
	constexpr result write_data(uint8_t data) {
		RESULT_ASSERT(cur_state == state::WRITE_DATA_EC || cur_state == state::WRITE_DATA, 
				"STATE_NOT_WRITE_DATA");
		if (!this->data)
			this->data = frame_data.end();
		RESULT_ASSERT(frame_data.push(data), "WRITE_DATA_FAILED");
		int missing_bytes = missing_data_bytes();
		if (missing_bytes == 0 && tcp_header)
			cur_state = state::FINAL;
		else if (missing_bytes == 0)
			cur_state = state::WRITE_CRC_0;
		else
			cur_state = state::WRITE_DATA;
		return OK;
	}
	constexpr result write_data(std::span<uint8_t> data) {
		if (data.data()) {
			for (uint8_t b: data)
				RESULT_ASSERT(write_data(b) == OK, "WRITE_DATA_FAILED");
		}
		else  {
			for ([[maybe_unused]] size_t i : std::ranges::iota_view{size_t(0), data.size()})
				RESULT_ASSERT(write_data(0) == OK, "WRITE_DATA_NULL_FAILED");
		}
		return OK;
	}
	constexpr result write_ec(exception_code ec) {
		RESULT_ASSERT(cur_state == state::WRITE_DATA_EC, "STATE_NOT_WRITE_EC");
		this->ec = frame_data.end();
		RESULT_ASSERT(frame_data.push(uint8_t(ec)), "WRITE_EC_FAILED");
		if (tcp_header)
			cur_state = state::FINAL;
		else
		cur_state = state::WRITE_CRC_0;
		return OK;
	}
	constexpr result write_checksum(uint16_t crc) {
		RESULT_ASSERT(cur_state == state::WRITE_CRC_0, "STATE_NOT_WRITE_CRC");
		RESULT_ASSERT(frame_data.push(l_byte(crc)), "FAILED_CRC_WRITE_0");
		RESULT_ASSERT(frame_data.push(h_byte(crc)), "FAILED_CRC_WRITE_1");
		cur_state = state::FINAL;
		RESULT_ASSERT(checksum::calculate_crc16(frame_data.span()) == 0, INVALID_CRC);
		return OK;
	}
	constexpr result write_checksum(uint8_t crc) {
		RESULT_ASSERT(cur_state == state::WRITE_CRC_0 || cur_state == state::WRITE_CRC_1,
			"STATE_NOT_WRITE_CRC");
		RESULT_ASSERT(frame_data.push(crc), "FAILED_CRC_WRITE");
		if (cur_state == state::WRITE_CRC_0)
			cur_state = state::WRITE_CRC_1;
		else
			cur_state = state::FINAL;
		if (cur_state == state::FINAL)
			RESULT_ASSERT(checksum::calculate_crc16(frame_data.span()) == 0, INVALID_CRC);
		return OK;
	}
	// ---------------------------------------------------------------------------------------
	// Receive functions
	// ---------------------------------------------------------------------------------------
	constexpr result process(uint8_t b) {
		switch (cur_state) {
		case modbus_frame<MAX_SIZE>::state::WRITE_ADDR_START_MBAP:
		case modbus_frame<MAX_SIZE>::state::WRITE_ADDR:
			return write_addr(b);
		case modbus_frame<MAX_SIZE>::state::WRITE_FC:
			return write_fc(function_code(b));
		case modbus_frame<MAX_SIZE>::state::WRITE_LENGTH:
			return write_length(b);
		case modbus_frame<MAX_SIZE>::state::WRITE_DATA:
		case modbus_frame<MAX_SIZE>::state::WRITE_DATA_EC:
			return write_data(b);
		case modbus_frame<MAX_SIZE>::state::WRITE_CRC_0:
		case modbus_frame<MAX_SIZE>::state::WRITE_CRC_1:
			return write_checksum(b);
		case modbus_frame<MAX_SIZE>::state::FINAL:
			return "NO_WRITE_IN_FINAL_STATE";
		}
		return "INVALID_STATE";
	}
};

}

#undef RESULT_ASSERT

