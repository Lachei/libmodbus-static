#pragma once

#include "common.h"
#include <cstring>
#include <array>
#include <ranges>
#include <cstdint>
#include <bit>

//TODO: remove
#include <iostream>

namespace libmodbus_static {
template<int N>
using mod_string = std::array<char, N>;
template<typename T>
std::string_view to_string_view(const T &s) {
	std::string_view r{s.begin(), s.end()};
	return r.substr(0, r.find_first_of('\0'));
}

template <typename T>
std::span<uint8_t> to_byte_span(T& t) {
	return std::span<uint8_t>{ reinterpret_cast<uint8_t*>(&t), sizeof(t) };
}

uint32_t popcount(std::span<uint8_t> bytes) {
	uint32_t bitcount{};
	for (uint8_t b: bytes)
		bitcount += std::popcount(b);
	return bitcount;
}

template<typename T>
constexpr uintptr_t to_uint_ptr(T member) {
	return *reinterpret_cast<uintptr_t*>(&member);
}

template<typename T>
struct swap_byte_order {
	constexpr void operator()(T src, T& dst) const { 
		constexpr int s{sizeof(T)};
		for (int byte: std::ranges::iota_view{0, s})
			std::memcpy(reinterpret_cast<uint8_t*>(&dst) + (s - 1 - byte), 
				reinterpret_cast<uint8_t*>(&src) + byte, sizeof(uint8_t)); 
	}
};
template<typename T> 
requires requires (T t) { sizeof(*t.data()) == 1; t = t; }
struct swap_byte_order<T> {
	constexpr void operator()(const T &src, T &dst) const {
		dst = src;
	}
};

constexpr std::string_view IN_PROGRESS{"IN_PROGRESS"};
constexpr std::string_view INVALID_RESPONSE{"RESPONSE_FROM_SERVER_INVALID"};
constexpr std::string_view WRONG_ADDR{"WRONG_ADDR"};
struct result_err {
	std::span<uint8_t> res{};
	std::string_view err{OK};
};
struct r_tie {
	std::span<uint8_t> &res;
	std::string_view &err;
	r_tie& operator=(const result_err &r) { res = r.res; err = r.err; return *this; }
};

/**
 * Example Layout which can be used
 * #pragma pack(push, 1)
 * struct example_layout {
 *	struct bits_layout {
 *		constexpr static int OFFSET{0};
 *		bool enabled: 1{};
 *		bool active: 1{};
 *		bool visible: 1{};
 *		bool trusted: 1{};
 *		bool uknown: 1{};
 *	] bits_registers;
 *	// if eg. no discrete inputs are used, just leave them out else
 *	// struct bits_write_layout {
 *	// 	constexpr static int OFFSET{256};
 *	//	bool setable: 1{};
 *	// } bits_write_registers;
 *	struct halfs_layout {
 *		constexpr static int OFFSET{40000};
 *		float a{};
 *		float b{};
 *		uint16_t another{};
 *		char string_field[32]{"Default"}; // careful with default inited strings, have to be byte swapped
 *		uint32_t events{};
 *	} halfs_registeres;
 *	struct halfs_write_layout {
 *		constexpr static int OFFSET{60000};
 *		float a{};
 *		float b{};
 *		uint16_t others{};
 *	} halfs_write_registers;
 * };
 * #pragma pack(pop)
 */

template<typename S, typename M>
using mem_t = std::decay_t<decltype(std::declval<S>().*std::declval<M>())>;
template<typename L, typename S, typename M>
concept IsMemberChain = requires (L l, S s, M m) { l.*s.*m; };
template<typename L, typename S>
concept IsBitsRegisters = requires (L l, S s) {l.bits_registers = s;} || requires(L l, S s) {l.bits_write_registers = s;};
template<typename S>
concept HasOffset = requires(S s) { std::is_same_v<decltype(s.OFFSET), int>; };
template<typename L>
concept HasBits = requires(L l) { l.bits_registers; };
template<typename L>
concept HasWriteBits = requires(L l) { l.bits_write_registers; };
template<typename L>
concept HasHalfs = requires(L l) { l.halfs_registers; };
template<typename L>
concept HasWriteHalfs = requires(L l) { l.halfs_write_registers; };
template<typename L, typename R>
constexpr register_t type_to_register() { 
	static_assert(false, "The type could not be converted, make sure your layout has the correct members"); 
	return register_t::NONE;
}
template<typename L, typename R> requires requires (L l, R r) {l.bits_registers = r;}
constexpr register_t type_to_register() { return register_t::BITS; }
template<typename L, typename R> requires requires (L l, R r) {l.bits_write_registers = r;}
constexpr register_t type_to_register() { return register_t::BITS_WRITE; }
template<typename L, typename R> requires requires (L l, R r) {l.halfs_registers = r;}
constexpr register_t type_to_register() { return register_t::HALFS; }
template<typename L, typename R> requires requires (L l, R r) {l.halfs_write_registers = r;}
constexpr register_t type_to_register() { return register_t::HALFS_WRITE; }
template<typename L, typename R>
constexpr R& get_register_ref(L &l) { 
	static_assert(false, "The type could not be converted, make sure your layout has the correct members"); 
}
template<typename L, typename R> requires requires (L l, R r) {l.bits_registers = r;}
constexpr R& get_register_ref(L &l) { return l.bits_registers; }
template<typename L, typename R> requires requires (L l, R r) {l.bits_write_registers = r;}
constexpr R& get_register_ref(L &l) { return l.bits_write_registers; }

#define MEMBER_CHAIN(l, r, m) (&l::r##_registers), (&l::r##_layout::m)
#define MEMBER_CHAIN_RANGE(l, r, m1, m2) (&l::r##_registers), (&l::r##_layout::m1), (&l::r##_layout::m2)

template<typename Layout, uint8_t Address = 0, int MAX_SIZE = 256>
struct modbus_register {
	static modbus_register& Default() { static modbus_register r{}; return r; }

	constexpr static uint8_t addr = Address;
	Layout storage{}; // do not access directly, contains data already byte swapped, read and write with read() and write()
	modbus_frame<MAX_SIZE> buffer{};
	struct last_completed{
		uint8_t addr{};
		function_code fc{};
		uint16_t i1{};
		uint16_t i2{};
		uint16_t crc{};
		constexpr bool operator==(const last_completed &o) const {
			return addr == o.addr && fc == o.fc && i1 == o.i1 && i2 == o.i2 && crc == o.crc;
		}
		constexpr bool operator!=(const last_completed &o) const { return !(*this == o); }
	} lc {};
	

	constexpr void switch_to_request() {
		buffer.clear(); 
		buffer.set_type({.REQUEST = true});
	}
	constexpr void switch_to_response() { 
		buffer.clear(); 
		buffer.set_type({.RESPONSE = true});
	}

	// Does all the magic
	// processes incoming data from any source and returns a non_empty span
	// if result frame is ready to be sent
	// if client is waiting for the result the result can be checked against ACK:
	// if (process(byte) == ACK)
	//	... done waiting
	constexpr result_err process_rtu(uint8_t b) {
		return _process(b);
	}
	constexpr result_err process_ascii(uint8_t b) {
		return {.err = "NOT_YET_IMPLEMENTED"};
	}
	constexpr result_err process_tcp(uint8_t b) {
		if (buffer.cur_state == modbus_frame<MAX_SIZE>::state::WRITE_ADD_START_MBAP) {
			if (storage.frame_data.size() > sizeof(*storage.frame_data.tcp_header)) {
				storage.clear();
				return {.err = "FATAL_TOO_LARGE_SIZE_FOR_TCP_HEADER"};
			}
			result r = buffer.frame_data.push(b);
			if (r != OK) {
				buffer.clear();
				return {.err = r};
			}
			// done with tcp header recieving
			if (buffer.frame_data.size() == sizeof(*buffer.frame_data.tcp_header)) {
				buffer.frame_data.tcp_header = reinterpret_cast<modbus_frame<MAX_SIZE>::mbap_header*>(buffer.frame_data.frame_data.data());
				uint16_t l = buffer.frame_data.tcp_header->length;
				buffer.frame_data.tcp_header->length = (l_byte(l) << 8) | h_byte(l);
				buffer.cur_state = modbus_frame<MAX_SIZE>::state::WRITE_ADDR;
			}
			return {.err = IN_PROGRESS};
		}
		if (!buffer.tcp_header) {
			buffer.clear();
			return {.err = "FATAL_MISSING_TCP_HEADER_IN_FRAME"};
		}
		if (buffer.frame_data.size() > buffer.tcp_header->lenngth + sizeof(*buffer.frame_data.tcp_header)) {
			buffer.clear();
			return {.err = "FATAL_TCP_FRAME_LENGTH_FULL"};
		}
		return _process(b);
	}

	constexpr result start_rtu_frame(uint8_t addr) {
		buffer.clear();
		return buffer.write_addr(addr);
	}
	constexpr result start_ascii_frame(uint8_t addr) {
		buffer.clear();
		result r = buffer.write_ascii_start();
		if (r != OK)
			return r;
		return buffer.write_addr(addr);
	}
	constexpr result start_tcp_frame(uint16_t trans, uint8_t addr) {
		buffer.clear();
		typename modbus_frame<MAX_SIZE>::mbap_header header{.transactionId = trans, .addreess = addr};
		return buffer.write_mbap(header);
	}

	template<typename Reg, typename MemA, typename MemB>
	requires IsMemberChain<Layout, Reg, MemA> && IsMemberChain<Layout, Reg, MemB> && HasOffset<mem_t<Layout, Reg>>
	constexpr result_err get_frame_read(Reg reg, MemA member_a, MemB member_b) {
		using Register = std::decay_t<decltype(storage.*reg)>;
		uint32_t start_str = to_uint_ptr(member_a);
		uint32_t end_str = to_uint_ptr(member_b) + sizeof(storage.*reg.*member_b);
		uint32_t start_reg = start_str / sizeof(uint16_t) + Register::OFFSET;
		uint8_t *str_addr = reinterpret_cast<uint8_t*>(&(storage.*reg));
		register_t reg_type = type_to_register<Layout, Register>();
		return _get_frame_read_req(reg_type, start_reg, (end_str - start_str) / sizeof(uint16_t));
	}
	template<typename Reg, typename Mem>
	requires IsMemberChain<Layout, Reg, Mem> && HasOffset<mem_t<Layout, Reg>>
	constexpr result_err get_frame_read(Reg reg, Mem mem) { return get_frame_read<Reg, Mem, Mem>(reg, mem, mem); };

	template<typename Reg>
	requires IsBitsRegisters<Layout, Reg>
	constexpr result_err get_frame_read(Reg start_bit, Reg end_bit) { 
		std::span<uint8_t> start_bytes = to_byte_span(start_bit);
		std::span<uint8_t> end_bytes = to_byte_span(end_bit);
		if (1 != popcount(start_bytes))
			return {.err = "EXACTLY_1_BIT_HAS_TO_BE_SET_IN_START_BIT"};
		if (1 != popcount(end_bytes))
			return {.err = "EXACTLY_1_BIT_HAS_TO_BE_SET_IN_END_BIT"};
		uint32_t start_byte = std::ranges::find_if(start_bytes, [](uint8_t e){ return e != 0; }) - start_bytes.begin();
		uint32_t end_byte = std::ranges::find_if(end_bytes, [](uint8_t e){ return e != 0; }) - end_bytes.begin();
		uint32_t start_str = start_byte * 8 + std::countr_zero(start_bytes[start_byte]);
		uint32_t end_str = end_byte * 8 + std::countl_zero(end_bytes[end_byte]) + 1;
		uint32_t start_reg = start_str + Reg::OFFSET;
		register_t reg_type = type_to_register<Layout, Reg>();
		return _get_frame_read_req(reg_type, start_reg, end_str - start_str);
	}

	template<typename Reg>
	requires IsBitsRegisters<Layout, Reg>
	constexpr result_err get_frame_read(const Reg &read_mask) { return get_frame_read(read_mask, read_mask); }

	template<typename Reg, typename MemA, typename MemB>
	requires IsMemberChain<Layout, Reg, MemA> && IsMemberChain<Layout, Reg, MemB> && HasOffset<mem_t<Layout, Reg>>
	constexpr result_err get_frame_write(Reg reg, MemA member_a, MemB member_b) {
		using Register = std::decay_t<decltype(storage.*reg)>;
		register_t type = type_to_register<Layout, Register>();
		uint32_t start_str = to_uint_ptr(member_a);
		uint32_t end_str = to_uint_ptr(member_b) + sizeof(storage.*reg.*member_b);
		uint32_t start_reg = start_str / sizeof(uint16_t) + Register::OFFSET;
		uint8_t *str_addr = reinterpret_cast<uint8_t*>(&(storage.*reg));
		return _get_frame_write_req(type, start_reg, std::span<uint8_t>{str_addr + start_str, str_addr + end_str});
	}

	template<typename Reg, typename Mem>
	requires IsMemberChain<Layout, Reg, Mem> && HasOffset<mem_t<Layout, Reg>>
	constexpr result_err get_frame_write(Reg reg, Mem mem) { return get_frame_write<Reg, Mem, Mem>(reg, mem, mem); }

	template<typename Reg>
	requires IsBitsRegisters<Layout, Reg>
	constexpr result_err get_frame_write(Reg start_bit, Reg end_bit) { 
		std::span<uint8_t> start_bytes = to_byte_span(start_bit);
		std::span<uint8_t> end_bytes = to_byte_span(end_bit);
		if (1 != popcount(start_bytes))
			return {.err = "EXACTLY_1_BIT_HAS_TO_BE_SET_IN_START_BIT"};
		if (1 != popcount(end_bytes))
			return {.err = "EXACTLY_1_BIT_HAS_TO_BE_SET_IN_END_BIT"};
		uint32_t start_str = std::ranges::find_if(start_bytes, [](uint8_t e){ return e != 0; }) - start_bytes.begin();
		uint32_t end_str = std::ranges::find_if(end_bytes, [](uint8_t e){ return e != 0; }) - start_bytes.begin() + 1;
		uint32_t start_reg = start_str + Reg::OFFSET;
		uint8_t *str_addr = reinterpret_cast<uint8_t*>(&get_register_ref<Layout, Reg>(storage));
		return _get_frame_write_req(start_reg, std::span<uint8_t>{str_addr + start_str, str_addr + end_str});
	}

	template<typename Reg>
	requires IsBitsRegisters<Layout, Reg>
	constexpr result_err get_frame_write(const Reg &read_mask) { return get_frame_write(read_mask, read_mask); }

	template<typename Reg, typename Mem, typename MemT = mem_t<mem_t<Layout, Reg>, Mem>>
	requires IsMemberChain<Layout, Reg, Mem>
	constexpr MemT read(Reg src_reg, Mem src) {
		MemT res{};
		swap_byte_order<MemT>{}(storage.*src_reg.*src, res);
		return res;
	}

	template<typename Reg, typename Mem, typename MemT = mem_t<mem_t<Layout, Reg>, Mem>>
	requires IsMemberChain<Layout, Reg, Mem>
	constexpr void write(const MemT &src, Reg dst_reg, Mem dst) {
		swap_byte_order<MemT>{}(src, storage.*dst_reg.*dst);
	}


	// ---------------------------------------------------------------------------------------
	// Internal frame fill functions
	// ---------------------------------------------------------------------------------------
	// Client
	#define RES_ERR_ASSERT(cond, msg) if (cond != OK) return {.err = msg}
	#define RES_FORWARD(stm) if (result r = stm; r != OK) return {.err = r}
	constexpr result_err _get_frame_read_req(register_t reg_type, uint32_t reg_offset, uint32_t reg_count) {
		switch (reg_type) {
			case register_t::BITS:        RES_FORWARD(buffer.write_fc(function_code::READ_COILS)); break;
			case register_t::BITS_WRITE:  RES_FORWARD(buffer.write_fc(function_code::READ_DISCRETE_INPUTS)); break;
			case register_t::HALFS:       RES_FORWARD(buffer.write_fc(function_code::READ_HOLDING_REGISTERS)); break;
			case register_t::HALFS_WRITE: RES_FORWARD(buffer.write_fc(function_code::READ_INPUT_REGISTERS)); break;
		}

		RES_ERR_ASSERT(buffer.write_data(h_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		RES_ERR_ASSERT(buffer.write_data(l_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		RES_ERR_ASSERT(buffer.write_data(h_byte(reg_count)), "WRITE_REG_COUNT_ERR");
		RES_ERR_ASSERT(buffer.write_data(l_byte(reg_count)), "WRITE_REG_COUNT_ERR");
		if (buffer.is_ascii()) {
			return {.err = "ASCII not implemented yet"};
		} else {
			uint16_t crc = checksum::calculate_crc16(buffer.frame_data.span());
			RES_FORWARD(buffer.write_checksum(crc));
		}
		if (buffer.tcp_header) {
			buffer.tcp_header->length = buffer.frame_data.size() - sizeof(*buffer.tcp_header);
		}
		lc = get_last_completed();
		return {buffer.frame_data.span()};
	}
	constexpr result_err _get_frame_write_req(register_t reg_type, uint32_t reg_offset, std::span<uint8_t> data) {
		switch (reg_type) {
			case register_t::BITS:        return {.err = "BITS_NOT_ALLOWED"};
			case register_t::BITS_WRITE:  
				if (data.size() == 1) {
					RES_FORWARD(buffer.write_fc(function_code::WRITE_SINGLE_COIL)); 
				}
				else {
					RES_FORWARD(buffer.write_fc(function_code::WRITE_MULTIPLE_COILS)); 
				}
				break;
			case register_t::HALFS:       return {.err = "HALFS_NOT_ALLOWED"};
			case register_t::HALFS_WRITE:
				if (data.size() == 2) {
					RES_FORWARD(buffer.write_fc(function_code::WRITE_SINGLE_REGISTER));
				}
				else {
					RES_FORWARD(buffer.write_fc(function_code::WRITE_MULTIPLE_REGISTERS));
				}
				break;
			default: return {.err = "INVALID_REGISTER_TYPE"};
		}
		RES_ERR_ASSERT(buffer.write_data(h_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		RES_ERR_ASSERT(buffer.write_data(l_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		if (function_code(*buffer.fc) == function_code::WRITE_MULTIPLE_COILS || 
			function_code(*buffer.fc) == function_code::WRITE_MULTIPLE_REGISTERS) {
			uint16_t reg_count = data.size() / 2;
			RES_ERR_ASSERT(buffer.write_data(h_byte(reg_count)), "WRITE_REG_COUNT_ERR");
			RES_ERR_ASSERT(buffer.write_data(l_byte(reg_count)), "WRITE_REG_COUNT_ERR");
			RES_ERR_ASSERT(buffer.write_data(data.size()), "WRITE_BYTE_SIZE_ERR");
		}
		RES_ERR_ASSERT(buffer.write_data(data), "WRITE_DATA_ERR");

		if (buffer.is_ascii()) {
			return {.err = "ASCII not implemented yet"};
		} else {
			uint16_t crc = checksum::calculate_crc16(buffer.frame_data.span());
			RES_FORWARD(buffer.write_checksum(crc));
		}
		if (buffer.tcp_header) {
			swap_byte_order<uint16_t>{}(buffer.frame_data.size() - sizeof(*buffer.tcp_header), 
				buffer.tcp_header->length);
		}
		lc = get_last_completed();
		return {buffer.frame_data.span()};
	}
	// Server
	constexpr result_err _get_frame_read_res();
	constexpr result_err _get_frame_write_res();

	constexpr result_err _process(uint8_t b) {
		result r = buffer.process(b);
		if (r != OK) {
			buffer.clear();
			return {.err = r};
		}
		if (r == OK && buffer.cur_state != modbus_frame<MAX_SIZE>::state::FINAL)
			return {.err = IN_PROGRESS};
		uint16_t reg_offset = (l_byte(lc.i1) << 8) | h_byte(lc.i1);
		uint16_t reg_count = (l_byte(lc.i2) << 8) | h_byte(lc.i2);
		last_completed response_lc = get_last_completed();
		// modbus client
		if constexpr (addr == 0) { 
			// validation checks
			bool valid = true;
			bool is_bit{};
			switch(lc.fc) {
				case function_code::WRITE_SINGLE_COIL:
				case function_code::WRITE_SINGLE_REGISTER:
					valid = response_lc == lc;
					break;
				case function_code::READ_COILS:
				case function_code::READ_DISCRETE_INPUTS:
					is_bit = true;
				case function_code::READ_HOLDING_REGISTERS:
				case function_code::READ_INPUT_REGISTERS:
					valid = lc.addr == response_lc.addr && lc.fc == response_lc.fc &&
						is_bit ? (reg_count + 7) / 8 == l_byte(response_lc.i1): reg_count * 2 == l_byte(response_lc.i1);
			}
			if (!valid) {
				buffer.clear();
				return {.err = INVALID_RESPONSE};
			}
			// data extraction
			switch(lc.fc) {
			case function_code::READ_COILS:
				if constexpr (!HasBits<Layout>) {
					buffer.clear();
					return {.err = "LAYOUT_HAS_NO_BITS"};
				} else {
				// TODO: implement
				}
				break;
			case function_code::READ_DISCRETE_INPUTS:
				if constexpr (!HasWriteBits<Layout>) {
					buffer.clear();
					return {.err = "LAYOUT_HAS_NO_WRITE_BITS"};
				} else {
				// TODO: implement
				storage.write_bits_registers;
				}
				break;
			case function_code::READ_HOLDING_REGISTERS:
				if constexpr (!HasHalfs<Layout>) {
					buffer.clear();
					return {.err = "LAYOUT_HAS_NO_HALFS"};
				} else {
				constexpr int START = decltype(storage.halfs_registers)::OFFSET;
				constexpr int END = START + sizeof(storage.halfs_registers);
				if (reg_offset < START || reg_offset + reg_count > END) {
					buffer.clear();
					return {.err = "REG_OUT_OF_BOUNDS"};
				}
				if (!buffer.data || !buffer.byte_count) {
					buffer.clear();
					return {.err = "INCOMPLETE_RESPONSE"};
				}
				std::copy_n(buffer.data, *buffer.byte_count, 
					reinterpret_cast<uint8_t*>(&storage.halfs_registers) + (reg_offset - START) * 2);
				}
				break;
			case function_code::READ_INPUT_REGISTERS:
				if constexpr (!HasWriteHalfs<Layout>) {
					buffer.clear();
					return {.err = "LAYOUT_HAS_NO_WRITE_HALFS"};
				} else {
				constexpr int START = decltype(storage.halfs_write_registers)::OFFSET;
				constexpr int END = START + sizeof(storage.halfs_write_registers);
				if (reg_offset < START || reg_offset + reg_count > END) {
					buffer.clear();
					return {.err = "REG_OUT_OF_BOUNDS"};
				}
				if (!buffer.data || !buffer.byte_count) {
					buffer.clear();
					return {.err = "INCOMPLETE_RESPONSE"};
				}
				std::copy_n(buffer.data, *buffer.byte_count, 
					reinterpret_cast<uint8_t*>(&storage.halfs_write_registers) + (reg_offset - START) * 2);
				}
				break;
			}
		} else {
			// validation checks
			if (response_lc.addr != addr) {
				buffer.clear();
				return {.err = WRONG_ADDR};
			}
		}
		lc = response_lc;
		
		return {.res = buffer.frame_data.span()};
	}

	last_completed get_last_completed() {
		uint16_t *reg = reinterpret_cast<uint16_t*>(buffer.frame_data.begin());
		return last_completed{
			.addr = buffer.addr ? *buffer.addr: uint8_t(0),
			.fc = buffer.fc ? function_code(*buffer.fc): function_code::NONE,
			.i1 = reg[1],
			.i2 = reg[2],
			.crc = *(reinterpret_cast<uint16_t*>(buffer.frame_data.end()) - 1),
		};
	}
	#undef RES_ERR_ASSERT
};

}

