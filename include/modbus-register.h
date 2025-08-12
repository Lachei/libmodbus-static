#pragma once

#include "common.h"
#include <cstring>
#include <array>
#include <ranges>
#include <cstdint>

namespace libmodbus_static {
template<int N>
using mod_string = std::array<char, N>;
template<typename T>
std::string_view to_string_view(const T &s) {
	std::string_view r{s.begin(), s.end()};
	return r.substr(0, r.find_first_of('\0'));
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
struct result_err {
	std::span<uint8_t> res{};
	std::string_view err{OK};
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
template<typename S>
concept HasOffset = requires(S s) { std::is_same_v<decltype(s.OFFSET), int>; };

#define MEMBER_CHAIN(l, r, m) (&l::r##_registers), (&l::r##_layout::m)

template<typename Layout, int MAX_SIZE = 256, int SingletonInstannce = 0>
struct modbus_register {
	static modbus_register& Default(uint8_t addr = 0) { static modbus_register r{.addr = addr}; return r; }

	uint8_t addr{};
	Layout storage{}; // do not access directly, contains data already byte swapped, read and write with read() and write()
	modbus_frame<MAX_SIZE> buffer{};

	// Does all the magic
	// processes incoming data from any source and returns a non_empty span
	// if result frame is ready to be sent
	// if client is waiting for the result the result can be checked against ACK:
	// if (process(byte) == ACK)
	//	... done waiting
	constexpr result_err process_rtu(uint8_t b, uint64_t cur_t) {
		result r = _process(b, cur_t);
		if (r == OK && buffer.cur_state == modbus_frame<MAX_SIZE>::state::FINAL)
			return {.res = buffer.frame_data.sv()};
		if (r == OK)
			return {.err = IN_PROGRESS};
		return {.err = r};
	}
	constexpr result_err process_ascii(uint8_t b, uint64_t cur_t) {
		return {.err = "NOT_YET_IMPLEMENTED"};
	}
	constexpr result_err process_tcp(uint8_t b, uint64_t cur_t) {
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
		result r = _process(b, cur_t);
		if (r == OK && buffer.cur_state == modbus_frame<MAX_SIZE>::state::FINAL)
			return {.res = buffer.frame_data.sv()};
		if (r == OK)
			return {.err = IN_PROGRESS};
		return {.err = r};
	}

	constexpr result start_rtu_frame(uint8_t addr) {
		buffer.clear();
		buffer.write_addr(addr);
	}
	constexpr result start_ascii_frame(uint8_t addr) {
		buffer.clear();
		buffer.write_ascii_start();
		buffer.write_addr(addr);
	}
	constexpr result start_tcp_frame(uint16_t trans, uint8_t addr) {
		buffer.clear();
		typename modbus_frame<MAX_SIZE>::mbap_header header{.transactionId = trans, .addreess = addr};
		buffer.write_mbap(header);
	}

	template<typename Reg, typename MemA, typename MemB>
	requires IsMemberChain<Layout, Reg, MemA> && IsMemberChain<Layout, Reg, MemB> && HasOffset<mem_t<Layout, Reg>>
	constexpr std::span<uint8_t> get_frame_read(Reg reg, MemA member_a, MemB member_b) {
		uint32_t start_str = std::uintptr_t(reg) + std::uintptr_t(member_a);
		uint32_t end_str = std::uintptr_t(reg) + std::uintptr_t(member_b) + sizeof(storage.*reg.*member_b);
		uint32_t start_reg = start_str + storage.*reg.OFFSET;
		uint8_t *str_addr = reinterpert_cast<uint8_t>(&storage);
		return _get_frame_read_req(start_reg, std::span<uint8_t>{storage + start_str, storage + end_str});
	}
	template<typename Reg, typename Mem>
	requires IsMemberChain<Layout, Reg, Mem> && HasOffset<mem_t<Layout, Reg>>
	constexpr result_err get_frame_read(Reg reg, Mem mem) { return get_frame_read<Reg, Mem, Mem>(reg, mem, mem); };

	template<typename Reg, typename MemA, typename MemB>
	requires IsMemberChain<Layout, Reg, MemA> && IsMemberChain<Layout, Reg, MemB> && HasOffset<mem_t<Layout, Reg>>
	constexpr result_err get_frame_write(Reg reg, MemA mem_a, MemB mem_b);

	template<typename Reg, typename MemA, typename Mem>
	requires IsMemberChain<Layout, Reg, Mem> && HasOffset<mem_t<Layout, Reg>>
	constexpr result_err get_frame_write(Reg reg, Mem mem) { return get_frame_write<Reg, Mem, Mem>(reg, mem, mem); }

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
	// Internal process functions
	// ---------------------------------------------------------------------------------------
	result_err _process(uint8_t b, uint64_t cur_t) {
		switch (buffer.cur_state) {
		case modbus_frame<MAX_SIZE>::state::WRITE_ADDR_START_MBAP:
		case modbus_frame<MAX_SIZE>::state::WRITE_ADDR:
			return buffer.write_addr(b);
		case modbus_frame<MAX_SIZE>::state::WRITE_FC:
			return buffer.write_fc(b);
		case modbus_frame<MAX_SIZE>::state::WRITE_DATA:
		case modbus_frame<MAX_SIZE>::state::WRITE_CRC:
		case modbus_frame<MAX_SIZE>::state::FINALWRITE_CRC: break;
		}
		return {};
	}

	// ---------------------------------------------------------------------------------------
	// Internal frame fill functions
	// ---------------------------------------------------------------------------------------
	#define RES_ERR_ASSERT(cond, msg) if (!cond) return {.err = msg}
	#define RES_FORWARD(stm) if (result r = stm; r != OK) return {.err = r}
	constexpr result_err _get_frame_read_req(register_t reg_type, uint32_t reg_offset, uint32_t reg_count) {
		auto &mf = buffer.frame_data;
		switch (reg_type) {
			case register_t::BITS:        RES_FORWARD(buffer.write_fc(function_code::READ_COILS)); break;
			case register_t::BITS_WRITE:  RES_FORWARD(buffer.write_fc(function_code::READ_DISCRETE_INPUTS)); break;
			case register_t::HALFS:       RES_FORWARD(buffer.write_fc(function_code::READ_HOLDING_REGISTERS)); break;
			case register_t::HALFS_WRITE: RES_FORWARD(buffer.write_fc(function_code::READ_INPUT_REGISTERS)); break;
		}

		RES_ERR_ASSERT(mf.push(h_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		RES_ERR_ASSERT(mf.push(l_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		RES_ERR_ASSERT(mf.push(h_byte(reg_count)), "WRITE_REG_COUNT_ERR");
		RES_ERR_ASSERT(mf.push(l_byte(reg_count)), "WRITE_REG_COUNT_ERR");
		if (mf[0] == ':') {
			return {.err = "ASCII not implemented yet"};
		} else {
			uint16_t crc = checksum::calculate_crc16(mf.span());
			RES_FORWARD(buffer.write_crc(crc));
		}
		if (buffer.tcp_header) {
			buffer.tcp_header->length = mf.size() - sizeof(*buffer.tcp_header);
		}
		return mf.span();
	}
	constexpr result_err _get_frame_write_req(register_t reg_type, uint32_t reg_offset, std::span<uint8_t> data) {
		auto &mf = buffer.frame_data;
		switch (reg_type) {
			case register_t::BITS:        return {.err = "BITS_NOT_ALLOWED"};
			case register_t::BITS_WRITE:  
				if (data.size() == 1)
					RES_FORWARD(buffer.write_fc(function_code::WRITE_SINGLE_COIL)); 
				else
					RES_FORWARD(buffer.write_fc(function_code::WRITE_MULTIPLE_COILS)); 
				break;
			case register_t::HALFS:       return {.err = "HALFS_NOT_ALLOWED"};
			case register_t::HALFS_WRITE:
				if (data.size() == 2)
					RES_FORWARD(buffer.write_fc(function_code::WRITE_SINGLE_REGISTER));
				else
					RES_FORWARD(buffer.write_fc(function_code::WRITE_MULTIPLE_REGISTERS));
				break;
		}
		RES_ERR_ASSERT(mf.push(h_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		RES_ERR_ASSERT(mf.push(l_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		if (*buffer.fc == function_code::WRITE_MULTIPLE_COILS || 
			*buffer.fc == function_code::WRITE_MULTIPLE_REGISTERS) {
			uint16_t reg_count = data.size() / 2;
			RES_ERR_ASSERT(mf.push(h_byte(reg_count)), "WRITE_REG_COUNT_ERR");
			RES_ERR_ASSERT(mf.push(l_byte(reg_count)), "WRITE_REG_COUNT_ERR");
			RES_ERR_ASSERT(mf.push(data.size()), "WRITE_BYTE_SIZE_ERR");
		}
		for (uint8_t b: data)
			RES_ERR_ASSERT(mf.push(b), "WRITE_DATA_ERR");

		if (mf[0] == ':') {
			return {.err = "ASCII not implemented yet"};
		} else {
			uint16_t crc = checksum::calculate_crc16(mf.span());
			RES_FORWARD(buffer.write_crc(crc));
		}
		if (buffer.tcp_header) {
			buffer.tcp_header->length = mf.size() - sizeof(*buffer.tcp_header);
		}
		return mf.span();
	}
	constexpr result_err _get_frame_read_res();
	constexpr result_err _get_frame_write_res();
	#undef RES_ERR_ASSERT
};

}

