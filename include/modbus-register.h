#pragma once

#include "common.h"
#include <cstring>
#include <array>
#include <ranges>
#include <cstdint>
#include <bit>

//TODO: remove
#include <iostream>
#include <bitset>

namespace libmodbus_static {

constexpr std::string_view IN_PROGRESS{"IN_PROGRESS"};
constexpr std::string_view INVALID_RESPONSE{"RESPONSE_FROM_SERVER_INVALID"};
constexpr std::string_view WRONG_ADDR{"WRONG_ADDR"};
constexpr std::string_view REGISTER_NOT_FULLY_COVERED{"REGISTER_NOT_FULLY_COVERED"};
constexpr std::string_view BITS_NOT_FULLY_COVERED{"BITS_NOT_FULLY_COVERED"};

template<int N>
using mod_string = std::array<char, N>;
template<typename T>
constexpr std::string_view to_string_view(const T &s) {
	std::string_view r{s.begin(), s.end()};
	return r.substr(0, r.find_first_of('\0'));
}

template <typename T>
constexpr std::span<const uint8_t> to_byte_span(T& t) {
	return std::span<const uint8_t>{ reinterpret_cast<const uint8_t*>(&t), sizeof(t) };
}

constexpr uint32_t popcount(std::span<const uint8_t> bytes) {
	uint32_t bitcount{};
	for (uint8_t b: bytes)
		bitcount += std::popcount(b);
	return bitcount;
}

template<typename T>
constexpr uintptr_t to_uint_ptr(T member) {
	return *reinterpret_cast<uintptr_t*>(&member);
}

// byte swapping for little endian systems
template<typename T>
struct swap_byte_order {
	constexpr void operator()(T src, T& dst) const { 
		constexpr int s{sizeof(T)};
		for (int byte: std::ranges::iota_view{0, s})
			std::memcpy(reinterpret_cast<uint8_t*>(&dst) + (s - 1 - byte), 
				reinterpret_cast<uint8_t*>(&src) + byte, sizeof(uint8_t)); 
	}
};
// no byte swapping for byte sequences and if system is big endian (same ordering as modbus bytes)
template<typename T>
concept IsByteSequence = requires (T t) { sizeof(*t.data()) == 1; t = t; };
template<typename T> 
requires IsByteSequence<T> || (std::endian::native == std::endian::big)
struct swap_byte_order<T> {
	constexpr void operator()(const T &src, T &dst) const {
		dst = src;
	}
};
template<typename T>
constexpr T to_hb_first(T e) {
	T res{};
	swap_byte_order<T>{}(e, res);
	return res;
}
template<typename T>
constexpr static result is_register_covered(uint32_t reg_offset, uint32_t reg_count) {
	constexpr int START = T::OFFSET;
	constexpr int END = START + sizeof(T) / 2;
	if (reg_offset < START || END < reg_offset + reg_count)
		return REGISTER_NOT_FULLY_COVERED;
	return OK;
}
template<typename T>
constexpr static result is_bit_covered(uint32_t reg_offset, uint32_t reg_count) {
	constexpr int START = T::OFFSET;
	constexpr int END = START + sizeof(T) * 8;
	if (reg_offset < START || END < reg_offset + reg_count)
		return BITS_NOT_FULLY_COVERED;
	return OK;
}
template<typename T>
constexpr static uint8_t* get_start_addr(T &r, uint32_t reg_offset) {
	constexpr int START = T::OFFSET;
	return reinterpret_cast<uint8_t*>(&r) + (reg_offset - START) * 2;
}
template<typename T>
constexpr static uint8_t* get_bit_start_addr(T &r, uint32_t reg_offset) {
	constexpr int START = T::OFFSET;
	return reinterpret_cast<uint8_t*>(&r) + (reg_offset - START) / 8;
}
template<typename T>
constexpr static void read_bits_from_storage(T &bits, int reg_offset, int reg_count, uint8_t *dst) {
	uint8_t *data = reinterpret_cast<uint8_t*>(&bits);
	int start_bit = reg_offset - std::decay_t<decltype(bits)>::OFFSET;
	for (int cur_bit = start_bit; cur_bit < (start_bit + reg_count); cur_bit += 8) {
		uint8_t i = cur_bit / 8;
		uint8_t r = cur_bit % 8;
		uint8_t s = reg_count - (cur_bit - start_bit);
		uint8_t t = s < 8 ? ((1 << s) - 1): 0xff;
		uint8_t byte = (data[i] >> r) & t;
		if (r && s > r)
			byte |= (data[i + 1] << (8 - r)) & t;
		*dst++ = byte;
	}
}
template<typename T>
constexpr static void write_bits_to_storage(T &bits, int reg_offset, int reg_count, uint8_t *data) {
	constexpr int START = T::OFFSET;
	uint8_t *dst = reinterpret_cast<uint8_t*>(&bits);
	for (int cur_bit = reg_offset - START, cur_data = 0; 
		cur_bit < reg_offset - START + reg_count; 
		cur_bit += 8, ++cur_data) {
		// clear bits to be written via or
		dst[cur_bit / 8] &= 0xff >> (8 - (cur_bit % 8));
		dst[cur_bit / 8] |= data[cur_data] << (cur_bit % 8);
		if (cur_bit % 8) {
			dst[cur_bit / 8 + 1] &= 0xff << (cur_bit % 8);
			dst[cur_bit / 8 + 1] |= data[cur_data] >> (8 - (cur_bit % 8));
		}
	}
}

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

template<typename L, typename Reg>
concept IsBitsRegister = requires(L l, Reg b) { l.bits_registers = b; };
template<typename L, typename Reg>
concept IsBitsWriteRegister = requires(L l, Reg b) { l.bits_write_registers = b; };
template<typename L, typename Mp>
concept IsHalfsRegister = requires(L l, Mp m) { l.halfs_registers.*m; };
template<typename L, typename Mp>
concept IsHalfsWriteRegister = requires(L l, Mp m) { l.halfs_write_registers.*m; };
template<typename L, typename Mp>
concept IsValidRegister = IsHalfsRegister<L, Mp> || IsHalfsWriteRegister<L, Mp>;

template<typename L, typename Mp> requires IsBitsRegister<L, Mp>
constexpr decltype(std::declval<L>().bits_registers) RegisterTypeH();
template<typename L, typename Mp> requires IsBitsWriteRegister<L, Mp>
constexpr decltype(std::declval<L>().bits_write_registers) RegisterTypeH();
template<typename L, typename Mp> requires IsHalfsRegister<L, Mp>
constexpr decltype(std::declval<L>().halfs_registers) RegisterTypeH();
template<typename L, typename Mp> requires IsHalfsWriteRegister<L, Mp>
constexpr decltype(std::declval<L>().halfs_write_registers) RegisterTypeH();
template<typename L, typename Mp>
using RegisterType = std::decay_t<decltype(RegisterTypeH<L, Mp>())>;
template<typename L, typename Mp>
using MemberType = std::decay_t<decltype(std::declval<RegisterType<L,Mp>>().*std::declval<Mp>())>;

template<typename L, typename Mp> requires IsBitsRegister<L, Mp>
constexpr uint32_t OFFSET() { return decltype(std::declval<L>().bits_registers)::OFFSET; }
template<typename L, typename Mp> requires IsBitsWriteRegister<L, Mp>
constexpr uint32_t OFFSET() { return decltype(std::declval<L>().bits_write_registers)::OFFSET; }
template<typename L, typename Mp> requires IsHalfsRegister<L, Mp>
constexpr uint32_t OFFSET() { return decltype(std::declval<L>().halfs_registers)::OFFSET; }
template<typename L, typename Mp> requires IsHalfsWriteRegister<L, Mp>
constexpr uint32_t OFFSET() { return decltype(std::declval<L>().halfs_write_registers)::OFFSET; }

template<typename L, typename Mp> requires IsBitsRegister<L, Mp>
constexpr RegisterType<L, Mp>& register_ref(L &l) { return l.bits_registers; }
template<typename L, typename Mp> requires IsBitsWriteRegister<L, Mp>
constexpr RegisterType<L, Mp>& register_ref(L &l) { return l.bits_write_registers; }
template<typename L, typename Mp> requires IsHalfsRegister<L, Mp>
constexpr RegisterType<L, Mp>& register_ref(L &l) { return l.halfs_registers; }
template<typename L, typename Mp> requires IsHalfsWriteRegister<L, Mp>
constexpr RegisterType<L, Mp>& register_ref(L &l) { return l.halfs_write_registers; }

template<typename L, typename Mp> requires IsBitsRegister<L, Mp>
constexpr register_t type_to_register() { return register_t::BITS; }
template<typename L, typename Mp> requires IsBitsWriteRegister<L, Mp>
constexpr register_t type_to_register() { return register_t::BITS_WRITE; }
template<typename L, typename Mp> requires IsHalfsRegister<L, Mp>
constexpr register_t type_to_register() { return register_t::HALFS; }
template<typename L, typename Mp> requires IsHalfsWriteRegister<L, Mp>
constexpr register_t type_to_register() { return register_t::HALFS_WRITE; }
template<typename L, typename R>
constexpr R& get_register_ref(L &l) { 
	static_assert(false, "The type could not be converted, make sure your layout has the correct members"); 
}
template<typename L, typename R> requires requires (L l, R r) {l.bits_registers = r;}
constexpr R& get_register_ref(L &l) { return l.bits_registers; }
template<typename L, typename R> requires requires (L l, R r) {l.bits_write_registers = r;}
constexpr R& get_register_ref(L &l) { return l.bits_write_registers; }

template<typename Layout, int MAX_SIZE = 256>
struct modbus_register {
	template<int slot = 0>
	static modbus_register& Default(uint8_t address) { static modbus_register r{.addr = address}; return r; }

	const uint8_t addr{};
	Layout storage{};
	modbus_frame<MAX_SIZE> buffer{};
	struct last_completed{
		transport_t transport{};
		uint16_t tcp_tid{};
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
		if (buffer.cur_state == modbus_frame<MAX_SIZE>::state::WRITE_ADDR_START_MBAP) {
			if (buffer.frame_data.size() > int(sizeof(*buffer.tcp_header))) {
				buffer.clear();
				return {.err = "FATAL_TOO_LARGE_SIZE_FOR_TCP_HEADER"};
			}
			if (!buffer.frame_data.push(b)) {
				buffer.clear();
				return {.err = "ERR_WRITE_TCP_HEADER"};
			}
			// done with tcp header recieving
			if (buffer.frame_data.size() == sizeof(*buffer.tcp_header)) {
				buffer.transport = transport_t::TCP;
				buffer.tcp_header = reinterpret_cast<modbus_frame<MAX_SIZE>::mbap_header*>(buffer.frame_data.begin());
				uint16_t l = buffer.tcp_header->length;
				buffer.tcp_header->length = (l_byte(l) << 8) | h_byte(l);
				buffer.cur_state = modbus_frame<MAX_SIZE>::state::WRITE_ADDR;
			}
			return {.err = IN_PROGRESS};
		}
		if (!buffer.tcp_header) {
			buffer.clear();
			return {.err = "FATAL_MISSING_TCP_HEADER_IN_FRAME"};
		}
		if (buffer.frame_data.size() > int(buffer.tcp_header->length + sizeof(*buffer.tcp_header))) {
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
		result r = buffer.write_mbap({.transaction_id = trans});
		if (r != OK)
			return r;
		return buffer.write_addr(addr);
	}
	constexpr std::span<const uint8_t> get_current_frame() const { return buffer.frame_data.span();}

	template<typename MemA, typename MemB>
	requires IsValidRegister<Layout, MemA> && IsValidRegister<Layout, MemB>
	constexpr result_err get_frame_read(MemA member_a, MemB member_b) {
		auto reg = register_ref<Layout, MemA>(storage);
		uint32_t start_str = to_uint_ptr(member_a);
		uint32_t end_str = to_uint_ptr(member_b) + sizeof(reg.*member_b);
		uint32_t start_reg = start_str / sizeof(uint16_t) + OFFSET<Layout, MemA>();
		uint8_t *str_addr = reinterpret_cast<uint8_t*>(&reg);
		register_t reg_type = type_to_register<Layout, MemA>();
		return _get_frame_read_req(reg_type, start_reg, (end_str - start_str) / sizeof(uint16_t));
	}
	template<typename Mem>
	requires IsValidRegister<Layout, Mem>
	constexpr result_err get_frame_read(Mem mem) { return get_frame_read<Mem, Mem>(mem, mem); };

	template<typename Reg>
	requires IsBitsRegisters<Layout, Reg>
	constexpr result_err get_frame_read(const Reg &mask) { 
		std::span<const uint8_t> bytes = to_byte_span(mask);
		if (1 != popcount(bytes) && 2 != popcount(bytes))
			return {.err = "EXACTLY_1_OR_2_BIT_HAS_TO_BE_SET_IN_START_BIT"};
		uint32_t start_byte = std::ranges::find_if(bytes, [](uint8_t e){ return e != 0; }) - bytes.begin();
		uint32_t end_byte{uint32_t(bytes.size()) - 1};
		for (;end_byte >= 0 && bytes[end_byte] == 0; --end_byte);
		uint32_t start_str = start_byte * 8 + std::countr_zero(bytes[start_byte]);
		uint32_t end_str = end_byte * 8 + 8 - std::countl_zero(bytes[end_byte]);
		uint32_t start_reg = start_str + Reg::OFFSET;
		register_t reg_type = type_to_register<Layout, Reg>();
		return _get_frame_read_req(reg_type, start_reg, end_str - start_str);
	}

	template<typename MemA, typename MemB>
	requires IsValidRegister<Layout, MemA> && IsValidRegister<Layout, MemB>
	constexpr result_err get_frame_write(MemA member_a, MemB member_b) {
		auto reg = register_ref<Layout, MemA>(storage);
		register_t type = type_to_register<Layout, MemA>();
		uint32_t start_str = to_uint_ptr(member_a);
		uint32_t end_str = to_uint_ptr(member_b) + sizeof(reg.*member_b);
		uint32_t start_reg = start_str / sizeof(uint16_t) + OFFSET<Layout, MemA>();
		uint8_t *str_addr = reinterpret_cast<uint8_t*>(&reg);
		return _get_frame_write_req(type, start_reg, std::span<uint8_t>{str_addr + start_str, str_addr + end_str});
	}

	template<typename Mem>
	requires IsValidRegister<Layout, Mem>
	constexpr result_err get_frame_write(Mem mem) { return get_frame_write<Mem, Mem>(mem, mem); }

	template<typename Reg>
	requires IsBitsRegisters<Layout, Reg>
	constexpr result_err get_frame_write(const Reg &mask) { 
		std::span<const uint8_t> bytes = to_byte_span(mask);
		if (1 != popcount(bytes) && 2 != popcount(bytes))
			return {.err = "EXACTLY_1_OR_2_BIT_HAS_TO_BE_SET_IN_START_BIT"};
		uint32_t start_byte = std::ranges::find_if(bytes, [](uint8_t e){ return e != 0; }) - bytes.begin();
		uint32_t end_byte{uint32_t(bytes.size()) - 1};
		for (;end_byte >= 0 && bytes[end_byte] == 0; --end_byte);
		uint32_t start_str = start_byte * 8 + std::countr_zero(bytes[start_byte]);
		uint32_t end_str = end_byte * 8 + 8 - std::countl_zero(bytes[end_byte]);
		uint32_t start_reg = start_str + Reg::OFFSET;
		register_t reg_type = type_to_register<Layout, Reg>();
		auto reg = register_ref<Layout, Reg>(storage);
		uint8_t *start_addr = reinterpret_cast<uint8_t*>(&reg);
		return _get_frame_write_req(reg_type, start_reg, std::span<uint8_t>{start_addr, sizeof(reg)}, start_str, end_str - start_str);
	}

	#define RES_ERR_ASSERT(cond, msg) if (cond != OK) {buffer.clear(); return {.err = msg};}
	#define RES_FORWARD(stm) if (result r = stm; r != OK) {buffer.clear(); return {.err = r};}
	#define RES_BOOL_ASSERT(cond, msg) if (!(cond)) {buffer.clear(); return {.err = msg};}
	constexpr result_err get_frame_response() {
		if (buffer.cur_state != modbus_frame<MAX_SIZE>::state::FINAL)
			return {.err = "FRAME_NOT_DONE"};

		// header information
		uint16_t reg_offset = (l_byte(lc.i1) << 8) | h_byte(lc.i1);
		uint16_t reg_count = (l_byte(lc.i2) << 8) | h_byte(lc.i2);
		switch_to_response();
		switch(lc.transport) {
		case transport_t::ASCII: RES_FORWARD(buffer.write_ascii_start()); break;
		case transport_t::TCP: RES_FORWARD(buffer.write_mbap({lc.tcp_tid})); break;
		default: break;
		}
		RES_FORWARD(buffer.write_addr(lc.addr));
		RES_FORWARD(buffer.write_fc(lc.fc));

		// data information
		switch(function_code(*buffer.fc)) {
		case function_code::READ_COILS:
			if constexpr (!HasBits<Layout>) {
				buffer.clear();
				return {.err = "LAYOUT_HAS_NO_BITS"};
			} else {
				RES_FORWARD(is_bit_covered<decltype(storage.bits_registers)>(reg_offset, reg_count));
				uint16_t n_bytes = (reg_count + 7) / 8;
				RES_FORWARD(buffer.write_length(n_bytes));
				uint8_t *dst = buffer.frame_data.end();
				RES_FORWARD(buffer.write_data(std::span<uint8_t>((uint8_t*)nullptr, n_bytes)));
				read_bits_from_storage(storage.bits_registers, reg_offset, reg_count, dst);
			}
			break;
		case function_code::READ_DISCRETE_INPUTS:
			if constexpr (!HasWriteBits<Layout>) {
				buffer.clear();
				return {.err = "LAYOUT_HAS_NO_BITS"};
			} else {
				RES_FORWARD(is_bit_covered<decltype(storage.bits_write_registers)>(reg_offset, reg_count));
				uint16_t n_bytes = (reg_count + 7) / 8;
				RES_FORWARD(buffer.write_length(n_bytes));
				uint8_t *dst = buffer.frame_data.end();
				RES_FORWARD(buffer.write_data(std::span<uint8_t>((uint8_t*)nullptr, n_bytes)));
				read_bits_from_storage(storage.bits_write_registers, reg_offset, reg_count, dst);
			}
			break;
		case function_code::READ_HOLDING_REGISTERS:
			if constexpr (!HasHalfs<Layout>) {
				buffer.clear();
				return {.err = "LAYOUT_HAS_NO_HALFS"};
			} else {
				RES_FORWARD(is_bit_covered<decltype(storage.halfs_registers)>(reg_offset, reg_count));
				RES_FORWARD(buffer.write_length(reg_count * 2));
				uint8_t *start_addr = get_start_addr(storage.halfs_registers, reg_offset);
				RES_FORWARD(buffer.write_data(std::span<uint8_t>{start_addr, reg_count * 2u}));
			}
			break;
		case function_code::READ_INPUT_REGISTERS:
			if constexpr (!HasWriteHalfs<Layout>) {
				buffer.clear();
				return {.err = "LAYOUT_HAS_NO_WRITE_HALFS"};
			} else {
				RES_FORWARD(is_register_covered<decltype(storage.halfs_write_registers)>(reg_offset, reg_count));
				RES_FORWARD(buffer.write_length(reg_count * 2));
				uint8_t *start_addr = get_start_addr(storage.halfs_write_registers, reg_offset);
				RES_FORWARD(buffer.write_data(std::span<uint8_t>{start_addr, reg_count * 2u}));
			}
			break;
		case function_code::WRITE_SINGLE_COIL:
			if constexpr (!HasWriteBits<Layout>) {
				buffer.clear();
				return {.err = "LAYOUT_HAS_NO_WRITE_HALFS"};
			} else {
				RES_FORWARD(is_bit_covered<decltype(storage.bits_write_registers)>(reg_offset, 1));
				RES_BOOL_ASSERT(buffer.data, "MISSING_DATA_IN_FRAME");
				int bit = reg_offset - decltype(storage.bits_write_registers)::OFFSET;
				bool bit_on{};
				switch ((*buffer.data << 8) | (*(buffer.data + 1))) {
					case 0xff00: bit_on = true; break;
					case 0x0000: bit_on = true; break;
					default: buffer.clear(); return {.err = "INVALID_COIL_WRITE_DATA"};
				}
				uint8_t *data = reinterpret_cast<uint8_t*>(&storage.bits_write_registers);
				if (bit_on)
					data[bit / 8] |= 1 << (bit % 8);
				else
					data[bit / 8] &= ~(1 << (bit % 8));
			}
			break;
		case function_code::WRITE_SINGLE_REGISTER:
			if constexpr (!HasWriteHalfs<Layout>) {
				buffer.clear();
				return {.err = "LAYOUT_HAS_NO_WRITE_HALFS"};
			} else {
				RES_FORWARD(is_register_covered<decltype(storage.halfs_write_registers)>(reg_offset, 1));
				RES_BOOL_ASSERT(buffer.data, "MISSING_DATA_IN_FRAME");
				uint8_t *start_addr = get_start_addr(storage.halfs_write_registers, reg_offset);
				std::copy_n(buffer.data, 2, start_addr);
			}
			break;
		case function_code::WRITE_MULTIPLE_COILS:
			if constexpr (!HasWriteBits<Layout>) {
				buffer.clear();
				return {.err = "LAYOUT_HAS_NO_WRITE_HALFS"};
			} else {
				RES_FORWARD(is_bit_covered<decltype(storage.bits_registers)>(reg_offset, reg_count));
				RES_BOOL_ASSERT(buffer.data && buffer.byte_count, "MISSING_DATA_IN_FRAME");
				write_bits_to_storage(storage.bits_registers, reg_offset, reg_count, buffer.data);
			}
			break;
		case function_code::WRITE_MULTIPLE_REGISTERS:
			if constexpr (!HasWriteHalfs<Layout>) {
				buffer.clear();
				return {.err = "LAYOUT_HAS_NO_WRITE_HALFS"};
			} else {
				RES_FORWARD(is_register_covered<decltype(storage.halfs_write_registers)>(reg_offset, reg_count));
				RES_BOOL_ASSERT(buffer.data, "MISSING_DATA_IN_FRAME");
				uint8_t *start_addr = get_start_addr(storage.halfs_write_registers, reg_offset);
				std::copy_n(buffer.data, reg_count * 2, start_addr);
			}
			break;
		default: break;
		}

		// footer (crc) information
		switch(lc.transport) {
		case transport_t::RTU: RES_FORWARD(buffer.write_checksum(checksum::calculate_crc16(buffer.frame_data.span()))); break;
		case transport_t::TCP: swap_byte_order<uint16_t>{}(buffer.frame_data.size() - sizeof(*buffer.tcp_header), buffer.tcp_header->length); break;
		case transport_t::ASCII: RES_BOOL_ASSERT(false, "NOT_IMPLEMENTED");
		default: break;
		}

		return {buffer.frame_data.span()};
	}
	constexpr result_err get_frame_error_response(result err) {
		switch_to_response();
		buffer.t.EXCEPTION = true;
		switch(lc.transport) {
		case transport_t::ASCII: RES_FORWARD(buffer.write_ascii_start()); break;
		case transport_t::TCP: RES_FORWARD(buffer.write_mbap({lc.tcp_tid})); break;
		default: break;
		}
		RES_FORWARD(buffer.write_addr(lc.addr));
		RES_FORWARD(buffer.write_fc(lc.fc));
		if (err == REGISTER_NOT_FULLY_COVERED) {
			RES_FORWARD(buffer.write_ec(exception_code::ILLEGAL_DATA_ADDRESS));
		} else {
			RES_FORWARD(buffer.write_ec(exception_code::SLAVE_DEVICE_FAILURE));
		}

		// footer (crc) information
		switch(lc.transport) {
		case transport_t::RTU: RES_FORWARD(buffer.write_checksum(checksum::calculate_crc16(buffer.frame_data.span()))); break;
		case transport_t::TCP: swap_byte_order<uint16_t>{}(buffer.frame_data.size() - sizeof(*buffer.tcp_header), buffer.tcp_header->length); break;
		case transport_t::ASCII: RES_BOOL_ASSERT(false, "NOT_IMPLEMENTED");
		default: break;
		}

		return {buffer.frame_data.span()};
	}

	template<typename Mem, typename MemT = MemberType<Layout, Mem>>
	requires IsValidRegister<Layout, Mem>
	constexpr MemT read(Mem src) {
		MemT res{};
		swap_byte_order<MemT>{}(register_ref<Layout, Mem>(storage).*src, res);
		return res;
	}

	template<typename Mem, typename MemT = MemberType<Layout, Mem>>
	requires IsValidRegister<Layout, Mem>
	constexpr void write(const MemT &src, Mem dst) {
		swap_byte_order<MemT>{}(src, register_ref<Layout, Mem>(storage).*dst);
	}


	// ---------------------------------------------------------------------------------------
	// Internal frame fill functions
	// ---------------------------------------------------------------------------------------
	// Client
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
		switch(buffer.transport) {
		case transport_t::RTU: RES_FORWARD(buffer.write_checksum(checksum::calculate_crc16(buffer.frame_data.span()))); break;
		case transport_t::TCP: swap_byte_order<uint16_t>{}(buffer.frame_data.size() - sizeof(*buffer.tcp_header), buffer.tcp_header->length); break;
		case transport_t::ASCII: RES_BOOL_ASSERT(false, "NOT_IMPLEMENTED");
		default: break;
		}
		lc = get_last_completed();
		return {buffer.frame_data.span()};
	}
	constexpr result_err _get_frame_write_req(register_t reg_type, uint32_t reg_offset, std::span<uint8_t> data, uint16_t start_bit = 0, uint16_t bit_count = 0) {
		switch (reg_type) {
			case register_t::BITS:        return {.err = "BITS_NOT_ALLOWED"};
			case register_t::BITS_WRITE:  
				if (bit_count == 1) {
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

		// write offset and size
		RES_ERR_ASSERT(buffer.write_data(h_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		RES_ERR_ASSERT(buffer.write_data(l_byte(reg_offset)), "WRITE_REG_OFF_ERR");
		if (function_code(*buffer.fc) == function_code::WRITE_MULTIPLE_COILS || 
			function_code(*buffer.fc) == function_code::WRITE_MULTIPLE_REGISTERS) {
			uint16_t reg_count = reg_type == register_t::HALFS_WRITE ? data.size() / 2 : bit_count;
			uint8_t byte_count = reg_type == register_t::HALFS_WRITE ? data.size() / 2 : (bit_count + 7) / 8;
			RES_ERR_ASSERT(buffer.write_data(h_byte(reg_count)), "WRITE_REG_COUNT_ERR");
			RES_ERR_ASSERT(buffer.write_data(l_byte(reg_count)), "WRITE_REG_COUNT_ERR");
			buffer.byte_count = buffer.frame_data.end();
			RES_ERR_ASSERT(buffer.write_data(byte_count), "WRITE_BYTE_SIZE_ERR");
		}

		// write data
		switch (reg_type) {
			case register_t::BITS_WRITE:
				if (bit_count == 1) {
					uint8_t bit = data[start_bit / 8] & (1 << (start_bit % 8));
					RES_FORWARD(buffer.write_data(bit ? 0xff: 0));
					RES_FORWARD(buffer.write_data(0));
				} else {
					for (uint32_t cur_bit = start_bit; cur_bit < bit_count; cur_bit += 8) {
						uint8_t byte = data[cur_bit / 8] >> (cur_bit % 8);
						if (cur_bit % 8)
							byte |= data[cur_bit / 8 + 1] << (8 - (cur_bit % 8));
						RES_FORWARD(buffer.write_data(byte));
					}
				}
				break;
			case register_t::HALFS_WRITE:
				RES_ERR_ASSERT(buffer.write_data(data), "WRITE_DATA_ERR");
				break;
			default: break;
		}

		if (buffer.is_ascii()) {
			return {.err = "ASCII not implemented yet"};
		} else if (buffer.is_rtu()) {
			uint16_t crc = checksum::calculate_crc16(buffer.frame_data.span());
			RES_FORWARD(buffer.write_checksum(crc));
		}
		if (buffer.is_tcp()) {
			swap_byte_order<uint16_t>{}(buffer.frame_data.size() - sizeof(*buffer.tcp_header), 
				buffer.tcp_header->length);
		}
		lc = get_last_completed();
		return {buffer.frame_data.span()};
	}

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
		if (addr == 0) { 
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
					break;
				default: break;
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
					RES_FORWARD(is_bit_covered<decltype(storage.bits_registers)>(reg_offset, reg_count));
					RES_BOOL_ASSERT(buffer.data && buffer.byte_count, "INCOMPLETE_RESPONSE");
					write_bits_to_storage(storage.bits_registers, reg_offset, reg_count, buffer.data);
				}
				break;
			case function_code::READ_DISCRETE_INPUTS:
				if constexpr (!HasWriteBits<Layout>) {
					buffer.clear();
					return {.err = "LAYOUT_HAS_NO_WRITE_BITS"};
				} else {
					RES_FORWARD(is_bit_covered<decltype(storage.bits_write_registers)>(reg_offset, reg_count));
					RES_BOOL_ASSERT(buffer.data && buffer.byte_count, "INCOMPLETE_RESPONSE");
					write_bits_to_storage(storage.bits_write_registers, reg_offset, reg_count, buffer.data);
				}
				break;
			case function_code::READ_HOLDING_REGISTERS:
				if constexpr (!HasHalfs<Layout>) {
					buffer.clear();
					return {.err = "LAYOUT_HAS_NO_HALFS"};
				} else {
					constexpr int START = decltype(storage.halfs_registers)::OFFSET;
					RES_FORWARD(is_register_covered<decltype(storage.halfs_registers)>(reg_offset, reg_count));
					RES_BOOL_ASSERT(buffer.data && buffer.byte_count, "INCOMPLETE_RESPONSE");
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
					RES_FORWARD(is_register_covered<decltype(storage.halfs_write_registers)>(reg_offset, reg_count));
					RES_BOOL_ASSERT(buffer.data && buffer.byte_count, "INCOMPLETE_RESPONSE");
					std::copy_n(buffer.data, *buffer.byte_count, 
						reinterpret_cast<uint8_t*>(&storage.halfs_write_registers) + (reg_offset - START) * 2);
					}
				break;
			default: break;
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
		return last_completed{
			.transport = buffer.transport,
			.tcp_tid = buffer.tcp_header ? to_hb_first(buffer.tcp_header->transaction_id): uint16_t(0),
			.addr = buffer.addr ? *buffer.addr: uint8_t(0),
			.fc = buffer.fc ? function_code(*buffer.fc): function_code::NONE,
			.i1 = *reinterpret_cast<uint16_t*>(buffer.fc + 1),
			.i2 = *reinterpret_cast<uint16_t*>(buffer.fc + 3),
			.crc = *(reinterpret_cast<uint16_t*>(buffer.frame_data.end()) - 1),
		};
	}
	#undef RES_ERR_ASSERT
	#undef RES_FORWARD
	#undef RES_BOOL_ASSERT
};

}

