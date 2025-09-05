#pragma once

#include "modbus-register.h"
#include <chrono>

namespace libmodbus_static {

using ms = std::chrono::milliseconds;

constexpr std::string_view TIMEOUT = "TIMEOUT";
constexpr std::string_view CLIENT_CANT_QUERY = "CLIENT_CANT_QUERY";
constexpr std::string_view SERVER_CANT_RESPOND = "SERVER_CANT_RESPOND";

/**
* Modbus actor to be used as a simple full modbus actor based on the modbus-register
* The template struct CONFIG needs to have the following structure to be used internally
* struct test_io {
*	constexpr transport_t TRANSPORT_TYPE;
*	void init();
*	void deinit();
*	std::span<uint8_t> read_bytes(std::chrono::milliseconds max_timeout);
*	void write_bytes(std::span<uint8_t> data);
* };
*/
template<typename Layout, typename DATA_IO>
struct modbus_actor: public modbus_register<Layout> {
	modbus_actor(uint8_t address): modbus_register<Layout>(address) { io.init(); }
	~modbus_actor() { io.deinit(); }

	DATA_IO io{};
	uint16_t _tcp_trans{1};

	result poll_update_state(ms max_timeout) {
		if (this->addr == 0)
			return SERVER_CANT_RESPOND;
		std::string_view state{IN_PROGRESS};
		std::span<uint8_t> data = io.read_bytes(max_timeout);
		for (uint8_t b: data) {
			if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::RTU) {
				state  = this->process_tcp(b).err;
			} else if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::TCP) {
				state  = this->process_tcp(b).err;
			}
			if (state == IN_PROGRESS)
				continue;
			std::span<uint8_t> frame;
			if (state != OK) {
				r_tie{frame, state} = this->get_frame_error_response(state);
				if (state != OK) {
					this->switch_to_request();
					return state;
				}
			}
			r_tie{frame, state} = this->get_frame_response();
			if (state != OK) {
				r_tie{frame, state} = this->get_frame_error_response(state);
				if (state != OK) {
					this->switch_to_request();
					return state;
				}
			}
			this->write_bytes(frame);
			this->switch_to_request();
		}
		return state;
	};
	
	template<typename MemA, typename MemB>
	requires IsValidRegister<Layout, MemA> && IsValidRegister<Layout, MemB>
	constexpr result read_remote(uint8_t addr, MemA member_a, MemB member_b, ms timeout = ms(20000)) {
		if (this->addr != 0)
			return CLIENT_CANT_QUERY;
		if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::RTU) {
			if (result r = start_modbus_frame(this->addr); r != OK) return r;
		} else if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::TCP) {
			if (result r = this->start_tcp_frame(_tcp_trans++, addr); r != OK) return r;
		}
		auto [res, err] = this->get_frame_read(member_a, member_b);
		if (err != OK)
			return err;
		io.write_bytes(res);
		auto start = std::chrono::steady_clock::now();
		result state = IN_PROGRESS;
		std::span<uint8_t> data{};
		while (state == IN_PROGRESS && (std::chrono::steady_clock::now() - start) < timeout) {
			if (data.empty())
				data = io.read_bytes(timeout - std::chrono::duration_cast<ms>(std::chrono::steady_clock::now() - start));
			if (data.empty()) {
				state = TIMEOUT;
				continue;
			}
			if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::RTU)
				state = this->process_rtu(data[0]).err;
			else if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::TCP)
				state = this->process_tcp(data[0]).err;
		}
		return state;
	}
	template<typename Mem>
	requires IsValidRegister<Layout, Mem>
	constexpr result read_remote(uint8_t addr, Mem mem, ms timeout = ms(20e3)) { return read_remote<Mem, Mem>(addr, mem, mem, timeout); }
	template<typename Reg>
	requires IsBitsRegisters<Layout, Reg>
	constexpr result read_remote(uint8_t addr, const Reg &mask, ms timeout = ms(20e3)) {
		if (this->addr != 0)
			return CLIENT_CANT_QUERY;
		if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::RTU) {
			if (result r = this->start_modbus_frame(addr); r != OK) return r;
		} else if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::TCP) {
			if (result r = this->start_tcp_frame(_tcp_trans++, addr); r != OK) return r;
		}
		auto [res, err] = this->get_frame_read(mask);
		if (err != OK)
			return err;
		io.write_bytes(res);
		auto start = std::chrono::steady_clock::now();
		result state = IN_PROGRESS;
		std::span<uint8_t> data{};
		while (state == IN_PROGRESS && std::chrono::steady_clock::now() - start < timeout) {
			if (data.empty())
				data = io.read_bytes(timeout - (std::chrono::steady_clock::now() - start));
			if (data.empty())
				state = TIMEOUT;
			if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::RTU)
				state = this->process_rtu(data[0]).err;
			else if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::TCP)
				state = this->process_tcp(data[0]).err;
		}
		return state;
	}

	template<typename MemA, typename MemB>
	requires IsValidRegister<Layout, MemA> && IsValidRegister<Layout, MemB>
	constexpr result write_remote(uint8_t addr, MemA member_a, MemB member_b, ms timeout = ms(20e3)) {
		if (this->addr != 0)
			return CLIENT_CANT_QUERY;
		if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::RTU) {
			if (result r = this->start_modbus_frame(addr); r != OK) return r;
		} else if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::TCP) {
			if (result r = this->start_tcp_frame(_tcp_trans++, addr); r != OK) return r;
		}
		auto [res, err] = this->get_frame_write(member_a, member_b);
		if (err != OK)
			return err;
		io.write_bytes(res);
		auto start = std::chrono::steady_clock::now();
		result state = IN_PROGRESS;
		std::span<uint8_t> data{};
		while (state == IN_PROGRESS && std::chrono::steady_clock::now() - start < timeout) {
			if (data.empty())
				data = io.read_bytes(timeout - (std::chrono::steady_clock::now() - start));
			if (data.empty())
				state = TIMEOUT;
			if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::RTU)
				state = this->process_rtu(data[0]).err;
			else if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::TCP)
				state = this->process_tcp(data[0]).err;
		}
		return state;
	}
	template<typename Mem>
	requires IsValidRegister<Layout, Mem>
	constexpr result write_remote(uint8_t addr, Mem mem, ms timeout = ms(20e3)) { return write_remote<Mem, Mem>(addr, mem, mem, timeout); }
	template<typename Reg>
	requires IsBitsRegisters<Layout, Reg>
	constexpr result write_remote(uint8_t addr, const Reg &mask, ms timeout = ms(20e3)) {
		if (this->addr != 0)
			return CLIENT_CANT_QUERY;
		if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::RTU) {
			if (result r = this->start_modbus_frame(addr); r != OK) return r;
		} else if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::TCP) {
			if (result r = this->start_tcp_frame(_tcp_trans++, addr); r != OK) return r;
		}
		auto [res, err] = this->get_frame_write(mask);
		if (err != OK)
			return err;
		io.write_bytes(res);
		auto start = std::chrono::steady_clock::now();
		result state = IN_PROGRESS;
		std::span<uint8_t> data{};
		while (state == IN_PROGRESS && std::chrono::steady_clock::now() - start < timeout) {
			if (data.empty())
				data = io.read_bytes(timeout - (std::chrono::steady_clock::now() - start));
			if (data.empty())
				state = TIMEOUT;
			if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::RTU)
				state = this->process_rtu(data[0]).err;
			else if constexpr (DATA_IO::TRANSPORT_TYPE == transport_t::TCP)
				state = this->process_tcp(data[0]).err;
		}
		return state;
	}
};

}

