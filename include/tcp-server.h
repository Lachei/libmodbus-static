#pragma once
#include "modbus-register.h"

//TODO: implement

namespace libmodbus_static::modbus_tcp_server {

constexpr auto noop = [](auto& cur_request){};
constexpr auto nolog = [](std::string_view){};

/**
 * @brief Is an easier abstraction for creating a modbus tcp server with the underlying modbus_register class,
 * however uses only functions from modbus_register for all implementations
 *
 * This wrapper class is still independent of the underlying transport adapter.
 * See the `examples` folder for examples of how to add such an adapter for your platform (example for linux given)
 */
template<typename Layout, uint8_t Address, int MaxSize, typename Socket, typename ReadFn, typename WriteFn, typename PreResponseFn = decltype(noop), typename LogFn = decltype(nolog)>
requires requires(ReadFn rf, WriteFn wf, PreResponseFn prf) {
	std::is_same_v<decltype(rf(std::declval<Socket>())), uint8_t>;
	wf(std::declval<Socket>(), uint8_t{});
	prf(std::declval<modbus_frame<MaxSize>>());}
struct tcp_server: public modbus_register<Layout, Address, MaxSize> {
	tcp_server(Socket s, ReadFn read_fn, WriteFn write_fn, PreResponseFn pre_response_fn = noop, LogFn log_fn = nolog):
		s{s}, read_fn{read_fn}, write_fn{write_fn}, pre_response_fn{pre_response_fn}, log_fn{log_fn} {}
	Socket s{};
	ReadFn read_fn{};
	WriteFn write_fn{};
	PreResponseFn pre_response_fn{};
	LogFn log_fn{};

	using modbus_reg = modbus_register<Layout, Address, MaxSize>;
	/*
	* @brief main server function which does 
	* 1. process incoming bytes with read_fn
	* 2. calls pre_response_fn after a frame has received and the repsonse not yet has been sent
	* 3. sends out the response via write_fn
	*
	* All errors etc. are automatically handled and do not interrupt the control flow, for logging proved log_fn
	*/
	void step() {
		std::string_view r = process_tcp(read_fn(s));
		if (r == IN_PROGRESS)
			return;
		if (r != OK) {
			modbus_reg::switch_to_request();
			log_fn(r);
			return;
		}
		pre_response_fn(modbus_reg::buffer); // buffer is defined in modbus_register
		auto [res, err] = modbus_reg::get_frame_response();
		if (err != OK) {
			modbus_reg::switch_to_request();
			log_fn(err);
			return;
		}
		for (uint8_t b: res)
			write_fn(s, b);
		modbus_reg::switch_to_request();
	}
};

}

