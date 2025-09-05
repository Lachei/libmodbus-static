#include <cassert>
#include <modbus-register.h>
#include <iostream>
#include <vector>
#include <print>
#include <ranges>

#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

using namespace libmodbus_static;

bool operator==(std::span<uint8_t> a, std::span<uint8_t> b) {
	if (a.size() != b.size())
		return false;
	for (size_t i: std::ranges::iota_view{size_t(0), a.size()})
		if (a[i] != b[i])
			return false;
	return true;
}

struct ExcludeLast{};
template<typename T>
constexpr auto operator|(const T &v, ExcludeLast) { return v | std::ranges::views::take(v.size() - 1); }

struct bitset_test {
	constexpr static int OFFSET{20};
	bool a: 1{};
	bool b: 1{};
	bool c: 1{};
	bool d: 1{};
	bool e: 1{};
	bool f: 1{};
	bool g: 1{};
	bool h: 1{};
	bool i: 1{};
	bool j: 1{};
	bool k: 1{};
	bool l: 1{};
	bool m: 1{};
	bool n: 1{};
	bool o: 1{};
	bool p: 1{};
	bool q: 1{};
	bool r: 1{};
	bool s: 1{};
	bool t: 1{};
	bool u: 1{};
	bool v: 1{};
	bool w: 1{};
	bool x: 1{};
	bool y: 1{};
	bool z: 1{};
};
struct bitset_test_2 {
	constexpr static int OFFSET{10};
	bool a: 1{};
	bool b: 1{};
	bool c: 1{};
	bool d: 1{};
	bool e: 1{};
	bool f: 1{};
	bool g: 1{};
	bool h: 1{};
	bool i: 1{};
	bool j: 1{};
	bool k: 1{};
	bool l: 1{};
	bool m: 1{};
	bool n: 1{};
	bool o: 1{};
	bool p: 1{};
	bool q: 1{};
	bool r: 1{};
	bool s: 1{};
	bool t: 1{};
	bool u: 1{};
	bool v: 1{};
	bool w: 1{};
	bool x: 1{};
	bool y: 1{};
	bool z: 1{};
};

#pragma pack(push, 1)
struct example_layout {
	struct bits_layout {
		constexpr static int OFFSET{0};
		bool enabled: 1{};
		bool active: 1{};
		bool visible: 1{};
		bool trusted: 1{};
		bool uknown: 1{};
	} bits_registers;
	// if eg. no discrete inputs are used, just leave them out else
	// struct bits_write_layout {
	// 	constexpr static int OFFSET{256};
	//	bool setable: 1{};
	// } bits_write_registers;
	struct halfs_layout {
		constexpr static int OFFSET{40000};
		float a{};
		float b{};
		uint16_t another{};
		mod_string<32> string_field{"Default"}; // careful with default inited strings, have to be byte swapped
		uint32_t events{};
	} halfs_registers;
	struct halfs_write_layout {
		constexpr static int OFFSET{60000};
		float a{};
		float b{};
		uint16_t others{};
	} halfs_write_registers;
};
struct test_layout {
	bitset_test bits_registers{};
	bitset_test_2 bits_write_registers{};
	struct halfs_layout {
		constexpr static int OFFSET{0};
		uint16_t r1{};
		uint16_t r2{};
		uint16_t r3{};
		uint16_t r4{};
	} halfs_registers;
	struct halfs_write_layout {
		constexpr static int OFFSET{0};
		uint16_t r1{};
		uint16_t r2{};
		uint16_t r3{};
		uint16_t r4{};
	} halfs_write_registers;
};
#pragma pack(pop)
using e = example_layout;
using t = test_layout;

int main() {
	std::cout << "---------------------------------------------------------------------------------------\n";
	std::cout << "Base tests\n";
	std::cout << "---------------------------------------------------------------------------------------\n";
	// checksum tests
	std::vector<uint8_t> test{0x01, 0x04, 0x02, 0xFF, 0xFF};
	uint16_t crc = checksum::calculate_crc16(test);
	assert(crc == 0x80B8);
	test = {0x01, 0x04, 0x02, 0xFF, 0xFF, 0xB8, 0x80};
	crc = checksum::calculate_crc16(test);
	assert(crc == 0);

	static_assert(sizeof(bitset_test) > 2);
	bitset_test bs{
		.a = true,
		.i = true,
	};
	std::span<uint8_t> bs_span{reinterpret_cast<uint8_t*>(&bs), sizeof(bs)};
	std::println("Bytes {}, {}", bs_span, int('i' - 'a'));

	static_assert(sizeof(example_layout::bits_registers) == 1);
	static_assert(sizeof(example_layout::halfs_registers) == 46);
	static_assert(sizeof(example_layout::halfs_write_registers) == 10);
	static_assert(sizeof(example_layout) == 57);

	modbus_register<example_layout>& modbus{modbus_register<example_layout>::Default(20)};
	
	std::cout << "Done. \n\n";

	std::cout << "---------------------------------------------------------------------------------------\n";
	std::cout << "Read/write tests\n";
	std::cout << "---------------------------------------------------------------------------------------\n";
	assert(modbus.addr == 20);

	static_assert(IsHalfsRegister<example_layout, decltype(&e::halfs_layout::string_field)>, "This is a halfs register");
	
	std::string_view sv = to_string_view(modbus.read(&e::halfs_layout::string_field));
	assert(sv == "Default");
	modbus.write(mod_string<32>{"Another"}, &e::halfs_layout::string_field);
	sv = to_string_view(modbus.read(&e::halfs_layout::string_field));
	assert(sv == "Another");
	assert(modbus.read(&e::halfs_layout::a) == 0);
	modbus.write(20.0f, &e::halfs_layout::a);
	assert(modbus.read(&e::halfs_layout::a) == 20.0f);
	assert(modbus.storage.halfs_registers.a != 20.0f);
	assert(modbus.read(&e::halfs_write_layout::others) == 0);
	modbus.write(uint16_t(0x00ff), &e::halfs_write_layout::others);
	assert(modbus.storage.halfs_write_registers.others == 0xff00);

	// bits structs should be written directly
	modbus.storage.bits_registers.enabled = true;

	std::cout << "Done.\n\n";

	std::cout << "---------------------------------------------------------------------------------------\n";
	std::cout << "Client tests\n";
	std::cout << "---------------------------------------------------------------------------------------\n";

	modbus_register<test_layout>& client_test{modbus_register<test_layout>::Default(0)};

	std::println("Test read bits from bits registers");
	assert(client_test.start_rtu_frame(1) == OK);
	auto [res, err] = client_test.get_frame_read(bitset_test{.c = true, .g = true});
	assert(err == OK);
	std::println("Read bits frame: {}", res);
	std::vector<uint8_t> valid_bits_read_ref{1, 1, 0, 22, 0, 5, 29, 205};
	assert(res == valid_bits_read_ref);
	
	std::println("Test read bits from bits write registers");
	assert(client_test.start_rtu_frame(1) == OK);
	r_tie(res, err) = client_test.get_frame_read(bitset_test_2{.x = true});
	assert(err == OK);
	std::println("Read bits write frame: {}", res);
	std::vector<uint8_t> valid_bits_write_read_ref{1, 2, 0, 33, 0, 1, 233, 192};
	assert(res == valid_bits_write_read_ref);

	std::println("Test write single bit to bits");
	client_test.storage.bits_write_registers.z = true;
	assert(client_test.start_rtu_frame(233) == OK);
	r_tie(res, err) = client_test.get_frame_write(bitset_test_2{.z = true});
	assert(err == OK);
	std::vector<uint8_t> valid_write_single_bit{233, 5, 0, 35, 255, 0, 106, 216};
	std::println("Write bit frame: {}", res);
	assert(res == valid_write_single_bit);
	client_test.switch_to_response();
	for (uint8_t res_byte: valid_write_single_bit | ExcludeLast{})
		assert(client_test.process_rtu(res_byte).err == IN_PROGRESS);
	assert(client_test.process_rtu(valid_write_single_bit.back()).err == OK);

	std::println("Test write multiple bits");
	client_test.storage.bits_write_registers.a = true;
	client_test.storage.bits_write_registers.c = true;
	client_test.storage.bits_write_registers.d = true;
	client_test.storage.bits_write_registers.e = true;
	client_test.storage.bits_write_registers.f = true;
	client_test.storage.bits_write_registers.i = true;
	// byte bits ordering:
	// h g f e d c b a   $ $ $ $ $ $ $ i
	// which should result in the follwing bits
	// 0 0 1 1 1 1 0 1   0 0 0 0 0 0 0 1
	// which is in hex
	// 0x3d 0x1
	//
	// with offset 2:
	// j i h g f e d c   $ $ $ $ $ $ $ $
	// which should result in the follwing bits
	// 0 1 0 0 1 1 1 1   0 0 0 0 0 0 0 0
	// which is in hex
	// 0x4f 0x00
	assert(client_test.start_rtu_frame(134) == OK);
	r_tie(res, err) = client_test.get_frame_write(bitset_test_2{.a = true, .z = true});
	assert(err == OK);
	std::println("Write bits frame: {}", res);
	std::vector<uint8_t> write_multiple_bits_all_ref{134, 15, 0, 10, 0, 26, 4, 61, 1, 0, 2, 47, 182};
	assert(res == write_multiple_bits_all_ref);
	assert(client_test.start_rtu_frame(134) == OK);
	r_tie(res, err) = client_test.get_frame_write(bitset_test_2{.c = true, .z = true});
	assert(err == OK);
	std::println("Write bits frame start bit 3: {}", res);
	std::vector<uint8_t> write_multiple_bits_offset_ref{134, 15, 0, 12, 0, 24, 3, 79, 0, 128, 10, 49};
	assert(res == write_multiple_bits_offset_ref);

	std::println("Test read register from read registers");
	assert(client_test.start_rtu_frame(1) == OK);
	r_tie(res, err) = client_test.get_frame_read(&t::halfs_layout::r1, &t::halfs_layout::r2);
	assert(err == OK);
	std::vector<uint8_t> valid_read_holding = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b};
	std::vector<uint8_t> valid_read_response = {0x01, 0x03, 0x04, 0x00, 0x06, 0x00, 0x05, 0xda, 0x31};
	std::println("should be: {:}", valid_read_holding);
	std::println("is       : {:}", res);
	assert(std::span<uint8_t>(valid_read_holding) == res);
	std::println("Result bad crc");
	client_test.switch_to_response();
	for (uint8_t b: (valid_read_response | std::ranges::views::take(valid_read_response.size() - 1)))
		assert(client_test.process_rtu(b).err == IN_PROGRESS);
	assert(client_test.process_rtu(0x20).err == INVALID_CRC);
	std::println("Result bad return size");
	client_test.switch_to_response();
	std::vector<uint8_t> invalid_read_response = {0x01, 0x03, 0x02, 0x00, 0x06};
	for (uint8_t b: invalid_read_response)
		assert(client_test.process_rtu(b).err == IN_PROGRESS);
	uint16_t cs = checksum::calculate_crc16(client_test.buffer.frame_data.span());
	assert(client_test.process_rtu(l_byte(cs)).err == IN_PROGRESS);
	assert(client_test.process_rtu(h_byte(cs)).err == INVALID_RESPONSE);
	std::println("Valid response");
	client_test.switch_to_response();
	for (uint8_t b: (valid_read_response | std::ranges::views::take(valid_read_response.size() - 1)))
		assert(client_test.process_rtu(b).err == IN_PROGRESS);
	assert(client_test.process_rtu(valid_read_response.back()).err == OK);
	assert(client_test.read(&t::halfs_layout::r1) == 6);
	assert(client_test.read(&t::halfs_layout::r2) == 5);

	std::println("\nTest read single register from write registers");
	client_test.write(uint16_t(44), &t::halfs_write_layout::r1);
	assert(client_test.start_rtu_frame(1) == OK);
	r_tie(res, err) = client_test.get_frame_read(&t::halfs_write_layout::r1);
	assert(err == OK);
	std::vector<uint8_t> valid_read_input = {0x01, 0x04, 0x00, 0x00, 0x00, 0x01, 49, 202};
	std::vector<uint8_t> valid_read_response_input = {0x01, 0x04, 0x02, 0x00, 0x00, 0xb9, 0x30};
	std::println("should be: {:}", valid_read_input);
	std::println("is       : {:}", res);
	assert(std::span<uint8_t>(valid_read_input) == res);
	std::println("Valid response");
	client_test.switch_to_response();
	for (uint8_t b: (valid_read_response_input | std::ranges::views::take(valid_read_response_input.size() - 1)))
		assert(client_test.process_rtu(b).err == IN_PROGRESS);
	assert(client_test.process_rtu(valid_read_response_input.back()).err == OK);
	assert(client_test.read(&t::halfs_write_layout::r1) == 0);

	std::println("\nTest write single register");
	// invalid target register
	assert(client_test.start_rtu_frame(2) == OK);
	r_tie(res, err) = client_test.get_frame_write(&t::halfs_layout::r1);
	assert(err == "HALFS_NOT_ALLOWED");
	client_test.write(uint16_t(3), &t::halfs_write_layout::r1);
	assert(client_test.start_rtu_frame(17) == OK);
	r_tie{res, err} = client_test.get_frame_write(&t::halfs_write_layout::r1);
	std::println("{}", err);
	assert(err == OK);
	std::vector<uint8_t> solution1 = {0x11, 0x06, 0x00, 0x00, 0x00, 0x03, 203, 91};
	std::println("should be: {:}", solution1);
	std::println("is       : {:}", res);
	assert(std::span<uint8_t>(solution1) == res);
	std::println("Check response frame for validity, bad frame");
	assert(client_test.process_rtu(solution1[0]).err == "NO_WRITE_IN_FINAL_STATE");
	client_test.switch_to_response();
	// bad checksum
	assert(client_test.process_rtu(solution1[0]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[1]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[2]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[3]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[4]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[5]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[6]).err == IN_PROGRESS);
	assert(client_test.process_rtu(16).err == INVALID_CRC);

	// bad response
	assert(client_test.process_rtu(solution1[0]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[1]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[2]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[3]).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1[4]).err == IN_PROGRESS);
	assert(client_test.process_rtu(42).err == IN_PROGRESS);
	cs = checksum::calculate_crc16(client_test.buffer.frame_data.span());
	assert(client_test.process_rtu(l_byte(cs)).err == IN_PROGRESS);
	assert(client_test.process_rtu(h_byte(cs)).err == INVALID_RESPONSE);

	// correct response
	for (uint8_t b: (solution1 | std::ranges::views::take(solution1.size() - 1)))
		assert(client_test.process_rtu(b).err == IN_PROGRESS);
	assert(client_test.process_rtu(solution1.back()).err == OK);
	
	std::println("Done.\n");

	std::cout << "---------------------------------------------------------------------------------------\n";
	std::cout << "Server tests\n";
	std::cout << "---------------------------------------------------------------------------------------\n";

	modbus_register<test_layout>& test_server{modbus_register<test_layout>::Default<1>(1)};
	test_server.storage.bits_registers.a = true;
	test_server.storage.bits_registers.c = true;
	test_server.storage.bits_registers.d = true;
	test_server.storage.bits_registers.e = true;
	test_server.storage.bits_registers.f = true;
	test_server.storage.bits_registers.i = true;
	test_server.storage.bits_write_registers.b = true;
	test_server.storage.bits_write_registers.c = true;
	test_server.storage.bits_write_registers.d = true;
	test_server.storage.bits_write_registers.f = true;
	test_server.storage.bits_write_registers.i = true;
	test_server.write(uint16_t(5), &t::halfs_layout::r3);
	test_server.write(uint16_t(6), &t::halfs_layout::r4);
	client_test.storage.bits_write_registers = {};

	std::println("\nRead bits");
	client_test.start_rtu_frame(1);
	// i h g f e d c b
	// 0 0 0 1 1 1 1 0 - 0x1e
	r_tie{res, err} = client_test.get_frame_read(bitset_test{.b = true, .f = true});
	println("Read bits request: {}", res);
	assert(err == OK);
	for (uint8_t b: res | ExcludeLast{})
		assert(test_server.process_rtu(b).err == IN_PROGRESS);
	assert(test_server.process_rtu(res.back()).err == OK);
	r_tie{res, err} = test_server.get_frame_response();
	assert(err == OK);
	println("Read bits response: {}", res);
	std::vector<uint8_t> valid_read_coils_response{1, 1, 1, 30, 209, 128};
	assert(res == valid_read_coils_response);
	test_server.switch_to_request();

	std::println("Read write bits");
	client_test.start_rtu_frame(1);
	// i h g f e d c b
	// 0 0 0 1 0 1 1 1 - 0x17
	r_tie{res, err} = client_test.get_frame_read(bitset_test_2{.b = true, .f = true});
	println("Read bits request: {}", res);
	assert(err == OK);
	for (uint8_t b: res | ExcludeLast{})
		assert(test_server.process_rtu(b).err == IN_PROGRESS);
	assert(test_server.process_rtu(res.back()).err == OK);
	r_tie{res, err} = test_server.get_frame_response();
	assert(err == OK);
	println("Read bits response: {}", res);
	std::vector<uint8_t> valid_read_coils_server_response{1, 2, 1, 23, 225, 134};
	assert(res == valid_read_coils_server_response);
	client_test.switch_to_response();
	for (uint8_t b: res | ExcludeLast{})
		assert(client_test.process_rtu(b).err == IN_PROGRESS);
	assert(client_test.process_rtu(res.back()).err == OK);
	assert(client_test.storage.bits_write_registers.a == test_server.storage.bits_write_registers.a);
	assert(client_test.storage.bits_write_registers.b == test_server.storage.bits_write_registers.b);
	assert(client_test.storage.bits_write_registers.c == test_server.storage.bits_write_registers.c);
	assert(client_test.storage.bits_write_registers.d == test_server.storage.bits_write_registers.d);
	assert(client_test.storage.bits_write_registers.e == test_server.storage.bits_write_registers.e);
	assert(client_test.storage.bits_write_registers.i != test_server.storage.bits_write_registers.i);
	test_server.switch_to_request();

	std::println("Read halfs");
	std::println("Bad address");
	client_test.start_rtu_frame(31);
	r_tie{res, err} = client_test.get_frame_read(&t::halfs_layout::r3, &t::halfs_layout::r4);
	assert(err == OK);
	std::println("Read halfs frame: {}", res);
	for (uint8_t b: res | std::ranges::views::take(res.size() - 1))
		assert(test_server.process_rtu(b).err == IN_PROGRESS);
	assert(test_server.process_rtu(res.back()).err == WRONG_ADDR);

	std::println("Valid read halfs");
	client_test.start_rtu_frame(1);
	r_tie{res, err} = client_test.get_frame_read(&t::halfs_layout::r3, &t::halfs_layout::r4);
	std::println("Read halfs frame: {}", res);
	assert(err == OK);
	for (uint8_t b: res | std::ranges::views::take(res.size() - 1))
		assert(test_server.process_rtu(b).err == IN_PROGRESS);
	assert(test_server.process_rtu(res.back()).err == OK);
	r_tie{res, err} = test_server.get_frame_response();
	std::println("Read halfs frame responnse: {}", res);
	std::vector<uint8_t> halfs_readout{1, 3, 4, 0, 5, 0, 6, 106, 48};
	assert(err == OK);
	assert(res == halfs_readout);

	std::println("Valid read write_halfs");
	test_server.write(uint16_t(6), &t::halfs_write_layout::r3);
	test_server.write(uint16_t(2), &t::halfs_write_layout::r4);
	client_test.start_rtu_frame(1);
	r_tie{res, err} = client_test.get_frame_read(&t::halfs_write_layout::r1, &t::halfs_write_layout::r4);
	std::println("Read halfs frame: {}", res);
	assert(err == OK);
	test_server.switch_to_request();
	for (uint8_t b: res | std::ranges::views::take(res.size() - 1))
		assert(test_server.process_rtu(b).err == IN_PROGRESS);
	assert(test_server.process_rtu(res.back()).err == OK);
	r_tie{res, err} = test_server.get_frame_response();
	std::vector<uint8_t> halfs_write_readout{1, 4, 8, 0, 0, 0, 0, 0, 6, 0, 2, 69, 205};
	std::println("Read halfs frame responnse: {}", res);
	assert(err == OK);
	assert(res == halfs_write_readout);

	std::println("Done.\n\n");

	std::cout << "---------------------------------------------------------------------------------------\n";
	std::cout << "TCP test\n";
	std::cout << "---------------------------------------------------------------------------------------\n";

	assert(client_test.start_tcp_frame(10, 1) == OK);
	r_tie{res, err} = client_test.get_frame_read(&t::halfs_layout::r4);
	std::vector<uint8_t> tcp_valid_read{0, 10, 0, 0, 0, 6, 1, 3, 0, 3, 0, 1};
	std::println("tcp request frame: {}, {}", err, res);
	assert(err == OK);
	assert(res == tcp_valid_read);

	test_server.write(uint16_t(0x1805), &t::halfs_layout::r4);
	test_server.switch_to_request();
	for (uint8_t b: res | std::ranges::views::take(res.size() - 1))
		assert(test_server.process_tcp(b).err == IN_PROGRESS);
	assert(test_server.process_tcp(res.back()).err == OK);
	r_tie{res, err} = test_server.get_frame_response();
	std::vector<uint8_t> tcp_valid_response{0, 10, 0, 0, 0, 5, 1, 3, 2, 24, 5};
	std::println("tcp response frame: {}, {}", err, res);
	assert(err == OK);
	assert(res == tcp_valid_response);

	std::println("Done.\n");

	std::println("");
	std::println(ANSI_COLOR_GREEN "[  PASS  ] All tests work" ANSI_COLOR_RESET);

	return 0;
}

