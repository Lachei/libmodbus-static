# libmodbus-static

A c++ template based modbus library which is independent of the underlying transport (eg. Modbus-RTU, Modbus-TCP).

This enables high portability as no transport specific code is included, albeit at the cost of having to write a bit of additional code for working transport adoption.

The main approach to using the library is to create either a modbus-server or modbus-client object and use this object to create and anlyze outgoing and incomming connection frames.

