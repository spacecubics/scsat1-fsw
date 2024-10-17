cmake_minimum_required(VERSION 3.20.0)

set(BOARD scobc_module1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scsat1-bootloader)

target_sources(app PRIVATE src/main.c)
