cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(blinkiki)

target_sources(app PRIVATE main.c)
