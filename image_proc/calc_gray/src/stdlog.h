//
// Created by u01 on 11/21/18.
//

#ifndef __STDLOG_H__
#define __STDLOG_H__

#include <string>
#include <chrono>
#include <iostream>

#define LOG(str) std::cout \
					<< "\033[32m[ INFO] [" \
					+ std::to_string(std::chrono::system_clock::now().time_since_epoch().count() / 1000000) \
					+ "]: " + str + "\033[0m" << std::endl;

#define WARN(str) std::cout \
					<< "\033[33m[ WARN] [" \
					+ std::to_string(std::chrono::system_clock::now().time_since_epoch().count() / 1000000) \
					+ "]: " + str + "\033[0m" << std::endl;

#define ERROR(str) std::cout \
					<< "\033[31m[ERROR] [" \
					+ std::to_string(std::chrono::system_clock::now().time_since_epoch().count() / 1000000) \
					+ "]: " + str + "\033[0m" << std::endl;

#define INFO(str) LOG(str)

#endif // stdlog.h
