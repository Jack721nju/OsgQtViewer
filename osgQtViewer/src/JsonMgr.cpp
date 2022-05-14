/* Copyright© 2022 Jack721 */
#include "JsonMgr.h"

#include <iostream>
#include <fstream>

Json::Value JsonMgr::getReadValue(const std::string &jsonName) {
	Json::Value rootValue;
	Json::Reader reader;

	std::ifstream jsonFile(jsonName, std::ios::binary);
	if (!jsonFile.is_open()) {
		std::cout << "open json file failed" << std::endl;
	}

	if (!reader.parse(jsonFile, rootValue)) {
		std::cout << "json file parse failed" << std::endl;
	}
	return std::move(rootValue);
}
