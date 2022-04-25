#pragma once

#include <iostream>
#include <fstream>
#include <json/json.h>

class JsonMgr {
public:
	JsonMgr();
	~JsonMgr();

	static Json::Value getReadValue(const std::string jsonName);	
};

JsonMgr::JsonMgr(){

}

JsonMgr::~JsonMgr(){

}

Json::Value JsonMgr::getReadValue(const std::string jsonName) {
	Json::Value rootValue;
	Json::Reader reader;

	std::ifstream jsonFile(jsonName, std::ios::binary);
	if (!jsonFile.is_open()) {
		std::cout << "open json file failed" << std::endl;
	}

	if (!reader.parse(jsonFile, rootValue)) {
		std::cout << "json file parse failed" << std::endl;
	}
	return rootValue;
}
