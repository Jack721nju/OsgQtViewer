#pragma once

#include <iostream>
#include <fstream>
#include <json/json.h>

const std::string jsonFilePath = "etc/config.json";

class JsonMgr {
public:
	explicit JsonMgr(){}
	~JsonMgr(){}

	static Json::Value getReadValue(const std::string &jsonName = jsonFilePath);
};
