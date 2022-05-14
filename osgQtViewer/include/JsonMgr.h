/* Copyright© 2022 Jack721 */
#pragma once

#include <json/json.h>
#include <string>

const char jsonFilePath[] = "etc/config.json";

class JsonMgr {
 public:
    JsonMgr() {}
	~JsonMgr() {}

	static Json::Value getReadValue(const std::string &jsonName = jsonFilePath);
};
