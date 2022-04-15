#pragma once

#include "struct.h"

#include <Windows.h>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgGA/GUIEventHandler>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/AnimationPath>
#include <osg/LineWidth>
#include <osg/ComputeBoundsVisitor>
#include <osg/PolygonMode>
#include <osg/Point>

#include <osg/ShapeDrawable>
#include <osgSim/ColorRange>
#include <osgSim/ScalarBar>

#include <osgDB/readFile>
#include <osgGA/TrackballManipulator>
#include <osgUtil/LineSegmentIntersector>

#include <osgText/Text>
#include <osgText/Font>

#include <osgManipulator/Selection>
#include <osgManipulator/Dragger>

#include <QTime>
#include <QFile>
#include <QTextStream>
#include <QColor>

#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <fstream>
#include <ostream>

#include <liblas/liblas.hpp>

enum POINT_FILE_TYPE {
	LAS = 0,
	TXT = 1,
	TXT_COLOR = 2 
};

class PointCloud : public osg::Geode
{
public:
	explicit PointCloud(osg::ref_ptr<osg::Group> root = nullptr);
	virtual ~PointCloud();
	
public:
	size_t point_num;//点数量
	float point_size;
	std::string point_name;
	osg::ref_ptr<osg::Geometry> geo_point;//点数据几何体指针

	QColor point_color;
	POINT_FILE_TYPE m_Type;
	bool b_isSelected;

public:

	void setSelected(bool isSelected) {
		b_isSelected = isSelected;
	}

	bool isSelected() const {
		return b_isSelected;
	}

	void setName(const std::string & fullName) {
		point_name = fullName;
	};

	void setType(POINT_FILE_TYPE fileType) {
		m_Type = fileType;
	};

	POINT_FILE_TYPE getType() {
		return m_Type;
	};

	const std::string & getName() {
		return point_name;
	};

	void setPointNum(size_t num) {
		point_num = num;
	};

	const size_t getPointNum() {
		return point_num;
	};

	void readLasData(const std::string & openfileName);

	void readLasData(const std::string & openfileName, int & rate);

	void readPoints(const std::string & openfileName, int & rate, bool & isCancel);

	void readPoints(const std::string & openfileName) {
		int rate;
		bool cancel = false;
		readPoints(openfileName, rate, cancel);
	};
};

class PCloudManager {
private:
	PCloudManager(){};
	~PCloudManager(){ 
		all_pcloud_map.clear();
	};

	PCloudManager(const PCloudManager&);
	PCloudManager & operator=(const PCloudManager&);

public:
	static PCloudManager * getInstance() {
		static PCloudManager instance;
		return &instance;
	}

	void addPointCloud(PointCloud* pcloud) {
		if (pcloud) {
			all_pcloud_map.emplace(pcloud->getName(), pcloud);
		}
	};

	void removePointCloud(PointCloud* pcloud) {
		if (pcloud) {
			auto iter = all_pcloud_map.begin();
			while (iter != all_pcloud_map.end()) {
				if (iter->first == pcloud->getName()) {
					all_pcloud_map.erase(iter++);
					break;
				}
				iter++;
			}
		}
	};

	void removePointCloud(const std::string & pName) {
		if (!pName.empty()) {
			auto iter = all_pcloud_map.begin();
			while (iter != all_pcloud_map.end()) {
				if (iter->first == pName) {
					all_pcloud_map.erase(iter++);
					break;
				}
				iter++;
			}
		}
	};

	PointCloud* getPointCloud(const std::string & pName) {
		if (pName.empty()) {
			return nullptr;
		}
		auto iter = all_pcloud_map.find(pName);
		if (iter != all_pcloud_map.end()) {
			return iter->second;
		}
		return nullptr;
	};

public:
	std::map<std::string, PointCloud*> all_pcloud_map;
	std::vector<PointCloud*> selected_pcloud_list;
};