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
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>

#include "lasreader.hpp"
#include "laswriter.hpp"

enum POINT_FILE_TYPE {
	LAS = 0,
	TXT = 1,
	TXT_COLOR = 2,
	UNKNOWN
};

class PointCloud : public osg::Geode
{
private:
	explicit PointCloud(osg::ref_ptr<osg::Group> root = nullptr);
	virtual ~PointCloud();

	PointCloud(const PointCloud &other) = delete;
	PointCloud & operator=(const PointCloud& other) = delete;

	friend class PCloudManager;
	
private:
	size_t point_num;//点数量
	float point_size;
	std::string point_name;

	osg::ref_ptr<osg::Geometry> geo_point;//点数据几何体指针

	osg::ref_ptr<osg::Geode> geo_bounding_node;
	osg::ref_ptr<osg::Geometry> geo_bounding_box;//外接矩形框几何体指针

	QColor point_color;
	POINT_FILE_TYPE m_Type;
	bool b_isSelected;

private:
	void clearData() {
		this->removeDrawables(0, this->getNumDrawables());//剔除节点内所有的几何体
		geo_point = nullptr;
	}

	void initBoundingBox();

	void setShowBoundingBox(bool isShow);

public:
	void setSelected(bool isSelected) {
		b_isSelected = isSelected;
		setShowBoundingBox(isSelected);
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

	void PointCloud::readLasDataByLibLas(const std::string & openfileName);

	void readLasData(const std::string & openfileName, int & rate, bool & isCancel);

	void readPoints(const std::string & openfileName, int & rate, bool & isCancel);
};

class PCloudManager {
private:
	PCloudManager(osg::ref_ptr<osg::Group> root = nullptr);

	~PCloudManager();

	PCloudManager(const PCloudManager& other) {	};

	PCloudManager & operator=(const PCloudManager& other) {};

public:
	static PCloudManager * Instance(osg::ref_ptr<osg::Group> root = nullptr) {
		static PCloudManager instance(root);
		return &instance;
	}

	PointCloud * addPointCloud(const std::string & pName);

	void removePointCloud(PointCloud* pcloud);

	void removePointCloud(const std::string & pName);

	void setShowPointCloud(const std::string & pName, bool isShow);

	void setShowBox(const std::string & pName, bool isShow);

	PointCloud* getPointCloud(const std::string & pName);

	void clearSelectedState();

	bool setSelectState(const std::string & pName, bool isSelected);

	size_t selectedPcloudNum() const;

	void saveSelectedToFile(const std::string & saveFileName);

public:
	osg::ref_ptr<osg::Group> m_root;
	std::map<std::string, PointCloud*> all_pcloud_map;
	std::list<PointCloud*> selected_pcloud_list;
};