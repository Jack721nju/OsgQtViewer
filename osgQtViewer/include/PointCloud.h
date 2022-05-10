#pragma once

#include "struct.h"
#include "osgQt.h"

#include <Windows.h>

#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <fstream>
#include <ostream>
#include <iomanip>

#include <pcl/io/io.h> 

#include "TimerClock.h"

#include <liblas/liblas.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>

#include "lasreader.hpp"
#include "laswriter.hpp"

enum POINT_FILE_TYPE {
	LAS = 0,
	TXT = 1,
	TXT_COLOR = 2,
	PCD = 3,
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
	size_t point_size;
	std::string point_name;

	osg::ref_ptr<osg::Geometry> geo_point;//点数据几何体指针

	osg::ref_ptr<osg::Geode> geo_bounding_node;

	osg::ref_ptr<osg::Geode> geo_octree_node;

	osg::ref_ptr<osg::Geometry> geo_bounding_box;//外接矩形框几何体指针

	bool hasBuildBox{false};

	QColor point_color;
	POINT_FILE_TYPE m_Type;
	bool b_isSelected;

	point_MAXMIN * Max_area{ nullptr };

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_loadCloud;

private:
	void clearData();		

	void initBoundingBox();

	void setShowBoundingBox(bool isShow);

public:
	//获取给定点云数据的最大最小范围
	point_MAXMIN* getMinMaxXYZ_POINTS();

	template <typename type>
	auto getVertArry() {
		if (geo_point == nullptr) {
			return (type *)nullptr;
		}
		return dynamic_cast<type *>(geo_point->getVertexArray());
	}

	osg::Geometry * getGeoPoint() {
		return geo_point.get();
	}

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

	const size_t getPointSize() {
		return point_size;
	}

	const QColor & getPointColor() {
		return point_color;
	}

	void setPointColor(const QColor & color) {
		if (geo_point) {
			osg::ref_ptr<osg::Vec4Array> colorList = new osg::Vec4Array;//创建颜色数组,逆时针排序
			osg::Vec4 new_color(color.red() / 255., color.green() / 255., color.blue() / 255., color.alpha() / 255.);
			colorList->push_back(new_color);

			geo_point->setColorArray(colorList);
			geo_point->setColorBinding(osg::Geometry::BIND_OVERALL);
			point_color = color;
		}
	}

	void setPointColorArry(osg::ref_ptr<osg::Vec4Array> colorList) {
		if (geo_point) {
			geo_point->setColorArray(colorList);
			geo_point->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		}
	}

	void setPointSize(size_t size) {
		osg::Point * point = new osg::Point();
		point->setSize(size);
		this->getOrCreateStateSet()->setAttribute(point);
		point_size = size;
	}

	void readLasData(const std::string & openfileName);

	void readLasDataByLibLas(const std::string & openfileName);

	void readLasData(const std::string & openfileName, int & rate, bool & isCancel);

	void readTxtData(const std::string & openfileName, int & rate, bool & isCancel);

	void readPCDData(const std::string & openfileName);

	void buildOtree(float minSize);
};

class PCloudManager {
private:
	PCloudManager(osg::ref_ptr<osg::Group> root = nullptr);

	~PCloudManager();

	PCloudManager(const PCloudManager& other) = delete;

	PCloudManager & operator=(const PCloudManager& other) = delete;

public:
	static PCloudManager * Instance(osg::ref_ptr<osg::Group> root = nullptr) {
		static PCloudManager instance(root);
		return &instance;
	}

	PointCloud * addPointCloud(const std::string & pName);

	void removePointCloud(PointCloud* pcloud);

	void removeSeletedPointCloud();

	void removeAllPointCloud();

	void removePointCloud(const std::string & pName);

	void setShowPointCloud(const std::string & pName, bool isShow);

	void setShowBox(const std::string & pName, bool isShow);

	PointCloud* getPointCloud(const std::string & pName);

	void clearSelectedState();

	bool setSelectState(const std::string & pName, bool isSelected);

	size_t selectedPcloudNum() const;

	void saveSelectedToFile(const std::string & saveFileName);

	void setSelectPointSize(size_t size);

	void setSelectPointColor(const QColor & color);

public:
	osg::ref_ptr<osg::Group> m_root;
	std::map<std::string, PointCloud*> all_pcloud_map;
	std::list<PointCloud*> selected_pcloud_list;
};