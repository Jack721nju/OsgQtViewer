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

	QColor point_color;
	POINT_FILE_TYPE m_Type;
	bool b_isSelected;

private:
	void clearData() {
		this->removeDrawables(0, this->getNumDrawables());//剔除节点内所有的几何体
		geo_point = nullptr;
	}

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

	void PointCloud::readLasDataByLibLas(const std::string & openfileName);

	void readLasData(const std::string & openfileName, int & rate, bool & isCancel);

	void readPoints(const std::string & openfileName, int & rate, bool & isCancel);
};

class PCloudManager {
private:
	PCloudManager(osg::ref_ptr<osg::Group> root = nullptr): m_root(root){
	};

	~PCloudManager(){ 
		all_pcloud_map.clear();
		selected_pcloud_list.clear();
		m_root = nullptr;
	};

	PCloudManager(const PCloudManager& other) {	};

	PCloudManager & operator=(const PCloudManager& other) {};

public:
	static PCloudManager * Instance(osg::ref_ptr<osg::Group> root = nullptr) {
		static PCloudManager instance(root);
		return &instance;
	}

	PointCloud * addPointCloud(const std::string & pName) {
		PointCloud * pcloud = new PointCloud();
		pcloud->setName(pName);
		all_pcloud_map.emplace(pcloud->getName(), pcloud);
		if (m_root) {
			m_root->addChild(pcloud);
		}
		return pcloud;
	}

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
			m_root->removeChild(pcloud);
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

	void clearSelectedState() {
		for (const auto & item : selected_pcloud_list) {
			item->setSelected(false);
		}
		selected_pcloud_list.clear();
	}

	bool setSelectState(const std::string & pName, bool isSelected) {
		if (pName.empty()) {
			return false;
		}
		PointCloud * curPCl = getPointCloud(pName);
		if (curPCl) {
			curPCl->setSelected(isSelected);
			if (isSelected) {
				selected_pcloud_list.emplace_back(curPCl);
			}
			else{
				selected_pcloud_list.remove(curPCl);
			}
			return true;
		}		
		return false;
	};

	size_t selectedPcloudNum() const {
		return selected_pcloud_list.size();
	}

	void saveSelectedToFile(const std::string & saveFileName) {
		if (saveFileName.empty() || saveFileName.find_last_of(".") < 0) {
			return;
		}
		
		std::ofstream outf(saveFileName, std::ios::out | std::ios::binary);
		if (!outf.is_open()) {
			return;
		}

		osg::Vec3f singP;
		osg::ref_ptr<osg::Vec3Array> vertAll  = new osg::Vec3Array;
		size_t allPointNum = 0;
		for (const auto item : selected_pcloud_list) {
			osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(item->geo_point->getVertexArray());
			allPointNum += item->getPointNum();
			vertAll->resizeArray(allPointNum);
			vertAll->assign(vertices->begin(), vertices->end());
		}

		int pos = saveFileName.find_last_of('.');
		const std::string &fileFormat = saveFileName.substr(pos + 1);
		if (fileFormat == "txt") {			
			for (const auto & curVert : *vertAll) {
				outf << curVert.x() << " " << curVert.y() << " " << curVert.z() << " " << std::endl;
			}
		}
		else if (fileFormat == "las") {		
			LASwriteOpener writerOpener;
			writerOpener.set_file_name(saveFileName.c_str());
			LASheader header;
			header.x_scale_factor = 0.001;
			header.y_scale_factor = 0.001;
			header.z_scale_factor = 0.001;
			header.point_data_format = 1;
			header.point_data_record_length = 28;
			header.number_of_point_records = allPointNum;
			LASwriter *writer = writerOpener.open(&header);
			//header.set_bounding_box();

			LASpoint point;
			point.init(&header, header.point_data_format, header.point_data_record_length, nullptr);

			for (const auto & curVert : *vertAll) {
				point.set_x(curVert.x());
				point.set_y(curVert.y());
				point.set_z(curVert.z());
				writer->write_point(&point);
				writer->update_inventory(&point);
			}

			writer->update_header(&header, true);
			writer->close();
			delete writer;
			writer = nullptr;
		}

		outf.flush();
		outf.close();
	}

public:
	osg::ref_ptr<osg::Group> m_root;
	std::map<std::string, PointCloud*> all_pcloud_map;
	std::list<PointCloud*> selected_pcloud_list;
};