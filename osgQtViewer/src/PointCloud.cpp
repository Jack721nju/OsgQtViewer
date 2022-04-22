#include "PointCloud.h"

using namespace std;

PointCloud::PointCloud(osg::ref_ptr<osg::Group> root) {
	point_num = 0;
	point_size = 1.0;
	geo_bounding_node = nullptr;
	geo_bounding_box = nullptr;
	geo_point = nullptr;
}

PointCloud::~PointCloud() {
	point_num = 0;
	point_size = 1.0;
	point_name = "";
	this->removeDrawables(0, this->getNumDrawables());
	geo_point = nullptr;
}

void PointCloud::readLasData(const std::string & openfileName) {
	//fstream ifs;
	//ifs.open(openfileName.c_str(), ios::in | ios::binary);
	//if (!ifs)
	//	return;

	LASreadOpener opener;
	opener.set_file_name(openfileName.c_str());
	if (false == opener.active()){
		return;
	}
	LASreader * reader = opener.open();
	if (reader == nullptr) {
		return;
	}

	geo_point = new osg::Geometry;//创建一个几何体对象
	osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec3Array> normal = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;//创建颜色数组,逆时针排序
	osg::ref_ptr<osg::DrawElementsUByte> point = new osg::DrawElementsUByte(GL_POINTS);

	size_t count = 0;
	std::string errInfo;
	try
	{
		while (reader->read_point()) {
			double  x, y, z;
			double  r, g, b, w;

			const LASpoint & p = reader->point;

			x = p.get_x();
			y = p.get_y();
			z = p.get_z();

			r = p.get_R() / 255.0;
			g = p.get_G() / 255.0;
			b = p.get_B() / 255.0;
			w = 1.0;

			osg::Vec3 single_point(x, y, z);
			osg::Vec4 single_color(r, g, b, w);

			vert->push_back(single_point);
			color->push_back(single_color);

			point->push_back(count);
			++count;//points num
		}

		reader->close();
		delete reader;
		reader = nullptr;
	}
	catch (std::exception & e)
	{
		errInfo = e.what();
	}
	catch (...)
	{
		errInfo = "get unknown exception";
	}
	

	this->point_num = count;

	normal->push_back(osg::Vec3(0.0, 0.0, 1.0));

	geo_point->setVertexArray(vert.get());
	geo_point->setNormalArray(normal.get());
	geo_point->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geo_point->setColorArray(color.get());
	geo_point->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geo_point->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, count));

	this->removeDrawables(0, this->getNumDrawables());//读取文件前先剔除节点内所有的几何体
	this->addDrawable(geo_point.get());//加入当前新的点云几何绘制体
}

void PointCloud::readLasDataByLibLas(const std::string & openfileName) {
	fstream ifs;
	ifs.open(openfileName.c_str(), ios::in | ios::binary);
	if (!ifs)
		return;

	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	geo_point = new osg::Geometry;//创建一个几何体对象
	osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec3Array> normal = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;//创建颜色数组,逆时针排序
	osg::ref_ptr<osg::DrawElementsUByte> point = new osg::DrawElementsUByte(GL_POINTS);

	size_t count = 0;
	std::string errInfo;
	try
	{
		while (reader.ReadNextPoint()) {
			double  x, y, z;
			double  r, g, b, w;

			liblas::Point const& p = reader.GetPoint();

			x = p.GetX();
			y = p.GetY();
			z = p.GetZ();

			r = p.GetColor().GetRed() / 255.0;
			g = p.GetColor().GetGreen() / 255.0;
			b = p.GetColor().GetBlue() / 255.0;
			w = 1.0;

			osg::Vec3 single_point(x, y, z);
			osg::Vec4 single_color(r, g, b, w);

			vert->push_back(single_point);
			color->push_back(single_color);

			point->push_back(count);
			++count;//points num
		}
	}
	catch (std::exception & e)
	{
		errInfo = e.what();
	}
	catch (...)
	{
		errInfo = "get unknown exception";
	}


	this->point_num = count;

	normal->push_back(osg::Vec3(0.0, 0.0, 1.0));

	geo_point->setVertexArray(vert.get());
	geo_point->setNormalArray(normal.get());
	geo_point->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geo_point->setColorArray(color.get());
	geo_point->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geo_point->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, count));

	this->removeDrawables(0, this->getNumDrawables());//读取文件前先剔除节点内所有的几何体
	this->addDrawable(geo_point.get());//加入当前新的点云几何绘制体
}

void PointCloud::readLasData(const std::string & openfileName, int & rate, bool & isCancel) {
	//std::fstream ifs;
	//ifs.open(openfileName.c_str(), std::ios::in | std::ios::binary);
	//if (!ifs)
	//	return;

	if (isCancel) {
		return;
	}

	LASreadOpener opener;
	opener.set_file_name(openfileName.c_str());
	if (false == opener.active()) {
		return;
	}
	LASreader * reader = opener.open();
	if (reader == nullptr) {
		return;
	}

	geo_point = new osg::Geometry;//创建一个几何体对象
	osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec3Array> normal = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;//创建颜色数组,逆时针排序
	osg::ref_ptr<osg::DrawElementsUByte> point = new osg::DrawElementsUByte(GL_POINTS);

	size_t numInterval = this->getPointNum() / 100;
	size_t point_count = 0;
	int m_Rate = 1;
	std::string errInfo;
	try
	{
		while (reader->read_point()) {
			if (isCancel) {
				break;
			}
			const LASpoint & p = reader->point;
			osg::Vec3 single_point(p.get_x(), p.get_y(), p.get_z());
			osg::Vec4 single_color(p.get_R() / 255.0, p.get_G() / 255.0, p.get_B() / 255.0, 1.0);

			vert->push_back(single_point);
			color->push_back(single_color);
			point->push_back(point_count);

			if (++point_count > numInterval * m_Rate) {
				rate = m_Rate++;
			}
		}
	}
	catch (std::exception & e) {
		errInfo = e.what();
	}
	catch (...) {
		errInfo = "get unknown exception";
	}

	if (this->getPointNum() != point_count) {
		errInfo = "The point num is not same";
	}

	reader->close();
	delete reader;
	reader = nullptr;

	if (isCancel) {
		clearData();
		return;
	}

	normal->push_back(osg::Vec3(0.0, 0.0, 1.0));

	geo_point->setVertexArray(vert.get());
	geo_point->setNormalArray(normal.get());
	geo_point->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geo_point->setColorArray(color.get());
	geo_point->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geo_point->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, point_count));

	this->removeDrawables(0, this->getNumDrawables());//读取文件前先剔除节点内所有的几何体
	this->addDrawable(geo_point.get());//加入当前新的点云几何绘制体
}

void PointCloud::readPoints(const std::string & openfileName, int & rate, bool & isCancel) {
	QFile text_file(QString::fromStdString(openfileName));
	if (!text_file.open(QFile::ReadOnly | QIODevice::Text))	{
		return;
	}

	if (isCancel) {
		return;
	}
		
	geo_point = new osg::Geometry;//创建一个几何体对象
	osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec3Array> normal = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;//创建颜色数组,逆时针排序
	osg::ref_ptr<osg::DrawElementsUByte> point = new osg::DrawElementsUByte(GL_POINTS);

	size_t numInterval = (size_t)(this->point_num / 100);
	int m_Rate = 1;
	size_t point_count = 0;
	uchar * fpr = text_file.map(0, text_file.size());
		
	char *ss = strdup((char*)fpr);
	char delim[] = " ,";
	char *ppp;
	char *lineSubstr = nullptr;

	osg::Vec3 single_point(0.0, 0.0, 0.0);
	osg::Vec4 single_color(0.0, 0.0, 1.0, 1.0);//默认为蓝色点

	char * substr = strtok_s(ss, "\n", &ppp);
	vector<float> xyz_list;

	bool isOnlyXYZ = (this->getType() == TXT_COLOR ? false : true);
	if (isOnlyXYZ) {
		QColor curColor = Qt::blue;
		point_color = curColor;
		color->push_back(single_color);

		while (substr) {
			if (isCancel) {
				break;
			}
			xyz_list.clear();
			lineSubstr = strtok(substr, delim);
			while (lineSubstr != nullptr) {
				xyz_list.push_back(atof(lineSubstr));
				lineSubstr = strtok(nullptr, delim);
			}

			if (xyz_list.size() != 3) {
				continue;
			}

			single_point.set(xyz_list[0], xyz_list[1], xyz_list[2]);
			vert->push_back(single_point);

			if (++point_count > numInterval * m_Rate) {
				rate = m_Rate++;
			}
			substr = strtok_s(NULL, "\n", &ppp);
		}
	}
	else {
		while (substr) {
			if (isCancel) {
				break;
			}
			xyz_list.clear();
			lineSubstr = strtok(substr, delim);
			while (lineSubstr != nullptr) {
				xyz_list.push_back(atof(lineSubstr));
				lineSubstr = strtok(nullptr, delim);
			}

			if (xyz_list.size() != 7) {
				continue;
			}

			single_point.set(xyz_list[0], xyz_list[1], xyz_list[2]);
			single_color.set(xyz_list[3], xyz_list[4], xyz_list[5], xyz_list[6]);

			vert->push_back(single_point);
			color->push_back(single_color);

			if (++point_count > numInterval * m_Rate) {
				rate = m_Rate++;
			}
			substr = strtok_s(nullptr, "\n", &ppp);
		}
	}	

	if (isCancel) {
		clearData();
		return;
	}

	this->setPointNum(point_count);

	normal->push_back(osg::Vec3(0.0, 0.0, 1.0));
	geo_point->setVertexArray(vert.get());
	geo_point->setNormalArray(normal.get());
	geo_point->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geo_point->setColorArray(color.get());
	geo_point->setColorBinding(isOnlyXYZ ? osg::Geometry::BIND_OVERALL : osg::Geometry::BIND_PER_VERTEX);

	geo_point->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, point_count));
	this->removeDrawables(0, this->getNumDrawables());//读取文件前先剔除节点内所有的几何体
	this->addDrawable(geo_point.get());//加入当前新的点云几何绘制体
}

void PointCloud::setShowBoundingBox(bool isShow) {
	//首次初始化外接框
	initBoundingBox();
	if (geo_point && geo_point->getNodeMask()) {
		if (geo_bounding_node) {
			geo_bounding_node->setNodeMask(isShow);
		}
	}
}

void PointCloud::initBoundingBox() {
	if (nullptr == geo_point) {
		return;
	}
	if (geo_bounding_node && geo_bounding_box) {
		return;
	}

	geo_bounding_node = new osg::Geode;
	geo_bounding_box = new osg::Geometry;//创建一个几何体对象
	osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;//创建颜色数组,逆时针排序
	osg::ref_ptr<osg::DrawElementsUByte> quad = new osg::DrawElementsUByte(GL_LINES);

	float box_xmin = geo_point->getBoundingBox().xMin();
	float box_ymin = geo_point->getBoundingBox().yMin();
	float box_zmin = geo_point->getBoundingBox().zMin();
	float box_xmax = geo_point->getBoundingBox().xMax();
	float box_ymax = geo_point->getBoundingBox().yMax();
	float box_zmax = geo_point->getBoundingBox().zMax();

	//外接包围盒的世界坐标8至点
	osg::Vec3 p0(box_xmin, box_ymin, box_zmin);
	osg::Vec3 p1(box_xmax, box_ymin, box_zmin);
	osg::Vec3 p2(box_xmax, box_ymax, box_zmin);
	osg::Vec3 p3(box_xmin, box_ymax, box_zmin);
	osg::Vec3 p4(box_xmin, box_ymin, box_zmax);
	osg::Vec3 p5(box_xmax, box_ymin, box_zmax);
	osg::Vec3 p6(box_xmax, box_ymax, box_zmax);
	osg::Vec3 p7(box_xmin, box_ymax, box_zmax);

	osg::Vec4 out_line_color(1.0, 1.0, 0.0, 1.0);//定义外部线框的颜色

	//单一网格的八顶点
	vert->push_back(p0);
	vert->push_back(p1);
	vert->push_back(p2);
	vert->push_back(p3);
	vert->push_back(p4);
	vert->push_back(p5);
	vert->push_back(p6);
	vert->push_back(p7);

	for (int k = 0; k < 8; ++k)	{
		color->push_back(out_line_color);
	}

	{
		quad->push_back(0);
		quad->push_back(1);

		quad->push_back(0);
		quad->push_back(3);

		quad->push_back(0);
		quad->push_back(4);

		quad->push_back(2);
		quad->push_back(1);

		quad->push_back(2);
		quad->push_back(3);

		quad->push_back(2);
		quad->push_back(6);

		quad->push_back(5);
		quad->push_back(1);

		quad->push_back(5);
		quad->push_back(4);

		quad->push_back(5);
		quad->push_back(6);

		quad->push_back(7);
		quad->push_back(3);

		quad->push_back(7);
		quad->push_back(4);

		quad->push_back(7);
		quad->push_back(6);
	}

	geo_bounding_box->setVertexArray(vert.get());
	geo_bounding_box->setColorArray(color.get());
	geo_bounding_box->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geo_bounding_box->addPrimitiveSet(quad);
	geo_bounding_node->addDrawable(geo_bounding_box.get());
	this->addChild(geo_bounding_node.get());

	osg::ref_ptr<osg::StateSet> stateset = geo_bounding_box->getOrCreateStateSet();
	stateset->setMode(GL_BLEND, osg::StateAttribute::ON);//开启Alpha混合，实现透明度
	stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);//设置渲染模式		
	stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);//取消深度测试
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);//关闭关照效果，这样任意面均可实现半透明效果
	osg::ref_ptr<osg::PolygonMode> polyMode = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);//设置网格模式
	stateset->setAttribute(polyMode);

	osg::ref_ptr<osg::LineWidth> line_width = new osg::LineWidth(1.0);
	stateset->setAttribute(line_width);
}

//获取给点云数据的最大最小范围
point_MAXMIN* PointCloud::getMinMaxXYZ_POINTS() {
	if (Max_area) {
		return Max_area;
	}

	Max_area = new point_MAXMIN;
	vector<float> x_list, y_list, z_list;
	osg::Vec3Array * pointArry = this->getVertArry<osg::Vec3Array>();

	if (pointArry == nullptr) {
		return nullptr;
	}

	for (const auto & curP : *pointArry) {
		x_list.push_back(curP.x());
		y_list.push_back(curP.y());
		z_list.push_back(curP.z());
	}

	vector<float>::iterator xmax = max_element(begin(x_list), end(x_list));
	vector<float>::iterator ymax = max_element(begin(y_list), end(y_list));
	vector<float>::iterator zmax = max_element(begin(z_list), end(z_list));

	vector<float>::iterator xmin = min_element(begin(x_list), end(x_list));
	vector<float>::iterator ymin = min_element(begin(y_list), end(y_list));
	vector<float>::iterator zmin = min_element(begin(z_list), end(z_list));

	Max_area->xmax = *xmax;
	Max_area->ymax = *ymax;
	Max_area->zmax = *zmax;
	Max_area->xmin = *xmin;
	Max_area->ymin = *ymin;
	Max_area->zmin = *zmin;

	return Max_area;
}

PCloudManager::PCloudManager(osg::ref_ptr<osg::Group> root) {
	m_root = root;
}

PCloudManager::~PCloudManager() {
	all_pcloud_map.clear();
	selected_pcloud_list.clear();
	m_root = nullptr;
}

PointCloud * PCloudManager::addPointCloud(const std::string & pName) {
	PointCloud * pcloud = new PointCloud();
	pcloud->setName(pName);
	all_pcloud_map.emplace(pcloud->getName(), pcloud);
	if (m_root) {
		m_root->addChild(pcloud);
	}
	return pcloud;
}

void PCloudManager::removePointCloud(PointCloud* pcloud) {
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
}

void PCloudManager::removePointCloud(const std::string & pName) {
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
}

void PCloudManager::setShowPointCloud(const std::string & pName, bool isShow) {
	PointCloud * curP = getPointCloud(pName);
	if (curP) {
		curP->setNodeMask(isShow);
	}
}

void PCloudManager::setShowBox(const std::string & pName, bool isShow) {
	PointCloud * curP = getPointCloud(pName);
	if (curP) {
		curP->setShowBoundingBox(isShow);
	}
}

PointCloud* PCloudManager::getPointCloud(const std::string & pName) {
	if (pName.empty()) {
		return nullptr;
	}
	auto iter = all_pcloud_map.find(pName);
	if (iter != all_pcloud_map.end()) {
		return iter->second;
	}
	return nullptr;
}

void PCloudManager::clearSelectedState() {
	for (const auto & curP : selected_pcloud_list) {
		curP->setSelected(false);
	}
	selected_pcloud_list.clear();
}

bool PCloudManager::setSelectState(const std::string & pName, bool isSelected) {
	if (pName.empty()) {
		return false;
	}
	PointCloud * curPCl = getPointCloud(pName);
	if (curPCl) {
		curPCl->setSelected(isSelected);
		if (isSelected) {
			selected_pcloud_list.emplace_back(curPCl);
		}
		else {
			selected_pcloud_list.remove(curPCl);
		}
		return true;
	}
	return false;
}

size_t PCloudManager::selectedPcloudNum() const {
	return selected_pcloud_list.size();
}

void PCloudManager::saveSelectedToFile(const std::string & saveFileName) {
	if (saveFileName.empty() || saveFileName.find_last_of(".") < 0) {
		return;
	}

	std::ofstream outf(saveFileName, std::ios::out | std::ios::binary);
	if (!outf.is_open()) {
		return;
	}

	osg::Vec3f singP;
	osg::ref_ptr<osg::Vec3Array> vertAll = new osg::Vec3Array;
	size_t allPointNum = 0;
	for (const auto item : selected_pcloud_list) {
		osg::Vec3Array* vertices = item->getVertArry<osg::Vec3Array>();
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

void PCloudManager::setSelectPointSize(size_t size) {
	for (auto curP : selected_pcloud_list) {
		curP->setPointSize(size);
	}
}

void PCloudManager::setSelectPointColor(const QColor & color) {
	for (auto curP : selected_pcloud_list) {
		curP->setPointColor(color);
	}
}