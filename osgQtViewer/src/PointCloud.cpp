#include "PointCloud.h"

using namespace std;

PointCloud::PointCloud(osg::ref_ptr<osg::Group> root) {
	point_num = 0;
	point_size = 1.0;
}

PointCloud::~PointCloud() {
	point_num = 0;
	point_size = 1.0;
	point_name = "";
	this->removeDrawables(0, this->getNumDrawables());
	geo_point = nullptr;
}


void PointCloud::readLasData(const std::string & openfileName) {
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
	std::fstream ifs;
	ifs.open(openfileName.c_str(), std::ios::in | std::ios::binary);
	if (!ifs)
		return;

	if (isCancel) {
		return;
	}

	liblas::ReaderFactory f;
	liblas::Reader & reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	geo_point = new osg::Geometry;//创建一个几何体对象
	osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec3Array> normal = new osg::Vec3Array;//创建顶点数组,逆时针排序
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;//创建颜色数组,逆时针排序
	osg::ref_ptr<osg::DrawElementsUByte> point = new osg::DrawElementsUByte(GL_POINTS);

	size_t pointAllNum = this->getPointNum() / 100;
	size_t point_count = 0;
	int m_Rate = 1;
	std::string errInfo;
	try
	{
		while (reader.ReadNextPoint()) {
			if (isCancel) {
				break;
			}
			liblas::Point const& p = reader.GetPoint();
			osg::Vec3 single_point(p.GetX(), p.GetY(), p.GetZ());
			osg::Vec4 single_color(p.GetColor().GetRed() / 255.0, p.GetColor().GetGreen() / 255.0, p.GetColor().GetBlue() / 255.0, 1.0);

			vert->push_back(single_point);
			color->push_back(single_color);
			point->push_back(point_count);

			if (++point_count > pointAllNum * m_Rate) {
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