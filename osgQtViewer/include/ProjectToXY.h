#pragma once
#include "osgQt.h"
#include "AlphaShape.h"

class PaintArea : public QWidget {
public:
	explicit PaintArea(QWidget* parent = nullptr);
	PaintArea(int widthX, int heightY);

	void paintEvent(QPaintEvent *);
	void drawAxis();
	void drawPoints(QPointF points[], int point_num, int point_size = 1.0, const QColor& point_color = Qt::blue);
	void drawCircles(const std::vector<osg::Vec2> &center_list, int radius);
	void drawCircles(const vector<osg::Vec3> &circle_list, const std::vector<int> & Size_List);
	void drawLines(const vector<Edge> & line_list);
	void drawText(QPointF pos, QString text);

	void drawGrid(SingleGrid2D* eachGrid);
	void drawGridWithFillColor(SingleGrid2D* eachGrid, const QColor& curGridColor, int delt_x = -10, int delt_y = 32);

	void drawDegreeLines(QString x_axis_name, QString y_axis_name, float base_x, float base_y, float delt_x, float delt_y);
	
private:
	QImage *image{nullptr};

public:
	int origin_point_X;
	int origin_point_Y;//坐标系原点

	int axis_width;//确定坐标轴宽度和高度
	int axis_height;
};