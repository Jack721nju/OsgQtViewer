#include "ProjectToXY.h"

PaintArea::PaintArea(QWidget* parent) : QWidget(parent)
{
	//初始化绘制区域范围
	image = new QImage(600, 600, QImage::Format_RGB32);
	QColor backcolor = qRgb(255, 255, 255);
	image->fill(backcolor);
}

PaintArea::PaintArea(int widthX, int heightY)
{
	//初始化绘制区域范围
	image = new QImage(widthX, heightY, QImage::Format_RGB32);
	QColor backcolor = qRgb(255, 255, 255);
	image->fill(backcolor);
}

void PaintArea::paintEvent(QPaintEvent *)
{
	QPainter painter(this);
	painter.drawImage(20, 20, *image);
}

void PaintArea::drawPoints(QPointF points[], int point_num, int point_size, QColor point_color)
{
	if (!image)
		return;

	QPen pen;
	pen.setWidth(point_size);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(point_color);

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);

	for (int k = 0; k < point_num; k++)
	{
		points[k] = QPointF(points[k].x(), (points[k].y()));
	}

	painter.drawPoints(points, point_num);
}

void PaintArea::drawText(QPointF pos, QString text)
{
	if (!image)
		return;

	QPen pen;
	pen.setWidth(2);
	pen.setColor(QColor(0, 0, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);

	painter.drawText(pos, text);
}

void PaintArea::drawGridWithFillColor(SingleGrid2D* eachGrid, QColor curGridColor)
{
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(Qt::black);

	QBrush brush;
	brush.setColor(curGridColor);
	brush.setStyle(Qt::SolidPattern);

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);
	painter.setBrush(brush);//设置画刷形式

	QPoint point_LeftBottom(eachGrid->curGridInfo.Min_X, eachGrid->curGridInfo.Min_Y);
	QPoint point_RightBottom(eachGrid->curGridInfo.Max_X, eachGrid->curGridInfo.Min_Y);

	QPoint point_LeftTop(eachGrid->curGridInfo.Min_X, eachGrid->curGridInfo.Max_Y);
	QPoint poitn_RightTop(eachGrid->curGridInfo.Max_X, eachGrid->curGridInfo.Max_Y);

	float originX = eachGrid->curGridInfo.Min_X;
	float originY = eachGrid->curGridInfo.Min_Y;
	float width = eachGrid->curGridInfo.Max_X - eachGrid->curGridInfo.Min_X;
	float height = eachGrid->curGridInfo.Max_Y - eachGrid->curGridInfo.Min_Y;
	painter.drawRect(originX, originY, width, height);

}

void PaintArea::drawGrid(SingleGrid2D* eachGrid)
{
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(QColor(0, 0, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);

	QPoint point_LeftBottom(eachGrid->curGridInfo.Min_X, eachGrid->curGridInfo.Min_Y);
	QPoint point_RightBottom(eachGrid->curGridInfo.Max_X, eachGrid->curGridInfo.Min_Y);

	QPoint point_LeftTop(eachGrid->curGridInfo.Min_X, eachGrid->curGridInfo.Max_Y);
	QPoint poitn_RightTop(eachGrid->curGridInfo.Max_X, eachGrid->curGridInfo.Max_Y);

	painter.drawLine(point_LeftBottom, point_RightBottom);
	painter.drawLine(point_RightBottom, poitn_RightTop);
	painter.drawLine(poitn_RightTop, point_LeftTop);
	painter.drawLine(point_LeftTop, point_LeftBottom);

}

void PaintArea::drawLines(vector<Edge> line_list)
{
	if (!image)
		return;

	QPen pen;
	pen.setWidth(2);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(QColor(255, 0, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);

	for (int k = 0; k < line_list.size(); k++)
	{
		QPoint pointA(line_list[k].point_A.x(), line_list[k].point_A.y());
		QPoint pointB(line_list[k].point_B.x(), line_list[k].point_B.y());

		painter.drawLine(pointA, pointB);
	}
}

void PaintArea::drawCircles(vector<osg::Vec2> center_list, int radius)
{
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(QColor(0, 255, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);

	for (int i = 0; i < center_list.size(); i++)
	{
		QPoint center(center_list[i].x(), center_list[i].y());
		painter.drawEllipse(center, radius, radius);
	}
}

void PaintArea::drawCircles(vector<osg::Vec3> circle_list, vector<int> Size_List)
{
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);


	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式


	for (int i = 0; i < circle_list.size(); i++)
	{
		QPoint center(circle_list[i].x(), circle_list[i].y());
		int radius = (int)(circle_list[i].z());

		if (Size_List[i] == 0)
		{
			pen.setColor(QColor(0, 255, 0, 100));
		}
		else
			if (Size_List[i] == 1)
			{
				pen.setColor(QColor(0, 0, 200, 200));
			}
			else
				if (Size_List[i] == 2)
				{
					pen.setColor(QColor(250, 100, 0));
				}

		painter.setPen(pen);
		painter.drawEllipse(center, radius, radius);
	}
}

void PaintArea::drawAxis()
{
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(QColor(0, 0, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);

	origin_point_X = 50;
	origin_point_Y = 500;//坐标系原点

	axis_width = 500;//确定坐标轴长度
	axis_height = 500;//高度

					  //painter.drawRect(5, 5, 520 - 5, 520 - 5);//绘制区域的矩形范围，保留5左右的间隙
	painter.drawLine(origin_point_X - 20, origin_point_Y, axis_width + origin_point_X, origin_point_Y);//绘制坐标轴X
	painter.drawLine(origin_point_X, origin_point_Y + 20, origin_point_X, origin_point_Y - axis_height);//绘制坐标轴Y

}

void PaintArea::drawDegreeLines(QString x_axis_name, QString y_axis_name, float base_x, float base_y, float delt_x, float delt_y)
{
	if (!image)
		return;

	QPen pen_text;
	pen_text.setWidth(2);
	pen_text.setColor(QColor(0, 0, 0));

	QPainter painter(image);
	painter.setPen(pen_text);
	painter.drawText(QPoint(280, 530), x_axis_name);
	painter.drawText(QPoint(10, 250), y_axis_name);

	//绘制刻度线
	QPen penDegree;
	penDegree.setColor(Qt::black);
	penDegree.setWidth(2);
	painter.setPen(penDegree);

	int divde_num = 10;

	for (int i = 0; i < divde_num; i++)
	{
		painter.drawLine((i + 1)*axis_width / divde_num, origin_point_Y, (i + 1)*axis_width / divde_num, origin_point_Y + 3);
		painter.drawText((i + 1)*axis_width / divde_num - 7, origin_point_Y + 15, QString::number((int)(base_x + (i + 1) * (abs(delt_x)) / divde_num)));
	}

	for (int i = 0; i < divde_num; i++)
	{
		painter.drawLine(origin_point_X, axis_height - (i + 1)*axis_height / divde_num, origin_point_X - 3, axis_height - (i + 1)*axis_height / divde_num);
		painter.drawText(origin_point_X - 15, axis_height - (i + 1)*axis_height / divde_num + 10, QString::number((int)(base_y + (i + 1) * (abs(delt_y)) / divde_num)));
	}
}
