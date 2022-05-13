#include "ProjectToXY.h"

PaintArea::PaintArea(QWidget* parent) : QWidget(parent) {
	//初始化绘制区域范围
	image = new QImage(600, 600, QImage::Format_RGB32);
	const QColor &backcolor = qRgb(255, 255, 255);
	image->fill(backcolor);
}

PaintArea::PaintArea(int widthX, int heightY) {
	//初始化绘制区域范围
	image = new QImage(widthX, heightY, QImage::Format_RGB32);
	const QColor & backcolor = qRgb(255, 255, 255);
	image->fill(backcolor);
}

void PaintArea::paintEvent(QPaintEvent *) {
	QPainter painter(this);
	painter.drawImage(20, 20, *image);
}

void PaintArea::drawPoints(QPointF points[], int point_num, int point_size, const QColor& point_color) {
	if (!image)
		return;

	QPen pen;
	pen.setWidth(point_size);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(point_color);

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);
	painter.drawPoints(points, point_num);
}

void PaintArea::drawText(QPointF pos, QString text) {
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

void PaintArea::drawGridWithFillColor(float xmin, float ymin, float xmax, float ymax, const QColor& curGridColor, int delt_x, int delt_y) {
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

	float originX = xmin;
	float originY = ymin;
	float width = xmax - xmin;
	float height = ymax - ymin;
	painter.drawRect(originX + delt_x, originY + delt_y, width, height);
}

void PaintArea::drawGridWithFillColor(SingleGrid2D* eachGrid, const QColor& curGridColor, int delt_x, int delt_y) {
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
	painter.drawRect(originX + delt_x, originY + delt_y, width, height);
}

void PaintArea::drawGrid(SingleGrid2D* eachGrid) {
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

void PaintArea::drawLines(const vector<Edge> &line_list) {
	if (!image)
		return;

	QPen pen;
	pen.setWidth(2);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(QColor(255, 0, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);

	for (const auto & curLine : line_list) {
		const auto & pointA = curLine.point_A;
		const auto & pointB = curLine.point_B;
		painter.drawLine(QPoint(pointA.x(), pointA.y()), QPoint(pointB.x(), pointB.y()));
	}
}

void PaintArea::drawCircles(const vector<osg::Vec2> &center_list, int radius) {
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(QColor(0, 255, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式
	painter.setPen(pen);

	for (const auto &curCenter : center_list) {
		painter.drawEllipse(QPoint(curCenter.x(), curCenter.y()), radius, radius);
	}
}

void PaintArea::drawCircles(const vector<osg::Vec3> &circle_list, const vector<int> &Size_List) {
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式

	for (int i = 0; i < circle_list.size(); ++i) {
		QPoint center(circle_list[i].x(), circle_list[i].y());
		int radius = (int)(circle_list[i].z());

		switch (Size_List[i]) {
			case 0 :
				pen.setColor(QColor(0, 255, 0, 100));
				break;
			case 2:
				pen.setColor(QColor(0, 0, 200, 200));
				break;
			case 3:
				pen.setColor(QColor(250, 100, 0));
				break;
			default:
				break;
		}

		painter.setPen(pen);
		painter.drawEllipse(center, radius, radius);
	}
}

//绘制坐标轴
void PaintArea::drawAxis() {
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
	origin_point_Y = 530;//坐标系原点

	axis_width = 500;//确定坐标轴长度
	axis_height = 500;//高度

	//painter.drawRect(5, 5, 520 - 5, 520 - 5);//绘制区域的矩形范围，保留5左右的间隙
	painter.drawLine(origin_point_X - 20, origin_point_Y, origin_point_X + axis_width, origin_point_Y);//绘制坐标轴X
	painter.drawLine(origin_point_X, origin_point_Y + 20, origin_point_X, origin_point_Y - axis_height);//绘制坐标轴Y
}

//绘制坐标轴刻度线
void PaintArea::drawDegreeLines(QString x_axis_name, QString y_axis_name, float base_x, float base_y, float delt_x, float delt_y) {
	if (!image)
		return;

	QPen pen_text;
	pen_text.setWidth(2);
	pen_text.setColor(QColor(0, 0, 0));

	QPainter painter(image);
	painter.setPen(pen_text);
	painter.drawText(QPoint(280, 560), x_axis_name);
	painter.drawText(QPoint(10, 280), y_axis_name);

	//绘制刻度线
	QPen penDegree;
	penDegree.setColor(Qt::black);
	penDegree.setWidth(2);
	painter.setPen(penDegree);

	int divde_num = 10;

	for (int i = 0; i < divde_num; ++i)	{
		painter.drawLine((i + 1)*axis_width / divde_num, origin_point_Y, (i + 1)*axis_width / divde_num, origin_point_Y + 3);
		painter.drawText((i + 1)*axis_width / divde_num - 7, origin_point_Y + 18, QString::number((int)(base_x + (i + 1) * (abs(delt_x)) / divde_num)));
	}

	for (int i = 0; i < divde_num; ++i ) {
		painter.drawLine(origin_point_X, axis_height - (i + 1)*axis_height / divde_num, origin_point_X - 3, axis_height - (i + 1)*axis_height / divde_num);
		painter.drawText(origin_point_X - 30, axis_height - (i + 1)*axis_height / divde_num + 10, QString::number((int)(base_y + (i + 1) * (abs(delt_y)) / divde_num)));
	}
}
