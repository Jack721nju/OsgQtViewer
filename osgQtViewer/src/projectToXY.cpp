#include "ProjectToXY.h"

PaintArea::PaintArea(QWidget* parent) : QWidget(parent) {
	//��ʼ����������Χ
	image = new QImage(600, 600, QImage::Format_RGB32);
	const QColor &backcolor = qRgb(255, 255, 255);
	image->fill(backcolor);
}

PaintArea::PaintArea(int widthX, int heightY) {
	//��ʼ����������Χ
	image = new QImage(widthX, heightY, QImage::Format_RGB32);
	const QColor & backcolor = qRgb(255, 255, 255);
	image->fill(backcolor);
}

void PaintArea::paintEvent(QPaintEvent *) {
	QPainter painter(this);
	painter.drawImage(20, 20, *image);
}

void PaintArea::drawPoints(QPointF points[], int point_num, int point_size, QColor point_color) {
	if (!image)
		return;

	QPen pen;
	pen.setWidth(point_size);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(point_color);

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//���÷����ģʽ
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
	painter.setRenderHint(QPainter::Antialiasing, true);//���÷����ģʽ
	painter.setPen(pen);
	painter.drawText(pos, text);
}

void PaintArea::drawGridWithFillColor(SingleGrid2D* eachGrid, QColor curGridColor) {
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
	painter.setRenderHint(QPainter::Antialiasing, true);//���÷����ģʽ
	painter.setPen(pen);
	painter.setBrush(brush);//���û�ˢ��ʽ

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

void PaintArea::drawGrid(SingleGrid2D* eachGrid) {
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(QColor(0, 0, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//���÷����ģʽ
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

void PaintArea::drawLines(vector<Edge> line_list) {
	if (!image)
		return;

	QPen pen;
	pen.setWidth(2);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(QColor(255, 0, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//���÷����ģʽ
	painter.setPen(pen);

	for (const auto & curLine : line_list) {
		QPoint pointA(curLine.point_A.x(), curLine.point_A.y());
		QPoint pointB(curLine.point_B.x(), curLine.point_B.y());
		painter.drawLine(pointA, pointB);
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
	painter.setRenderHint(QPainter::Antialiasing, true);//���÷����ģʽ
	painter.setPen(pen);

	for (const auto &curCenter : center_list) {
		QPoint center(curCenter.x(), curCenter.y());
		painter.drawEllipse(center, radius, radius);
	}
}

void PaintArea::drawCircles(const PointV3List &circle_list, const vector<int> &Size_List) {
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//���÷����ģʽ

	for (int i = 0; i < circle_list.size(); i++) {
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

void PaintArea::drawAxis() {
	if (!image)
		return;

	QPen pen;
	pen.setWidth(1);
	pen.setStyle(Qt::SolidLine);
	pen.setColor(QColor(0, 0, 0));

	QPainter painter(image);
	painter.setRenderHint(QPainter::Antialiasing, true);//���÷����ģʽ
	painter.setPen(pen);

	origin_point_X = 50;
	origin_point_Y = 500;//����ϵԭ��

	axis_width = 500;//ȷ�������᳤��
	axis_height = 500;//�߶�

	//painter.drawRect(5, 5, 520 - 5, 520 - 5);//��������ľ��η�Χ������5���ҵļ�϶
	painter.drawLine(origin_point_X - 20, origin_point_Y, axis_width + origin_point_X, origin_point_Y);//����������X
	painter.drawLine(origin_point_X, origin_point_Y + 20, origin_point_X, origin_point_Y - axis_height);//����������Y
}

void PaintArea::drawDegreeLines(QString x_axis_name, QString y_axis_name, float base_x, float base_y, float delt_x, float delt_y) {
	if (!image)
		return;

	QPen pen_text;
	pen_text.setWidth(2);
	pen_text.setColor(QColor(0, 0, 0));

	QPainter painter(image);
	painter.setPen(pen_text);
	painter.drawText(QPoint(280, 530), x_axis_name);
	painter.drawText(QPoint(10, 250), y_axis_name);

	//���ƿ̶���
	QPen penDegree;
	penDegree.setColor(Qt::black);
	penDegree.setWidth(2);
	painter.setPen(penDegree);

	int divde_num = 10;

	for (int i = 0; i < divde_num; ++i)	{
		painter.drawLine((i + 1)*axis_width / divde_num, origin_point_Y, (i + 1)*axis_width / divde_num, origin_point_Y + 3);
		painter.drawText((i + 1)*axis_width / divde_num - 7, origin_point_Y + 15, QString::number((int)(base_x + (i + 1) * (abs(delt_x)) / divde_num)));
	}

	for (int i = 0; i < divde_num; ++i ) {
		painter.drawLine(origin_point_X, axis_height - (i + 1)*axis_height / divde_num, origin_point_X - 3, axis_height - (i + 1)*axis_height / divde_num);
		painter.drawText(origin_point_X - 15, axis_height - (i + 1)*axis_height / divde_num + 10, QString::number((int)(base_y + (i + 1) * (abs(delt_y)) / divde_num)));
	}
}
