#pragma once
#include <vector>

//��һ��Ľṹ�嶨��
struct Point {
	//����
	float p_x;
	float p_y;
	float p_z;

	//��ɫ
	float C_r;
	float C_g;
	float C_b;
	float C_w;

	std::vector<int> octreeIndex;
	int voxel_index[3];
	int color_type;
	//vector<int> p_depth;
};

//��ά���񶥵�����ṹ��
struct point_MAXMIN {
	float xmin, ymin, zmin;
	float xmax, ymax, zmax;
};

//��ά���񶥵�����ṹ��
struct point2D_MAXMIN {
	float xmin, ymin;
	float xmax, ymax;

	point2D_MAXMIN() = default;
	point2D_MAXMIN(const point2D_MAXMIN & other) {
		this->xmin = other.xmin;
		this->ymin = other.ymin;
		this->xmax = other.xmax;
		this->ymax = other.ymax;
	}
};
