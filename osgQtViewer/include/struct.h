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

//��Χ�������߽�ṹ��
struct point_MAXMIN {
	float xmin, ymin, zmin;
	float xmax, ymax, zmax;
};
