#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>


#define __cv3__ *(glm::fvec3*)
#define __cv4__ *(glm::fvec4*)
#define __cm4__ *(glm::fmat4x4*)

glm::fvec3 tr_pt(const glm::fmat4x4& mat, const glm::fvec3& p) {
	glm::fvec4 _p(p, 1.f);
	_p = mat * _p;
	return glm::fvec3(_p.x / _p.w, _p.y / _p.w, _p.z / _p.w);
}

glm::fvec3 tr_vec(const glm::fmat4x4& mat, const glm::fvec3& v) {
	glm::fmat3x3 r = mat;
	glm::fvec3 _v = r * v;
	return _v;
}

int get_first_integer(const char *v) {
	int ival;
	std::string s(v);
	std::replace(s.begin(), s.end(), '/', ' ');
	sscanf(s.c_str(), "%d", &ival);
	return ival;
}

namespace helper {
	bool loadObj(std::string filename, std::vector<float> &coords, std::vector<int> &tris) {
		float x, y, z;
		char line[1024], v0[1024], v1[1024], v2[1024];

		// open the file, return if open fails
		//FILE *fp = fopen(filename, "r");
		//if (!fp) return;

		std::ifstream in(filename);
		if (!in.is_open())
			return false;

		// read lines from the file, if the first character of the
		// line is 'v', we are reading a vertex, otherwise, if the
		// first character is a 'f' we are reading a facet
		while (in.getline(line, 1024)) {
			if (line[0] == 'v' && line[1] != 'n') {
				sscanf(line, "%*s%f%f%f", &x, &y, &z);
				coords.push_back(x);
				coords.push_back(y);
				coords.push_back(z);
			} else if (line[0] == 'f') {
				sscanf(line, "%*s%s%s%s", v0, v1, v2);
				tris.push_back(get_first_integer(v0) - 1);
				tris.push_back(get_first_integer(v1) - 1);
				tris.push_back(get_first_integer(v2) - 1);
			}
		}

		in.close();

		std::cout << "[Info] " << filename << " is successfully loaded" << std::endl;
		std::cout << "       - # of vertices: " << coords.size() / 3 << std::endl;
		std::cout << "       - # of faces: " << tris.size() / 3 << std::endl;

		return true;
	}

	std::vector<float> depth2Pointcloud(cv::Mat& depth_image, float fx, float fy, float cx, float cy) {
		if (!depth_image.data) {
			std::cerr << "No depth data!!!" << std::endl;
			exit(EXIT_FAILURE);
		}

		float factor = 1000.f;

		std::vector<float> pointcloud_xyz;

#pragma omp parallel for
		for (int v = 0; v < depth_image.rows; ++v) {
			for (int u = 0; u < depth_image.cols; ++u) {
				float Z = depth_image.at<unsigned short>(v, u) / factor;

				float x, y, z;
				z = -Z; // 카메라의 -z-axis 위에 있음
				x = (u - cx) * Z / fx;
				y = -(v - cy) * Z / fy;

				if (x == 0.0 && y == 0.0 && z == 0.0)
					continue;

				pointcloud_xyz.emplace_back(x);
				pointcloud_xyz.emplace_back(y);
				pointcloud_xyz.emplace_back(z);
			}
		}

		return pointcloud_xyz;
	}
}