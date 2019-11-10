#define _SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING

#include <vector>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include "PCACalculator.h"

int main() {

	pcl::PointCloud<pcl::PointXYZ> scanned_pointcloud_pcl, model_pointcloud, save_pointcloud;
	//pcl::io::loadOBJFile("data/scan_at_origin.obj", scanned_pointcloud_pcl);
	pcl::io::loadPLYFile("data/partialy_deleted.ply", scanned_pointcloud_pcl);
	pcl::io::loadOBJFile("data/body_half_cut.obj", model_pointcloud);

	std::vector<float> scanned_xyz;
	std::vector<float> model_xyz;

	int scanned_points = scanned_pointcloud_pcl.points.size();
	int model_points = model_pointcloud.points.size();

	for (int i = 0; i < scanned_points; ++i) {
		scanned_xyz.emplace_back((float)scanned_pointcloud_pcl.points.at(i).x);
		scanned_xyz.emplace_back((float)scanned_pointcloud_pcl.points.at(i).y);
		scanned_xyz.emplace_back((float)scanned_pointcloud_pcl.points.at(i).z);
	}
	for (int i = 0; i < model_points; ++i) {
		model_xyz.emplace_back((float)model_pointcloud.points.at(i).x);
		model_xyz.emplace_back((float)model_pointcloud.points.at(i).y);
		model_xyz.emplace_back((float)model_pointcloud.points.at(i).z);
	}



	PCACalculator pca(scanned_xyz, model_xyz);
	pca.calc_and_apply_rot();


	// Scan Data와 Model Data 한 ply로 합치기
	//for (int i = 0; i < model_pointcloud.points.size(); ++i) {
	//	scanned_pointcloud_pcl.push_back(model_pointcloud.points.at(i));
	//}
	//pcl::io::savePLYFile("save.ply", scanned_pointcloud_pcl);


	return 0;
}