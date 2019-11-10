#define _SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING
#include "PCACalculator.h"
#include <cmath>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");

PCACalculator::PCACalculator(){
	std::cout << "Must set data before using" << std::endl;
}
PCACalculator::PCACalculator(const std::vector<float>& scanned, const std::vector<float>& model){
	scanned_.assign(scanned.begin(), scanned.end());
	model_.assign(model.begin(), model.end());

	move_obj_to_origin();

	//pcl::PointCloud<pcl::PointXYZ> save_scan, save_model;
	//for (int i = 0; i < scanned_.size(); i += 3) {
	//	pcl::PointXYZ point;
	//	point.x = scanned_.at(i);
	//	point.y = scanned_.at(i + 1);
	//	point.z = scanned_.at(i + 2);
	//	save_scan.push_back(point);
	//}
	//for (int i = 0; i < model_.size(); i += 3) {
	//	pcl::PointXYZ point;
	//	point.x = model_.at(i);
	//	point.y = model_.at(i + 1);
	//	point.z = model_.at(i + 2);
	//	save_model.push_back(point);
	//}
	//pcl::io::savePLYFile("scan_at_origin.ply", save_scan);
	//pcl::io::savePLYFile("model_at_origin.ply", save_model);
}
PCACalculator::~PCACalculator(){
}
void PCACalculator::set_data(const std::vector<float>& scanned, const std::vector<float>& model) {
	scanned_.assign(scanned.begin(), scanned.end());
	model_.assign(model.begin(), model.end());
	move_obj_to_origin();
}

void PCACalculator::move_obj_to_origin() {
	std::vector<double> mean1, mean2;
	mean1 = calc_mean(scanned_);
	mean2 = calc_mean(model_);

	int size = scanned_.size();
	for (int i = 0; i < size; i += 3) {
		scanned_[i] -= mean1[0];
		scanned_[i + 1] -= mean1[1];
		scanned_[i + 2] -= mean1[2];
	}

	size = model_.size();
	for (int i = 0; i < size; i += 3) {
		model_[i] -= mean2[0];
		model_[i + 1] -= mean2[1];
		model_[i + 2] -= mean2[2];
	}

	translation_v_ = Eigen::Vector3f(mean1[0], mean1[1], mean1[2]);
}
Eigen::Matrix4f PCACalculator::get_init_mat(){
	return init_mat;
}
Eigen::Matrix4f PCACalculator::get_init_mat(const std::vector<float>& scanned, const std::vector<float>& model){
	scanned_.assign(scanned.begin(), scanned.end());
	model_.assign(model.begin(), model.end());
	return get_init_mat();
}

void PCACalculator::init_matching(std::vector<float>& floating, float* translation, float* rotation) {
	calc_and_apply_rot();

	floating.clear();
	//std::copy(scanned_.begin(), scanned_.end(), floating);
	floating.assign(scanned_.begin(), scanned_.end());

	translation[3 * 4 + 0] = -translation_v_(0);
	translation[3 * 4 + 1] = -translation_v_(1);
	translation[3 * 4 + 2] = -translation_v_(2);

	rotation[0*4 + 0] = rotation_m_(0, 0);	rotation[1*4 + 0] = rotation_m_(0, 1);	rotation[2*4 + 0] = rotation_m_(0, 2);
	rotation[0*4 + 1] = rotation_m_(1, 0);	rotation[1*4 + 1] = rotation_m_(1, 1);	rotation[2*4 + 1] = rotation_m_(1, 2);
	rotation[0*4 + 2] = rotation_m_(2, 0);	rotation[1*4 + 2] = rotation_m_(2, 1);	rotation[2*4 + 2] = rotation_m_(2, 2);
}

void PCACalculator::applyMat_ToScan(const Eigen::Matrix3f &rot_mat) {
	Eigen::Vector3f vec;
	int size = scanned_.size();

	for (int i = 0; i < size; i+=3) {
		vec.x() = scanned_[i];
		vec.y() = scanned_[i+1];
		vec.z() = scanned_[i+2];
		
		vec = rot_mat * vec;
		
		scanned_[i] = vec.x();
		scanned_[i+1] = vec.y();
		scanned_[i+2] = vec.z();
	}
}

void PCACalculator::calc_and_apply_rot() {
	Eigen::Matrix3f scan_cov_mat = calc_cov_mat(scanned_);
	Eigen::Matrix3f model_cov_mat = calc_cov_mat(model_);

	std::cout << "Scan Data Covariance Mat" << std::endl;
	std::cout << scan_cov_mat.format(OctaveFmt) << std::endl;
	std::cout << "Model Data Covariance Mat" << std::endl;
	std::cout << model_cov_mat.format(OctaveFmt) << std::endl;

	std::vector<Eigen::Vector3f> eigenvec_scan, eigenvec_model;
	std::vector<float> eigenval_scan, eigenval_model;
	
	// SelfAdjointEigenSolver
	// symmetric한 matrix의 eigen value와 vector를 구해준다.
	// symmentric의 성질을 이용하여, EigenSolver, ComplexEigenSolver 함수보다 더 빠르고 정확하게 값을 구한다.
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es_scan, es_model;
	es_scan.compute(scan_cov_mat);
	std::cout << es_scan.eigenvalues().format(OctaveFmt) << std::endl;
	std::cout << es_scan.eigenvectors().format(OctaveFmt) << std::endl;
	for (int i = 0; i < 3; i++)  eigenvec_scan.emplace_back(es_scan.eigenvectors().col(i));
	eigenval_scan.emplace_back(es_scan.eigenvalues().x());
	eigenval_scan.emplace_back(es_scan.eigenvalues().y());
	eigenval_scan.emplace_back(es_scan.eigenvalues().z());

	es_model.compute(model_cov_mat);
	std::cout << es_model.eigenvalues().format(OctaveFmt) << std::endl;
	std::cout << es_model.eigenvectors().format(OctaveFmt) << std::endl;
	for (int i = 0; i < 3; i++)  eigenvec_model.emplace_back(es_model.eigenvectors().col(i));
	eigenval_model.emplace_back(es_model.eigenvalues().x());
	eigenval_model.emplace_back(es_model.eigenvalues().y());
	eigenval_model.emplace_back(es_model.eigenvalues().z());

#if 1
	for (int i = 0; i < 500; i++) {
		scanned_.emplace_back(eigenvec_scan[0](0) * i);
		scanned_.emplace_back(eigenvec_scan[0](1) * i);
		scanned_.emplace_back(eigenvec_scan[0](2) * i);
	}
	for (int i = 0; i < 1000; i++) {
		scanned_.emplace_back(eigenvec_scan[1](0) * i);
		scanned_.emplace_back(eigenvec_scan[1](1) * i);
		scanned_.emplace_back(eigenvec_scan[1](2) * i);
	}
	for (int i = 0; i < 1500; i++) {
		scanned_.emplace_back(eigenvec_scan[2](0) * i);
		scanned_.emplace_back(eigenvec_scan[2](1) * i);
		scanned_.emplace_back(eigenvec_scan[2](2) * i);
	}

	for (int i = 0; i < 500; i++) {
		model_.emplace_back(eigenvec_model[0](0) * i);
		model_.emplace_back(eigenvec_model[0](1) * i);
		model_.emplace_back(eigenvec_model[0](2) * i);
	}
	for (int i = 0; i < 1000; i++) {
		model_.emplace_back(eigenvec_model[1](0) * i);
		model_.emplace_back(eigenvec_model[1](1) * i);
		model_.emplace_back(eigenvec_model[1](2) * i);
	}
	for (int i = 0; i < 1500; i++) {
		model_.emplace_back(eigenvec_model[2](0) * i);
		model_.emplace_back(eigenvec_model[2](1) * i);
		model_.emplace_back(eigenvec_model[2](2) * i);
	}

	pcl::PointCloud<pcl::PointXYZ> model_pc, scan_pc;
	for (int i = 0; i < scanned_.size(); i += 3) {
		pcl::PointXYZ point;
		point.x = scanned_.at(i);
		point.y = scanned_.at(i + 1);
		point.z = scanned_.at(i + 2);

		scan_pc.push_back(point);
	}
	for (int i = 0; i < model_.size(); i += 3) {
		pcl::PointXYZ point;
		point.x = model_.at(i);
		point.y = model_.at(i + 1);
		point.z = model_.at(i + 2);

		model_pc.push_back(point);
	}
	pcl::io::savePLYFile("eigen_dir_plus_scan.ply", scan_pc);
	pcl::io::savePLYFile("eigen_dir_plus_model.ply", model_pc);
#endif

	// 우리가 가지고 있는 model은 eigen vector가 비교적 xyz축에 잘 맞춰져있다.
	// 따라서 회전축으로 model의 회전축을 사용하는것이 좋아보인다.
	// 가장작은 eigen value를 갖는 eigen vector가 scan, model data 모두 z방향이 major 한 eigen vector이다.

	//std::cout <<
	//	eigenvec_scan[0].x() * eigenvec_model[0].x()
	//	+ eigenvec_scan[0].y() * eigenvec_model[0].y()
	//	+ eigenvec_scan[0].z() * eigenvec_model[0].z()
	//	<< std::endl << std::endl;
	//std::cout <<
	//	  eigenvec_scan[1].x() * eigenvec_model[1].x()
	//	+ eigenvec_scan[1].y() * eigenvec_model[1].y()
	//	+ eigenvec_scan[1].z() * eigenvec_model[1].z()
	//	<< std::endl << std::endl;

	//double cos_ = eigenvec_scan[1].dot(eigenvec_model[1]);
	//double rad_ = acos(cos_);
	//std::cout << "cos : " << cos_ << std::endl;
	//std::cout << "rad : " << rad_ << std::endl;

	//for (int i = 0; i < 3; i++) {
	//	std::cout << "Scan Eigen Vec " << i << std::endl;
	//	for (int j = 0; j < 3; j++) {
	//		std::cout << eigenvec_scan[i](j) << std::endl;
	//	}
	//}
	//for (int i = 0; i < 3; i++) {
	//	std::cout << "Model Eigen Vec " << i << std::endl;
	//	for (int j = 0; j < 3; j++) {
	//		std::cout << eigenvec_model[i](j) << std::endl;
	//	}
	//}

	double cos = eigenvec_scan[1].dot(eigenvec_model[1]);
	double rad = acos(cos);
	std::cout << "cos : " << cos << std::endl;
	std::cout << "rad : " << rad << std::endl;

	Eigen::Matrix3f rot_mat;
	rot_mat = Eigen::AngleAxisf(rad, eigenvec_model[2]);
	applyMat_ToScan(rot_mat);

	pcl::PointCloud<pcl::PointXYZ> save_scan;
	for (int i = 0; i < scanned_.size(); i += 3) {
		pcl::PointXYZ point;
		point.x = scanned_.at(i);
		point.y = scanned_.at(i + 1);
		point.z = scanned_.at(i + 2);
		save_scan.push_back(point);
	}
	pcl::io::savePLYFile("rotated.ply", save_scan);


	//rotation_m_ = rot_mat;

	//if (eigenscanned_[0](2) * eigenmodel_[0](2) > 0) {
	//	eigenscanned_[0] = -eigenscanned_[0];
	//}
	//cos = eigenscanned_[0].dot(eigenmodel_[0]);
	//rad = acos(cos);
	//std::cout << "cos : " << cos << std::endl;
	//std::cout << "rad : " << rad << std::endl;

	//rot_mat = Eigen::AngleAxisf(rad, eigenscanned_[2]);
	//apply_mat(rot_mat);

	//rotation_m_ = rot_mat* rotation_m_;
}

Eigen::Matrix3f PCACalculator::calc_cov_mat(const std::vector<float> &data){
	std::vector<double> mean(3);
	mean = calc_mean(data);
	//mean[0] = 0;
	//mean[1] = 0;
	//mean[2] = 0;

	Eigen::Matrix3f cov_mat;
	
	int size = data.size();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cov_mat(i,j) = 0.0;
			for (int k = 0; k < size; k += 3) {
				cov_mat(i, j) += (mean[i] - data[k + i]) * (mean[j] - data[k + j]);
			}
			cov_mat(i, j) /= (size / 3) - 1;
		}
	}
	return cov_mat;
}
std::vector<double> PCACalculator::calc_mean(const std::vector<float>& data){
	std::vector<double> mean(3, 0);
	
	int size = data.size();
	for (int i = 0; i < size; i += 3) {
		mean[0] += data[i];
		mean[1] += data[i+1];
		mean[2] += data[i+2];
	}
	
	mean[0] /= (size / 3);
	mean[1] /= (size / 3);
	mean[2] /= (size / 3);

	return mean;
}