#pragma once
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

class PCACalculator{
public:
	PCACalculator();
	PCACalculator(const std::vector<float> &scanned, const std::vector<float> &model);
	~PCACalculator();

	Eigen::Matrix4f get_init_mat();
	Eigen::Matrix4f get_init_mat(const std::vector<float> &scanned, const std::vector<float> &model);
	void init_matching(std::vector<float>& floating, float* translation, float* rotation);
	void set_data(const std::vector<float> &scanned, const std::vector<float> &model);
	void calc_and_apply_rot();
private:
	Eigen::Matrix3f rotation_m_;
	Eigen::Vector3f translation_v_;

	std::vector<float> scanned_;
	std::vector<float> model_;
	Eigen::Matrix4f init_mat;
	
	void move_obj_to_origin();
	void applyMat_ToScan(const Eigen::Matrix3f &rot_mat);
	
	Eigen::Matrix3f calc_cov_mat(const std::vector<float> &data);
	std::vector<double> calc_mean(const std::vector<float> &data);
};