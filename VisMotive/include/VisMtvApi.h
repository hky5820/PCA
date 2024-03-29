/*
Copyright (c) 2019, Dongjoon Kim
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. Commercial
purposed SW that uses this library must be approved by the main contributor of 
Dongjoon Kim (korfriend@gmail.com).

Neither the name of the Seoul National University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#pragma once
#define __dojostatic extern "C" __declspec(dllexport)
#define __dojosclass class __declspec(dllexport)

#include <string>

//#define VERSION "1.0.a" // released at 19.07.30
//#define VERSION "1.0.b" // released at 19.08.02
//#define VERSION "1.0.c" // released at 19.08.14
//#define VERSION "1.0.d" // released at 19.08.29
//#define VERSION "1.0.e" // released at 19.09.06
//#define VERSION "1.0.f" // released at 19.09.10
//#define VERSION "1.0.g" // released at 19.09.18
//#define VERSION "1.1.a" // released at 19.10.03
//#define VERSION "1.1.b" // released at 19.10.18
#define VERSION "1.1.c__test" // released at 19.10.22

// Dongjoon's VisMotive interface.
namespace vzm
{
	struct CameraParameters
	{
		float pos[3], view[3], up[3]; // WS coordinates
		int projection_mode; // 0 : undefined, 1 : use ip_w, ip_h instead of fov_y, aspect_ratio, 2 : vice versa, 3: AR mode
		union {
			struct { // projection_mode == 1
				float ip_w, ip_h; // defined in CS
			};
			struct { // projection_mode == 2
				// fov_y should be smaller than PI
				float fov_y, aspect_ratio; // ip_w / ip_h
			};
			struct { // projection_mode == 3
				// AR mode (normal camera intrinsics..)
				float fx, fy, sc, cx, cy;
			};
		};
		int w, h; // resolution. note that the aspect ratio is recomputed w.r.t. w and h during the camera setting.
		bool is_rgba_write; // if false, use BGRA order
		CameraParameters() {
			projection_mode = 0; is_rgba_write = false; 
		};
	};

	struct ObjStates
	{
		float os2ws[16]; // 4x4 matrix col-major (same as in glm::fmat4x4)
		float emission, diffusion, specular, sp_pow; // Phong's material reflection model
		bool is_visible;
		bool ignore_vertex_color; // regardless, color[3] is always used for the transparency
		float color[4]; // rgba [0,1]
		float point_thickness; // when the object consists of point cloud
		float line_thickness; // not available for the wireframe's line
		bool is_wireframe; // only for polygonal mesh
		bool ignore_vertex_wirecolor; // regardless, color[3] is always used for the transparency
		float wire_color[4]; // rgba [0,1].. only for wireframe object
		// to do ... add more in future...

		ObjStates()
		{
			memset(os2ws, 0, sizeof(float) * 16);
			os2ws[0] = os2ws[5] = os2ws[10] = os2ws[15] = 1.f;
			emission = 0.2f, diffusion = 0.4f, specular = 0.4f, sp_pow = 10.f;
			is_visible = true;
			is_wireframe = false;
			ignore_vertex_color = false;
			ignore_vertex_wirecolor = true;
			color[0] = color[1] = color[2] = color[3] = 1.f;
			memset(wire_color, 0, sizeof(float) * 4);
			wire_color[3] = 1.f;
			point_thickness = 0;
			line_thickness = 0;
		}
	};

	struct SceneEnvParameters
	{
		float pos_light[3], dir_light[3];
		bool is_pointlight; // 'true' uses pos_light, 'false' uses dir_light
		bool is_on_camera; // 'true' sets cam params to light source. in this case, it is unnecessary to set pos_light and dir_light (ignore both)
		
		// to do ... add more in future...
	};

	__dojostatic bool InitEngineLib();
	__dojostatic bool DeinitEngineLib();

	// here, obj_id (without const) is [in/out].. in : when a registered object of obj_id exists, out : when there is no registered object of obj_id
	__dojostatic bool LoadModelFile(const std::string& filename, int& obj_id);
	__dojostatic bool GenerateArrowObject(const float* pos_s, const float* pos_e, const float radius, int& obj_id);
	// optional : rgb_list (if NULL, this is not used)
	__dojostatic bool GenerateSpheresObject(const float* xyzr_list, const float* rgb_list, const int num_spheres, int& obj_id);
	__dojostatic bool GenerateCylindersObject(const float* xyz_01_list, const float* radius_list, const float* rgb_list, const int num_cylinders, int& obj_id);
	// when line_thickness = 0, the line thickness is not used, depending on the line renderer of the rendering API.
	__dojostatic bool GenerateLinesObject(const float* xyz_01_list, const float* rgb_list, const int num_lines, int& obj_id);
	__dojostatic bool GenerateTrianglesObject(const float* xyz_012_list, const float* rgb_list, const int num_tris, int& obj_id);
	// optional : nrl_list, rgb_list, tex_list, idx_prims (if NULL, this is not used)
	// stride_prim_idx : 1 ==> point cloud, 2 ==> line, 3 ==> triangle
	__dojostatic bool GeneratePrimitiveObject(const float* xyz_list, const float* nrl_list, const float* rgb_list, const float* tex_list, const int num_vtx, const unsigned int* idx_prims, const int num_prims, const int stride_prim_idx, int& obj_id);
	// optional : nrl_list, rgb_list (if NULL, this is not used)
	__dojostatic bool GeneratePointCloudObject(const float* xyz_list, const float* nrl_list, const float* rgb_list, const int num_points, int& obj_id);
	__dojostatic bool GenerateTextObject(const float* xyz_LT_view_up, const std::string& text, const float font_height, bool bold, bool italic, int& obj_id);

	__dojostatic bool ReplaceOrAddSceneObject(const int scene_id, const int obj_id, const ObjStates& obj_states);
	__dojostatic bool GetSceneObjectState(const int scene_id, const int obj_id, ObjStates& obj_states);
	__dojostatic bool RemoveSceneObject(const int scene_id, const int obj_id);
	__dojostatic bool RemoveScene(const int scene_id);
	__dojostatic bool DeleteObject(const int obj_id); // the obj is deleted in memory
	__dojostatic bool SetSceneEnvParameters(const int scene_id, const SceneEnvParameters& env_params);
	// cam id is corresponding to a specific renderer and ip states
	__dojostatic bool SetCameraParameters(const int scene_id, const CameraParameters& cam_params, const int cam_id = 0);
	__dojostatic bool GetCameraParameters(const int scene_id, CameraParameters& cam_params, const int cam_id = 0);

	__dojostatic bool RenderScene(const int scene_id, const int cam_id = 0);
	__dojostatic bool GetRenderBufferPtrs(const int scene_id, unsigned char** ptr_rgba, float** ptr_zdepth, int* fbuf_w, int* fbuf_h, const int cam_id = 0);

	// etc
	__dojostatic bool GetPModelData(const int obj_id, float** pos_vtx, float** nrl_vtx, float** rgb_vtx, float** tex_vtx, int& num_vtx, unsigned int** idx_prims, int& num_prims, int& stride_prim_idx);

	// picking
	__dojostatic bool ValidatePickTarget(const int obj_id);
	__dojostatic bool PickObject(int& pick_obj_id, const int x, const int y, const int scene_id, const int cam_id = 0);
}

namespace vzmproc
{
	__dojostatic bool SimplifyPModelByUGrid(const int obj_src_id, const float cell_width, int& obj_dst_id);
	__dojostatic bool ComputePCA(const int obj_id, float(&egvals)[3], float(&egvecs)[9]);

	// kar-breast part // to do
	//__dojostatic bool ComputeMatchingTransform(const rgbd, obj_id, mat_tr[16]); // based on special-care ICP	//
}

namespace helpers
{
	//__dojostatic bool CopyObject(const int src_obj_id, const int dst_obj_id);
	__dojostatic bool ComputePCAc(const float* xyz_list, const int num_points, float(&egvals)[3], float(&egvecs)[9]);

	// at least 3 point-pairs are requested
	// matching based on least squares
	// assume each point pair has the same index of the point list (one-to-one);
	__dojostatic bool ComputeRigidTransform(const float* xyz_from_list, const float* xyz_to_list, const int num_pts, float(&mat_tr)[16]);

	// mat_ext : glm::fmat4x3 format, conventional camera system's extrinsic parameters (y down and z as view direction)
	// fx, fy : focusing parameters
	// sc : skew coefficient
	// cx, cy : principal point position
	// w, h : screen size
	// zn, zf : near and far plane
	// api_mode : 0 => opengl, 1 => direct3d
	// mat_ws2cs (view matrix), mat_cs2ps (projection matrix) : glm::fmat4x3 format, output
	// mat_ps2ss : glm::fmat4x3 format, output
	__dojostatic bool ComputeCameraRendererMatrice(const float* mat_ext,
		const float fx, const float fy, const float sc, const float cx, const float cy,
		const int w, const int h, const float zn, const float zf, const int api_mode,
		float* mat_ws2cs, float* mat_cs2ps, float* mat_ps2ss);
	__dojostatic bool ComputeCameraRendererParameters(const float* pos_xyz_ws, const float* pos_xy_ss, const int num_mks,
		float* cam_pos, float* cam_view, float* cam_up, float* fx, float* fy, float* sc, float* cx, float* cy);

	__dojostatic bool ComputeArCameraCalibrateInfo(const float* mat_camrb2ws, const float* calrb_xyz_ws, const float* calrb_xy_ss, const int num_mks,
		float* cam_pos_crbs, float* cam_view_crbs, float* cam_up_crbs, vzm::CameraParameters& cam_ar_mode_params);

	struct cam_pose
	{
		float pos[3], view[3], up[3]; // WS coordinates
	};

	__dojosclass arcball
	{
	public:
		arcball();
		~arcball();
		// stage_center .. fvec3
		bool intializer(const float* stage_center, const float stage_radius);
		// pos_xy .. ivec2
		bool start(const int* pos_xy, const float* screen_size, const cam_pose& cam_pose, const float np = 0.1f, const float fp = 100.f, const float sensitivity = 1.0);
		// pos_xy .. ivec2
		// mat_r_onmove .. fmat4x4
		bool move(const int* pos_xy, cam_pose& cam_pose);	// target is camera
		bool move(const int* pos_xy, float* mat_r_onmove);	// target is object
	};
}