#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <igh3d.hpp>
#include <vector>
#include <array>
#include <stdexcept>

namespace py = pybind11;


// テンプレート作成器
class pyTCreator {
	iGH3DTcreator *creator;
	
public:
	pyTCreator() : creator(0) {
		creator = CreateGH3DTcreator();
	}			
	~pyTCreator() {
		creator->destroy();
	}

	// パラメータセット
	bool setParam(py::dict param) {
		GH3DParamTcreator gparam;
		
		if (param.contains("PlanePicker")) {
			py::dict pp = param["PlanePicker"];
			if (pp.contains("n_pixels")) gparam.planepik.n_pixels = pp["n_pixels"].cast<int>();
			if (pp.contains("d_threshold")) gparam.planepik.d_threshold = pp["d_threshold"].cast<float>();
		}

		if (param.contains("EdgeExtractor")) {
			py::dict pp = param["EdgeExtractor"];
			if (pp.contains("canny_low")) gparam.edgeextr.canny_low = pp["canny_low"].cast<double>();
			if (pp.contains("canny_upp")) gparam.edgeextr.canny_upp = pp["canny_upp"].cast<double>();
			if (pp.contains("sobel_kernel")) gparam.edgeextr.sobel_kernel = pp["sobel_kernel"].cast<int>();
			if (pp.contains("depth_radius")) gparam.edgeextr.depth_radius = pp["depth_radius"].cast<int>();
			if (pp.contains("d_threshold")) gparam.edgeextr.d_threshold = pp["d_threshold"].cast<float>();
		}

		if (param.contains("pix_per_mm")) gparam.pix_per_mm = param["pix_per_mm"].cast<float>();
		if (param.contains("rmdistance")) gparam.rmdistance = param["rmdistance"].cast<float>();
		if (param.contains("rtab_delta")) gparam.rtab_delta = param["rtab_delta"].cast<float>();
		
		return creator->setParameters(&gparam);
	}

	// カメラパラメータセット
	void setCamera(std::array<double, 9> K, std::array<double, 3> rvec, std::array<double, 3> tvec,
				   int width, int height) {
		creator->setCameraParam(K, rvec, tvec, width, height);
	}

	// 点群データセット
	void setPointCloud(std::vector<float> xyz, std::vector<unsigned char> tex,
					   int roi_x, int roi_y, int roi_w, int roi_h) {
		creator->setPointCloud(xyz, tex, roi_x, roi_y, roi_w, roi_h);
	}

	// テンプレート作成実行
	bool exec() {
		return creator->exec();
	}

	// ファイルに保存
	bool save(std::string filename) {
		return creator->save(filename);
	}

	// テンプレート検出結果をカラー画像にして返す
	py::array_t<unsigned char> getImage() {
		unsigned char *image;
		int width, height;
		size_t step;
		creator->getimage(&image, &width, &height, &step);

		unsigned char *retim = new unsigned char [ height * width * 3 ];
		unsigned char *rP = retim;
		for (int j = 0; j < height; j++) {
			unsigned char *iP = image;
			for (int i = 0; i < width; i++, iP += 3) {
				// OpenCVのBGRからRGB順にする				
				*rP++ = iP[2];
				*rP++ = iP[1];
				*rP++ = iP[0];
			}
			image += step;
		}

		py::capsule free_when_done(retim, [](void *f) {
			unsigned char *retim = reinterpret_cast<unsigned char*>(f);
			delete [] retim;
		});

		return py::array_t<unsigned char>(
			{ height, width, 3 },	// shape
			{ width * 3, 3, 1 }, // strides
			retim,	// data pointer
			free_when_done);
	}   	
};



class pyDetector {
	iGH3DDetector *detector;
	int max_planes_;
	int n_detect_;
	
public:
	pyDetector() : detector(0), max_planes_(1), n_detect_(0) {
		detector = CreateGH3DDetector();
	}
	~pyDetector() {
		detector->destroy();
	}

	// パラメータセット
	bool setParam(py::dict param) {
		GH3DParamDetector gparam;
		
		if (param.contains("PlanePicker")) {
			py::dict pp = param["PlanePicker"];
			if (pp.contains("n_pixels")) gparam.planepik.n_pixels = pp["n_pixels"].cast<int>();
			if (pp.contains("d_threshold")) gparam.planepik.d_threshold = pp["d_threshold"].cast<float>();
		}

		if (param.contains("EdgeExtractor")) {
			py::dict pp = param["EdgeExtractor"];
			if (pp.contains("canny_low")) gparam.edgeextr.canny_low = pp["canny_low"].cast<double>();
			if (pp.contains("canny_upp")) gparam.edgeextr.canny_upp = pp["canny_upp"].cast<double>();
			if (pp.contains("sobel_kernel")) gparam.edgeextr.sobel_kernel = pp["sobel_kernel"].cast<int>();
			if (pp.contains("depth_radius")) gparam.edgeextr.depth_radius = pp["depth_radius"].cast<int>();
			if (pp.contains("d_threshold")) gparam.edgeextr.d_threshold = pp["d_threshold"].cast<float>();
		}

		if (param.contains("rmdistance")) gparam.rmdistance = param["rmdistance"].cast<float>();
		if (param.contains("vote_minimum")) gparam.vote_minimum = param["vote_minimum"].cast<float>();
		if (param.contains("overlap_thr")) gparam.overlap_thr = param["overlap_thr"].cast<float>();
		if (param.contains("x_delta")) gparam.x_delta = param["x_delta"].cast<int>();
		if (param.contains("y_delta")) gparam.y_delta = param["y_delta"].cast<int>();
		if (param.contains("a_delta")) gparam.a_delta = param["a_delta"].cast<int>();
		
		return detector->setParameters(&gparam);
	}

	// カメラパラメータセット
	void setCamera(std::array<double, 9> K, std::array<double, 3> rvec, std::array<double, 3> tvec,
				   int width, int height) {
		detector->setCameraParam(K, rvec, tvec, width, height);
	}

	// テンプレートデータ読み込み
	bool load(std::string filename) {
		return detector->load(filename);
	}
	
	// 点群データセット
	void setPointCloud(std::vector<float> xyz, std::vector<unsigned char> tex,
					   int roi_x, int roi_y, int roi_w, int roi_h) {		
		detector->setPointCloud(xyz, tex, roi_x, roi_y, roi_w, roi_h);
	}

	// 物体検出実行
	int exec(int max_planes) {
		if (max_planes < 1) max_planes = 1;
		return (n_detect_ = detector->exec(max_planes_ = max_planes));
	}

	// スコアを返す
	py::array_t<float> getScores() {
		if (n_detect_ == 0) return py::array_t<float>();
		
		float *scores = new float [n_detect_];
		detector->copyScores(scores);

		py::capsule free_when_done(scores, [](void *f) {
			float *scores = reinterpret_cast<float *>(f);
			delete [] scores;
		});

		return py::array_t<float>(
			{ n_detect_ },	// shape
			{ sizeof(float) },	// strides
			scores,	// data pointer
			free_when_done);		
	}
	
	// RTを返す
	py::array_t<float> getRTs() {
		if (n_detect_ == 0) return py::array_t<float>();
		
		float *rtmats = new float [n_detect_ * 16];
		detector->copyRTMats(rtmats);

		py::capsule free_when_done(rtmats, [](void *f) {
			float *rtmats = reinterpret_cast<float *>(f);
			delete [] rtmats;
		});

		return py::array_t<float>(
			{ n_detect_, 4, 4 },	// shape
			{ 4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float) },	// strides
			rtmats,	// data pointer
			free_when_done);
	}

	
	// 物体検出結果をカラー画像にして返す
	py::array_t<unsigned char> getImage(int n_plane) {
		if ((n_plane < 0) || (n_plane >=max_planes_)) return py::array_t<unsigned char>();
		
		unsigned char *image;
		int width, height;
		size_t step;
		detector->getimage(&image, &width, &height, &step, n_plane);

		unsigned char *retim = new unsigned char [ height * width * 3 ];
		unsigned char *rP = retim;
		for (int j = 0; j < height; j++) {
			unsigned char *iP = image;
			for (int i = 0; i < width; i++, iP += 3) {
				// OpenCVのBGRからRGB順にする				
				*rP++ = iP[2];
				*rP++ = iP[1];
				*rP++ = iP[0];
			}
			image += step;
		}

		py::capsule free_when_done(retim, [](void *f) {
			unsigned char *retim = reinterpret_cast<unsigned char*>(f);
			delete [] retim;
		});

		return py::array_t<unsigned char>(
			{ height, width, 3 },	// shape
			{ width * 3, 3, 1 }, // strides
			retim,	// data pointer
			free_when_done);
	}   	

	// テンプレートのアンカー点の座標値
	std::array<float, 3> getAnchor3D() {
		return detector->getAnchor3D();
	}

	// 結果をファイルに保存
	bool save(std::string filename) {
		return detector->save(filename);
	}
};





PYBIND11_MODULE(pygh3d, m) {
	m.doc() = "GH3D module";
	py::class_<pyTCreator> template_class(m, "tcreator");
	template_class.def(py::init<>());
	template_class.def("setParam", &pyTCreator::setParam, "parameters of template creator", py::arg("param"));
	template_class.def("setCamera", &pyTCreator::setCamera, "set camera parameter",
					   py::arg("K"), py::arg("rvec"), py::arg("tvec"), py::arg("width"), py::arg("height"));
	template_class.def("setPointCloud", &pyTCreator::setPointCloud, "set point cloud",
					   py::arg("xyz"), py::arg("tex"),
					   py::arg("roi_x") = 0, py::arg("roi_y") = 0, py::arg("roi_w") = -1, py::arg("roi_h") = -1);
	template_class.def("exec", &pyTCreator::exec, "execute create template");
	template_class.def("save", &pyTCreator::save, "save template to file",
					   py::arg("filename") = "data.ght");
	template_class.def("getImage", &pyTCreator::getImage, "get result image");	



	
	py::class_<pyDetector> detector_class(m, "detector");
	detector_class.def(py::init<>());
	detector_class.def("setParam", &pyDetector::setParam, "parameters of template creator", py::arg("param"));
	detector_class.def("setCamera", &pyDetector::setCamera, "set camera parameter",
					   py::arg("K"), py::arg("rvec"), py::arg("tvec"), py::arg("width"), py::arg("height"));
	detector_class.def("load", &pyDetector::load, "load template file", py::arg("filename"));
	detector_class.def("setPointCloud", &pyDetector::setPointCloud, "set point cloud",
					   py::arg("xyz"), py::arg("tex"),
					   py::arg("roi_x") = 0, py::arg("roi_y") = 0, py::arg("roi_w") = -1, py::arg("roi_h") = -1);
	detector_class.def("exec", &pyDetector::exec, "execute detect object", py::arg("max_planes") = 1);
	detector_class.def("getScores", &pyDetector::getScores, "get score (NCC)");					   					   
	detector_class.def("getRTs", &pyDetector::getRTs, "get transform matrix");					   					   
	detector_class.def("getImage", &pyDetector::getImage, "get result image",
					   py::arg("n_plane") = 1);
	detector_class.def("save", &pyDetector::save, "save results to file",
					   py::arg("filename") = "result.yaml");
	detector_class.def("getAnchor3D", &pyDetector::getAnchor3D, "get template anchor point coordinate(3D)");
}
