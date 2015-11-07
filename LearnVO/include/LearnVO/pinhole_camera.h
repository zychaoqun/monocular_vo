#ifndef PINHOLE_CAMERA_H_
#define PINHOLE_CAMERA_H_

#include <opencv2/opencv.hpp>

class PinholeCamera
{
public:
	// ���ǻ������k1,k2,p1,p2,k3,��Ŀǰ����KITTI���ݼ��У�ͼ���Ѿ����л��䴦��
	PinholeCamera(double width, double height,
		double fx, double fy, double cx, double cy,
		double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);

	~PinholeCamera();

	inline int width() const { return width_; }
	inline int height() const { return height_; }
	inline double fx() const { return fx_; };
	inline double fy() const { return fy_; };
	inline double cx() const { return cx_; };
	inline double cy() const { return cy_; };
	inline double k1() const { return d_[0]; };
	inline double k2() const { return d_[1]; };
	inline double p1() const { return d_[2]; };
	inline double p2() const { return d_[3]; };
	inline double k3() const { return d_[4]; };

private:
	double width_, height_;  //!<ͼ��Ŀ�Ⱥ͸߶�
	double fx_, fy_;      //!<����Ľ���
	double cx_, cy_;      //!< ��������ĵ�
	bool distortion_;             //!< �ǵ�����С�����ģ�ͣ����Ǵ��о�����䣿
	double d_[5];                 //!< ����������ο� http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

};

#endif // PINHOLE_CAMERA_H_
