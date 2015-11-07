#ifndef VISUAL_ODOMETRY_H_
#define VISUAL_ODOMETRY_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "pinhole_camera.h"

class VisualOdometry
{
public:
	//Ŀǰ����֡��ͨ�������������õ�ȷ����һ֡�͵ڶ�֡ͼ������ȷ����ʼλ��
	enum FrameStage {
		STAGE_FIRST_FRAME,//��һ֡
		STAGE_SECOND_FRAME,//�ڶ�֡
		STAGE_DEFAULT_FRAME//Ĭ��֡
	};

	VisualOdometry(PinholeCamera* cam);
	virtual ~VisualOdometry();

	/// �ṩһ��ͼ��
	void addImage(const cv::Mat& img, int frame_id);

	/// ��ȡ��ǰ֡����ת
	cv::Mat getCurrentR() { return cur_R_; }
	/// ��õ�ǰ֡��ƽ��
	cv::Mat getCurrentT() { return cur_t_; }

protected:
	/// �����һ֡
	virtual bool processFirstFrame();
	/// ����ڶ�֡
	virtual bool processSecondFrame();
	/// �����꿪ʼ������֡��������֡
	virtual bool processFrame(int frame_id);
	/// ������Գ߶�
	double getAbsoluteScale(int frame_id);
	/// �������
	void featureDetection(cv::Mat image, std::vector<cv::Point2f> &px_vec);
	/// ��������
	void featureTracking(cv::Mat image_ref, cv::Mat image_cur,
		std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur, std::vector<double>& disparities);

protected:
	FrameStage frame_stage_;                 //!< ��ǰΪ�ڼ�֡
	PinholeCamera *cam_;                     //!< ���
	cv::Mat new_frame_;                      //!< ��ǰ֡
	cv::Mat last_frame_;                     //!< �������֡

	cv::Mat cur_R_;//!< �������ǰ����̬�����ڽ�ͼ�����̬���з�װΪ֡
	cv::Mat cur_t_;//!< ��ǰ��ƽ��

	std::vector<cv::Point2f> px_ref_;      //!< �ڲο�֡�����ڸ��ٵ�������
	std::vector<cv::Point2f> px_cur_;      //!< �ڵ�ǰ֡�и��ٵ�������
	std::vector<double> disparities_;      //!< ��һ֡��ڶ�֡��Ӧ�������ٵ�����֮������ز�ֵ

	double focal_;//!<�������
	cv::Point2d pp_; //!<������ĵ�

};

#endif // VISUAL_ODOMETRY_H_
