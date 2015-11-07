#include <fstream>
#include <iostream>
#include <iomanip>
#include "visual_odometry.h"
#include "assert.h"
using namespace std;
using namespace cv;
int main(int argc, char *argv[])
{

	PinholeCamera *cam = new PinholeCamera(1241.0, 376.0,
    718.8560, 718.8560, 607.1928, 185.2157);
	VisualOdometry vo(cam);

	std::ofstream out("position.txt");

	char text[100];
	int font_face = cv::FONT_HERSHEY_PLAIN;
	double font_scale = 1;
	int thickness = 1;
	cv::Point text_org(10, 50);
	cv::namedWindow("Road facing camera", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);
    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);//

    double x=0.0, y=0.0,z=0.0;
	for (int img_id = 0; img_id < 2000; ++img_id)
	{
        cout<<img_id<<endl;
		std::stringstream ss;
		ss <<  "/home/chaoqun/Downloads/dataset/sequences/00/image_1/"
			<< std::setw(6) << std::setfill('0') << img_id << ".png";

		cv::Mat img(cv::imread(ss.str().c_str(), 0));
		assert(!img.empty());

		vo.addImage(img, img_id);
		cv::Mat cur_t = vo.getCurrentT();
		if (cur_t.rows!=0)
		{
			x = cur_t.at<double>(0);
			y = cur_t.at<double>(1);
			z = cur_t.at<double>(2);
		}
		out << x << " " << y << " " << z << std::endl;

		int draw_x = int(x) + 300;
		int draw_y = int(z) + 100;
		cv::circle(traj, cv::Point(draw_x, draw_y), 1, Scalar(255, 0, 0), 2);

        cv::rectangle(traj, cv::Point(10, 30), cv::Point(580, 60), Scalar(0, 0, 0), CV_FILLED);
		sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, y, z);
		cv::putText(traj, text, text_org, font_face, font_scale, cv::Scalar::all(255), thickness, 8);

		cv::imshow("Road facing camera", img);
		cv::imshow("Trajectory", traj);

		cv::waitKey(1);
	}

	delete cam;
	out.close();
	getchar();
	return 0;
}
