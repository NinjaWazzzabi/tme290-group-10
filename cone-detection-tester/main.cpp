/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

#define SAVE_PATH "../imgs/result.png"

using namespace cv;

int32_t main(int32_t argc, char **argv)
{
	CommandLineParser parser(argc, argv, "{@input | lena.jpg | input image}");
	Mat img = imread(samples::findFile(parser.get<String>("@input")), IMREAD_COLOR);
	if (img.empty())
	{
		std::cout << "Could not open or find the image!\n"
			 << std::endl;
		std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
		return -1;
	}

	// equalizeHist(img, img);
	// Mat croppedFrame(img, Rect(0, 270, 1278, 450));

	// Mask out the front of the kiwi
	rectangle(img, Point(240,718), Point(1000,600), Scalar(0,0,0), -1, 8);

	GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );
	Mat src_gray;
	cvtColor(img, src_gray, COLOR_BGR2GRAY);

	// Homemade Diagonal filter
	Mat dst1,dst2,abs_dst1,abs_dst2,diag_filtered;

	float m1[9] = {0,1,2,-1,0,1,-2,-1,0};
	Mat kernelOne(Size(3,3), CV_32F, m1);

	float m2[9] = {-2,-1,0,-1,0,1,0,1,2};
	Mat kernelTwo(Size(3,3), CV_32F, m2);

	filter2D(src_gray, dst1, -1, kernelOne);
	filter2D(src_gray, dst2, -1, kernelTwo);

	convertScaleAbs( dst1, abs_dst1 );
	convertScaleAbs( dst2, abs_dst2 );


	addWeighted( abs_dst1, 0.5, abs_dst2, 0.5, 0, diag_filtered );




	Mat hsv;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	Scalar hsvLow(110, 40, 40);
	Scalar hsvHi(130, 255, 255);
	Mat blueCones;
	inRange(hsv, hsvLow, hsvHi, blueCones);

	uint32_t iterations{5};
	Mat dilate;
	cv::dilate(blueCones, dilate, Mat(), Point(-1, -1), iterations, 1, 1);

	Mat erode;
	cv::erode(dilate, erode, Mat(), Point(-1, -1), iterations, 1, 1);

	Mat result = diag_filtered;

	// Canny Detection
	/*Mat canny;
	Canny(croppedFrame, canny, 30, 90, 3);

	std::vector<Vec2f> lines;
	HoughLines(canny, lines, 1, CV_PI/180, 150, 0, 0);

	//Draw the lines
	Mat hough;
	cvtColor(canny, hough, COLOR_GRAY2BGR);
	for(size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0];
		float theta = lines [i][1];
		double a = cos(theta);
		double b = sin(theta);
		double x0 = a * rho;
		double y0 = b * rho;

		Point pt1;
		Point pt2;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * a);
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * a);
		line(hough, pt1, pt2, Scalar(0,255,255), 3, LINE_AA);
	}*/

	// // Invert colors
	// bitwise_not(img, img);

	// // Draw a red rectangle
	// rectangle(img, Point(50, 50), Point(100, 100), Scalar(0,0,255));

	// Display image.
	imwrite( SAVE_PATH, img);
	imshow("Image", result);
	waitKey(100000);
}
