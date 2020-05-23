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
#include <vector>

#define SAVE_PATH "../imgs/result.png"

using namespace cv;


void MatchingMethod( int, void* , Mat img, Mat templ );

int32_t main(int32_t argc, char **argv)
{
	CommandLineParser parser(argc, argv, "{@input | lena.jpg | input image}");
	Mat img = imread(samples::findFile(parser.get<String>("@input")), CV_8UC4);
	Mat templ = imread("../imgs/capture.png", IMREAD_GRAYSCALE);
	if (img.empty())
	{
		std::cout << "Could not open or find the image!\n"
			 << std::endl;
		std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
		return -1;
	}	
	
	/*
	// Canny Detection
	Mat canny;
	Canny(img, canny, 30, 100, 3);

	// Masking car and top half of picture
	rectangle(canny, Point(img.cols/5,img.rows-200), Point(img.cols*4/5,img.rows), Scalar(0,0,0), -1, 8);
	rectangle(canny, Point(0,0), Point(img.cols,img.rows*9/16), Scalar(0,0,0), -1, 8);


	MatchingMethod( 0, 0 ,canny, templ);
	*/
	Rect myROI(0, img.rows*9.4/16, img.cols, img.rows*4.2/16);
	Mat croppedImage = img(myROI);

	GaussianBlur( croppedImage, croppedImage, Size(3,3), 0, 0, BORDER_DEFAULT );

	Mat  hsv;
	cvtColor(croppedImage , hsv , cv::COLOR_BGR2HSV );


 	std::vector<Mat> planes;
	split(hsv, planes);
	Scalar avg( cv::mean(planes[0])[0], cv::mean(planes[1])[0], cv::mean(planes[2])[0] );

	Scalar  hsvLow(avg[0] - 40, avg[1] - 20, avg[2] - 50);
	Scalar  hsvHi(avg[0] + 40, avg[1] + 40, avg[2] + 80);

	
  
	std::cout << avg << std::endl;
	std::cout << hsvLow << std::endl;


	Mat1b  background_filter_inv;
	inRange(hsv , hsvLow , hsvHi , background_filter_inv );

	
	Mat1b  background_filter;
	bitwise_not(background_filter_inv, background_filter);

	Mat res = croppedImage;
	bitwise_and(croppedImage,croppedImage, res, background_filter);

	Scalar  bluLow(100,40,40);
	Scalar  bluHi(130,255,255);

	Mat blue_mask;
	inRange(hsv , bluLow , bluHi , blue_mask);

	Mat blue_cones;
	bitwise_and(res,res, blue_cones, blue_mask);


	Scalar  yellowLow(20,41,50);
	Scalar  yellowHi(40,255,255);

	Mat yellow_mask;
	inRange(hsv,yellowLow,yellowHi, yellow_mask);

	Mat cones_mask;
	bitwise_or(blue_mask,yellow_mask,cones_mask);

	Mat cones;
	bitwise_and(res, res, cones, cones_mask);


	Mat  dilate;
	uint32_t  iterations = 3;
	cv::dilate(cones_mask , dilate , Mat(), Point(-1,  -1), iterations , 1, 1);
	Mat  erode;
	cv::erode(dilate , erode , Mat(), Point(-1,  -1), iterations , 1, 1);

	cones_mask = erode;

	Mat canny_out;
	Canny( cones_mask, canny_out, 30, 90, 3 );


	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;
	std::vector<Rect> boundRect( contours.size() );
	findContours( canny_out, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, Point(0, 0) );
	//RNG rng(12345);
	//Mat drawing = Mat::zeros( canny_out.size(), CV_8UC3 );
	for( int i = 0; i < (int) contours.size(); i++ )
	{
		std::vector<Point> contour_poly;
        approxPolyDP( contours.at(i), contour_poly, 3, true );
		Scalar color = Scalar( 255,255,255 );
		//drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		rectangle( res, boundingRect( contours.at(i)).tl(), boundingRect( contours.at(i)).br(), color, 2 );

	}


	imwrite( SAVE_PATH, img);
	imshow("img",img); 
	imshow("hsv",hsv); 
	imshow("mask",background_filter); 
	imshow("blue_mask",blue_mask);
	imshow("yellow_mask",yellow_mask);
	imshow("cones_mask",cones_mask);
	imshow("cones",cones);
	imshow( "Contours", res );
	waitKey(100000);


}

void MatchingMethod( int, void*, Mat img, Mat templPre )
{

	Mat templ;
	templPre.convertTo(templ, img.type());

	/// Source image to display
	Mat img_display;
	img.copyTo( img_display );

	Mat result;

	/// Do the Matching and Normalize
	matchTemplate( img, templ, result, 4 );
	normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

	
	float threshold = 0.85;
	
	std::vector<Point> top_positions;

	// Find Positions of best matches
	for(int i = 0; i < result.rows -1; ++i)
	{
		for(int j = 0; j < result.cols - 1; ++j)
		{
			float pixel = result.at<float>(i, j);
			if (pixel >= threshold)
			{
				top_positions.push_back(Point(j,i));
			}
		}
	}

	// Draw rectangle around best matches.
	for (Point pos : top_positions)
	{
		rectangle( img_display, pos, Point( pos.x + templ.cols , pos.y + templ.rows ), Scalar(255,255,255),  2, 8, 0 );
		rectangle( result, pos, Point( pos.x + templ.cols , pos.y + templ.rows ), Scalar(255,255,255),  2, 8, 0);			
	}


	/// Localizing the best match with minMaxLoc
	double minVal; double maxVal; Point minLoc; Point maxLoc;
	Point matchLoc;

	minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
	matchLoc = maxLoc;



	/// Draw rectangle
	rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar(255,255,255), 2, 8, 0 );
	rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar(255,255,255), 2, 8, 0 );
	

	imwrite( SAVE_PATH, img);
	imshow("result", result);
	imshow("Image", img_display);
	waitKey(100000);

  return;
}