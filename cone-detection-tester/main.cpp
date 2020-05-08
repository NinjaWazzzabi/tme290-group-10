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
	Mat img = imread(samples::findFile(parser.get<String>("@input")), IMREAD_GRAYSCALE);
	Mat templ = imread("../imgs/capture.png", IMREAD_GRAYSCALE);
	if (img.empty())
	{
		std::cout << "Could not open or find the image!\n"
			 << std::endl;
		std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
		return -1;
	}	
	
	GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );
	
	// Canny Detection
	Mat canny;
	Canny(img, canny, 30, 100, 3);

	// Masking car and top half of picture
	rectangle(canny, Point(img.cols/5,img.rows-200), Point(img.cols*4/5,img.rows), Scalar(0,0,0), -1, 8);
	rectangle(canny, Point(0,0), Point(img.cols,img.rows*9/16), Scalar(0,0,0), -1, 8);


	MatchingMethod( 0, 0 ,canny, templ);

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

	
	float threshold = 0.8;
	
	std::vector<Point> top_positions;

	// Find Positions of best matches
	for(int i = 0; i < result.rows -1; ++i)
	{
		for(int j = 0; j < result.cols - 1; ++j)
		{
			float pixel = result.at<float>(i, j);
			if (pixel >= threshold)
			{
				std::cout << "Pos: " << i << ", " << j << " Threshold: " << pixel << std::endl;
				top_positions.push_back(Point(j,i));
			}
		}
	}

	// Draw rectangle around best matches.
	for (Point pos : top_positions)
	{
		std::cout << "Point: " << pos.x << ", " << pos.y << std::endl;
		rectangle( img_display, pos, Point( pos.x + templ.cols , pos.y + templ.rows ), Scalar(255,255,255),  2, 8, 0 );
		rectangle( result, pos, Point( pos.x + templ.cols , pos.y + templ.rows ), Scalar(255,255,255),  2, 8, 0);			
	}


	/// Localizing the best match with minMaxLoc
	double minVal; double maxVal; Point minLoc; Point maxLoc;
	Point matchLoc;

	minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
	matchLoc = maxLoc;
	std::cout << matchLoc.x << ", " << matchLoc.y << "  Val:  " << maxVal << std::endl;



	/// Draw rectangle
	rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar(255,255,255), 2, 8, 0 );
	rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar(255,255,255), 2, 8, 0 );
	

	imwrite( SAVE_PATH, img);
	imshow("result", result);
	imshow("Image", img_display);
	waitKey(100000);

  return;
}