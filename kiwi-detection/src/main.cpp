#include "cluon-complete.hpp"
#include "messages.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/dnn.hpp>

#include <stdint.h>
#include <iostream>
#include <memory>
#include <mutex>

#include "kiwi_location.hpp"
#include "serializer.hpp"

using namespace cv;
using namespace dnn;
using namespace std;





// Initialize the parameters
float confThreshold = 0.2; // Confidence threshold
float nmsThreshold = 0.1;  // Non-maximum suppression threshold
int inpWidth = 480;//608;  // Width of network's input image
int inpHeight = 480;//608; // Height of network's input image

const uint32_t WIDTH = 1280; // Width of image
const uint32_t HEIGHT = 720; // Height of image
const double CAMERA_FOV_X = 97.6; // Field of view of Kiwi Camera
const double CAMERA_FOV_Y = 54.9; // Field of view of Kiwi Camera
const double KIWI_WIDTH = 0.16; // Width of Kiwi in meters

std::vector<std::string> classes{"kiwi"};

// Remove low confidence bboxes
void postprocess(Mat& frame, const vector<Mat>& outs, cluon::OD4Session &session);

// Draw the predicted bboxes
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net);

int32_t main(int32_t argc, char **argv)
{
	int32_t retCode{1};
	auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    const std::string NAME{commandlineArguments["name"]};
	const bool SIMULATION{static_cast<bool>(std::stoi(commandlineArguments["simulation"]))};
	const uint16_t CID{111};

	String modelConfiguration = "kiwi.cfg";
    String modelWeights = "kiwi_intersect.weights";

    // Load the network
    Net net = readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);


	// Attach to the shared memory.
	std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
	if (sharedMemory && sharedMemory->valid())
	{
		std::clog << " Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

		// Interface to a running OpenDaVINCI session; here, you can send and receive messages.
		cluon::OD4Session od4{CID};

		while (od4.isRunning())
		{
			Mat img;

			// Wait for a notification of a new frame.
			sharedMemory->wait();

			// Lock the shared memory.
			sharedMemory->lock();
			{
				// Copy image into cvMat structure.
				// Be aware of that any code between lock/unlock is blocking
				// the camera to provide the next frame. Thus, any
				// computationally heavy algorithms should be placed outside
				// lock/unlock
				Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
				img = wrapped.clone();
			}
			sharedMemory->unlock();

			if (SIMULATION)
			{
				Mat hsv;
				cvtColor(img, hsv,cv::COLOR_BGR2HSV);
				//Red filter
				Scalar  redLow(0, 0, 10);
				Scalar  redHi(180,255,12);
				Mat red_mask;
				inRange(hsv , redLow , redHi , red_mask);

				std::vector<std::vector<Point>> red_contours;
				std::vector<Vec4i> red_hierarchy;
				findContours( red_mask, red_contours, red_hierarchy, cv::RETR_EXTERNAL , cv::CHAIN_APPROX_SIMPLE, Point(0, 0) );
				std::vector <KiwiLocation> kiwi_locations;
				for( int i = 0; i < (int) red_contours.size(); i++ )
				{
					
					Rect red_bounding_rect = boundingRect( red_contours.at(i));
					if (red_bounding_rect.area() > 5000 && red_bounding_rect.y < 500)
					{
						Rect box = red_bounding_rect;
						// std::cout << "box.width: "  <<  box.width << std::endl;
						KiwiLocation kiwi_location = KiwiLocation( box.x, box.y, box.width,box.height, WIDTH, HEIGHT, CAMERA_FOV_X, CAMERA_FOV_Y, box.width, box.x + box.width/2.0f);
						std::cout << "distance: "  <<  kiwi_location.distance() << "  y: " << kiwi_location.y() << std::endl;
						
						kiwi_locations.push_back(kiwi_location);
						//rectangle( img, red_bounding_rect.tl(), red_bounding_rect.br(), Scalar( 255,255,255 ), 3 );
					}
				}

				string serialized_kiwi_locations = Serializer::encode(kiwi_locations);
				opendlv::robo::KiwiLocation kiwi_location_message;
				kiwi_location_message.data(serialized_kiwi_locations);
				od4.send(kiwi_location_message);
			}
			else
			{
				Mat frame, blob;
				cvtColor(img, frame, 1);

				// Create a 4D blob from img
				blobFromImage(frame, blob, 1/255.0, Size(inpWidth, inpHeight), Scalar(0,0,0), true, false);
				net.setInput(blob);

				vector<Mat> outs;
				net.forward(outs, getOutputsNames(net));

				// Remove the bounding boxes with low confidence
				postprocess(frame, outs, od4);

				// show inference speed
				vector<double> layersTimes;
				double freq = getTickFrequency() / 1000;
				double t = net.getPerfProfile(layersTimes) / freq;
				string label = format("Inference time for a frame : %.2f ms", t);
				//std::cout << label << std::endl;
				putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
				Mat detectedFrame;
				frame.convertTo(detectedFrame, CV_8U);
			}

		}
	}
	retCode = 0;
	return retCode;
}

// Remove the bounding boxes with low confidence
void postprocess(Mat& frame, const vector<Mat>& outs, cluon::OD4Session &session)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;
    
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. 
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                
				classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

	vector<KiwiLocation> kiwi_locations;
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
		KiwiLocation kiwi_location = KiwiLocation( box.x, box.y, box.width,box.height, WIDTH, HEIGHT, CAMERA_FOV_X, CAMERA_FOV_Y, box.width, box.x + box.width/2.0f);
		kiwi_locations.push_back(kiwi_location);
		int classId = classIds[idx];
		float conf = confidences[idx];
        drawPred(classId, conf , box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }

	// Send kiwi locations over network session
	string serialized_kiwi_locations = Serializer::encode(kiwi_locations);
	opendlv::robo::KiwiLocation kiwi_location_message;
	kiwi_location_message.data(serialized_kiwi_locations);
	session.send(kiwi_location_message);
}

void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{

    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);    
    string label = format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0),1);
}

vector<String> getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        vector<int> outLayers = net.getUnconnectedOutLayers();
        vector<String> layersNames = net.getLayerNames();
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}