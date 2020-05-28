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
float confThreshold = 0.3; // Confidence threshold
float nmsThreshold = 0.2;  // Non-maximum suppression threshold
int inpWidth = 608;  // Width of network's input image
int inpHeight = 608; // Height of network's input image

const uint32_t WIDTH = 1280; // Width of image
const uint32_t HEIGHT = 720; // Height of image
const double CAMERA_FOV = 53.0; // Field of view of Kiwi Camera
const double KIWI_WIDTH = 0.16; // Width of Kiwi in meters

std::vector<std::string> classes{"kiwi"};

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(Mat& frame, const vector<Mat>& outs, cluon::OD4Session &session);

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net);

int32_t main(int32_t, char **)
{
int32_t retCode{1};

	//const double TO_DEGREES{180 / 3.141592653589793};
	const std::string NAME{"img.argb"};
	//const bool VERBOSE{true};
	const uint16_t CID{111};

	String modelConfiguration = "kiwi.cfg";
    String modelWeights = "kiwi_5000.weights";

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

		


		// Handler to receive distance readings (realized as C++ lambda).
		std::mutex distancesMutex;
		float front{0};
		float rear{0};
		float left{0};
		float right{0};
		auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env) {
			auto senderStamp = env.senderStamp();

			// Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
			opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));

			// Store distance readings.
			std::lock_guard<std::mutex> lck(distancesMutex);
			switch (senderStamp)
			{
			case 0:
				front = dr.distance();
				break;
			case 2:
				rear = dr.distance();
				break;
			case 1:
				left = dr.distance();
				break;
			case 3:
				right = dr.distance();
				break;
			}
		};
		// Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
		od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);


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
			Mat frame, blob;
			cvtColor(img, frame, 1);

			// Create a 4D blob from a frame.
			blobFromImage(frame, blob, 1/255.0, Size(inpWidth, inpHeight), Scalar(0,0,0), true, false);

			
			
			//Sets the input to the network
			net.setInput(blob);
			
			// Runs the forward pass to get output of the output layers
			vector<Mat> outs;
			net.forward(outs, getOutputsNames(net));
			
			// Remove the bounding boxes with low confidence
			postprocess(frame, outs, od4);
			
			// Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
			vector<double> layersTimes;
			double freq = getTickFrequency() / 1000;
			double t = net.getPerfProfile(layersTimes) / freq;
			string label = format("Inference time for a frame : %.2f ms", t);
			putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
			
			// Write the frame with the detection boxes
			Mat detectedFrame;
			frame.convertTo(detectedFrame, CV_8U);


			//for( size_t i = 0; i < kiwis.size(); i++ )
			//{
			//	cv::rectangle(img, Point(kiwis[i].x,kiwis[i].y), Point(kiwis[i].x + kiwis[i].width, kiwis[i].y + kiwis[i].height), Scalar(255,0,0));
			//}


			imshow("kiwis", frame);
			waitKey(1);

		}
	}
	retCode = 0;
	return retCode;
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(Mat& frame, const vector<Mat>& outs, cluon::OD4Session &session)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;
    
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }
    
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
	vector<KiwiLocation> kiwi_locations;
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];

		KiwiLocation kiwi_location = KiwiLocation(WIDTH, CAMERA_FOV, box.width, box.x);
		kiwi_locations.push_back(kiwi_location);
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }

	// Send kiwi locations over network session
	string serialized_kiwi_locations = Serializer::encode(kiwi_locations);
	opendlv::robo::KiwiLocation kiwi_location_message;
	kiwi_location_message.data(serialized_kiwi_locations);
	session.send(kiwi_location_message);
}

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);
    
    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }
    
    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0),1);
}

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();
        
        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();
        
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}