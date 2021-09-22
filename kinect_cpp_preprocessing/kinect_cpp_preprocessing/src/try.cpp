#include <Windows.h>
#include <Ole2.h>

//#include "opencv2/opencv.hpp" 
//#include "opencv2/highgui/highgui.hpp"

//#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <Kinect.h>
#include <iostream>



#define width 512
#define height 424



// Kinect variables
IKinectSensor* sensor;         // Kinect sensor
IDepthFrameReader* reader;     // Kinect color data source

//error handling variable
HRESULT hResult = S_OK;
BOOLEAN boolResult = true;

class rgba {
public:
    double blue;
    double green;
    double red;
    double alpha;
};

int initKinect() {

    hResult = GetDefaultKinectSensor(&sensor);
    if (FAILED(hResult)) {
        std::cout << "ERROR: couldn't get the kinect sensor(GetDefaultKinectSensor)" << std::endl;
        exit;
    }

    hResult = sensor->Open();
    if (FAILED(hResult)) {
        std::cout << "ERROR: couldn't start streaming data(sensor>>Open())" << std::endl;
        exit;

    }
    //sensor->Open();
    // BOOLEAN isOpen;
    // BOOLEAN isAvialable;
     hResult=sensor->get_IsAvailable(&boolResult);
     if (FAILED(hResult)) {
         std::cout << "ERROR: the kinect opened a stream but isn't available(get_IsAvailable failed)" << std::endl;
         sensor->Close();
         exit;
     }


     IDepthFrameSource* framesource = NULL;
     hResult=sensor->get_DepthFrameSource(&framesource);
     if (FAILED(hResult)) {
         std::cout << "ERROR: couldn't get a depth frame source from the sensor(get_DepthFrameSource failed)" << std::endl;
         sensor->Close();
         exit;
     }
     framesource->get_IsActive(&boolResult);
     if (FAILED(hResult)) {
         std::cout << "ERROR: got depth frame source from sensor but it isn't active(framesource->get_IsActive)";
         sensor->Close();
         exit;
     }

      hResult=framesource->OpenReader(&reader);
      if (FAILED(hResult)) {
          std::cout << "ERROR: couldn't create a frame reader for the depth frame source.(framesource->OpenReader)" << std::endl;
          sensor->Close();
          exit;
      }
      if (framesource) {
          framesource->Release();
          framesource = NULL;
      }
      return true;
  

}

void getKinectData() {
    BYTE* dest = new BYTE[515*424*4];
    IDepthFrame* frame = NULL;
    while (1) {
        hResult = reader->AcquireLatestFrame(&frame);
        if (SUCCEEDED(hResult)) {

            unsigned int capacity;
            unsigned short* buf;
            hResult = frame->AccessUnderlyingBuffer(&capacity, &buf);
            if (FAILED(hResult)) {
                std::cout << "ERROR: couldn't access the depth frame data(no pointer retuned from frame->AccessUnderlyingBuffer) " << std::endl;
                sensor->Close();
                exit;
            }

            const unsigned short* curr = (const unsigned short*)buf;
            const unsigned short* dataEnd = curr + (width * height);
            int i = 0;
            while (curr < dataEnd) {
                i++;
                //BYTE desValue = dest[i-1];
                // Get depth in millimeters
                unsigned short depth = (*curr++);

                // Draw a grayscale image of the depth:
                // B,G,R are all set to depth%256, alpha set to 1.
                for (int i = 0; i < 3; ++i) {
                    *dest++ = (BYTE)depth % 256;
                }
                *dest++ = 0xff;

                
            }
            if (frame) frame->Release();
            return;
               
        }
        else {
            //      std::cout << "ERROR: couldn't acquire the lastest frame from the reader.(reader->AcquireLatestFrame)" << std::endl;
            //      return;
            std::cout << "ERROR: couldn't acquire the lastest frame from the reader.(reader->AcquireLatestFrame). trying again." << std::endl;
        }
    }
  //  if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
  //      frame->CopyConvertedFrameDataToArray(width * height * 4, dataa, ColorImageFormat_Bgra);
  //  }
  //  if (frame) frame->Release();
}

int main() {
    //cv::Mat Yoda = cv::imread("Yoda.jpeg");
    //cv::imshow("little Yoda", Yoda);
    //cv::waitKey(0);
    initKinect();
    while (1) {
        getKinectData();
    }
    sensor->Close();
   
}

