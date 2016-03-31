#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
  //VideoCapture cap("TLDtracker-motorbike.avi");
  VideoCapture cap(0);
  // Clos program if video source is not open
  if(!cap.isOpened()){
    return(-1);
  }

  Mat frame;

  char c;

  // Define simple window
  const char* WIN = "Main"; 
  namedWindow(WIN, WINDOW_AUTOSIZE);
  moveWindow(WIN, 400       , 0);         //750,  2 (bernat =0)

  while(cap.isOpened()) {
    // Read frame
    cap >> frame;
    // Does frame has data
    if (!frame.data) {
     continue;
    }

    // Change to gray values
    cvtColor(frame, frame, CV_BGR2GRAY);

    imshow(WIN, frame);
    
    c = (char)waitKey(25);
    if(27 == c) {
      break;
    }    
  }

  return(0);
}
