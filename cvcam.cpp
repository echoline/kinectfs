/*
** 
*/

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp> 

int main()
{
	cv::VideoCapture cap(0); // the fist webcam connected to your PC
	if (!cap.isOpened())
	{
		std::cerr << "Webcam not detected." << std::endl;
		return -1;
	}
	cv::Mat frame;
	while (1)
	{
		std::vector<uchar> buf;
		cap >> frame; // outputs the webcam image to a Mat
		cv::imencode(".jpg", frame, buf);
		std::cout << "--ffserver\r\n";
		std::cout << "Content-Type: image/jpeg\r\n";
		std::cout << "Content-Length: " << buf.size() << "\r\n";
		std::cout << "\r\n";
		for (uchar &c : buf)
			std::cout << c;
		std::cout << "\r\n";
		if (cv::waitKey(1000) == 'q') break;
	}
	return 0;
}

