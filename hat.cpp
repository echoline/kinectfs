/*
** hat.cpp
** 
** the definitive opencv for unix multihat tool
** 
** compilation:
** 	g++ -o hat hat.cpp `pkg-config --cflags --libs opencv`
** usage:
**	cd path/to/opencv/data/haarcascades/
** 	ffmpeg -i /dev/video0 -f mpjpeg - | path/to/hat path/to/hatImage.png | ffplay -
*/

#include <iostream>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp> 

void putHatOn(cv::Mat &frame, cv::CascadeClassifier &faceCascade, cv::Mat &hat)
{
	int x, y;
	cv::Mat grayscale;
	cv::Mat green(frame.rows*2, frame.cols*3, CV_8UC3, cv::Scalar(0,0,0));
	cv::cvtColor(frame, grayscale, CV_BGR2GRAY);
	cv::equalizeHist(grayscale, grayscale); // enhance image contrast 
	std::vector<cv::Rect> faces;
	static std::vector<cv::Rect> lastfaces;
	static int lastfacescount = 0;
	faceCascade.detectMultiScale(grayscale, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(150, 150));
	if (faces.size() == 0) {
		lastfacescount++;
		if (lastfacescount > 4)
			lastfaces = faces;
		if (lastfaces.size() == 0)
			return;
		faces = lastfaces;
	} else
		lastfacescount = 0;

	double width = faces[0].br().x - faces[0].tl().x;
	width *= 2;
	double ratio = width / hat.cols;
	double height = ratio * hat.rows;
	cv::Mat resizedHat;
	cv::resize(hat, resizedHat, cv::Size(width, height), ratio, ratio);
	cv::Rect hatRect(faces[0].tl().x + frame.cols - width/4, faces[0].tl().y - 3*height/4 + frame.rows, resizedHat.cols, resizedHat.rows);
	resizedHat.copyTo(green(hatRect));

	for (y = 0; y < frame.rows; y++) for (x = 0; x < frame.cols; x++) {
		cv::Vec3b color = green.at<cv::Vec3b>(cv::Point(x + frame.cols,y + frame.rows));
		if (color[0] != 0 || color[1] != 0 || color[2] != 0)
			frame.at<cv::Vec3b>(cv::Point(x,y)) = color;
	}

	lastfaces = faces;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		std::cerr << "error: args\n";
		return -1;
	}

	cv::Mat hat = cv::imread(argv[1]);
	if (!hat.data)
	{
		std::cerr << "Could not load hat." << std::endl;
		return -1;
	}

	cv::CascadeClassifier faceCascade;
	if (!faceCascade.load("./haarcascade_frontalface_alt.xml"))
	{
		std::cerr << "Could not load face detector." << std::endl;
		return -1;
	}

	cv::Mat frame;
	char *charbuf = (char*)calloc(1, 8);
	while (1)
	{
		std::vector<uchar> inbuf;
		std::vector<uchar> buf;
		int success = 0;
		while (read(0, &charbuf[3], 1) == 1) {
			if (!strcmp(charbuf, "\r\n\r\n")) {
				success = 1;
				break;
			}
			memmove(charbuf, &charbuf[1], 3);
		}
		if (!success)
			continue;
		memset(charbuf, 0, 4);
		success = 0;
		while (read(0, &charbuf[3], 1) == 1) {
			if (!strcmp(charbuf, "\r\n--")) {
				success = 1;
				inbuf.pop_back(); inbuf.pop_back(); inbuf.pop_back();
				break;
			}
			inbuf.push_back(charbuf[3]);
			memmove(charbuf, &charbuf[1], 3);
		}
		if (!success)
			continue;
		cv::Mat inmat = cv::Mat( 1, inbuf.size(), CV_8UC3, inbuf.data() );
		frame = cv::imdecode(inmat, CV_LOAD_IMAGE_COLOR);
		if (frame.data == NULL)
			continue;
		putHatOn(frame, faceCascade, hat);
		cv::imencode(".jpg", frame, buf);
		std::cout << "--ffserver\r\n";
		std::cout << "Content-Type: image/jpeg\r\n";
		std::cout << "Content-Length: " << buf.size() << "\r\n";
		std::cout << "\r\n";
		for (uchar &c : buf)
			std::cout << c;
		std::cout << "\r\n";
	}
	return 0;
}
