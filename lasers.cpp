/*
** lasers.cpp
** 
** the definitive opencv for unix laser vision
** 
** compilation:
** 	g++ -o lasers lasers.cpp `pkg-config --cflags --libs opencv`
** usage:
**	cd path/to/opencv/data/haarcascades/
** 	ffmpeg -i /dev/video0 -f mpjpeg - | path/to/lasers | ffplay -
*/

#include <iostream>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp> 

void lasersBaby(cv::Mat &frame, cv::CascadeClassifier &faceCascade, cv::CascadeClassifier &eyeCascade)
{
	cv::Mat grayscale;
	cv::cvtColor(frame, grayscale, CV_BGR2GRAY);
	cv::equalizeHist(grayscale, grayscale); // enhance image contrast 
	std::vector<cv::Rect> faces;
	faceCascade.detectMultiScale(grayscale, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(150, 150));
	if (faces.size() == 0)
		return;

	cv::Mat face = grayscale(faces[0]); // crop the face
	std::vector<cv::Rect> eyes;
	eyeCascade.detectMultiScale(face, eyes, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));

	if (eyes.size() != 2)
		return;

	if (fabs(eyes[0].tl().x - eyes[1].tl().x) < 50)
		return;

	for (cv::Rect &eyeRect : eyes)
	{
		cv::Mat eye = face(eyeRect); // crop the eye
		cv::Point p = faces[0].tl() + (eyeRect.tl() + eyeRect.br()) * 0.5;
		cv::Point q = p;
		q.y = grayscale.rows - 1;
		q.x += (rand() % 255) - 127;
		cv::line(frame, p, q, cv::Scalar(0,0,255), 10);
	}
}

int main()
{
	srand(time(NULL));
	char *charbuf;

	cv::CascadeClassifier faceCascade;
	cv::CascadeClassifier eyeCascade;
	if (!faceCascade.load("./haarcascade_frontalface_alt.xml"))
	{
		std::cerr << "Could not load face detector." << std::endl;
		return -1;
	}
	if (!eyeCascade.load("./haarcascade_eye_tree_eyeglasses.xml"))
	{
		std::cerr << "Could not load eye detector." << std::endl;
		return -1;
	}

	cv::Mat frame;
	charbuf = (char*)calloc(1, 8);
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
		lasersBaby(frame, faceCascade, eyeCascade);
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

