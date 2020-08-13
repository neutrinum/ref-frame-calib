#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

int thres = 29, thrMax = 255;
char Threshold[50];


static void on_trackbar(int, void*) {}


/**
 * @function main
 */
int main(int argc, char** argv)
{

	Mat src, dst, lplc, bi, bi2, color, lplc_out, lplc_gray;
	
	int kernel_lplc = 9;
	int ddepth = CV_16S;  //CV_8U, CV_16U, CV_16S, CV_32F or CV_64F
	double scale = 0.02;  // .02 for 9, .2 for 7
	int HEIGHT, WIDTH;

	/// Load image
	src = imread("E:/MScEE/IAM/PROJECT/pics_26_11/long11_1.bmp", CV_LOAD_IMAGE_GRAYSCALE); //pics_08_12/shut3_7
	HEIGHT = src.size().height;
	WIDTH = src.size().width;
	

	if (!src.data)
	{
		return -1;
	}

	GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
	imshow("Blur", src);
	waitKey(0);
	Laplacian(src, lplc, ddepth, kernel_lplc, scale);

	imshow("Laplacian", lplc);
	waitKey(0);

	// converting back to CV_8U
	//convertScaleAbs(lplc, lplc_);

	//lplc.convertTo(lplc_out,CV_16U);

	//cout << lplc.at<short>(0, 0) << endl;
	Mat lplc_(HEIGHT, WIDTH, CV_8UC1, Scalar(0));
	for (int i = 0; i < HEIGHT; i++) {
		for (int j = 0; j < WIDTH; j++) {
			//cout << lplc.at<short>(i, j) / 256 + 127 << endl;
			lplc_.at<uchar>(i, j) = lplc.at<short>(i, j) / 256 + 127;
		}
	}

	imshow("Laplacian2", lplc_);
	waitKey(0);
	//cvtColor(lplc_out, lplc_gray, COLOR_BGR2GRAY);
	imwrite("E:/MScEE/IAM/PROJECT/lapl.png", lplc_);


	// Create trackbar
	namedWindow("threshold", WINDOW_AUTOSIZE); // Create Window

	sprintf_s(Threshold, "T %d", thrMax);
	createTrackbar(Threshold, "threshold", &thres, thrMax, on_trackbar);



	/// HISTOGRAM
	cout << src.channels() << endl;


	/// Establish the number of bins
	int histSize = 256;

	/// Set the ranges
	float range[] = { 0, 256 };
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;

	Mat bw_hist, bw_hist_out;

	/// Compute the histograms:
	calcHist(&src, 1, 0, Mat(), bw_hist, 1, &histSize, &histRange, uniform, accumulate);


	// Draw the histograms for BW
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound((double)hist_w / histSize);

	while (true)
	{

		Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

		/// Normalize the result to [ 0, histImage.rows ]
		normalize(bw_hist, bw_hist_out, 0, histImage.rows, NORM_MINMAX, -1, Mat());


		/// Draw for each channel
		for (int i = 1; i < histSize; i++)
		{
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(bw_hist_out.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(bw_hist_out.at<float>(i))),
				Scalar(255, 255, 255), 1, 8, 0);
		}
		/// Threshold
		thres = getTrackbarPos(Threshold, "threshold");
		line(histImage, Point(bin_w * thres, 0), Point(bin_w * thres, hist_h), Scalar(0, 0, 255));

		threshold(src, bi, thres, 255, 0);
		//threshold(src, bi, 0, 255, THRESH_BINARY + THRESH_OTSU);

		
		/*
		cvtColor(bi, bi2, COLOR_RGB2GRAY);

		//cout << src.channels() << endl;
		//cout << bin2.channels() << endl;

		Moments mu = moments(bi2);
		Point centroid = Point(mu.m10 / mu.m00, mu.m01 / mu.m00);

		circle(src, centroid, 5, Scalar(0, 0, 255), 2);
		*/
		/// Display histogram and image
		namedWindow("histogram", CV_WINDOW_AUTOSIZE);
		imshow("histogram", histImage);

		namedWindow("source image", CV_WINDOW_AUTOSIZE);
		imshow("source image", src);

		namedWindow("binary image", CV_WINDOW_AUTOSIZE);
		imshow("binary image", bi);



		if (waitKey(25) >= 0) { 
			
			imwrite("E:/MScEE/IAM/PROJECT/Pictures 8_11/out1.png", bi);

			break; 
		}
	}

	return 0;
}