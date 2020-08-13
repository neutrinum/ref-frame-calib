#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <cmath>

#define LONGITUDINAL 0
#define TRANSVERSAL 1
#define ROTX 0
#define ROTY 1
#define ROTZ 2
#define FIXED 0
#define CURRENT 1
#define PI 3.14159265358979323846264338327950288419716939937510
#define EPS 1e-7
#define GAMMA 3.0e5

using namespace std;
using namespace cv;

int getIndex(int i, int j) { return (int(-0.5*double(pow(i, 2)) + 6.5*double(i) - 1 + double(j))); }

vector<double> crossProd(vector<double> a, vector<double> b) {
	vector<double> c(3);
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
	return c;
}

vector<vector<double>> rot2euler(vector<double> x, vector<double> y, vector<double> z, int mode) { //http://www.gregslabaugh.net/publications/euler.pdf
	vector<vector<double>> angles;
	angles.resize(2, vector<double>(3,0)); // rows 3, colums 2
	
	if (mode==FIXED) {
		if (fabs(x[2]) != 1) {
			angles[0][1] = -asin(x[2]);
			angles[1][1] = PI - angles[0][1];
			angles[0][0] = atan2((y[2]) / cos(angles[0][1]), (z[2]) / cos(angles[0][1]));
			angles[1][0] = atan2((y[2]) / cos(angles[1][1]), (z[2]) / cos(angles[1][1]));
			angles[0][2] = atan2((x[1]) / cos(angles[0][1]), (x[0]) / cos(angles[0][1]));
			angles[1][2] = atan2((x[1]) / cos(angles[1][1]), (x[0]) / cos(angles[1][1]));
		}
		else {
			angles[0][2] = 0;
			angles[1][2] = 0;
			if (x[2] == -1) {
				angles[0][1] = PI / 2;
				angles[1][1] = PI / 2;
				angles[0][0] = atan2(y[0], z[0]);
				angles[1][0] = atan2(y[0], z[0]);
			}
			else {
				angles[0][1] = -PI / 2;
				angles[1][1] = -PI / 2;
				angles[0][0] = atan2(-y[0], -z[0]);
				angles[1][0] = atan2(-y[0], -z[0]);
			}
		}
	}
	else {

			double x_aux = -asin(z[1]);
			angles[0][0] = -atan2((z[1]) / cos(x_aux), (z[2]) / cos(x_aux));
			angles[0][1] = asin(z[0]);
			angles[0][2] = -atan2((y[0]) / cos(x_aux), (x[0]) / cos(x_aux));

			/*
			angles[0][0] = -asin(x[2]);
			angles[1][0] = PI - angles[0][0];
			angles[0][1] = -atan2((y[2]) / cos(angles[0][0]), (z[2]) / cos(angles[0][0]));
			angles[1][1] = -atan2((y[2]) / cos(angles[1][0]), (z[2]) / cos(angles[1][0]));
			angles[0][2] = atan2((x[1]) / cos(angles[0][0]), (x[0]) / cos(angles[0][0]));
			angles[1][2] = atan2((x[1]) / cos(angles[1][0]), (x[0]) / cos(angles[1][0]));*/
	
		
	}
	return angles;
}


vector<vector<double>> rotMat2eulerWorld(Mat R) {
	return rot2euler({ R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0) },
		{ R.at<double>(0, 1), R.at<double>(1, 1), R.at<double>(2, 1) },
		{ R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2) },FIXED);
}

vector<vector<double>> rotMat2eulerCurrent(Mat R) {
	return rot2euler({ R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0) },
		{ R.at<double>(0, 1), R.at<double>(1, 1), R.at<double>(2, 1) },
		{ R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2) },CURRENT);
	/*for (int an = 0; an < 2; an++) {
		double xAngAux = angles[an][0];
		angles[an][0] = angles[an][1];
		angles[an][1] = -xAngAux;
	}*/
	//return angles;

}

Mat getRotMatrix(double angle, int axis) {
	Mat R = Mat::zeros(3, 3, CV_64F);
	switch (axis) {
		case ROTX: R.at<double>(0, 0) = 1;
			R.at<double>(1, 1) = cos(angle);
			R.at<double>(2, 1) = sin(angle);
			R.at<double>(2, 2) = cos(angle);
			R.at<double>(1, 2) = -sin(angle);	
			break;

		case ROTY: R.at<double>(1, 1) = 1;
			R.at<double>(0, 0) = cos(angle);
			R.at<double>(2, 0) = -sin(angle);
			R.at<double>(2, 2) = cos(angle);
			R.at<double>(0, 2) = sin(angle);
			break;

		case ROTZ: R.at<double>(2, 2) = 1;
			R.at<double>(0, 0) = cos(angle);
			R.at<double>(1, 0) = sin(angle);
			R.at<double>(1, 1) = cos(angle);
			R.at<double>(0, 1) = -sin(angle);
			break;
	}
	return R;
}


void customFilter(void);
/// Custom filter
	/*
	for (int i = 30; i < HEIGHT-2-30; i++)
		for (int j = 40; j < WIDTH-2-40; j++){
			avg = 0;
			src.at<Vec3b>(i + 1, j + 1) += Vec3b(0,0,128);
			//imshow("progress", src);
			//waitKey(1);
			for (int m = -1; m < 2; m++)
				for (int n = -1; n < 2; n++) {
					// laplacian
					if (m==0 && n==0)
						avg += 8* blk.at<uchar>(i + 1 + m, j + 1 + n);
					else
						avg -= blk.at<uchar>(i + 1 + m, j + 1 + n);
					// lowpass
					//avg += gray.at<uchar>(i + 1 + m, j + 1 + n);
				}
			cout << avg << endl;
			cnt.at<uchar>(i+1, j+1) = int(avg/9);
		}
	*/


pair<double,double> calcDist(int displacementType, double displacement, double focal, double x1, double x2) {
	double x_world, z_world;
	switch (displacementType) {
	case 0: x_world = displacement * (x1*x2) / ((x1 - x2) * focal);
			z_world = x2 * displacement / (x1 - x2);
			break;
	case 1: x_world = displacement * x1 / (x1 - x2);
			z_world = focal * displacement / (x1 - x2);
			break;
	}

	return make_pair(x_world, z_world);
}


void getLapl(Mat src, Mat& blr, Mat& lplc, int ddepth, int kernel_lplc, double scale, int HEIGHT, int WIDTH) {//(iooppppp)

	Mat lplc_; // auxiliar variable
	GaussianBlur(src, blr, Size(3, 3), 0, 0, BORDER_DEFAULT);
	Laplacian(blr, lplc_, ddepth, kernel_lplc, scale);

	for (int i = 0; i < HEIGHT; i++) {
		for (int j = 0; j < WIDTH; j++) {
			lplc.at<uchar>(i, j) = lplc_.at<short>(i, j) / 256 + 128;
		}
	}
	/*namedWindow("lplc", CV_WINDOW_AUTOSIZE);
	imshow("lplc", lplc);
	waitKey(0);*/
}


void binarize(Mat src, Mat& bi, int thres) {//(iop)
	// get binary image
	threshold(src, bi, thres, 255, 0);

	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(bi, bi, MORPH_ERODE, kernel);

	Mat kernel1 = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
	morphologyEx(bi, bi, MORPH_DILATE, kernel1);

	namedWindow("bi", CV_WINDOW_AUTOSIZE);
	imshow("bi", bi);
	//waitKey(0);
}


void contours(Mat bi, Mat cnt[], Mat roi[], Mat& cnts, int HEIGHT, int WIDTH) {//(iooopp)
	Mat aux(HEIGHT, WIDTH, CV_8UC1, Scalar(0));
	int searchIdx = 0, x = 0, y = 0, m = 0;

	while (x < HEIGHT) {
		//cout << x << endl;
		if (bi.at<uchar>(x, y) > 0 && aux.at<uchar>(x, y) == 0) {

			int k = 0, dir, d = 1, i = 0, j = 0, turned, search_x, search_y;
			bool finish = false, new_pix = true;

			while (!finish) {
				if (new_pix) {
					cnt[m].at<uchar>(x + i, y + j) = 255;
					turned = 0;
					dir = d;
					new_pix = false;
					d = (dir + 3) % 4;
				}
				else {
					turned++;
					if (turned == 4)
						break;
					d = (d + 1) % 4;
				}

				switch (d) {
				case 0: search_x = x + i - 1; search_y = y + j; break;
				case 1: search_x = x + i; search_y = y + j + 1; break;
				case 2: search_x = x + i + 1; search_y = y + j; break;
				case 3: search_x = x + i; search_y = y + j - 1; break;
				}

				if (cnt[m].at<uchar>(search_x, search_y) != 0 && search_x == x && search_y == y) {
					finish = true; break;
				}
				else if (bi.at<uchar>(search_x, search_y) > 0) {
					new_pix = true; i = search_x - x; j = search_y - y;
				}

			}

			// Fill found contour
			add(roi[m], cnt[m], roi[m]);
			add(cnts, cnt[m], cnts);

			floodFill(roi[m], Point(y, x - 1), 127); // invert image

			subtract(Scalar::all(255), roi[m], roi[m]);

			threshold(roi[m], roi[m], 130, 0, THRESH_TOZERO_INV);

			threshold(roi[m], roi[m], 125, 255, THRESH_BINARY_INV);

			add(aux, roi[m], aux);

			m++;
		}

		searchIdx++;

		y = searchIdx % WIDTH;
		x = searchIdx / WIDTH;
	}
	
}




void assignCentroids(double centr[][2], double mean[]) {

	vector<double> vecProds(8,0); // vectorial products, initializes as a vector of zeros with length 8
	vector<double> dist(8); // distances to mean
	double mindist = 10000, maxdist = 0;
	vector<int> diagIdx(2);

	for (int c = 0; c < 8; c++) { // find diagonal vector, formed by furthest and nearest points to centroid (points 1 and 3)
		dist[c] = sqrt(pow(centr[c][0] - mean[0], 2) + pow(centr[c][1] - mean[1], 2));
		if (dist[c] < mindist) {
			mindist = dist[c];
			diagIdx[1] = c;
		}
		if (dist[c] > maxdist) {
			maxdist = dist[c];
			diagIdx[0] = c;
		}
	}
	vector<double> diagVec = { centr[diagIdx[0]][0] - centr[diagIdx[1]][0], centr[diagIdx[0]][1] - centr[diagIdx[1]][1], 0};
	vector<double> minVecProds = { 1.0e20,1.0e20 }, maxVecProds = { 0., 0. };
	vector<int> minVPidx(2), maxVPidx(2);
	

	for (int c = 0; c < 8; c++) {
		if (c != diagIdx[0] && c != diagIdx[1]) {

			vector<double> vecProd = crossProd(diagVec, vector<double> { centr[c][0] - mean[0], centr[c][1] - mean[1], 0 });
			vecProds[c] = vecProd[2];

			// Finding 2 mins vectorial products (points 4 and 5)
			double upBound = -1, upPos = 0;
			for (int k = 0; k < 2; k++) { // Pick max value of minVecProds, which will be substituted (minimized)
				if (fabs(minVecProds[k]) > upBound) {
					upBound = fabs(minVecProds[k]);
					upPos = k;
				}
			}
			if (fabs(vecProd[2]) < fabs(minVecProds[upPos])) { // substitute max value of minVecProds
				minVecProds[upPos] = vecProd[2];
				minVPidx[upPos] = c;
			}


			// Finding 2 maxs vectorial products (points 0 and 7)
			double lowBound = 1e20, lowPos = 0;
			for (int k = 0; k < 2; k++) { // Pick min value of maxVecProds, which will be substituted (minimized)
				if (fabs(maxVecProds[k]) < lowBound) {
					lowBound = fabs(maxVecProds[k]);
					lowPos = k;
				}
			}
			if (fabs(vecProd[2]) > fabs(maxVecProds[lowPos])) { // substitute min value of maxVecProds
				maxVecProds[lowPos] = vecProd[2];
				maxVPidx[lowPos] = c;
			}
		}
	}
	/*cout << diagIdx[0] << endl;
	cout << diagIdx[1] << endl;
	cout << minVPidx[0] << endl;
	cout << minVPidx[1] << endl;
	cout << maxVPidx[0] << endl;
	cout << maxVPidx[1] << endl;*/
	
	
	// Assigning new index
	double centr_aux[8][2];
	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 2; j++)
			centr_aux[i][j] = centr[i][j];

	for (int i = 0; i < 2; i++) {
		centr[1][i] = centr_aux[diagIdx[0]][i];
		centr[3][i] = centr_aux[diagIdx[1]][i];

		if (dist[minVPidx[0]] < dist[minVPidx[1]]) {
			centr[4][i] = centr_aux[minVPidx[0]][i];
			centr[5][i] = centr_aux[minVPidx[1]][i];
		}
		else {
			centr[5][i] = centr_aux[minVPidx[0]][i];
			centr[4][i] = centr_aux[minVPidx[1]][i];
		}

		if (maxVecProds[0] > maxVecProds[0]) { // check if the positive and negative are the one we think they are
			centr[7][i] = centr_aux[maxVPidx[0]][i];
			centr[0][i] = centr_aux[maxVPidx[1]][i];
		}
		else {
			centr[0][i] = centr_aux[maxVPidx[0]][i];
			centr[7][i] = centr_aux[maxVPidx[1]][i];
		}
	}
	
	vector<int> lastIdx(2);
	int k = 0;

	for (int c = 0; c < 8; c++) { // assing last 2 points (points 2 and 6)
		if (c != diagIdx[0] && c != diagIdx[1] && c != minVPidx[0] && c != minVPidx[1] && c != maxVPidx[0] && c != maxVPidx[1]) {
			lastIdx[k++] = c;
		}
	}

	for (int i = 0; i < 2; i++) {
		if (vecProds[lastIdx[0]] < vecProds[lastIdx[1]]) {
			centr[6][i] = centr_aux[lastIdx[0]][i];
			centr[2][i] = centr_aux[lastIdx[1]][i];
		}
		else {
			centr[2][i] = centr_aux[lastIdx[0]][i];
			centr[6][i] = centr_aux[lastIdx[1]][i];
		}
	}

}

void centroids(Mat roi[], Mat lplc, Mat& src, double centr[][2], double mean[], int HEIGHT, int WIDTH) {//(iiooppp)

	const int samples = 16;
	int valley_values[8][samples]; // values defining the local mimima of Laplacian for each mire
	int valley_xy[8][samples][2]; // coords of local minima

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < samples; j++) {
			valley_values[i][j] = 255;
		}
	}
	/// Find centroids
	for (int r = 0; r < 8; r++) { // 8 regions of interest
		for (int i = 0; i < HEIGHT; i++) {
			for (int j = 0; j < WIDTH; j++) {
				if (roi[r].at<uchar>(i, j)) {
					int maxVal = -1, pos_max = 0;
					for (int k = 0; k < samples; k++) { // Pick max value of valleyvalues, which will be substituted
						if (valley_values[r][k] > maxVal) {
							maxVal = valley_values[r][k];
							pos_max = k;
						}
					}
					if (lplc.at<uchar>(i, j) < valley_values[r][pos_max]) { // substitute max value of  valleyvalues
						valley_values[r][pos_max] = lplc.at<uchar>(i, j);
						valley_xy[r][pos_max][0] = i;
						valley_xy[r][pos_max][1] = j;
					}

				}
			}
		}
		int m0 = 0, wx = 0, wy = 0; // m0: moment 0 (sum of weights), w: weighted sum 
		for (int k = 0; k < samples; k++) {
			m0 += (255-valley_values[r][k]);
			wx += (255-valley_values[r][k]) * valley_xy[r][k][0];
			wy += (255-valley_values[r][k]) * valley_xy[r][k][1];
		}
		centr[r][0] = double(wx) / m0;
		mean[0] += centr[r][0] / 8;
		//if (centr[r][0] > bounds[1][0]) bounds[1][0] = centr[r][0];
		//if (centr[r][0] < bounds[0][0]) bounds[0][0] = centr[r][0];
		centr[r][1] = double(wy) / m0;
		mean[1] += centr[r][1] / 8;
		//if (centr[r][1] > bounds[1][1]) bounds[1][1] = centr[r][1];
		//if (centr[r][1] < bounds[0][1]) bounds[0][1] = centr[r][1];

		//cout << centr[r][0] << endl;
		//cout << centr[r][1] << endl;
		src.at<uchar>(int(centr[r][0]), int(centr[r][1])) = 0;
		src.at<uchar>(int(centr[r][0] + 1), int(centr[r][1]) + 1) = 0;
		//putText(src, to_string(r), Point(int(centr[r][1]), int(centr[r][0])), FONT_HERSHEY_SIMPLEX, 0.8, 255);

	}

	
	/// Debug write label
	Mat src_cpy = src.clone();
	src_cpy.at<uchar>(int(mean[0]), int(mean[1])) = 255;
	src_cpy.at<uchar>(int(mean[0] + 1), int(mean[1]) + 1) = 255;
	for (int r = 0; r < 8; r++)
		putText(src_cpy, to_string(r), Point(int(centr[r][1]), int(centr[r][0])), FONT_HERSHEY_SIMPLEX, 0.8, 255);
	//namedWindow("src_cpy", CV_WINDOW_AUTOSIZE);
	//imshow("src_cpy", src_cpy);

	
	/// Sort centroids
	assignCentroids(centr, mean);

	/// Write label
	for (int r = 0; r < 8; r++)
		putText(src, to_string(r), Point(int(centr[r][1]), int(centr[r][0])), FONT_HERSHEY_SIMPLEX, 0.8, 255);

	namedWindow("src", CV_WINDOW_AUTOSIZE);
	imshow("src", src);
	waitKey(1);

}


// optimization function for R
double F1(Mat R, Mat a, Mat NN){
	Mat RR = Mat::zeros(R.size() * 28, CV_64F);

	for (int i = 0; i < 28; i++) {
		Mat RRaux = RR.colRange(i*3, i*3 + 3).rowRange(i*3, i*3 + 3);
		R.copyTo(RRaux);
	}
	Mat x = RR.t() * a;
	Mat F1 = x.t() * NN * x;

	return F1.at<double>(0,0);
}

// optimization function for t
double F2(Mat t, Mat RR, Mat Qa, Mat pp, Mat Et) {

	Mat x = RR * pp + Et * t;
	Mat F2 = x.t() * Qa * x;
	return F2.at<double>(0, 0);
}


void runGradientDescent1(vector<double> &x, int iter, Mat a, Mat NN) {

	Mat R = Mat(getRotMatrix(x[0], ROTX) * getRotMatrix(x[1], ROTY) * getRotMatrix(x[2], ROTZ));
	double F = F1(R, a, NN);
	vector<double> dF = { 1,1,1 };
	int64 t0, t1;
	
	t0 = getTickCount(); // start timer for optimization

	int n=0;
	// iterate until tolerance threshold
	while (abs(dF[0]) > 5e-13 || abs(dF[1]) > 5e-13 || abs(dF[2]) > 5e-13) {

		// compute numerical grdient
		for (int i = 0; i < 3; i++) { 
			vector<double> dx(x);
			dx[i] += EPS;
			
			R = Mat(getRotMatrix(dx[0], ROTX) * getRotMatrix(dx[1], ROTY) * getRotMatrix(dx[2], ROTZ));
			//cout << R << endl;
			dF[i] = F1(R, a, NN) - F;
			//cout << "dloss" << endl;
			//cout << dF[i] << endl;
		}

		// update variables
		x[0] -= GAMMA * dF[0];
		x[1] -= GAMMA * dF[1];
		x[2] -= GAMMA * dF[2];

		R = Mat(getRotMatrix(x[0], ROTX) * getRotMatrix(x[1], ROTY) * getRotMatrix(x[2], ROTZ));
		F = F1(R, a, NN);


		// print results periodically
		if (n % 1000 == 0) {
			cout << n << " iteration" << endl;
			//cout << "x " << endl;

			cout << "dF " << endl;
			cout << dF[0] << endl;
			cout << dF[1] << endl;
			cout << dF[2] << endl;
			cout << "loss " << F << endl;
		}
		
		n++;
	}
	/*cout << scientific << setprecision(15) << x[0] << endl;
	cout << scientific << setprecision(15) << x[1] << endl;
	cout << scientific << setprecision(15) << x[2] << endl;*/

	t1 = getTickCount(); // end timer

	double secs = (t1 - t0) / getTickFrequency();
	cout << "Found in " << n << " iterations and " << secs << " sec." << endl;
}

void runGradientDescent2(vector<double> &x, int iter, Mat RR, Mat Qa, Mat pp, Mat Et) {

	Mat t = (Mat_<double>(3,1) << x[0], x[1], x[2]);
	double F = F2(t, RR, Qa, pp, Et);
	vector<double> dF = { 1,1,1 };
	int64 t0, t1;

	t0 = getTickCount();

	//for (int n = 0; n < iter; n++) 
	int n=0;
	while (abs(dF[0]) > 5e-13 || abs(dF[1]) > 5e-13 || abs(dF[2]) > 5e-13) {

		for (int i = 0; i < 3; i++) {
			vector<double> dx(x);
			dx[i] += EPS;
			t = (Mat_<double>(3, 1) << dx[0], dx[1], dx[2]);
			/*if (n%100 == 0){
				cout << "dt " << t << endl;
				waitKey(0);
			}*/
				
			dF[i] = F2(t, RR, Qa, pp, Et) - F;
		}
		x[0] -= GAMMA * dF[0];
		x[1] -= GAMMA * dF[1];
		x[2] -= GAMMA * dF[2];

		t = (Mat_<double>(3, 1) << x[0], x[1], x[2]);
		F = F2(t, RR, Qa, pp, Et);


		if (n % 5000 == 0) {

			cout << n << " iteration" << endl;
			/*cout << "x " << endl;
			cout << x[0] << endl;
			cout << x[1] << endl;
			cout << x[2] << endl;*/
			cout << "dF " << endl;
			cout << dF[0] << endl;
			cout << dF[1] << endl;
			cout << dF[2] << endl;
			cout << "loss " << F << endl;
		}
		n++;

	}
	t1 = getTickCount();
	double secs = (t1 - t0) / getTickFrequency();
	cout << "Found in " << n << " iterations and " << secs << " sec." << endl;

}


Mat getDisplcVec1(Mat R1, Mat R2, Mat t1, Mat t2) {
	return t1 - R1 * R2.t() * t2;
}

double getRotAngle(Mat R1, Mat R2) {
	return acos((trace(R1.t()*R2)[0] - 1) / 2);
}

vector<double> axisAngle(Mat R1, Mat R2) { // axis-angle representation of a rotation
	Mat R = R2* R1.t(); // rotation between two consecutive states
	double angle = getRotAngle(R1, R2);
	vector<double> axAng = { 0,0,0,angle };
	axAng[0] = (R.at<double>(2, 1) - R.at<double>(1, 2)) / (2 * sin(angle));
	axAng[1] = (R.at<double>(0, 2) - R.at<double>(2, 0)) / (2 * sin(angle));
	axAng[2] = (R.at<double>(1, 0) - R.at<double>(0, 1)) / (2 * sin(angle));
	return axAng;
}

double getDist2Frame(Mat displcVec, double rotAng) {
	double nrm = norm(displcVec);
	return (nrm / 2)*sqrt(pow((1 / tan(rotAng)), 2) + 1);
}

// This function is not used for final solution, was part of an initial approach which didn't include matlab
Mat getSkew(Mat v) {
	Mat skew = Mat::zeros(3, 3, CV_64F);
	skew.at<double>(0, 1) = -v.at<double>(2, 0);
	skew.at<double>(1, 0) = v.at<double>(2, 0);
	skew.at<double>(0, 2) = v.at<double>(1, 0);
	skew.at<double>(2, 0) = -v.at<double>(1, 0);
	skew.at<double>(1, 2) = -v.at<double>(0, 0);
	skew.at<double>(2, 1) = v.at<double>(0, 0);
	return skew;
}

// This function is not used for final solution, was part of an initial approach which didn't include matlab
Mat getRError(Mat displcVec, Mat displcVecTheor) {
	// displcVecTheor is a unit vector so its norm is 1
	//double norm = sqrt(Mat(displcVec.t()*displcVec).at<double>(0, 0));
	double nrm = norm(displcVec);
	double c = (Mat(displcVec.t()*displcVecTheor).at<double>(0, 0)) / nrm;
	Mat v = displcVec.cross(displcVecTheor) / nrm;
	Mat S = getSkew(v);

	Mat Rerror = Mat::eye(3, 3, CV_64F) + S + S * S*(1 / (1 + c));
	return Rerror;
}


// This function is not used for final solution, was part of an initial approach which didn't include matlab
Mat getDisplcVecTheor(double alpha, int rotType, int rotDir) {
	// rotType 0: yaw, 1: roll
	// rotDir 0: camera towards back / down for yaw / roll respectively, 1: opposite
	Mat displcVecTheor = Mat::zeros(3, 1, CV_64F);
	if (!rotType) {
		displcVecTheor.at<double>(1, 0) = -sin(alpha / 2);
		displcVecTheor.at<double>(2, 0) = (1-2*rotDir)*cos(alpha / 2);
	}
	else {
		displcVecTheor.at<double>(0, 0) = (1-2*rotDir)*cos(alpha / 2);
		displcVecTheor.at<double>(1, 0) = -sin(alpha / 2);
	}
	return displcVecTheor;
}


// ###########################################################################################################################################################

/// ##########################################################################################################################################################

// ###########################################################################################################################################################

/**
 * @function main
 */
int main(int argc, char** argv)
{
	bool rotationData = 1;
	vector<Mat> src;
	vector<int> rotDef;
	int thres;
	if (rotationData) { // set of images for SECOND series explained in the results of the report
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/rot1_1.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/rot1_2.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/rot1_3.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/rot1_4.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/rot1_5.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/rot1_6.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		rotDef = { 1, 0 };
		thres = 22;
	}
	else { // set of images for FIRST series explained in the results of the report
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/mot1_1.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/mot1_2.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/mot1_3.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/mot1_4.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/mot1_5.bmp", CV_LOAD_IMAGE_GRAYSCALE));
		rotDef = { 0, 0 };
		thres = 35;
	}
	
	/*src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/pp6.bmp", CV_LOAD_IMAGE_GRAYSCALE));
	src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/pp7.bmp", CV_LOAD_IMAGE_GRAYSCALE));*/

	
	int WIDTH = src[0].size().width, HEIGHT = src[0].size().height;

	

	// Here load p, N, obtained from calibration, and compute pp
	Mat N, p;
	FileStorage patternData("E:/MScEE/IAM/PROJECT/code/pattern.yml", FileStorage::READ);

	patternData["p"] >> p;
	patternData["N"] >> N;
	patternData.release();

	cout << "Loaded data." << endl;

	// Normalize
	p = p / 888;
	N = N / 888;

	Mat pp = Mat::zeros(3*28*2, 1, CV_64F);

	for (int i = 0; i < 7; i++) {
		for (int j = i + 1; j < 8; j++) {
			Mat ppaux1 = pp.rowRange(getIndex(i, j) * 6, getIndex(i, j) * 6 + 3).colRange(0,1);
			Mat ppaux2 = pp.rowRange(getIndex(i, j) * 6 + 3, getIndex(i, j) * 6 + 6).colRange(0, 1);

			Mat paux1 = p.rowRange(i*3, i*3 + 3).colRange(0, 1);
			Mat paux2 = p.rowRange(j*3, j*3 + 3).colRange(0, 1);
			
			paux1.copyTo(ppaux1);
			paux2.copyTo(ppaux2);
		}
	}
	

	// Now compute NN,
	Mat NN = Mat::zeros(3 * 28, 3 * 28, CV_64F), Et = Mat::zeros(3 * 28 * 2, 3, CV_64F);
	
	for (int i = 0; i < 28; i++) {
		//This NN matrix is precomputed (initialised in the program and fed to the function)
		Mat NNaux = NN.rowRange(i * 3, i * 3 + 3).colRange(i * 3, i * 3 + 3);
		Mat Naux = N.rowRange(i * 3, i * 3 + 3).colRange(0, 1);
		Mat Ni = Naux * Naux.t();
		Ni.copyTo(NNaux);
	}

	// Et,
	for (int i = 0; i < 28*2; i++) {
		Mat E = Mat::eye(3, 3, CV_64F);
		// Et will be precomputed
		Mat Etaux = Et.rowRange(i * 3, i * 3 + 3).colRange(0, 3);
		E.copyTo(Etaux);
	}

	cout << "Computed first parameters." << endl;


	// Now create Rotation and translation vectors which will store the data for all images
	vector<Mat> R(src.size()), t(src.size());
	
	// Loop for all images
	for (int im = 0; im < src.size(); im++) {

		// This is the image analysis part of the program (finding centroids), required to get a, aa
		Mat cnts, lplc;
		Mat roi[8], cnt[8];
		Mat blr, dst, bi;
		//int c_roi[8][2]; // centers of roi (maybe not necessary)
		double centr[8][2]; // centroids
		//double pattern[8][2] = { {-0.5,-0.5},{-0.5,0.5},{0.3277,-0.5},{0.3277,-0.3277},{0.4138,-0.4212},{0.5,-0.5},{0.5,-0.3277},{0.5,0.5} }; // matching pattern

		int kernel_lplc = 9;
		int ddepth = CV_16S;  //CV_8U, CV_16U, CV_16S, CV_32F or CV_64F
		double scale = 0.02;  // .02 for 9, .2 for 7

		lplc = Mat(HEIGHT, WIDTH, CV_8UC1, Scalar(0));
		cnts = Mat(HEIGHT, WIDTH, CV_8UC1, Scalar(0));
		for (int r = 0; r < 8; r++) {
			roi[r] = cnts.clone();
			cnt[r] = cnts.clone();
		}
		
		/// Laplacian of image
		getLapl(src[im], blr, lplc, ddepth, kernel_lplc, scale, HEIGHT, WIDTH);

		/// Binary image and morphological operations
		binarize(blr, bi, thres);

		/// Contour finding of binary image
		contours(bi, cnt, roi, cnts, HEIGHT, WIDTH);

		/// For all contours found, find centroid
		double mean[2] = { 0,0 }; // up left, bott right, x,y coordinates
		centroids(roi, lplc, src[im], centr, mean, HEIGHT, WIDTH);// , pattern);


		// Now compute a, Qa, as done in previous program for p, N (from the points coordinates in image)
		Mat a(3 * 28, 1, CV_64F);
		Mat Qa = Mat::zeros(3 * 28 * 2, 3 * 28 * 2, CV_64F);

		double focal = 20, cell_x = 0.0083, cell_y = 0.0086;
		double off_x, off_y;
		off_x = double(HEIGHT - 1) / 2;
		off_y = double(WIDTH - 1) / 2;

		for (int i = 0; i < 7; i++) {
			for (int j = i + 1; j < 8; j++) {
				double dist_x, dist_y; // ((image 1, image 2)
				dist_x = (centr[i][0] - centr[j][0])*cell_x;
				dist_y = (centr[i][1] - centr[j][1])*cell_y;

				// Get a vector with cross product
				vector<double> ai_vec = crossProd(vector<double>{ dist_x, dist_y, 0 }, vector<double>{ (centr[i][0] - off_x)*cell_x, (centr[i][1] - off_y)*cell_y, focal });
				double ai_norm = sqrt(pow(ai_vec[0], 2) + pow(ai_vec[1], 2) + pow(ai_vec[2], 2));
				ai_vec[0] /= ai_norm;
				ai_vec[1] /= ai_norm;
				ai_vec[2] /= ai_norm;

				// save result of vector in Mat
				Mat ai(3, 1, CV_64F, (double*)ai_vec.data());
				//memcpy(ai.data,ai_vec.data(),ai_vec.size()*sizeof(double))

				// copy results to a
				Mat a_aux = a.rowRange(getIndex(i, j) * 3, getIndex(i, j) * 3 + 3);
				ai.copyTo(a_aux);


				Mat Qa_aux1 = Qa.rowRange(getIndex(i, j) * 6, getIndex(i, j) * 6 + 3).colRange(getIndex(i, j) * 6, getIndex(i, j) * 6 + 3);
				Mat Qa_aux2 = Qa.rowRange(getIndex(i, j) * 6 + 3, getIndex(i, j) * 6 + 6).colRange(getIndex(i, j) * 6, getIndex(i, j) * 6 + 3);
				Mat ai2 = ai * ai.t();
				ai2.copyTo(Qa_aux1);
				ai2.copyTo(Qa_aux2);

			}
		}

		cout << "Computed second parameters, starting optimization." << endl;

		// OPTIMIZATION CODE

		// Here run F1 code and find R with gradient descent
		//Mat R;
		double f1, f2;
		vector<vector<double>> angles;
		//Mat eulAng(3, 1, CV_64F);
		vector<double> x1;
		if (im == 0)
			x1 = { 0.19753 , 0.06821 , -1.89653 }; // { 10.55555 / 180 * PI, 30.66666 / 180 * PI, -90. / 180 * PI }; // -5.74309 -2.53082 -90.5703 // -5. / 180 * PI, -2. / 180 * PI, -90. / 180 * PI
		else {
			angles = rotMat2eulerCurrent(R[im - 1]);
			x1 = { angles[0][0], angles[0][1], angles[0][2] }; // take previous here
		}
		int iter1 = 50000;

		// Initial R and F
		R[im] = Mat(getRotMatrix(x1[0], ROTX) * getRotMatrix(x1[1], ROTY) * getRotMatrix(x1[2], ROTZ));
		f1 = F1(R[im], a, NN);
		cout << endl << "Initial R loss: " << f1 << endl;
		//cout << R[im] << endl;

		// check angle (check rotation matrix to euler angle function)
		angles = rotMat2eulerCurrent(R[im]);
		//R[im] = Mat(getRotMatrix(angles[0][0], ROTX) * getRotMatrix(angles[0][1], ROTY) * getRotMatrix(angles[0][2], ROTZ));
		cout << R[im] << endl;
		for (auto i : angles) {
			for (auto j : i)
				std::cout << j / PI * 180 << ' ';
			cout << endl;
		}
	
		// run F1 optimization function
		runGradientDescent1(x1, iter1, a, NN);
		// Final R and F1
		R[im] = Mat(getRotMatrix(x1[0], ROTX) * getRotMatrix(x1[1], ROTY) * getRotMatrix(x1[2], ROTZ));
		cout << "Final R: " << endl;
		cout << R[im] << endl;

		// check angle
		angles = rotMat2eulerCurrent(R[im]);
		for (auto i : angles) {
			for (auto j : i)
				std::cout << j / PI * 180 << ' ';
			cout << endl;
		}

		// get F1 index value for finl optimization result
		f1 = F1(R[im], a, NN);
		cout << "with loss: " << f1 << endl << endl;

		// Now compute RR
		Mat RR = Mat::zeros(R[im].size() * 28 * 2, CV_64F);

		for (int i = 0; i < 28 * 2; i++) {
			Mat RRaux = RR.rowRange(i * 3, i * 3 + 3).colRange(i * 3, i * 3 + 3);
			R[im].copyTo(RRaux);
		}

		cout << "Computed first optimization, R." << endl;
		//waitKey(0);

		// Finally run F2 code and find t with gradient descent
		//Mat t;
		vector<double> x2;
		if (im == 0)
			x2 = { 0., 0., -6. }; // -5.74309 -2.53082 -90.5703 // -5. / 180 * PI, -2. / 180 * PI, -90. / 180 * PI
		else {
			x2 = { t[im-1].at<double>(0,0), t[im-1].at<double>(1,0), t[im-1].at<double>(2,0) }; // take previous here
		}
		int iter2 = 50000;

		// Initial t and F2
		t[im] = (Mat_<double>(3, 1) << x2[0], x2[1], x2[2]);
		f2 = F2(t[im], RR, Qa, pp, Et);
		cout << endl << "Initial t loss: " << f2 << endl;
		//cout << t[im] << endl;

		// run F2 optimization function
		runGradientDescent2(x2, iter2, RR, Qa, pp, Et);
		// Final t and F
		t[im] = (Mat_<double>(3, 1) << x2[0], x2[1], x2[2]);
		// get F2 index value for finl optimization result
		f2 = F2(t[im], RR, Qa, pp, Et);

		cout << "Final t: " << endl;
		cout << t[im] * 888 << endl; // *888 to scale up again, since parameters loaded were divided by 888
		cout << "with loss: " << f2 << endl << endl;

		cout << "Computed second optimiztion, t." << endl << endl;

		//waitKey(0);
	}


	// HERE THE PROGRAM OF ORIENTATION AND POSITION ESTIMATION ENDS

	// FROM HERE DOWN, THE CODE CORRESPONDS TO CAMERA CALIBRATION TASK

	// Now process R and t found in two images to do the calibration of camera
	cout << endl << "Now going to process tha data." << endl << endl;
	waitKey(0);

	Mat rotAxMean = Mat::zeros(3, 1, CV_64F);
	for (int im=1; im<src.size(); im++){ // for every two consecutive images of the sequence
		Mat displcVec = getDisplcVec1(R[im-1], R[im], t[im-1] * 888, t[im] * 888);

		vector<double> axAng = axisAngle(R[im - 1], R[im]); // axis-angle representation

		double rotAng = axAng[3]; // rotation angle
	
		Mat rotAx = (Mat_<double>(3, 1) << axAng[0], axAng[1], axAng[2]); // rotation axis
		rotAxMean += rotAx / (src.size()-1); // make an average of all rotation axes found

		double dist2frame = getDist2Frame(displcVec, rotAng);

		// The commented code was an initial approach to obtain the R calibration matrix, but it has been done in matlab in the final version

		/*Mat rotAxTheor = !rotDef[0] ?
			!rotDef[1] ? (Mat_<double>(3, 1) << -1, 0, 0) : (Mat_<double>(3, 1) << 1, 0, 0) :	// case 0, yaw angle (back, forth)
			!rotDef[1] ? (Mat_<double>(3, 1) << 0, 0, 1) : (Mat_<double>(3, 1) << 0, 0, -1);	// case 1, roll angle (down, up)
		double dist2frame = getDist2Frame(displcVec, rotAng);
		Mat displcVecTheor = getDisplcVecTheor(rotAng, rotDef[0], rotDef[1]);
		Mat Rerror = getRError(displcVec, displcVecTheor);

		Mat Rerror = getRError(rotAxTheor, rotAx);*/

		cout << im << endl;
		cout << "Measured rotation axis" << endl << rotAx << endl;
		//cout << "Theoretical rot axis" << endl << rotAxTheor << endl;
		cout << "Rotated angle " << rotAng * 180 / PI << endl;
		cout << "dist2frame " << dist2frame << endl;
		//cout << "R error matrix " << Rerror << endl << endl;

		/*vector<vector<double>> angles = rotMat2eulerCurrent(Rerror);
		for (auto i : angles) {
			for (auto j : i)
				std::cout << j / PI * 180 << ' ';
			cout << endl;
		}
		cout << endl;*/
	}
	cout << rotAxMean << endl;
	
	waitKey(0);

	return 0;
}