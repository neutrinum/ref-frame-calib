#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

#define LONGITUDINAL 0
#define TRANSVERSAL 1
#define PI 3.1415926535897932384626433

using namespace std;
using namespace cv;

// cross product function
vector<double> crossProd(vector<double> a, vector<double> b) {
	vector<double> c(3);
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
	return c;
}

// rotation matrix to euler angles
vector<vector<double>> rot2euler(vector<double> x, vector<double> y, vector<double> z) { //http://www.gregslabaugh.net/publications/euler.pdf
	vector<vector<double>> angles;
	angles.resize(2, vector<double>(3,0)); // rows 3, colums 2
	
	if (x[2] != 1) {
		angles[0][0] = -asin(-x[2]);
		angles[1][0] = PI - angles[0][0];
		angles[0][1] = atan2((y[2]) / cos(angles[0][0]), (z[2]) / cos(angles[0][0]));
		angles[1][1] = atan2((y[2]) / cos(angles[1][0]), (z[2]) / cos(angles[1][0]));
		angles[0][2] = atan2((x[1]) / cos(angles[0][0]), (x[0]) / cos(angles[0][0]));
		angles[1][2] = atan2((x[1]) / cos(angles[1][0]), (x[0]) / cos(angles[1][0]));
	}
	else {
		angles[0][2] = 0;
		angles[1][2] = 0;
		if (x[2] == -1) {
			angles[0][0] = PI / 2;
			angles[1][0] = PI / 2;
			angles[0][1] = atan2(y[0], z[0]);
			angles[1][1] = atan2(y[0], z[0]);
		}
		else {
			angles[0][0] = -PI / 2;
			angles[1][0] = -PI / 2;
			angles[0][1] = atan2(-y[0], -z[0]);
			angles[1][1] = atan2(-y[0], -z[0]);
		}
	}

	return angles;
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

// calculate coordinates of the mires in real world from image coordinates using pinhole camera model
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

// get laplacian (lplc) and also blurred image from gaussian filter (blr)
void getLapl(Mat src, Mat& blr, Mat& lplc, int ddepth, int kernel_lplc, double scale, int HEIGHT, int WIDTH) {//(iooppppp)
	Mat lplc_; // auxiliar variable

	GaussianBlur(src, blr, Size(3, 3), 0, 0, BORDER_DEFAULT);
	Laplacian(blr, lplc_, ddepth, kernel_lplc, scale);

	// change data type from CV_16S to CV_8UC1
	for (int i = 0; i < HEIGHT; i++) {
		for (int j = 0; j < WIDTH; j++) {
			lplc.at<uchar>(i, j) = lplc_.at<short>(i, j) / 256 + 128;
		}
	}
	/*namedWindow("lplc", CV_WINDOW_AUTOSIZE);
	imshow("lplc", lplc);
	waitKey(0);*/
}

// obtain binary image (bi), and also realize morphological transformations (erode and dilate)
void binarize(Mat src, Mat& bi, int thres) {//(iop)
	// get binary image
	threshold(src, bi, thres, 255, 0);

	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(bi, bi, MORPH_ERODE, kernel); // erosion transformation

	Mat kernel1 = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
	morphologyEx(bi, bi, MORPH_DILATE, kernel1); // dilatation transformation

	namedWindow("bi", CV_WINDOW_AUTOSIZE); 
	imshow("bi", bi);
	waitKey(0);
}

// controur finding function
void contours(Mat bi, Mat cnt[], Mat roi[], Mat& cnts, int HEIGHT, int WIDTH) {//(iooopp
	// bi: binary image
	// cnt: vector of images, each one having one contour  
	// roi: vector of images, with a region of interest (contours filled) in each one
	// cnts: single image containing all contours
	Mat aux(HEIGHT, WIDTH, CV_8UC1, Scalar(0));
	int searchIdx = 0, x = 0, y = 0, m = 0;

	// start searching whole image
	while (x < HEIGHT) {
		//cout << x << endl;

		// white pixel found and not belonging to an already found contour
		if (bi.at<uchar>(x, y) > 0 && aux.at<uchar>(x, y) == 0) {

			// create helper variables. the directions of search (d) are 1:up 2:right 3:down 4:left
			int k = 0, d = 1, i = 0, j = 0, turned, search_x, search_y;
			bool finish = false, new_pix = true;

			// keep looking for contour while termination condition is not given. see below
			while (!finish) {
				// When a new pixel has been found, assign it to the matrix storing the contour (cnt[m])
				// Also reset the turning counter (turned) and update the direction of search (d) to be 
				// -90 deg wrt the direction in which the pixel has been found
				if (new_pix) {
					cnt[m].at<uchar>(x + i, y + j) = 255;
					turned = 0;
					new_pix = false;
					d = (d + 3) % 4;
				}
				// if not found in last iteration, increase  the searching direction 90 deg 
				// and the turned counter. if it reaches 4, the search ends (this corresponds to a controur of just 1 pixel)
				else {
					turned++;
					if (turned == 4)
						break;
					d = (d + 1) % 4;
				}
				// update new search position according to search direction
				switch (d) {
				case 0: search_x = x + i - 1; search_y = y + j; break;
				case 1: search_x = x + i; search_y = y + j + 1; break;
				case 2: search_x = x + i + 1; search_y = y + j; break;
				case 3: search_x = x + i; search_y = y + j - 1; break;
				}
				// termination condition: pixel at next search position belongs already to contour
				// and it is also the initial pixel found in the contour (x,y)
				if (cnt[m].at<uchar>(search_x, search_y) != 0 && search_x == x && search_y == y) {
					finish = true; break;
				}
				// new white pixel found
				else if (bi.at<uchar>(search_x, search_y) > 0) {
					new_pix = true; i = search_x - x; j = search_y - y;
				}

			}

			// Fill and store found contour
			add(roi[m], cnt[m], roi[m]);
			add(cnts, cnt[m], cnts);

			floodFill(roi[m], Point(y, x - 1), 127); // invert image

			//subtract(Scalar::all(255), roi[m], roi[m]);

			threshold(roi[m], roi[m], 130, 0, THRESH_TOZERO_INV);

			threshold(roi[m], roi[m], 125, 255, THRESH_BINARY_INV);

			// update auxiliar image. it stores all ROI (blobs) found until this point, 
			// to avoid searching a contour again on the same place
			add(aux, roi[m], aux);

			m++;
		}

		searchIdx++;

		y = searchIdx % WIDTH;
		x = searchIdx / WIDTH;
	}
	
}

// function for sorting the centroids obtained
void assignCentroids(double centr[][2], double mean[]) {

	vector<double> vecProds(8, 0); // vectorial products, initializes as a vector of zeros with length 8
	vector<double> dist(8); // distances to mean (centroid of pattern)
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
	vector<double> diagVec = { centr[diagIdx[0]][0] - centr[diagIdx[1]][0], centr[diagIdx[0]][1] - centr[diagIdx[1]][1], 0 };
	vector<double> minVecProds = { 1.0e20,1.0e20 }, maxVecProds = { 0., 0. };
	vector<int> minVPidx(2), maxVPidx(2);


	for (int c = 0; c < 8; c++) {
		if (c != diagIdx[0] && c != diagIdx[1]) {

			vector<double> vecProd = crossProd(diagVec, vector<double> { centr[c][0] - mean[0], centr[c][1] - mean[1], 0 });
			vecProds[c] = vecProd[2];

			// Finding 2 min vectorial products (points 4 and 5)
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


			// Finding 2 max vectorial products (points 0 and 7)
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

	// crete a copy of the unsorted centroids
	double centr_aux[8][2];
	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 2; j++)
			centr_aux[i][j] = centr[i][j];

	// re-assign centroids according to rules explained
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

	// last two centroids to re-assign (2 and 6)
	for (int c = 0; c < 8; c++) { 
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

// centroid finding algorithm
void centroids(Mat roi[], Mat lplc, Mat& src, double centr[][2], double mean[], int HEIGHT, int WIDTH) {//(iiooppp)

	const int samples = 16;
	int valley_values[8][samples]; // values defining the local mimima of Laplacian for each mire
	int valley_xy[8][samples][2]; // coords of local minima

	for (int i = 0; i < 8; i++) { // Since this is a minimum value search algorithm, the array is
		for (int j = 0; j < samples; j++) { // initialized with the upper bound values.
			valley_values[i][j] = 255;
		}
	}
	/// Find centroids
	for (int r = 0; r < 8; r++) { // 8 regions of interest
		for (int i = 0; i < HEIGHT; i++) {
			for (int j = 0; j < WIDTH; j++) {
				if (roi[r].at<uchar>(i, j)) {
					int maxVal = -1, pos_max = 0; // This is an algorithm to search minimum values. Therefore, 
												// the entries of array valley_values are compared each iteration
												// against a lower bound value, maxVal
					for (int k = 0; k < samples; k++) { // Pick max value of valleyvalues, which will be substituted
						if (valley_values[r][k] > maxVal) {
							maxVal = valley_values[r][k];
							pos_max = k;
						}
					}
					if (lplc.at<uchar>(i, j) < valley_values[r][pos_max]) { // Compare maximum value of valley_values 
																			// with anew pixel of the image, and
						valley_values[r][pos_max] = lplc.at<uchar>(i, j);	// substitute it if the value in image is lower
						valley_xy[r][pos_max][0] = i;						// Also save its position.
						valley_xy[r][pos_max][1] = j;
					}

				}
			}
		}
		// After the relevant values of a mire have been found, proceed to compute centroids
		int m0 = 0, wx = 0, wy = 0; // m0: moment 0 (sum of weights), w: weighted sum 
		for (int k = 0; k < samples; k++) { // weighted average
			m0 += (255 - valley_values[r][k]);
			wx += (255 - valley_values[r][k]) * valley_xy[r][k][0];
			wy += (255 - valley_values[r][k]) * valley_xy[r][k][1];
		}
		centr[r][0] = double(wx) / m0; // centroid X coordinate
		mean[0] += centr[r][0] / 8;

		centr[r][1] = double(wy) / m0; // centroid Y coordinate
		mean[1] += centr[r][1] / 8;


		//cout << centr[r][0] << endl;
		//cout << centr[r][1] << endl;
		src.at<uchar>(int(centr[r][0]), int(centr[r][1])) = 0;
		src.at<uchar>(int(centr[r][0] + 1), int(centr[r][1]) + 1) = 0;
		//putText(src, to_string(r), Point(int(centr[r][1]), int(centr[r][0])), FONT_HERSHEY_SIMPLEX, 0.8, 255);

	}


	/// Debug purposes, write label
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

	//namedWindow("src", CV_WINDOW_AUTOSIZE);
	//imshow("src", src);
	waitKey(1);

}


/**
 * @function main
 */
int main(int argc, char** argv)
{
	vector<Mat> src;
	Mat blr[2],bi[2];
	//int c_roi[8][2]; // centers of roi (maybe not necessary)
	double centr[2][8][2]; // centroids for both images
	// pattern obtained by measurements with ruler, used for debug purposes and to assess the results of some algorithms
	// this vector expresses approximated normalized coordinates (divided by 887 mm, approx side of the pattern) with respec to the center of the pattern.
	double pattern[8][2] = { {-0.5,-0.5},{-0.5,0.5},{0.3277,-0.5},{0.3277,-0.3277},{0.4138,-0.4212},{0.5,-0.5},{0.5,-0.3277},{0.5,0.5} }; 
	// thresholds for the pair of images to be loaded.
	int thres[] = { 25, 25 };
	
	int kernel_lplc = 9;  // kernel for laplacian convolution
	int ddepth = CV_16S;  //CV_8U, CV_16U, CV_16S, CV_32F or CV_64F
	double scale = 0.02;  // .02 for 9, .2 for 7

	/// Load image, get properties,
	src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/long3_2.bmp", CV_LOAD_IMAGE_GRAYSCALE));
	src.push_back(imread("E:/MScEE/IAM/PROJECT/pics_26_11/long3_3.bmp", CV_LOAD_IMAGE_GRAYSCALE));
	int WIDTH = src[0].size().width, HEIGHT = src[0].size().height;

	/// ans create images based on the size of original
	Mat cnts[2], lplc[2]; // contours, laplacian
	Mat roi[2][8], cnt[2][8]; // regions of interest, contours

	// now process both images in order to find the centroids
	for (int n = 0; n < 2; n++) {
		lplc[n] = Mat(HEIGHT, WIDTH, CV_8UC1, Scalar(0));
		cnts[n] = Mat(HEIGHT, WIDTH, CV_8UC1, Scalar(0));
		for (int r = 0; r < 8; r++) {
			roi[n][r] = cnts[n].clone();
			cnt[n][r] = cnts[n].clone();
		}
		
		/// Laplacian of image
		// input: src(raw image). output: blr, lplc (blured image and laplacian of it)
		getLapl(src[n], blr[n], lplc[n], ddepth, kernel_lplc, scale, HEIGHT, WIDTH);

		/// Binary image and morphological operations
		// input: blr (blurred image). output: bi (binary image)
		binarize(blr[n], bi[n], thres[n]);

		/// Contour finding of binary image
		contours(bi[n], cnt[n], roi[n], cnts[n], HEIGHT, WIDTH);

		/// For all contours found, find centroid
		double mean[2] = { 0,0 }; // vector that will store the controid of the mires centroids
		centroids(roi[n], lplc[n], src[n], centr[n], mean, HEIGHT, WIDTH);// , pattern);
	}

	/// Find postions in real world (PCA approch)
	int displacementType = LONGITUDINAL; // two kinds of displacements were first tested, but the lateral/transversal one 
										// was not able to find both X and Y coordinates, so the final solution only uses longitudinal
	double focal = 20, cell_x = 0.0083, cell_y = 0.0086; // camera parameters (cell is the size of the pixels in the ccd sensor)
	int delta_d = 1015;									// DISPLACEMENT OF THE CAMERA
	double off_x, off_y, mires[8][3];
	double bounds3d[3][2] = { {-100000,100000}, {-100000,100000}, {-1000,1000000}, }; // up and low bound for X,Y,Z
	double means[3] = { 0,0,0 };
	off_x = double(HEIGHT - 1) / 2;
	off_y = double(WIDTH - 1) / 2;

	for (int m = 0; m < 8; m++) {
		double z1, z2, img_x[2], img_y[2];
		// image coordinates expressed in mm (in camera reference frame) for both images and both coordinates
		img_x[0] = (centr[0][m][0] - off_x)*cell_x;
		img_x[1] = (centr[1][m][0] - off_x)*cell_x;
		img_y[0] = (centr[0][m][1] - off_y)*cell_y;
		img_y[1] = (centr[1][m][1] - off_y)*cell_y;

		cout << "mire " << m << endl;
		// << img_y[0] << endl;
		//cout << img_y[1] << "\n" << endl;

		// Coordinates of the mire in the world
		if (displacementType==LONGITUDINAL) {
			tie(mires[m][0], z1) = calcDist(LONGITUDINAL, delta_d, focal, img_x[0], img_x[1]);
			tie(mires[m][1], z2) = calcDist(LONGITUDINAL, delta_d, focal, img_y[0], img_y[1]);
			mires[m][2] = z2;// (z1 + z2) / 2;
		}
		else {
			tie(mires[m][1], z2) = calcDist(TRANSVERSAL, delta_d, focal, img_y[0], img_y[1]);
			mires[m][0] = img_x[0] * mires[m][1] / img_y[0];
			mires[m][2] = z2;
		}
		/*
		// Define bounds (not used in final solution, these are some parameters to show results of PCA 2D projection)
		if (mires[m][0] > bounds3d[0][0]) bounds3d[0][0] = mires[m][0];
		if (mires[m][0] < bounds3d[0][1]) bounds3d[0][1] = mires[m][0];
		if (mires[m][1] > bounds3d[1][0]) bounds3d[1][0] = mires[m][1];
		if (mires[m][1] < bounds3d[1][1]) bounds3d[1][1] = mires[m][1];
		if (mires[m][2] > bounds3d[2][0]) bounds3d[2][0] = mires[m][2];
		if (mires[m][2] < bounds3d[2][1]) bounds3d[2][1] = mires[m][2];*/

		// Define means (for PCA) (actually the PCA class has also a mean function)
		if (m != 4) {
			means[0] += mires[m][0] / 7;
			means[1] += mires[m][1] / 7;
			means[2] += mires[m][2] / 7;
		}
		
		if (m == 0) { // show some results (used in error analysis)
			cout << "x " << centr[0][m][0] << endl;
			cout << "y " << centr[0][m][1] << endl;
			cout << "x coord " << mires[m][0] << endl;
			cout << "y coord " << mires[m][1] << endl;
		}
		
		if (displacementType==LONGITUDINAL) cout << z1 << endl;
		cout << z2 << endl;
		//cout << mires[m][2] << "\n" << endl;
	}
	/// Distnace Matrix Approach
	// from world distances between mires (distance matrix distMat) to coordinates (coordinates matrix mVecBase)
	// https://math.stackexchange.com/questions/156161/finding-the-coordinates-of-points-from-distance-matrix
	// https://stackoverflow.com/questions/10963054/finding-the-coordinates-of-points-from-distance-matrix
	if (displacementType==LONGITUDINAL)
	{
		double varianc = 0; 
		vector<double> dists(28); // vector for all 28 distances
		Mat distMat = Mat::zeros(8, 8, CV_64F); // distance matrix

		// loop to find all distances between mires
		for (int i = 0; i < 7; i++) {
			for (int j = i + 1; j < 8; j++) {
				int indx = int(-0.5*(double)pow(i, 2) + 6.5*(double)i - 1 + (double)j);
				double z, dist[2], dist_x[2], dist_y[2]; // ((image 1, image 2)
				dist_x[0] = (centr[0][i][0] - centr[0][j][0])*cell_x;
				dist_x[1] = (centr[1][i][0] - centr[1][j][0])*cell_x;
				dist_y[0] = (centr[0][i][1] - centr[0][j][1])*cell_y;
				dist_y[1] = (centr[1][i][1] - centr[1][j][1])*cell_y;

				dist[0] = sqrt(pow(dist_x[0], 2) + pow(dist_y[0], 2)); // distance in first image
				dist[1] = sqrt(pow(dist_x[1], 2) + pow(dist_y[1], 2)); // distance in second image

				// find real world distance
				tie(dists[indx], z) = calcDist(LONGITUDINAL, delta_d, focal, dist[0], dist[1]);

				// show some results and check discrepance with respect to pattern obtained with measurement tape to assess accuracy
				cout << "mire " << i << " to " << j << endl;
				double tape = 887 * sqrt(pow(pattern[i][0] - pattern[j][0], 2) + pow(pattern[i][1] - pattern[j][1], 2));
				cout << dists[indx] << " vs " << tape << endl;
				if (i != 4 && j != 4)
					varianc += pow((dists[indx] - tape), 2) / 20;

				cout << z << endl;
				distMat.at<double>(i, j) = dists[indx];
				distMat.at<double>(j, i) = dists[indx];

			}
		}
		// show variance respect to reference positions obtained with measuring tape
		cout << "variance " << varianc << endl << endl;
		cout << "std dev " << sqrt(varianc) << endl << endl;

		// now find basis from distance matrix otbained
		Mat mMat = Mat::zeros(8, 8, CV_64F); // M matrix described in report
		Mat mEig = Mat::zeros(8, 1, CV_64F);
		Mat mVec = Mat::zeros(8, 8, CV_64F);
		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 8; j++) {
				mMat.at<double>(i, j) = 0.5 * (pow(distMat.at<double>(1, j), 2) + pow(distMat.at<double>(i, 1), 2) - pow(distMat.at<double>(i, j), 2));
			}
		}
		eigen(mMat, mEig, mVec);
		cout << distMat << endl;
		cout << mMat << endl;
		cout << mVec << endl;
		cout << "Eigen values" << endl << mEig << endl;


		Mat mEigBase = Mat::zeros(2, 1, CV_64F);
		Mat mVecBase = Mat::zeros(8, 2, CV_64F);
		// mVecBase is actually the coordinates of the mires obtained from the distance matrix following the method explained in report
		for (int i = 0; i < 8; i++) {
			mVecBase.at<double>(i, 0) = mVec.at<double>(0, i) * sqrt(mEig.at<double>(0, 0));
			mVecBase.at<double>(i, 1) = mVec.at<double>(1, i) * sqrt(mEig.at<double>(1, 0));
		}
		cout << mVecBase << endl;

		// Save data
		FileStorage file("E:/MScEE/IAM/PROJECT/data2.yml", FileStorage::WRITE);

		file << "mVecBase" << mVecBase;
		file.release();
	}

	/// PCA 
	//Camera coordinate system to plane coordinate system

	Mat points3d(7, 3, CV_64F);
	Mat points2d(7, 3, CV_64F);
	int q = 0;
	bool skip = false;
	for (int p = 0; p < 8; p++) {
		for (int i = 0; i < 3; i++)
			p != 4 ? points3d.at<double>(p - q, i) = mires[p][i] : skip = true; //- (bounds3d[i][0] + bounds3d[i][1])/2
			//p != 4 ? points3d.at<double>(p - q, i) = mires[p][i] - means[i] : skip = true;
		if (skip) {
			q += 1, skip = false;
		}
	}
	cout << "poins3d" << endl;
	cout << points3d << endl;

	// retain 3 dimensions
	PCA pca(points3d, Mat(), CV_PCA_DATA_AS_ROW, 3);
	pca.project(points3d, points2d);

	cout << pca.mean << endl;

	cout << "points2d" << endl;
	cout << points2d << endl;

	cout << pca.eigenvalues << endl;

	cout << pca.eigenvectors << endl;

	/* FOR THIS PART OF THE PROGRAM, THE DIMENSIONS RETAINED SHOULD BE 2

	// Now translate and rotate the points to express them in a known coord frame
	Mat eigVec = pca.eigenvectors;
	vector<double> xVec, yVec;
	xVec.assign(pca.eigenvectors.ptr<double>(0), pca.eigenvectors.ptr<double>(0) + pca.eigenvectors.cols);
	yVec.assign(pca.eigenvectors.ptr<double>(1), pca.eigenvectors.ptr<double>(1) + pca.eigenvectors.cols);
	vector<double> zVec = crossProd(xVec,yVec); // = crossProd()
	for (auto i : zVec)
		std::cout << i << ' ';
	cout << endl;
	//eigVec.push_back(Mat(zVec).reshape(1,3));

	vector<vector<double>> angles = rot2euler(xVec, yVec, zVec);
	for (auto i : angles) {
		for (auto j : i)
			std::cout << j/PI*180 << ' ';
		cout << endl;
	}

	//Mat rot = (Mat_<double>(2, 2) << cos(PI / 2 - angles[1][2]), cos(-angles[1][2]), sin(PI / 2 - angles[1][2]), sin(-angles[1][2]));
	//Mat rot = (Mat_<double>(2, 2) << -sin(PI / 2 - angles[1][2]), sin(-angles[1][2]), -cos(PI / 2 - angles[1][2]), cos(-angles[1][2]));
	Mat rot = (Mat_<double>(2, 2) << eigVec.at<double>(0,0), eigVec.at<double>(1, 0), eigVec.at<double>(0, 1), eigVec.at<double>(1, 1));
	Mat points = (rot * points2d.t()).t();
	cout << "rot" << endl;
	cout << rot << endl;
	
	double bottx = points.at<double>(4, 0), botty = points.at<double>(4, 1);
	points.col(0) = points.col(0) - bottx;
	points.col(1) = points.col(1) - botty;

	cout << "points" << endl;
	cout << points << endl;

	
	double bounds2d[2][2] = { {-100000,100000}, {-100000,100000} };
	for (int p = 0; p < 7; p++) {
		for (int i = 0; i < 2; i++) {
			if (points.at<double>(p, i) > bounds2d[i][0]) bounds2d[i][0] = points.at<double>(p, i);
			if (points.at<double>(p, i) < bounds2d[i][1]) bounds2d[i][1] = points.at<double>(p, i);
		}
	}
	Mat results(int(.6*(bounds2d[0][0]- bounds2d[0][1])), int(.6*(bounds2d[1][0] - bounds2d[1][1])), CV_8UC1, Scalar(0));
	for (int p = 0; p < 7; p++) {
		circle(results, Point(int(0.05*(bounds2d[0][0] - bounds2d[0][1]) - 0.5*points.at<double>(p, 0)), int(0.55*(bounds2d[1][0] - bounds2d[1][1]) - 0.5*points.at<double>(p, 1))), 5, Scalar(255), 2);
	}
	*/

	/// Display images
	namedWindow("source image 1", CV_WINDOW_AUTOSIZE);
	imshow("source image 1", src[0]);

	namedWindow("source image 2", CV_WINDOW_AUTOSIZE);
	imshow("source image 2", src[1]);

	//namedWindow("result", CV_WINDOW_AUTOSIZE);
	//imshow("result", results);

	/*namedWindow("result", CV_WINDOW_AUTOSIZE);
	imshow("result", cnts[0]);

	namedWindow("aux", CV_WINDOW_AUTOSIZE);
	imshow("aux", roi[0][1]);*/


	//if (waitKey(30) >= 0) break;
	waitKey(0);

	//}
	
	return 0;

	/*
	*/
}