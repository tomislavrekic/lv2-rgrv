// RG_LV1.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"

#include <conio.h>
#include <vector>
#include <vtkAutoInit.h>
#include <math.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAxesActor.h>
#include <vtkTextActor.h>
#include <vtkTransform.h>
#include "RVL3DTools.h"
#include "Display.h"
#include "Body.h"
#include <cmath>
#include "opencv2\imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstddef>
#include <time.h>

#define PI	3.14159265359

using namespace cv;
using namespace std;

struct UserData {
	int pointNum;
	Subdiv2D subdiv;
	Mat image;
};

void onMouse(int event, int x, int y, int flags, void *userData) {
	if (event == CV_EVENT_LBUTTONDOWN) {
		UserData* data = (UserData*)userData;
		Subdiv2D subdiv = data->subdiv;
		printf("location: x: %d  y: %d\n", x, y);
		for (int i = 4; i < data->pointNum+4; i++) {
			int* firstEdge = new int;
			int firstVertIndex = i;

			Point2f* verts[3];
			verts[0] = new Point2f();
			verts[1] = new Point2f();
			verts[2] = new Point2f();

			*verts[0] = subdiv.getVertex(firstVertIndex, firstEdge);

			int secondEdge = subdiv.getEdge(*firstEdge, subdiv.NEXT_AROUND_LEFT);
			int secondVertIndex = subdiv.edgeOrg(secondEdge, verts[1]);

			int thirdEdge = subdiv.getEdge(secondEdge, subdiv.NEXT_AROUND_LEFT);	
			int thirdVertIndex = subdiv.edgeOrg(thirdEdge, verts[2]);
			if (firstVertIndex < 4 || secondVertIndex < 4 || thirdVertIndex < 4) {
				continue;
			}
			if ((firstVertIndex > secondVertIndex) && (firstVertIndex > thirdVertIndex)) {
				continue;
			}

			Point2f edgeVectors[3];
			Point2f edgeNormal[3];
			Point2f edgeMidpoint[3];

			int k = 0;
			for (int j = 0; j < 3; j++) {
				k = j + 1;
				if (j == 2) k = 0;
				edgeVectors[j] = Point2f(Point2f(verts[k]->x - verts[j]->x, verts[k]->y - verts[j]->y));

				float edgeModulus = sqrt((edgeVectors[j].x * edgeVectors[j].x) + (edgeVectors[j].y * edgeVectors[j].y));
				edgeNormal[j] = Point2f((edgeVectors[j].y/edgeModulus), (-edgeVectors[j].x/edgeModulus));

				edgeMidpoint[j] = Point2f((verts[j]->x + verts[k]->x) / 2, (verts[j]->y + verts[k]->y) / 2);
			}			

			//FIRST METHOD
			int counter = 0;
			for (int j = 0; j < 3; j++) {				
				if (((edgeNormal[j].x * (x - edgeMidpoint[j].x)) + (edgeNormal[j].y * (y - edgeMidpoint[j].y))) <= 0) {
					counter++;					
				}
				printf("%d ; %d :: %d -> %f\n", i, j, counter, ((edgeNormal[j].x * (x - edgeMidpoint[j].x)) + (edgeNormal[j].y * (y - edgeMidpoint[j].y))));
			}
			if (counter == 3) {
				printf("\nPoint I : %.0f, %.0f  ;  Point II : %.0f, %.0f  ;  Point III : %.0f, %.0f\n", verts[0]->x, verts[0]->y, verts[1]->x, verts[1]->y, verts[2]->x, verts[2]->y);
				Mat tempImage = data->image.clone();
				const Point2i* points[3] = {
					new Point2i((int)verts[0]->x, (int)verts[0]->y),
					new Point2i((int)verts[1]->x, (int)verts[1]->y),
					new Point2i((int)verts[2]->x, (int)verts[2]->y) };

				//printf("\nALTPoint I : %d, %d  ;  Point II : %d, %d  ;  Point III : %d, %d\n", points[0]->x, points[0]->y, points[1]->x, points[1]->y, points[2]->x, points[2]->y);
				circle(tempImage, *points[0], 4, Scalar(0, 0, 200), 2);
				circle(tempImage, *points[1], 4, Scalar(0, 0, 200), 2);
				circle(tempImage, *points[2], 4, Scalar(0, 0, 200), 2);
				int npt[] = { 3 };

				//fillPoly(tempImage, points, npt, 1, Scalar(0, 0, 200), 8);
				imshow("Display", tempImage);
				cv::waitKey();
			}

			//SECOND METHOD
			//float w1 = (verts[0]->x * (verts[2]->y - verts[0]->y) + (y - verts[0]->y) * (verts[2]->x - verts[0]->x) - x * (verts[2]->y - verts[0]->y))/
			//	((verts[1]->y - verts[0]->y) * (verts[2]->x - verts[0]->x) - (verts[1]->x - verts[0]->x) * (verts[2]->y - verts[0]->y));

			//float w2 = (y - verts[0]->y - w1* (verts[1]->y - verts[0]->y)) / (verts[2]->y - verts[0]->y);

			//printf("%.2f ; %.2f\n", w1, w2);
			//if ( (w1 >= 0) && (w2 >= 0) &&(w1+w2 <= 1)){
			//	printf("\nPoint I : %.0f, %.0f  ;  Point II : %.0f, %.0f  ;  Point III : %.0f, %.0f\n", verts[0]->x, verts[0]->y, verts[1]->x, verts[1]->y, verts[2]->x, verts[2]->y);
			//	Mat tempImage = data->image.clone();
			//	const Point2i* points[3] = { 
			//		new Point2i((int)verts[0]->x, (int)verts[0]->y),
			//		new Point2i((int)verts[1]->x, (int)verts[1]->y),
			//		new Point2i((int)verts[2]->x, (int)verts[2]->y) };

			//	//printf("\nALTPoint I : %d, %d  ;  Point II : %d, %d  ;  Point III : %d, %d\n", points[0]->x, points[0]->y, points[1]->x, points[1]->y, points[2]->x, points[2]->y);
			//	circle(tempImage, *points[0], 4, Scalar(0, 0, 200), 2);
			//	circle(tempImage, *points[1], 4, Scalar(0, 0, 200), 2);
			//	circle(tempImage, *points[2], 4, Scalar(0, 0, 200), 2);
			//	int npt[] = { 3 };
			//	
			//	//fillPoly(tempImage, points, npt, 1, Scalar(0, 0, 200), 8);
			//	imshow("Display", tempImage);
			//	cv::waitKey();
			//}

			
		}
	}	
}

int main(int argc, char *argv[])
{
	srand(time(NULL));
	int pointNum = 20;
	int imgOffset = 20;
	int divSize = 400;
	int imgSize = 440;

	
	Rect bounds = Rect(imgOffset, imgOffset, divSize, divSize);

	Subdiv2D div = Subdiv2D::Subdiv2D(bounds);

	Mat image = Mat(imgSize, imgSize, CV_8UC3);

	div.insert(Point(imgOffset, imgOffset));
	div.insert(Point(imgOffset, divSize + imgOffset-1));
	div.insert(Point(divSize + imgOffset - 1, imgOffset));
	div.insert(Point(divSize + imgOffset - 1, divSize + imgOffset - 1));

	for (int i = 0; i < pointNum-4; i++) {
		int x = imgOffset + rand() % (divSize + 1);
		int y = imgOffset + rand() % (divSize + 1);
		Point2f newPoint = Point2f(x,y);
		div.insert(newPoint);
	}

	for (int i = 4; i < pointNum + 4; i++) {
		Point2f temp = div.getVertex(i);
		circle(image, temp, 3, Scalar(0,0,0));
	}

	vector<Vec4f> edgeList;
	div.getEdgeList(edgeList);
	int edgeNum = edgeList.size();
	cout << edgeNum;

	int edgeCounter = 0;
	//256 is a number of edges, value is experimental
	for (int i = 0; i < 250; i++) {

		Point2f* tempOrg = new Point2f();
		Point2f* tempDst = new Point2f();

		int tempOrgIndex = div.edgeOrg(i, tempOrg);
		int tempDstIndex = div.edgeDst(i, tempDst);
		printf("Enter: %d, %d\n", tempOrgIndex, tempDstIndex);
		if ((tempOrgIndex < tempDstIndex) && ((tempOrgIndex > 3) && (tempDstIndex > 3))) {
			line(image, *tempOrg, *tempDst, Scalar(0,255,0), 1);
			printf("Drawing: %d, %d\n", tempOrgIndex, tempDstIndex);
			//circle(image, *tempOrg, 3, Scalar(0, 0, 0));
			//circle(image, *tempDst, 3, Scalar(0, 0, 0));
			edgeCounter++;
		}		
		
	}
	cout << edgeCounter;

	namedWindow("Display");

	if (image.data) {
		imshow("Display", image);
	}
	UserData* userData = new UserData();
	userData->pointNum = pointNum;
	userData->subdiv = div;
	userData->image = image;
	cv::setMouseCallback("Display", (MouseCallback)onMouse, userData);

	cv::waitKey();
	
}
