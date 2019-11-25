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
			int edges[3];
			int firstVertIndex = i;

			Point2f* verts[3];
			verts[0] = new Point2f();
			verts[1] = new Point2f();
			verts[2] = new Point2f();

			*verts[0] = subdiv.getVertex(firstVertIndex, &edges[0]);
			int beginningEdge = edges[0];
			int secondVertIndex, thirdVertIndex;

			//CYCLE THROUGH ALL TRIANGLES WHICH CONNECT TO THE FIRST VERTEX
			do {
				edges[0] = subdiv.getEdge(edges[0], subdiv.NEXT_AROUND_ORG);
				if (edges[0] == beginningEdge) {
					break;
				}
				edges[1] = subdiv.getEdge(edges[0], subdiv.NEXT_AROUND_LEFT);
				secondVertIndex = subdiv.edgeOrg(edges[1], verts[1]);

				edges[2] = subdiv.getEdge(edges[1], subdiv.NEXT_AROUND_LEFT);
				thirdVertIndex = subdiv.edgeOrg(edges[2], verts[2]);

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
					if (k == 3) k = 0;
					edgeVectors[j] = Point2f(Point2f(verts[k]->x - verts[j]->x, verts[k]->y - verts[j]->y));

					float edgeModulus = sqrt((edgeVectors[j].x * edgeVectors[j].x) + (edgeVectors[j].y * edgeVectors[j].y));
					edgeNormal[j] = Point2f((edgeVectors[j].y / edgeModulus), (-edgeVectors[j].x / edgeModulus));

					edgeMidpoint[j] = Point2f((verts[j]->x + verts[k]->x) / 2, (verts[j]->y + verts[k]->y) / 2);
				}

				//FIND TRIANGLE IN WHICH THE POINT IS
				int counter = 0;
				for (int j = 0; j < 3; j++) {
					if (((edgeNormal[j].x * (x - edgeMidpoint[j].x)) + (edgeNormal[j].y * (y - edgeMidpoint[j].y))) <= 0) {
						counter++;
					}
				}

				//DRAW POINTS AND EDGES IF PROPER TRIANGLE IS FOUND
				if (counter == 3) {
					printf("\nPoint I : %.0f, %.0f  ;  Point II : %.0f, %.0f  ;  Point III : %.0f, %.0f\n", verts[0]->x, verts[0]->y, verts[1]->x, verts[1]->y, verts[2]->x, verts[2]->y);
					Mat tempImage = data->image.clone();			

					int k = 0;
					for (int j = 0; j < 3; j++) {
						k = j + 1;
						if (k == 3) {
							k = 0;
						}

						circle(tempImage, *verts[j], 5, Scalar(0, 0, 200), 4);
						line(tempImage, *verts[j], *verts[k], Scalar(0, 0, 200), 3, 8);

						Point2f* neighbourVert = new Point2f();
						int neighbourVertIndex = subdiv.edgeOrg(subdiv.getEdge(edges[j], subdiv.NEXT_AROUND_RIGHT), neighbourVert);
						if (((neighbourVert->x >= 20) && (neighbourVert->x < 420)) && ((neighbourVert->y >= 20) && (neighbourVert->y < 420))) {
							circle(tempImage, *neighbourVert, 4, Scalar(200, 0, 0), 3);
							printf("\nPointBLUE : %.0f, %.0f\n", neighbourVert->x, neighbourVert->y);
							line(tempImage, *neighbourVert, *verts[j], Scalar(200, 0, 0), 2, 8);
							line(tempImage, *neighbourVert, *verts[k], Scalar(200, 0, 0), 2, 8);
						}						
					}
					imshow("Display", tempImage);
					cv::waitKey();
				}				
			} while (true);
			
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
			
	try {
		for (int i = 0; ; i++) {

			Point2f* tempOrg = new Point2f();
			Point2f* tempDst = new Point2f();

			int tempOrgIndex = div.edgeOrg(i, tempOrg);
			int tempDstIndex = div.edgeDst(i, tempDst);
			if ((tempOrgIndex < tempDstIndex) && ((tempOrgIndex > 3) && (tempDstIndex > 3))) {
				line(image, *tempOrg, *tempDst, Scalar(0, 255, 0), 1);
			}
		}
	}
	catch (Exception e) {
		printf("%s", e.msg);
	}	
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
