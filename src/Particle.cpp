#include "Particle.h"
#include <GL/glut.h>
#include <iostream>
#include <limits>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Particle::Particle(const Vector2f &ConstructPos, float mass, int index, TYPE type) : m_ConstructPos(ConstructPos), m_Position(ConstructPos), m_Velocity(Vector2f(0.0, 0.0)), mass(mass), index(index),
																					 type(type), MassCenter(ConstructPos), dimension(0.8)
{
	switch (type)
	{
	case NORMAL:
		rigid = 0;
		break;
	case RIGID:
	{
		rigid = 1;
		reset();
		setBoundingBox();
		break;
	}
	}
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vector2f(0.0, 0.0);
	m_Force = Vector2f(0.0, 0.0);
	x = m_ConstructPos;
	//rigid specific
	I = 0.0f;
	x = MassCenter;
	R = Matrix2f::Identity();
	P = Vector2f(0, 0); //M*v(t)
	L = 0.0f;			//I*w(t)
	omega = L / (I + 0.00000000001);
	torque = 0.0f;
	angle = 0.0f * M_PI / 180;
}

void Particle::draw() //draw a square
{
	const float h = 6.f;
	switch (type)
	{
	case NORMAL:
		// std::cout<<m_Position<<std::endl;
		glColor3f(1.f, 1.f, 1.f); //rgb
		glPointSize(h);
		glBegin(GL_POINTS);
		glVertex2f(m_Position[0], m_Position[1]);
		glEnd();
		break;
	case RIGID:
		// cout << "corners0: " << corners[0][0]*(1024/2) << corners[0][1]*(1024/2) << endl;
		// cout << "corners1: " << corners[1][0]*(1024/2) << corners[1][1]*(1024/2) << endl;
		// cout << "corners2: " << corners[2][0]*(1024/2) << corners[2][1]*(1024/2) << endl;
		// cout << "corners3: " << corners[3][0]*(1024/2) << corners[3][1]*(1024/2) << endl;
		// glClear(GL_COLOR_BUFFER_BIT);
		glColor3f(1.0, 1.0, 1.0);
		glBegin(GL_POLYGON);
		glVertex2f(corners[0][0], corners[0][1]);
		glVertex2f(corners[1][0], corners[1][1]);
		glVertex2f(corners[2][0], corners[2][1]);
		glVertex2f(corners[3][0], corners[3][1]);
		glVertex2f(corners[0][0], corners[0][1]);
		glEnd();
		break;
	}
}

void Particle::setBoundingBox()
{
	// local positions wrt masscenter, in order to deal with rotation
	// to be changed
	corners.push_back(Vector2f(-dimension / 2, dimension / 2));	 //local topleft
	corners.push_back(Vector2f(dimension / 2, dimension / 2));	 //local topright
	corners.push_back(Vector2f(dimension / 2, -dimension / 2));	 //local bottomright
	corners.push_back(Vector2f(-dimension / 2, -dimension / 2)); //local bottomleft

	// cout << "m_ConstructPos" <<m_ConstructPos << endl;              // 0.75
	// corners.push_back(Vector2f(0, dimension));//local top
	// corners.push_back(Vector2f(dimension , 0));//local right
	// corners.push_back(Vector2f(0 , -dimension));//local bottom
	// corners.push_back(Vector2f(-dimension , 0));//local left

	//corners rotated pos = corner pos*R + masscenter pos
	for (int k = 0; k < corners.size(); ++k)
	{
		corners[k] = R * corners[k] + x;
	}
}

// Do we confine corners to be local positions or global
vector<Vector2f> Particle::getBoundingBox()
{
	switch (type)
	{
	case NORMAL:
		break;
	case RIGID:
		return corners;
		break;
	}
}

vector<Vector2f> Particle::getClosestEdge(Vector2f point)
{
	int idx = 0, n = corners.size();
	float minDist = numeric_limits<float>::max(), currDist;
	for (int i = 0; i < n; i++)
	{
		currDist = minDistance(corners[i % n], corners[(i + 1) % n], point);
		if (minDist >= currDist)
		{
			idx = i;
			minDist = currDist;
			// cout << "idx: " << idx << endl;
			// cout << "mindist: " << minDist << endl;
		}
	}
	// cout << "idx: " << idx << endl;
	return vector<Vector2f>({corners[idx], corners[(idx + 1) % n]});
}

float Particle::minDistance(Vector2f p1, Vector2f p2, Vector2f p3)
{
	Vector2f e12 = p2 - p1, e23 = p3 - p2, e13 = p3 - p1;
	float d23 = e12.dot(e23), d13 = e12.dot(e13), minDist;
	if (d23 > 0)
	{
		minDist = e23.norm();
	}
	else if (d13 < 0)
	{
		minDist = e13.norm();
	}
	else
	{
		float mod = e12.norm();
		minDist = abs(e12[0] * e13[1] - e12[1] * e13[0]) / mod;
	}
	return minDist;
}

vector<Vector4f> Particle::BoundingGrid(int grid_N)
{
	vector<Vector4f> bound_grids;

	vector<Vector2i> corner_absolute;
	Vector4f temp4f, result;
	Vector2i temp2i;
	Vector2i top, bottom, left, right;
	Vector2f grid_center, vector_length;

	float grid_length = 2.0 / grid_N;

	for (int i = 0; i < 4; i++)
	{
		temp2i[0] = int((corners[i][0] - (-1)) / grid_length);
		temp2i[1] = int((1 - corners[i][1]) / grid_length);
		corner_absolute.push_back(temp2i);
	}

	left = corner_absolute[0];
	right = corner_absolute[0];
	top = corner_absolute[0];
	bottom = corner_absolute[0];
	for (int i = 1; i < 4; i++)
	{
		if (corner_absolute[i][0] > right[0])
		{
			right = corner_absolute[i];
		}
		if (corner_absolute[i][0] < left[0])
		{
			left = corner_absolute[i];
		}
		if (corner_absolute[i][1] < top[1])
		{
			top = corner_absolute[i];
		}
		if (corner_absolute[i][1] > bottom[1])
		{
			bottom = corner_absolute[i];
		}
	}

	// cout<<"left" << left[0] << " " << left[1] << endl;
	// cout<<"right" << right[0] << " " << right[1] << endl;
	// cout<<"bottom" << bottom[0] << " " << bottom[1] <<endl;
	// cout<<"top" << top[0] << " " << top[1] << endl;

	// cout<<"corner_absolute "<<corner_absolute[0][0]<<" "<<corner_absolute[0][1]<<endl;
	// cout<<"corner_absolute "<<corner_absolute[1][0]<<" "<<corner_absolute[1][1]<<endl;
	// cout<<"corner_absolute "<<corner_absolute[2][0]<<" "<<corner_absolute[2][1]<<endl;
	// cout<<"corner_absolute "<<corner_absolute[3][0]<<" "<<corner_absolute[3][1]<<endl;

	//case 1 平行
	if (corner_absolute[0][1] == corner_absolute[1][1] || corner_absolute[0][0] == corner_absolute[1][0])
	{
		// cout<<"running "<<endl;

		for (int i = left[0]; i < right[0]; i++)
		{
			grid_center[0] = i * grid_length + (grid_length / 2) - 1;
			grid_center[1] = (1 - top[1] * grid_length) - (grid_length / 2);
			vector_length = grid_center - m_Position;
			temp4f[0] = float(i + 1);				  //i
			temp4f[1] = float(grid_N - (top[1] + 1)); //j
			temp4f[2] = vector_length[0];
			temp4f[3] = vector_length[1];
			bound_grids.push_back(temp4f);
		}
		for (int j = top[1]; j < bottom[1]; j++)
		{
			grid_center[0] = right[0] * grid_length + (grid_length / 2) - 1;
			grid_center[1] = (1 - j * grid_length) - (grid_length / 2);
			vector_length = grid_center - m_Position;
			temp4f[0] = float(right[0] + 1);	 //i
			temp4f[1] = float(grid_N - (j + 1)); //j
			temp4f[2] = vector_length[0];
			temp4f[3] = vector_length[1];
			bound_grids.push_back(temp4f);
		}
		for (int i = right[0]; i > left[0]; i--)
		{
			grid_center[0] = i * grid_length + (grid_length / 2) - 1;
			grid_center[1] = (1 - bottom[1] * grid_length) - (grid_length / 2);
			vector_length = grid_center - m_Position;
			temp4f[0] = float(i + 1);					 //i
			temp4f[1] = float(grid_N - (bottom[1] + 1)); //j
			temp4f[2] = vector_length[0];
			temp4f[3] = vector_length[1];
			bound_grids.push_back(temp4f);
		}
		for (int j = bottom[1]; j > top[1]; j--)
		{
			grid_center[0] = left[0] * grid_length + (grid_length / 2) - 1;
			grid_center[1] = (1 - j * grid_length) - (grid_length / 2);
			vector_length = grid_center - m_Position;
			temp4f[0] = float(left[0] + 1);		 //i
			temp4f[1] = float(grid_N - (j + 1)); //j
			temp4f[2] = vector_length[0];
			temp4f[3] = vector_length[1];
			bound_grids.push_back(temp4f);
		}
	}
	else
	{
		// bound_grids.push_back(left);bound_grids.push_back();
		// bound_grids.push_back(temp4f);bound_grids.push_back(temp4f);

		// cout<<"corner "<<corners[0][0]<<" "<<corners[0][1]<<endl;
		// cout<<"corner "<<corners[1][0]<<" "<<corners[1][1]<<endl;
		// cout<<"corner "<<corners[2][0]<<" "<<corners[2][1]<<endl;
		// cout<<"corner "<<corners[3][0]<<" "<<corners[3][1]<<endl;

		float grid_diagonal = grid_length * sqrt(2);

		// topleft
		float x1 = corners[3][0], x2 = corners[0][0], y1 = corners[3][1], y2 = corners[0][1];
		for (int j = top[1]; j <= left[1]; j++)
		{
			for (int i = left[0]; i <= top[0]; i++)
			{
				if (i == left[0] && j == left[1])
				{
					continue;
				}
				grid_center[0] = i * grid_length + (grid_length / 2) - 1;
				grid_center[1] = (1 - j * grid_length) - (grid_length / 2);
				vector_length = grid_center - m_Position;
				float dist = abs((x1 - grid_center[0]) * (y2 - grid_center[1]) - (x2 - grid_center[0]) * (y1 - grid_center[1])) / sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
				// cout << "dist"<< dist << "diagonal/2" << grid_diagonal/2 << endl;
				if (dist <= grid_diagonal / 2)
				{
					temp4f[0] = float(i + 1);			 //i
					temp4f[1] = float(grid_N - (j + 1)); //j
					temp4f[2] = vector_length[0];
					temp4f[3] = vector_length[1];
					bound_grids.push_back(temp4f);
				}
			}
		}

		//top right
		x1 = corners[0][0];
		x2 = corners[1][0];
		y1 = corners[0][1];
		y2 = corners[1][1];
		for (int j = top[1]; j <= right[1]; j++)
		{
			for (int i = top[0]; i <= right[0]; i++)
			{
				if (i == top[0] && j == top[1])
				{
					continue;
				}
				grid_center[0] = i * grid_length + (grid_length / 2) - 1;
				grid_center[1] = (1 - j * grid_length) - (grid_length / 2);
				vector_length = grid_center - m_Position;
				float dist = abs((x1 - grid_center[0]) * (y2 - grid_center[1]) - (x2 - grid_center[0]) * (y1 - grid_center[1])) / sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
				// cout << "dist"<< dist << "diagonal/2" << grid_diagonal/2 << endl;
				if (dist <= grid_diagonal / 2)
				{
					temp4f[0] = float(i + 1);			 //i
					temp4f[1] = float(grid_N - (j + 1)); //j
					temp4f[2] = vector_length[0];
					temp4f[3] = vector_length[1];
					bound_grids.push_back(temp4f);
				}
			}
		}

		//bottomright
		x1 = corners[1][0];
		x2 = corners[2][0];
		y1 = corners[1][1];
		y2 = corners[2][1];
		for (int j = right[1]; j <= bottom[1]; j++)
		{
			for (int i = bottom[0]; i <= right[0]; i++)
			{
				if (i == right[0] && j == right[1])
				{
					continue;
				}
				grid_center[0] = i * grid_length + (grid_length / 2) - 1;
				grid_center[1] = (1 - j * grid_length) - (grid_length / 2);
				vector_length = grid_center - m_Position;
				float dist = abs((x1 - grid_center[0]) * (y2 - grid_center[1]) - (x2 - grid_center[0]) * (y1 - grid_center[1])) / sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
				// cout << "dist"<< dist << "diagonal/2" << grid_diagonal/2 << endl;
				if (dist <= grid_diagonal / 2)
				{
					temp4f[0] = float(i + 1);			 //i
					temp4f[1] = float(grid_N - (j + 1)); //j
					temp4f[2] = vector_length[0];
					temp4f[3] = vector_length[1];
					bound_grids.push_back(temp4f);
				}
			}
		}

		//bottomleft
		x1 = corners[2][0];
		x2 = corners[3][0];
		y1 = corners[2][1];
		y2 = corners[3][1];
		for (int j = left[1]; j <= bottom[1]; j++)
		{
			for (int i = left[0]; i <= bottom[0]; i++)
			{
				if (i == bottom[0] && j == bottom[1])
				{
					continue;
				}
				grid_center[0] = i * grid_length + (grid_length / 2) - 1;
				grid_center[1] = (1 - j * grid_length) - (grid_length / 2);
				vector_length = grid_center - m_Position;
				float dist = abs((x1 - grid_center[0]) * (y2 - grid_center[1]) - (x2 - grid_center[0]) * (y1 - grid_center[1])) / sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
				// cout << "dist"<< dist << "diagonal/2" << grid_diagonal/2 << endl;
				if (dist <= grid_diagonal / 2)
				{
					temp4f[0] = float(i + 1);			 //i
					temp4f[1] = float(grid_N - (j + 1)); //j
					temp4f[2] = vector_length[0];
					temp4f[3] = vector_length[1];
					bound_grids.push_back(temp4f);
				}
			}
		}
	}

	return bound_grids;
}

bool Particle::compareVectors(Vector2i g1, Vector2i g2)
{
	return (g1[1] < g2[1]);
}

vector<Vector2i> Particle::InnerGrid(vector<Vector4f> boundGrid4f)
{
	vector<Vector2i> boundGrid;
	vector<Vector2i> innerGrid;
	vector<int> rowGrid;
	Vector2i temp;
	int dist;

	for (int i = 0; i < boundGrid4f.size(); i++)
	{
		temp[0] = int(boundGrid4f[i][0]);
		temp[1] = int(boundGrid4f[i][1]);
		boundGrid.push_back(temp);
	}

	sort(boundGrid.begin(), boundGrid.end(), compareVectors);
	// cout<< "AFTER 1" << endl;
	// for (int i=0; i< boundGrid.size();i++){
	//    cout<< "bound_grids"<< boundGrid[i][0] << " " << boundGrid[i][1] <<endl;
	// }

	for (int i = 0; i < boundGrid.size() - 1; i++)
	{
		if (boundGrid.size() < 3)
		{
			break;
		}

		rowGrid.push_back(boundGrid[i][0]);

		if (boundGrid[i][1] != boundGrid[i + 1][1] || i == boundGrid.size() - 1)
		{
			if (i == boundGrid.size() - 1)
			{
				rowGrid.push_back(boundGrid[i + 1][0]);
			}

			// 		sort(rowGrid.begin(), rowGrid.end());
			// 		// cout<< "AFTER 2" << endl;
			// 		// for (int a=0; a< rowGrid.size();a++){
			// 		// 	cout<< "row_grids"<< rowGrid[a] <<endl;
			// 		// }

			// 		for (int j = 1; j < rowGrid.size(); j++)
			// 		{

			// 			dist = rowGrid[j] - rowGrid[j - 1];
			// 			if (dist > 1)
			// 			{
			// 				for (int k = 1; k < dist; k++)
			// 				{
			// 					temp[0] = rowGrid[j - 1] + k;
			// 					temp[1] = boundGrid[i][1];
			// 					innerGrid.push_back(temp);
			// 				}
			// 			}
			// 		}
			// 		vector<int>().swap(rowGrid);
		}
	}

	return innerGrid;
}

void Particle::drawBound()
{
	vector<Vector4f> boundgrids = BoundingGrid(128);
	float x, y, h;
	int i, j;
	h = 2.0f / 128;

	glBegin(GL_QUADS);

	for (int u = 0; u < boundgrids.size(); u++)
	{
		i = boundgrids[u][0], j = boundgrids[u][1];
		x = (i - 0.5f) * h;
		x = -1 + x;
		y = (j - 0.5f) * h;
		y = -1 + y;

		glColor3f(139, 0, 0);
		glVertex2f(x, y);
		glColor3f(139, 0, 0);
		glVertex2f(x + h, y);
		glColor3f(139, 0, 0);
		glVertex2f(x + h, y + h);
		glColor3f(139, 0, 0);
		glVertex2f(x, y + h);
	}

	glEnd();
}
