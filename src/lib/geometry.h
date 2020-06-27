#pragma once
#include <math.h>
#include "globals.h"

#define INF 10000 

// class Point 
// {
// public:
// 	float x; 
// 	float y; 

//     Point(float x_, float y_);
    
//     // Given three colinear points p, q, r, the function checks if 
//     // point q lies on line segment 'pr' 
//     bool onSegment(Point p, Point q, Point r);

//     // To find orientation of ordered triplet (p, q, r). 
//     // The function returns following values 
//     // 0 --> p, q and r are colinear 
//     // 1 --> Clockwise 
//     // 2 --> Counterclockwise 
//     float orientation(Point p, Point q, Point r);

//     // The function that returns true if line segment 'p1q1' 
//     // and 'p2q2' intersect. 
//     bool doIntersect(Point p1, Point q1, Point p2, Point q2);

//     // Returns true if the point p lies inside the polygon[] with n vertices 
//     bool isInside(Point polygon[], int n, Point p);

//     Point calc_newCOB(Point p, Point q, Point poly[], int n, float margin);
// }; 


struct Point
{
    float x;
    float y;
};

bool onSegment(Point p, Point q, Point r);

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
float orientation(Point p, Point q, Point r);

// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point p1, Point q1, Point p2, Point q2);

// Returns true if the point p lies inside the polygon[] with n vertices 
bool isInside(Point polygon[], int n, Point p);

/** line 1 is defined by p and q
 *  line 2 is defined by r and s
 *  Note: a parallel check should be performed before calling the function
 **/ 
Point line_intersection(Point p, Point q, Point r, Point s);

/**
 * Calculate new Center of body
 * p: swing leg
 * q: target support leg
 * s: support leg
 * t: support leg
 * poly[]: polygon containing all 4 points - doesnt matter when margin != -1
 * n: size of poly[]
 * margin: desired value of margin, float
 **/
Point calc_newCOB(Point p, Point q, Point s, Point t, Point poly[], int n, float margin);