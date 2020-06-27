#include "geometry.h"
/**
Point::Point(float x_, float y_)
{
    x = x_;
    y = y_;
}

bool Point::onSegment(Point p, Point q, Point r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
        return true; 
    return false; 
} 

float Point::orientation(Point p, Point q, Point r)
{ 
    float val = (q.y - p.y) * (r.x - q.x) - 
            (q.x - p.x) * (r.y - q.y); 

    if (val == 0) return 0; // colinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 

bool Point::doIntersect(Point p1, Point q1, Point p2, Point q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    float o1 = orientation(p1, q1, p2); 
    float o2 = orientation(p1, q1, q2); 
    float o3 = orientation(p2, q2, p1); 
    float o4 = orientation(p2, q2, q1); 

    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 

    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 

    return false; // Doesn't fall in any of the above cases 
} 

bool Point::isInside(Point polygon[], int n, Point p) 
{ 
    // There must be at least 3 vertices in polygon[] 
    if (n < 3) return false; 

    // Create a point for line segment from p to infinite 
    Point extreme(INF, p.y); 

    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0; 
    do
    { 
        int next = (i+1)%n; 

        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0) 
            return onSegment(polygon[i], p, polygon[next]); 

            count++; 
        } 
        i = next; 
    } while (i != 0); 

    // Return true if count is odd, false otherwise 
    return count&1; // Same as (count%2 == 1) 
} 

Point Point::calc_newCOB(Point p, Point q, Point poly[], int n, float margin)
{
    Point newCOB(0,0);
    if (margin != -1)
    {
        Point mid_point((p.x+q.x)/2, (p.y+q.y)/2);
        if (fabs(mid_point.x) < 0.0000000001)
        {
            mid_point.x = 0;
        }
        if (fabs(mid_point.y) < 0.0000000001)
        {
            mid_point.y = 0;
        }
        if ((p.y/q.y >= 0.99999) & (p.y/q.y <= 1.00001))
        {
            newCOB.x = mid_point.x;
            newCOB.y = mid_point.y + margin;
            if (isInside(poly, n, newCOB))
            {
                return newCOB;
            }
            else
            {
                newCOB.y = mid_point.y - margin;
                return newCOB;
            }
        }
        else if ((p.x/q.x >= 0.99999) & (p.x/q.x <= 1.00001))
        {
            newCOB.x = mid_point.x + margin;
            newCOB.y = mid_point.y;
            if (isInside(poly, n, newCOB))
            {
                return newCOB;
            }
            else
            {
                newCOB.x = mid_point.x - margin;
                return newCOB;
            }
        }
        else
        {
            float pq_slope = (p.y - q.y) / (p.x - q.x);
            float pq_offset = p.y - pq_slope * p.x;
            if (fabs(pq_offset) < 0.0000000001)
            {
                pq_offset = 0;
            }
            float k = -1/pq_slope;
            float b = mid_point.y - k * mid_point.x;
            float x = (b-pq_offset)/(pq_slope-k);
            float y = k*x+b;
            float diff_x = sqrt(pow(margin,2)/(pow(k,2)+1));
            float diff_y = k*diff_x;
            if (fabs(diff_x) < 0.0000000001)
            {
                diff_x = 0;
            }
            if (fabs(diff_y) < 0.0000000001)
            {
                diff_y = 0;
            }
            newCOB.x = x+diff_x;
            newCOB.y = y+diff_y;
            if (isInside(poly, n, newCOB))
            {
                return newCOB;
            }
            else
            {
                newCOB.x = x-diff_x;
                newCOB.y = y-diff_y;
                return newCOB;
            }
        }
    }
    else        // new_COB is the centroid. Only applies when the required margin is -1
    {
        float temp_1 = poly[0].x * poly[1].y - poly[1].x * poly[0].y;
        float temp_2 = poly[1].x * poly[2].y - poly[2].x * poly[1].y;
        float temp_3 = poly[2].x * poly[3].y - poly[3].x * poly[2].y;
        float temp_4 = poly[3].x * poly[0].y - poly[0].x * poly[3].y;
        float area = 0.5*(temp_1+temp_2+temp_3+temp_4);
        temp_1 = (poly[0].x+poly[1].x)*(poly[0].x*poly[1].y - poly[1].x*poly[0].y);
        temp_2 = (poly[1].x+poly[2].x)*(poly[1].x*poly[2].y - poly[2].x*poly[1].y);
        temp_3 = (poly[2].x+poly[3].x)*(poly[2].x*poly[3].y - poly[3].x*poly[2].y);
        temp_4 = (poly[3].x+poly[0].x)*(poly[3].x*poly[0].y - poly[0].x*poly[3].y);
        newCOB.x = (temp_1+temp_2+temp_3+temp_4)/(6*area);
        temp_1 = (poly[0].y+poly[1].y)*(poly[0].x*poly[1].y - poly[1].x*poly[0].y);
        temp_2 = (poly[1].y+poly[2].y)*(poly[1].x*poly[2].y - poly[2].x*poly[1].y);
        temp_3 = (poly[2].y+poly[3].y)*(poly[2].x*poly[3].y - poly[3].x*poly[2].y);
        temp_4 = (poly[3].y+poly[0].y)*(poly[3].x*poly[0].y - poly[0].x*poly[3].y);
        newCOB.y = (temp_1+temp_2+temp_3+temp_4)/(6*area);
        return newCOB;
    }
}

**/

bool onSegment(Point p, Point q, Point r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
        return true; 
    return false; 
} 

float orientation(Point p, Point q, Point r)
{ 
    float val = (q.y - p.y) * (r.x - q.x) - 
            (q.x - p.x) * (r.y - q.y); 

    if (val == 0) return 0; // colinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 

bool doIntersect(Point p1, Point q1, Point p2, Point q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    float o1 = orientation(p1, q1, p2); 
    float o2 = orientation(p1, q1, q2); 
    float o3 = orientation(p2, q2, p1); 
    float o4 = orientation(p2, q2, q1); 

    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 

    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 

    return false; // Doesn't fall in any of the above cases 
} 

bool isInside(Point polygon[], int n, Point p) 
{ 
    // There must be at least 3 vertices in polygon[] 
    if (n < 3) return false; 

    // Create a point for line segment from p to infinite 
    Point extreme={INF, p.y}; 

    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0; 
    do
    { 
        int next = (i+1)%n; 

        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0) 
            return onSegment(polygon[i], p, polygon[next]); 

            count++; 
        } 
        i = next; 
    } while (i != 0); 

    // Return true if count is odd, false otherwise 
    return count&1; // Same as (count%2 == 1) 
} 

Point line_intersection(Point p, Point q, Point r, Point s)
{
    if (p.x == q.x)
    {
        float rs_slope = (r.y-s.y)/(r.x-s.x);        // slope of the line rs
        float rs_offset = r.y - rs_slope * r.x;      // offset of the line rs
        Point point = {p.x, rs_slope * point.x + rs_offset};
        return point;
    }

    if (r.x == s.x)
    {
        float pq_slope = (p.y-q.y)/(p.x-q.x);        // slope of the line pq
        float pq_offset = p.y - pq_slope * p.x;      // offset of the line pq
        Point point = {r.x, pq_slope * point.x + pq_offset};
        return point;
    }

    if (p.y == q.y)
    {
        float rs_slope = (r.y-s.y)/(r.x-s.x);        // slope of the line rs
        float rs_offset = r.y - rs_slope * r.x;      // offset of the line rs
        Point point = {(point.y-rs_offset)/rs_slope, p.y};
        return point;
    }
    
    if (r.y == s.y)
    {
        float pq_slope = (p.y-q.y)/(p.x-q.x);             // slope of the line pq
        float pq_offset = p.y - pq_slope * p.x;           // offset of the line pq
        Point point = {(point.y-pq_offset)/pq_slope, r.y};
        return point;
    }
    // line pq and line rs are lines with non-zero and non-infinite slopes
    float pq_slope = (p.y-q.y)/(p.x-q.x);                 // slope of the line pq
    float pq_offset = p.y - pq_slope * p.x;               // offset of the line pq

    float rs_slope = (r.y-s.y)/(r.x-s.x);                 // slope of the line rs
    float rs_offset = r.y - rs_slope * r.x;               // offset of the line rs
    
    Point point = {-(rs_offset-pq_offset)/(rs_slope-pq_slope), pq_slope * point.x + pq_offset};
    return point;
}

/**
Point calc_newCOB(Point p, Point q, Point poly[], int n, float margin)
{
    Point newCOB={0,0};
    if (margin != -1)
    {
        Point mid_point={(p.x+q.x)/2, (p.y+q.y)/2};
        if (fabs(mid_point.x) < 0.0000000001)
        {
            mid_point.x = 0;
        }
        if (fabs(mid_point.y) < 0.0000000001)
        {
            mid_point.y = 0;
        }
        if ((p.y/q.y >= 0.99999) & (p.y/q.y <= 1.00001))
        {
            newCOB.x = mid_point.x;
            newCOB.y = mid_point.y + margin;
            if (isInside(poly, n, newCOB))
            {
                return newCOB;
            }
            else
            {
                newCOB.y = mid_point.y - margin;
                return newCOB;
            }
        }
        else if ((p.x/q.x >= 0.99999) & (p.x/q.x <= 1.00001))
        {
            newCOB.x = mid_point.x + margin;
            newCOB.y = mid_point.y;
            if (isInside(poly, n, newCOB))
            {
                return newCOB;
            }
            else
            {
                newCOB.x = mid_point.x - margin;
                return newCOB;
            }
        }
        else
        {
            float pq_slope = (p.y - q.y) / (p.x - q.x);
            float pq_offset = p.y - pq_slope * p.x;
            if (fabs(pq_offset) < 0.0000000001)
            {
                pq_offset = 0;
            }
            float k = -1/pq_slope;
            float b = mid_point.y - k * mid_point.x;
            float x = (b-pq_offset)/(pq_slope-k);
            float y = k*x+b;
            float diff_x = sqrt(pow(margin,2)/(pow(k,2)+1));
            float diff_y = k*diff_x;
            if (fabs(diff_x) < 0.0000000001)
            {
                diff_x = 0;
            }
            if (fabs(diff_y) < 0.0000000001)
            {
                diff_y = 0;
            }
            newCOB.x = x+diff_x;
            newCOB.y = y+diff_y;
            if (isInside(poly, n, newCOB))
            {
                return newCOB;
            }
            else
            {
                newCOB.x = x-diff_x;
                newCOB.y = y-diff_y;
                return newCOB;
            }
        }
    }
    else        // new_COB is the centroid. Only applies when the required margin is -1
    {
        if (n == 3)
        {
            newCOB.x = (poly[0].x+poly[1].x+poly[2].x)/3;
            newCOB.y = (poly[0].y+poly[1].y+poly[2].y)/3;
            return newCOB;
        }
        else
        {
            float temp_1 = poly[0].x * poly[1].y - poly[1].x * poly[0].y;
            float temp_2 = poly[1].x * poly[2].y - poly[2].x * poly[1].y;
            float temp_3 = poly[2].x * poly[3].y - poly[3].x * poly[2].y;
            float temp_4 = poly[3].x * poly[0].y - poly[0].x * poly[3].y;
            float area = 0.5*(temp_1+temp_2+temp_3+temp_4);
            temp_1 = (poly[0].x+poly[1].x)*(poly[0].x*poly[1].y - poly[1].x*poly[0].y);
            temp_2 = (poly[1].x+poly[2].x)*(poly[1].x*poly[2].y - poly[2].x*poly[1].y);
            temp_3 = (poly[2].x+poly[3].x)*(poly[2].x*poly[3].y - poly[3].x*poly[2].y);
            temp_4 = (poly[3].x+poly[0].x)*(poly[3].x*poly[0].y - poly[0].x*poly[3].y);
            newCOB.x = (temp_1+temp_2+temp_3+temp_4)/(6*area);
            temp_1 = (poly[0].y+poly[1].y)*(poly[0].x*poly[1].y - poly[1].x*poly[0].y);
            temp_2 = (poly[1].y+poly[2].y)*(poly[1].x*poly[2].y - poly[2].x*poly[1].y);
            temp_3 = (poly[2].y+poly[3].y)*(poly[2].x*poly[3].y - poly[3].x*poly[2].y);
            temp_4 = (poly[3].y+poly[0].y)*(poly[3].x*poly[0].y - poly[0].x*poly[3].y);
            newCOB.y = (temp_1+temp_2+temp_3+temp_4)/(6*area);
            return newCOB;
        }
    }
}
**/


Point calc_newCOB(Point p, Point q, Point s, Point t, Point poly[], int n, float margin)
{
    if (margin != -1)
    {
        Point s_1 = {0,0};
        Point t_1 = {0,0};
        Point s_2 = {0,0};
        Point t_2 = {0,0};
        if (s.x == t.x)         // st is perpendicular to x-axis
        {
            s_1.x = s.x - margin;
            s_1.y = s.y;
            t_1.x = t.x - margin;
            t_1.y = t.y;
            s_1.x = s.x + margin;
            s_1.y = s.y;
            t_1.x = t.x + margin;
            t_1.y = t.y;
        }
        else if (s.y == t.y)    // st is perpendicular to y-axis
        {
            s_1.x = s.x;
            s_1.y = s.y - margin;
            t_1.x = t.x;
            t_1.y = t.y - margin;
            s_1.x = s.x;
            s_1.y = s.y + margin;
            t_1.x = t.x;
            t_1.y = t.y + margin;
        }                       // st has a non-zero, non-infinite slope
        else
        {
            float st_slope_perp = -1/((s.y-t.y)/(s.x-t.x));
            float x_delta = sqrt((pow(margin,2))/(pow(st_slope_perp,2)+1));
            float y_delta = x_delta * st_slope_perp;
            s_1.x = s.x - x_delta;
            s_1.y = s.y - y_delta;
            t_1.x = t.x - x_delta;
            t_1.y = t.y - y_delta;

            s_2.x = s.x + x_delta;
            s_2.y = s.y + y_delta;
            t_2.x = t.x + x_delta;
            t_2.y = t.y + y_delta;
        }
        Point point_1 = line_intersection(s_1,t_1,p,q);
        Point point_2 = line_intersection(s_2,t_2,p,q);
        float dist_1 = sqrt(pow(point_1.x-q.x,2) + pow(point_1.y-q.y,2));
        float dist_2 = sqrt(pow(point_2.x-q.x,2) + pow(point_2.y-q.y,2));
        
        if (dist_1 > dist_2)
        {
            return point_2;
        }
        else
        {
            return point_1;
        }
    }
    else
    {
        Point newCOB = {0,0};
        if (n == 3)
        {
            newCOB.x = (poly[0].x+poly[1].x+poly[2].x)/3;
            newCOB.y = (poly[0].y+poly[1].y+poly[2].y)/3;
            return newCOB;
        }
        else
        {
            float temp_1 = poly[0].x * poly[1].y - poly[1].x * poly[0].y;
            float temp_2 = poly[1].x * poly[2].y - poly[2].x * poly[1].y;
            float temp_3 = poly[2].x * poly[3].y - poly[3].x * poly[2].y;
            float temp_4 = poly[3].x * poly[0].y - poly[0].x * poly[3].y;
            float area = 0.5*(temp_1+temp_2+temp_3+temp_4);
            temp_1 = (poly[0].x+poly[1].x)*(poly[0].x*poly[1].y - poly[1].x*poly[0].y);
            temp_2 = (poly[1].x+poly[2].x)*(poly[1].x*poly[2].y - poly[2].x*poly[1].y);
            temp_3 = (poly[2].x+poly[3].x)*(poly[2].x*poly[3].y - poly[3].x*poly[2].y);
            temp_4 = (poly[3].x+poly[0].x)*(poly[3].x*poly[0].y - poly[0].x*poly[3].y);
            newCOB.x = (temp_1+temp_2+temp_3+temp_4)/(6*area);
            temp_1 = (poly[0].y+poly[1].y)*(poly[0].x*poly[1].y - poly[1].x*poly[0].y);
            temp_2 = (poly[1].y+poly[2].y)*(poly[1].x*poly[2].y - poly[2].x*poly[1].y);
            temp_3 = (poly[2].y+poly[3].y)*(poly[2].x*poly[3].y - poly[3].x*poly[2].y);
            temp_4 = (poly[3].y+poly[0].y)*(poly[3].x*poly[0].y - poly[0].x*poly[3].y);
            newCOB.y = (temp_1+temp_2+temp_3+temp_4)/(6*area);
            return newCOB;
        }
    }
    
}
// Point calc_newCOB(Point p, Point q, Point A, Point B, float margin)
// {
// }