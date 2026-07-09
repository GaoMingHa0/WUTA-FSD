#include "pathplanning/structs/Line.h"
Line::Line() {
    p1.x = 0.0; p1.y = 0.0;
    p2.x = 0.0; p2.y = 0.0;
}
Line::Line(Point2d a, Point2d b) {
    p1 = a; p2 = b;
}
bool Line::operator==(const Line& other){
    return (this->p1 == other.p1 && this->p2 == other.p2)
        || (this->p1 == other.p2 && this->p2 == other.p1);
}