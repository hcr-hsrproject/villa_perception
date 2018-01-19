
#include <villa_octomap_server/Node.h>

Node::Node(unsigned depth, double x, double y, double z) {
    this->depth = depth;
    this->x = x;
    this->y = y;
    this->z = z;
}

unsigned Node::getDepth() {
    return depth;
}

double Node::getX() {
    return x;
}

double Node::getY() {
    return y;
}

double Node::getZ() {
    return z;
}

