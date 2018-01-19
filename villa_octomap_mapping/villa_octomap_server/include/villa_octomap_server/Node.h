
#ifndef NODE_H
#define NODE_H

class Node {
    private:
        unsigned depth;
        double x;
        double y;
        double z;

    public:
        Node(unsigned depth, double x, double y, double z);

        unsigned getDepth();
        double getX();
        double getY();
        double getZ();
};

#endif
