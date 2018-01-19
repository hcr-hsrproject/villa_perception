#ifndef BOUNDING_BOX_PERSON_DESC_H
#define BOUNDING_BOX_PERSON_DESC_H

class TrackingBox_Person_Desc
{
public:
  float tl_x;
  float tl_y;
  float width;
  float height;  
  TrackingBox_Person_Desc(float _tl_x, float _tl_y, 
  						  float _width, float _height); // Constructor
  ~TrackingBox_Person_Desc(); // Destructor
};

#endif
