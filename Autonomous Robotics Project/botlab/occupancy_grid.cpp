#include "occupancy_grid.hpp"
#include <fstream>
#include <cassert>

#include <iostream>
using namespace std;

OccupancyGrid::OccupancyGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(1.0 / metersPerCell_)
, globalOrigin_(0, 0)
{
}


OccupancyGrid::OccupancyGrid(float widthInMeters,
                             float heightInMeters,
                             float metersPerCell)
: metersPerCell_(metersPerCell)
, globalOrigin_(-widthInMeters/2.0f, -heightInMeters/2.0f)
{
    assert(widthInMeters  > 0.0f);
    assert(heightInMeters > 0.0f);
    assert(metersPerCell_ <= widthInMeters);
    assert(metersPerCell_ <= heightInMeters);
    
    cellsPerMeter_ = 1.0f / metersPerCell_;
    width_         = widthInMeters * cellsPerMeter_;
    height_        = heightInMeters * cellsPerMeter_;
    

    mats_.resize(width_ * height_);
    reset();
}

void OccupancyGrid::setOrigin(float x, float y){

    reset();
    globalOrigin_.x = widthInCells()/2 - x;
    globalOrigin_.y = heightInCells()/2 - y;


    //globalOrigin_.x = widthInCells() / 2;
    //globalOrigin_.y = heightInCells() / 2;
    
    
    printf("global origin: %f, %f\n", globalOrigin_.x, globalOrigin_.y);

   // cout << "set global origin to: " << x << ", " << y << "\n";
}

void OccupancyGrid::reset(void)
{
//    cout << "reset!\n";

    std::fill(mats_.begin(), mats_.end(), 5);
}


bool OccupancyGrid::isCellInGrid(int x, int y) const
{ 
    bool xCoordIsValid = (x >= 0) && (x < width_);
    bool yCoordIsValid = (y >= 0) && (y < height_);
    return xCoordIsValid && yCoordIsValid;
}




void OccupancyGrid::setMats(float x, float y, int mat){

    float sq_side_ft = 1.0; //1 sq ft is the key size
    
    float sq_side_meters = sq_side_ft / 3.28084; // convert feet to meters

    int cell_origin_x = x / metersPerCell_;
    int cell_origin_y = y / metersPerCell_;

    cell_origin_x += globalOrigin_.x;
    cell_origin_y += globalOrigin_.y;  

    //printf("%d, %d\n", cell_origin_x, cell_origin_y);

    int side_len_cells = sq_side_meters / metersPerCell_; //side length of square in cells

    for(int i = cell_origin_x - side_len_cells/2; i < cell_origin_x + side_len_cells/2; ++i){
        for(int j = cell_origin_y - side_len_cells/2; j < cell_origin_y + side_len_cells/2; ++j){
            if(isCellInGrid(i, j)){
                mats_[cellIndex(i, j)] = mat;
            }
        }
    }


    
}



bool OccupancyGrid::saveToFile_mats(const std::string& filename) const
{
    std::ofstream out(filename);
    if(!out.is_open())
    {
        std::cerr << "ERROR: OccupancyGrid::saveToFile_mats: Failed to save to " << filename << '\n';
        return false;
    }
    
    // Write header
    out << globalOrigin_.x << ' ' << globalOrigin_.y << ' ' << width_ << ' ' << height_ << ' ' << metersPerCell_ << '\n';
    
    // Write out each cell value
    for(int y = 0; y < height_; ++y)
    {
        for(int x = 0; x < width_; ++x)
        {
            // Unary plus forces output to be a a number rather than a character
             out << +mats_[cellIndex(x, y)] << ' '; //+mats_[cellIndex(x, y)]
        }
        out << '\n';
    }
    //printf("saved map");
    return out.good();
}

