




#include <common/point.hpp>
#include <vector>
  ///< Type used to represent the data in a cell

/**
* OccupancyGrid represents a simple two-dimensional occupancy grid, which is a discretized representation of the
* environment. The basic abstraction of the occupancy grid is the cell. Each cell corresponds to a small square
* in the world. A cell represents the log-odds that some part of the environment within its boundaries contains
* an object.
* 
* The log-odds for a cell are represented using the typedef CellOdds. A value greater than 0 means more likely occupied.
* A value less than 0 means more likely free. 0 means even odds.
* 
* Most robot functionality doesn't operate using discrete representations. They use continuous values, like the length
* of a laser ray, the pose of a robot, etc. Functions to convert coordinates in the global, continuous coordinate system
* into the discretized coordinate system of the occupancy grid are found in occupancy_grid_utils.hpp.
* 
* This simple OccupancyGrid implementation does not support resizing. You can complete A1 without needing to resize
* the grid. Just make it big enough to start with.
* 
* You can change the typedef to use a different underlying value. The default int8_t provided plenty of resolution
* for successful mapping, though. Furthermore, increasing to a larger value type will, at minimum, quadruple the
* amount of memory used by OccupancyGrid. Also, if you change the value, you'll need to update occupancy_grid_t
* accordingly.
* 
* The grid is represented in row-major order, that is, a 3x3 grid is in memory as:
* 
*     0     1     2     3     4     5     6     7     8         (memory index)
*   (0,0) (1,0) (2,0) (0,1) (1,1) (2,1) (0,2) (1,2) (2,2)       (cell coordinate)
* 
* Therefore, if iterating through all the cells in a grid using nested for-loops -- the standard approach -- you should
* put the y-loop on the outside:
* 
*       for(std::size_t y = 0; y < grid.heightInCells(); ++y)
*       {
*           for(std::size_t x = 0; x < grid.widthInCells(); ++x)
*           {
*               // Operations on grid(x, y)
*           }
*       }
*
* This loop will be anywhere between nominally and dramatically faster than the alternative.
*/
class OccupancyGrid
{
public:
    
    /**
    * Default constructor for OccupancyGrid.
    * 
    * Create an OccupancyGrid with a width and height of 0. metersPerCell is set to 0.05. The global origin is (0,0).
    * 
    * This constructor is intended for use when creating a grid from an LCM message.
    */

   OccupancyGrid(void);

   OccupancyGrid(float widthInMeters,
                             float heightInMeters,
                             float metersPerCell);
    
    
    
    
    
    // Accessors for the properties of the grid
    int   widthInCells (void) const { return width_; }
    //float widthInMeters(void) const { return width_ * metersPerCell_; }
    
    int   heightInCells (void) const { return height_; }
    //float heightInMeters(void) const { return height_ * metersPerCell_; }
    
    //float metersPerCell(void) const { return metersPerCell_; }
    float cellsPerMeter(void) const { return cellsPerMeter_; }
    
    Point<float> originInGlobalFrame(void) const { return globalOrigin_; }
    
    void setOrigin(float x, float y);

   
    void reset(void);
    

    bool isCellInGrid(int x, int y) const;

    bool saveToFile_mats(const std::string& filename) const;

    std::vector<int> mats_;
    void setMats(float x, float y, int mat);
    int cellIndex(int x, int y) const { return y*width_ + x; }
    int width_;                 ///< Width of the grid in cells
    int height_;                ///< Height of the grid in cells
    float metersPerCell_;
    float cellsPerMeter_;
    
    Point<float> globalOrigin_; 
    
private:
 
            ///< Origin of the grid in global coordinates
    
    // Convert between cells and the underlying vector index
    
};


