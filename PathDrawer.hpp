//
//  PathDrawer.hpp
//  Sokoban 2
//
//  Created by Olliver Ordell on 06/10/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef PathDrawer_hpp
#define PathDrawer_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include "VertexList.hpp"
#include "Image.hpp"
#include "PPMLoader.hpp"

using namespace rw::sensor;
using namespace rw::loaders;

class PathDrawer {
public:
    PathDrawer();
    ~PathDrawer();
    
    PathDrawer(unsigned int mapWidth, unsigned int mapHeight, Graph& map);
    
    PathDrawer(unsigned int mapWidth, unsigned int mapHeight, Graph& map, std::vector<VertexList>& robotPath);
    
    void drawMapAndSave(std::string fileName);
    void drawPathAndSave(std::string fileName);
    
protected:
    Graph* roadMap;
    std::vector<VertexList>* path;
    unsigned int width, height;
    
private:
    void initImage();
    void initConstants();
    void saveImage(std::string fileName);
    
    void drawMap();
    void drawPath();
    
    void drawWithMapScaler(int x, int y, int blue, int green, int red);
    
    void drawBlackGrid();
    void drawCircle(double x, double y, int radius, int blue, int green, int red);
    void drawArrowsOnAllVertex();
    
    void drawStartAndGoalPoint();
    
private:
    Image* img;
    
private:
    // Colors:
    unsigned int const RED_DIAMOND      = 200;
    unsigned int const BLUE_SOKOBAN     = 200;
    unsigned int const GREEN_GOAL       = 200;
    unsigned int const GRAY_FREESPACE   = 200;
    unsigned int const PATH_ROBOT       = 130;
    unsigned int const BLACK            = 0;
    
    // _MAP_SCALER scales up the image. if the initial width is 10, the scaler makes it 10*_MAP_SCALER. _ROBOT_WIDTH and _OBJECT_BOARDER is defined in initConstants(), and scales down the objects and robot radius according to the _MAP_SCALER.
    unsigned int const _MAP_SCALER  = 60;
    unsigned int _ROBOT_WIDTH;
    unsigned int _OBJECT_BOARDER;
};

#endif /* PathDrawer_hpp */
