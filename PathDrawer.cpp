//
//  PathDrawer.cpp
//  Sokoban 2
//
//  Created by Olliver Ordell on 06/10/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "PathDrawer.hpp"

PathDrawer::PathDrawer(){
    
}

PathDrawer::~PathDrawer(){
    
}

PathDrawer::PathDrawer(unsigned int mapWidth,
                       unsigned int mapHeight,
                       Graph& map):
width(mapWidth),
height(mapHeight),
roadMap(&map){
    initImage();
    initConstants();
}

PathDrawer::PathDrawer(unsigned int mapWidth,
                       unsigned int mapHeight,
                       Graph& map,
                       std::vector<VertexList>& robotPath):
width(mapWidth),
height(mapHeight),
roadMap(&map),path(&robotPath){
    initImage();
    initConstants();
}

PathDrawer::PathDrawer(unsigned int mapWidth,
                       unsigned int mapHeight,
                       Graph& map,
                       std::vector<int> dlZone):
width(mapWidth),
height(mapHeight),
roadMap(&map),
deadlockZone(dlZone){
    initImage();
    initConstants();
}

void PathDrawer::drawMap(bool dlZone){
    // Paint everything black:
    for (int i = 0 ; i < img->getWidth()/_MAP_SCALER ; i++) {
        for (int j = 0 ; j < img->getHeight()/_MAP_SCALER ; j++) {
            drawWithMapScaler(i, j, BLACK, BLACK, BLACK);
        }
    }
    
    // Paint gray 'free-space':
    for (int i = 0 ; i < roadMap->getNumOfVertex() ; i++) {
        // Get position:
        int x = roadMap->getVertex(i).getXPosition();
        int y = roadMap->getVertex(i).getYPosition();
        drawWithMapScaler(x, y, GRAY_FREESPACE, GRAY_FREESPACE, GRAY_FREESPACE);
    }
    if (dlZone) {
        for (int i = 0 ; i < deadlockZone.size() ; i++) {
            int x = roadMap->getVertex(deadlockZone[i]).getXPosition();
            int y = roadMap->getVertex(deadlockZone[i]).getYPosition();
            drawWithMapScaler(x, y, RED_DEADZONE, 0, 0);
        }
    }
    
    // draw black grid for the robot to follow:
    drawBlackGrid();
    
    // Paint objects:
    for (int i = 0 ; i < roadMap->getNumOfVertex() ; i++) {
        // Get position:
        int x = roadMap->getVertex(i).getXPosition();
        int y = roadMap->getVertex(i).getYPosition();
        int RED = 0, GREEN = 0, BLUE = 0;
        
        // Get the right color:
        if (roadMap->getVertex(i).getDiamond()) {
            RED = RED_DIAMOND;
        }
        if (roadMap->getVertex(i).getGoal()) {
            GREEN = GREEN_GOAL;
        }
        if (roadMap->getVertex(i).getSokoban()) {
            BLUE = BLUE_SOKOBAN;
        }
        
        // Paint the point:
        if ((roadMap->getVertex(i).getDiamond()) ||
            (roadMap->getVertex(i).getGoal()) ||
            (roadMap->getVertex(i).getSokoban())) {
            drawCircle(x*_MAP_SCALER + _MAP_SCALER/2, y*_MAP_SCALER + _MAP_SCALER/2, (_MAP_SCALER/2 - _OBJECT_BOARDER), BLUE, GREEN, RED);
        }
    }
}

void PathDrawer::drawPath(){
    drawMap();
    
    for (int i = 0 ; i < path->size()-1 ; i++) {
        int xCurrent = path->at(i).vertexTarget->getXPosition()*_MAP_SCALER + _MAP_SCALER/2;
        int yCurrent = path->at(i).vertexTarget->getYPosition()*_MAP_SCALER + _MAP_SCALER/2;
        
        int xPrev = path->at(i).vertexPrevious->getXPosition()*_MAP_SCALER + _MAP_SCALER/2;
        int yPrev = path->at(i).vertexPrevious->getYPosition()*_MAP_SCALER + _MAP_SCALER/2;
        
        int xDif = (xPrev - xCurrent);
        int yDif = (yPrev - yCurrent);
        
        if (xDif == 0) {
            for (int j = 0 ; j < _MAP_SCALER ; j++) {
                drawCircle(static_cast<double>(xCurrent),
                           static_cast<double>(yCurrent) + abs(yDif)*((static_cast<double>(j))/(yDif)),
                           _ROBOT_WIDTH, PATH_ROBOT, PATH_ROBOT, PATH_ROBOT);
                
            }
        } else if (yDif == 0) {
            for (int j = 0 ; j < _MAP_SCALER ; j++) {
                drawCircle(static_cast<double>(xCurrent) + abs(xDif)*((static_cast<double>(j))/(xDif)),
                           static_cast<double>(yCurrent), _ROBOT_WIDTH, PATH_ROBOT, PATH_ROBOT, PATH_ROBOT);
                
            }
        }
    }
    
    // Draw ONE extra gray circle at the startpoint:
    drawCircle(path->at(path->size()-1).vertexTarget->getXPosition()*_MAP_SCALER + _MAP_SCALER/2,
               path->at(path->size()-1).vertexTarget->getYPosition()*_MAP_SCALER + _MAP_SCALER/2,
               _ROBOT_WIDTH, PATH_ROBOT, PATH_ROBOT, PATH_ROBOT);
    
    drawStartAndGoalPoint();
    drawArrowsOnAllVertex();
}

void PathDrawer::drawMapAndSave(std::string fileName, bool dlZone){
    drawMap(dlZone);
    saveImage(fileName);
}

void PathDrawer::drawPathAndSave(std::string fileName, bool dlZone){
    drawPath();
    saveImage(fileName);
}

void PathDrawer::initImage(){
    img = new Image(width*_MAP_SCALER, height*_MAP_SCALER, Image::BGR, Image::Depth8U);
}

void PathDrawer::initConstants(){
    _ROBOT_WIDTH = _MAP_SCALER/10 + 1;
    _OBJECT_BOARDER = _MAP_SCALER/5;
}

void PathDrawer::saveImage(std::string fileName){
    img->saveAsPPM(fileName);
}

void PathDrawer::drawWithMapScaler(int x, int y, int blue, int green, int red){
    for (int i = x*_MAP_SCALER ; i < (x*_MAP_SCALER + _MAP_SCALER) ; i++){
        for (int j = y*_MAP_SCALER ; j < (y*_MAP_SCALER + _MAP_SCALER) ; j++) {
            img->setPixel8S(i, j, blue, green, red);
        }
    }
}

void PathDrawer::drawBlackGrid(){
    for (int i = _MAP_SCALER/2 ; i < img->getWidth() ; i+=_MAP_SCALER) {
        for (int j = _MAP_SCALER/2 ; j < img->getHeight() ; j+=_MAP_SCALER) {
            
            for (int k = i - _MAP_SCALER/2 ; k < i + _MAP_SCALER/2 ; k++) {
                img->setPixel8S(k, j, BLACK, BLACK, BLACK);
            }
            
            for (int k = j - _MAP_SCALER/2 ; k < j + _MAP_SCALER/2 ; k++) {
                img->setPixel8S(i, k, BLACK, BLACK, BLACK);
            }

        }
    }
}

void PathDrawer::drawCircle(double x, double y, int radius, int blue, int green, int red){
    for (int i = x-radius ; i < x+radius ; i++) {
        for (int j = y-radius ; j < y+radius ; j++) {
            if (radius > sqrt((x-i)*(x-i) + (y-j)*(y-j))) {
                img->setPixel8S(i, j, red, green, blue);
            }
        }
    }
}

void PathDrawer::drawArrowsOnAllVertex(){
    for (int i = 0 ; i < path->size() ; i++) {
        char direction = path->at(i).orientationOfRobot;
        int cutOfArrow = _MAP_SCALER/3.5;
        int lenghtOfArrow = _MAP_SCALER - cutOfArrow;
        
        // First draw a vertical or horizontal line:
        // Vertical:
        if (direction == 'n' || direction == 's') {
            int x = path->at(i).vertexTarget->getXPosition()*_MAP_SCALER + _MAP_SCALER/2;
            int y = path->at(i).vertexTarget->getYPosition()*_MAP_SCALER;
            for (int j = y+cutOfArrow ; j < y+lenghtOfArrow ; j++) {
                img->setPixel8S(x, j, BLACK, BLACK, BLACK);
                //Make the line 3 pixels thick:
                img->setPixel8S(x+1, j, BLACK, BLACK, BLACK);
                img->setPixel8S(x-1, j, BLACK, BLACK, BLACK);
            }
        }
        // Horizontal:
        else if (direction == 'w' || direction == 'e') {
            int x = path->at(i).vertexTarget->getXPosition()*_MAP_SCALER;
            int y = path->at(i).vertexTarget->getYPosition()*_MAP_SCALER + _MAP_SCALER/2;
            for (int j = x+cutOfArrow ; j < x+lenghtOfArrow ; j++) {
                img->setPixel8S(j, y, BLACK, BLACK, BLACK);
                //Make the line 3 pixels thick:
                img->setPixel8S(j, y+1, BLACK, BLACK, BLACK);
                img->setPixel8S(j, y-1, BLACK, BLACK, BLACK);
            }
        }
        //else { std::cerr << "Error: Vertex has no direction." << std::endl; }
        
        // Next, Draw two 45 degree lines:
        int lenghtOfArrowHead = lenghtOfArrow/5;
        if (direction == 'n') {
            int x = path->at(i).vertexTarget->getXPosition()*_MAP_SCALER + _MAP_SCALER/2;
            int y = path->at(i).vertexTarget->getYPosition()*_MAP_SCALER + cutOfArrow;
            for (int j = 0 ; j < lenghtOfArrowHead ; j++) {
                img->setPixel8S(x+j, y+j, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j, y+j, BLACK, BLACK, BLACK);
                //Make the line 3 pixels thick:
                img->setPixel8S(x+j, y+j+1, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j, y+j+1, BLACK, BLACK, BLACK);
                img->setPixel8S(x+j, y+j+2, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j, y+j+2, BLACK, BLACK, BLACK);
            }
        }
        else if (direction == 'e') {
            int x = path->at(i).vertexTarget->getXPosition()*_MAP_SCALER + _MAP_SCALER - cutOfArrow;
            int y = path->at(i).vertexTarget->getYPosition()*_MAP_SCALER + _MAP_SCALER/2;
            for (int j = 0 ; j < lenghtOfArrowHead ; j++) {
                img->setPixel8S(x-j, y+j, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j, y-j, BLACK, BLACK, BLACK);
                //Make the line 3 pixels thick:
                img->setPixel8S(x-j-1, y+j, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j-1, y-j, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j-2, y+j, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j-2, y-j, BLACK, BLACK, BLACK);
            }
        }
        else if (direction == 's') {
            int x = path->at(i).vertexTarget->getXPosition()*_MAP_SCALER + _MAP_SCALER/2;
            int y = path->at(i).vertexTarget->getYPosition()*_MAP_SCALER + _MAP_SCALER - cutOfArrow;
            for (int j = 0 ; j < lenghtOfArrowHead ; j++) {
                img->setPixel8S(x+j, y-j, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j, y-j, BLACK, BLACK, BLACK);
                //Make the line 3 pixels thick:
                img->setPixel8S(x+j, y-j-1, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j, y-j-1, BLACK, BLACK, BLACK);
                img->setPixel8S(x+j, y-j-2, BLACK, BLACK, BLACK);
                img->setPixel8S(x-j, y-j-2, BLACK, BLACK, BLACK);
            }
        }
        else if (direction == 'w') {
            int x = path->at(i).vertexTarget->getXPosition()*_MAP_SCALER + cutOfArrow;
            int y = path->at(i).vertexTarget->getYPosition()*_MAP_SCALER + _MAP_SCALER/2;
            for (int j = 0 ; j < lenghtOfArrowHead ; j++) {
                img->setPixel8S(x+j, y+j, BLACK, BLACK, BLACK);
                img->setPixel8S(x+j, y-j, BLACK, BLACK, BLACK);
                //Make the line 3 pixels thick:
                img->setPixel8S(x+j+1, y+j, BLACK, BLACK, BLACK);
                img->setPixel8S(x+j+1, y-j, BLACK, BLACK, BLACK);
                img->setPixel8S(x+j+2, y+j, BLACK, BLACK, BLACK);
                img->setPixel8S(x+j+2, y-j, BLACK, BLACK, BLACK);
            }
        }
    }
}

void PathDrawer::drawStartAndGoalPoint(){
    //Start point:
    int x = path->at(path->size()-1).vertexTarget->getXPosition()*_MAP_SCALER + _MAP_SCALER/2;
    int y = path->at(path->size()-1).vertexTarget->getYPosition()*_MAP_SCALER + _MAP_SCALER/2;
    
    //Draw cross:
    double startOfCross = sqrt(_ROBOT_WIDTH*_ROBOT_WIDTH/2);
    for (int i = x-startOfCross ; i < x+startOfCross ; i++) {
        img->setPixel8S(i, i+y-x, BLACK, BLACK, BLACK);
        img->setPixel8S(i, y-i+x, BLACK, BLACK, BLACK);
    }
    for (int i = x-_ROBOT_WIDTH ; i < x+_ROBOT_WIDTH ; i++) {
        img->setPixel8S(i, y, BLACK, BLACK, BLACK);
    }
    for (int i = y-_ROBOT_WIDTH ; i < y+_ROBOT_WIDTH ; i++) {
        img->setPixel8S(x, i, BLACK, BLACK, BLACK);
    }
    
    //Draw circle:
    for (int i = x-_ROBOT_WIDTH ; i < x+_ROBOT_WIDTH ; i++) {
        for (int j = y-_ROBOT_WIDTH ; j < y+_ROBOT_WIDTH ; j++) {
            if (_ROBOT_WIDTH == (int)sqrt((x-i)*(x-i) + (y-j)*(y-j))) {
                img->setPixel8S(i, j, BLACK, BLACK, BLACK);
            }
        }
    }
    
    //Goal point:
    x = path->at(0).vertexTarget->getXPosition()*_MAP_SCALER + _MAP_SCALER/2;
    y = path->at(0).vertexTarget->getYPosition()*_MAP_SCALER + _MAP_SCALER/2;
    
    //Draw circle (filled):
    for (int i = x-_ROBOT_WIDTH ; i < x+_ROBOT_WIDTH ; i++) {
        for (int j = y-_ROBOT_WIDTH ; j < y+_ROBOT_WIDTH ; j++) {
            if (_ROBOT_WIDTH > (int)sqrt((x-i)*(x-i) + (y-j)*(y-j))) {
                img->setPixel8S(i, j, BLACK, BLACK, BLACK);
            }
        }
    }
}

