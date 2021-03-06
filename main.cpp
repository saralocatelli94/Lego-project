//
//  main.cpp
//  Sokoban Solver
//
//  Created by Olliver Ordell on 21/09/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "Graph.hpp"
#include "AStar.hpp"

using namespace std;

int main() {
    
    vector<vector<char>> map_char;
    int map_width, map_height, numOfDimonds;
    ifstream map;
    map.open("map_specs.txt");
    if (!map.is_open()) {
        cerr << "ERROR: Could not open map description file." << endl;
        return 0;
    }
    else {
        int line_count = 0;
        for(string line; getline(map, line);)
        {
            if (line_count == 0){
                istringstream iss(line);
                if (!(iss >> map_width >> map_height >> numOfDimonds)) {
                    cerr << "ERROR: Top line in map description has wrong formate." << endl;
                    break;
                }
                vector<vector<char>> temp_vector(map_height);
                map_char = temp_vector;
            }
            else {
                for (int i = 0 ; i < line.length() ; i++){
                    map_char[line_count-1].push_back(line.at(i));
                }
            }
            line_count++;
        }
    }
    map.close();
    
    Graph road_map;
    int costOfDriving = 2;
    int costOfTurning = 3;
    char defaultSokobanDirection = 'n';
    
    /*
     (a) X - wall.
     (b) J - diamond.
     (c) G - goal.
     (d) . - walkable area.
     (e) M - sokoban man / the robot.
     (f)   - area outside the map
     */
    
    // Add all vertex's first:
    for (int i = 1 ; i < map_height-1 ; i++) {
        bool dimond = false, goal = false, sokoban = false;
        for (int j = 1 ; j < map_char[i].size()-1 ; j++) {
            
            // Add new vertex:
            // If vertex is accessible for Sokoban:
            if (map_char[i][j] == '.' ||
                map_char[i][j] == 'J' ||
                map_char[i][j] == 'G' ||
                map_char[i][j] == 'M') {
                
                // Set meta data:
                string name = to_string(i) + to_string(j);
                if      (map_char[i][j] == 'J') { dimond = true; }
                else if (map_char[i][j] == 'G') { goal = true; }
                else if (map_char[i][j] == 'M') { sokoban = true; }
                
                // Add vertex:
                if (map_char[i][j] == 'M')
                    road_map.addVertex(name, dimond, goal, sokoban, defaultSokobanDirection);
                else
                    road_map.addVertex(name, dimond, goal, sokoban);
                dimond = false, goal = false, sokoban = false;
            }
        }
    }
    
    // Now add all edges for all vertex's:
    for (int i = 1 ; i < map_height-1 ; i++) {
        for (int j = 1 ; j < map_char[i].size()-1 ; j++){
            
            // If vertex is accessible for Sokoban:
            if (map_char[i][j] == '.' ||
                map_char[i][j] == 'J' ||
                map_char[i][j] == 'G' ||
                map_char[i][j] == 'M') {
                
                string nameCurrent = to_string(i) + to_string(j);
                
                // Check for neighbour vertex's:
                // Above (direction ~ North):
                if (map_char[i-1][j] == '.' ||
                    map_char[i-1][j] == 'J' ||
                    map_char[i-1][j] == 'G' ||
                    map_char[i-1][j] == 'M') {
                    
                    string nameTarget = to_string(i-1) + to_string(j);
                    road_map.addEdge(nameCurrent, nameTarget, costOfDriving, 'n');
                }
                
                // Right (direction ~ East):
                if (map_char[i][j+1] == '.' ||
                    map_char[i][j+1] == 'J' ||
                    map_char[i][j+1] == 'G' ||
                    map_char[i][j+1] == 'M') {
                    
                    string nameTarget = to_string(i) + to_string(j+1);
                    road_map.addEdge(nameCurrent, nameTarget, costOfDriving, 'e');
                }
                
                // Below (direction ~ South):
                if (map_char[i+1][j] == '.' ||
                    map_char[i+1][j] == 'J' ||
                    map_char[i+1][j] == 'G' ||
                    map_char[i+1][j] == 'M') {
                    
                    string nameTarget = to_string(i+1) + to_string(j);
                    road_map.addEdge(nameCurrent, nameTarget, costOfDriving, 's');
                }
                
                // Left (direction ~ West):
                if (map_char[i][j-1] == '.' ||
                    map_char[i][j-1] == 'J' ||
                    map_char[i][j-1] == 'G' ||
                    map_char[i][j-1] == 'M') {
                    
                    string nameTarget = to_string(i) + to_string(j-1);
                    road_map.addEdge(nameCurrent, nameTarget, costOfDriving, 'w');
                }
            }
        }
    }

    road_map.printGraph();


    
    AStar aStarTest(road_map, road_map.getVertex("63"), road_map.getVertex("18"), 'n');
    aStarTest.runAStar();
    
    
    return 0;
}
