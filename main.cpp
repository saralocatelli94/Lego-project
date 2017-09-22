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

using namespace std;

void debugPrint(vector<vector<char>>& map_char, int map_height, int map_width, int n, int m){
    cout << "- 0 1 2 3 4 5 6 7 8 9" << endl;
    for (int i = 0 ; i < map_height ; i++){
        cout << i << " ";
        for (int j = 0 ; j < map_width ; j++) {
            if (n == i && m == j)
                cout << "O ";
            else
                cout << map_char[i][j] << " ";
        }
        cout << endl;
    }
}

int main() {
    
    vector<vector<char>> map_char;
    int map_width, map_height, numOfDimonds;
    ifstream map;
    map.open("map_specs.txt");
    if (!map.is_open()) {
        cerr << "ERROR: Could not open map description file." << endl;
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
    int costOfDriving = 10;
    int costOfTurning = 3;
    
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
    
    return 0;
}
