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
#include "PathDrawer.hpp"
#include "Image.hpp"
#include "PPMLoader.hpp"
#include "Solver.hpp"
#include "Solver_v2.hpp"
#include <ctime>

using namespace rw::sensor;
using namespace rw::loaders;

using namespace std;

int main() {
    
    vector<vector<char>> map_char;
    int map_width = 0, map_height = 0, numOfDimonds = 0;
    ifstream map; 
    //map.open("Bane-copy-2.txt");
    map.open("map_specs(4).txt");
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
    
    Graph start_map(numOfDimonds);
    int costOfDriving = 10;
    char defaultSokobanDirection = 's';
    
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
                string name = "";
                if (i < 10) {
                    name = name + "0" + to_string(i);
                } else {
                    name = name + to_string(i);
                }
                if (j < 10) {
                    name = name + "0" + to_string(j);
                } else {
                    name = name + to_string(j);
                }
                if      (map_char[i][j] == 'J') { dimond = true; }
                else if (map_char[i][j] == 'G') { goal = true; }
                else if (map_char[i][j] == 'M') { sokoban = true; }
                
                // Add vertex:
                if (map_char[i][j] == 'M')
                    start_map.addVertex(name, dimond, goal, sokoban, defaultSokobanDirection);
                else
                    start_map.addVertex(name, dimond, goal, sokoban);
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
                
                string nameCurrent = "";
                if (i < 10) {
                    nameCurrent = nameCurrent + "0" + to_string(i);
                } else {
                    nameCurrent = nameCurrent + to_string(i);
                }
                if (j < 10) {
                    nameCurrent = nameCurrent + "0" + to_string(j);
                } else {
                    nameCurrent = nameCurrent + to_string(j);
                }
                
                // Check for neighbour vertex's:
                // Above (direction ~ North):
                if (map_char[i-1][j] == '.' ||
                    map_char[i-1][j] == 'J' ||
                    map_char[i-1][j] == 'G' ||
                    map_char[i-1][j] == 'M') {
                    
                    string nameTarget = "";
                    if (i < 11) {
                        nameTarget = nameTarget + "0" + to_string(i-1);
                    } else {
                        nameTarget = nameTarget + to_string(i-1);
                    }
                    if (j < 10) {
                        nameTarget = nameTarget + "0" + to_string(j);
                    } else {
                        nameTarget = nameTarget + to_string(j);
                    }
                    start_map.addEdge(nameCurrent, nameTarget, costOfDriving, 'n');
                }
                
                // Right (direction ~ East):
                if (map_char[i][j+1] == '.' ||
                    map_char[i][j+1] == 'J' ||
                    map_char[i][j+1] == 'G' ||
                    map_char[i][j+1] == 'M') {
                    
                    string nameTarget = "";
                    if (i < 10) {
                        nameTarget = nameTarget + "0" + to_string(i);
                    } else {
                        nameTarget = nameTarget + to_string(i);
                    }
                    if (j < 9) {
                        nameTarget = nameTarget + "0" + to_string(j+1);
                    } else {
                        nameTarget = nameTarget + to_string(j+1);
                    }
                    start_map.addEdge(nameCurrent, nameTarget, costOfDriving, 'e');
                }
                
                // Below (direction ~ South):
                if (map_char[i+1][j] == '.' ||
                    map_char[i+1][j] == 'J' ||
                    map_char[i+1][j] == 'G' ||
                    map_char[i+1][j] == 'M') {
                    
                    string nameTarget = "";
                    if (i < 9) {
                        nameTarget = nameTarget + "0" + to_string(i+1);
                    } else {
                        nameTarget = nameTarget + to_string(i+1);
                    }
                    if (j < 10) {
                        nameTarget = nameTarget + "0" + to_string(j);
                    } else {
                        nameTarget = nameTarget + to_string(j);
                    }
                    start_map.addEdge(nameCurrent, nameTarget, costOfDriving, 's');
                }
                
                // Left (direction ~ West):
                if (map_char[i][j-1] == '.' ||
                    map_char[i][j-1] == 'J' ||
                    map_char[i][j-1] == 'G' ||
                    map_char[i][j-1] == 'M') {
                    
                    string nameTarget = "";
                    if (i < 10) {
                        nameTarget = nameTarget + "0" + to_string(i);
                    } else {
                        nameTarget = nameTarget + to_string(i);
                    }
                    if (j < 11) {
                        nameTarget = nameTarget + "0" + to_string(j-1);
                    } else {
                        nameTarget = nameTarget + to_string(j-1);
                    }
                    start_map.addEdge(nameCurrent, nameTarget, costOfDriving, 'w');
                }
            }
        }
    }
    
    start_map.printGraph();
    Graph goal_map = start_map;
    
    for (int i = 0 ; i < goal_map.getNumOfVertex() ; i++) {
        if (goal_map.getVertex(i).getGoal()){
            goal_map.getVertex(i).setDiamond(true);
        }
        if (goal_map.getVertex(i).getDiamond() &&
            !goal_map.getVertex(i).getGoal()) {
            goal_map.getVertex(i).setDiamond(false);
        }
    }
    
    cout << "Road map string: " << start_map.getGraphRepresentation() << endl;
    cout << "Goal map string: " << goal_map.getGraphRepresentation() << endl;
    
    PathDrawer startMap(map_width, map_height, start_map);
    startMap.drawMapAndSave("Images/map_start.ppm");
    
    Solver_v2 solution(start_map, goal_map, numOfDimonds, map_width, map_height);
    
    clock_t begin = clock();
    solution.startSolver();
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    int minutes = elapsed_secs / 60;
    int seconds = (int)elapsed_secs % 60;
    
    std::vector<std::vector<SolverNode_v2>> solutionList = solution.getSolution();
    
    for (int j = 0 ; j < solutionList.size() ; j++) {
        cout << "Solution " << j << "\n";
        for (int i = 0 ; i < solutionList[j].size() ; i++) {
            cout << "ID: " << solutionList[j][i].ID
            << " PrevID: " << solutionList[j][i].prevID
            << " Depth: " << solutionList[j][i].depthInTree
            << " Distance: " << solutionList[j][i].distanceTraveled << endl;
        }
    }
    
    
    cout << "\nFound " << solution.getSolution().size() << " solutions in " << minutes << " minuts, " << seconds << " seconds.\n";
    
    /*
    PathDrawer goalMap(map_width, map_height, goal_map);
    goalMap.drawMapAndSave("goalMap.ppm");
    
    AStar aStarTest(road_map, road_map.getVertex("63"), road_map.getVertex("22"), 'n');
    //AStar aStarTest(road_map, road_map.getVertex("18"), road_map.getVertex("63"), 'n');
    if (aStarTest.runAStar()) {
        std::vector<VertexList> path = aStarTest.getPath();
        
        PathDrawer Test(map_width, map_height, road_map, path);
        Test.drawPathAndSave("Map_test.ppm");
    }
    */
    return 0;
}
