#include "path_finder_main.hpp"
#include "utilities.hpp"

using namespace Eigen;
using namespace std;
namespace fs = std::filesystem;
using namespace std::chrono;

std::vector<std::vector<int>> readGridFromFile(const string& filename);

Astar_Planner::Astar_Planner() {
    grid_ = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0}
    };
    
    start_ = {0, 0};
    goal_ = {4, 4};
    rows_ = grid_.size();
    cols_ = grid_[0].size();

    // Data path
    std::string package_path; 
    try {
      package_path = ros::package::getPath("Digit_Ros");
      if (package_path.empty()) {
        throw 1;
      }
    } catch(...) {
      std::cerr << "package not found\n";
    }

    path_file_name_ = package_path + "/Data_Process/path.txt";
    grid_file_name_ = package_path + "/Data_Process/grid.txt";
};

void Astar_Planner::savePathToFile(){
  ofstream file(path_file_name_);
  for (const auto& node : path_) {
    file << node.first << " " << node.second << "\n";
  }
  file.close();

  ofstream grid_file(grid_file_name_);
  grid_file << grid_.size() << " " << grid_[0].size() << "\n";

  // Save grid (0 for free space, 1 for obstacles)
  for (const auto& row : grid_) {
      for (int cell : row) {
          grid_file << cell << " ";
      }
      grid_file << "\n";
  }

  grid_file.close();
}

void Astar_Planner::Plan_Path(){
  // Dimension Check
  assert(rows_ == grid_.size() && "row number does not match map");
  assert(cols_ == grid_[0].size() && "column number does not match map");

  // Define Data Structure
  //priority_queue<Node, vector<Node>, greater<Node>> openSet; // Min-heap (priority queue)
  //unordered_set<Node, NodeHash> openSetTracker;              // Stores all nodes in openSet
  unordered_set<Node, NodeHash> closedSet;                   // Stores visited nodes
  unordered_map<Node, Node, NodeHash> cameFrom;              // Stores path (backtracking)
  unordered_map<Node, double, NodeHash> gCost;               // Stores g value
  set<pair<double, Node>> openSet;                           // Uses fCost as key

  // Initialize the current Node
  // Node is structured as x,y,g,h,f
  Node StartNode = {start_.first, start_.first, 0, heuristic(start_.first, start_.second, goal_.first, goal_.second)};
  StartNode.f = StartNode.g + StartNode.h;
  openSet.insert({StartNode.f, StartNode});
  gCost[StartNode] = 0;

  // Run until no feasible path or optimal path
  while (!openSet.empty()){
    // Get current node
    Node current = openSet.begin()->second; // Get node with lowest fCost
    openSet.erase(openSet.begin()); // Remove from open set

    if (current.x == goal_.first && current.y == goal_.second){
      // Optimal Path found, reconstruct path
      std::vector<std::pair<int, int>> new_path;

      while (cameFrom.find(current) != cameFrom.end()) {
          new_path.push_back({current.x, current.y});
          current = cameFrom[current];
      }
      new_path.push_back({StartNode.x,StartNode.y});
      reverse(new_path.begin(), new_path.end());


      std::cout << "Success Found a Path!" << std::endl;
      std::cout << new_path << std::endl;
      path_ = new_path;
      this->savePathToFile();
    }
    else{
      // Current node is visited
      closedSet.insert(current);
      for (const auto& dir : DIRECTIONS_8_) {
        int nx = current.x + dir.first;
        int ny = current.y + dir.second;

        // Check bounds or collisions
        if (nx < 0 || ny < 0 || nx >= rows_ || ny >= cols_ || grid_[nx][ny] == 1) 
          continue;

        Node next_Node = {nx, ny};

        // Skip visited Nodes
        if (closedSet.find(next_Node) != closedSet.end()) continue;

        // Transition cost
        double temp_g =  gCost[current] + 1;
        // Checking Nodes
        if (gCost.find(next_Node) == gCost.end()){
          // For new nodes, add it to the list
          next_Node.g = temp_g;
          next_Node.h = heuristic(nx, ny, goal_.first, goal_.second);
          next_Node.f = next_Node.g + next_Node.h;

          gCost[next_Node] = next_Node.g;
          cameFrom[next_Node] = current;
          openSet.insert({next_Node.f, next_Node});
        }
        else{
          if(temp_g < gCost[next_Node]){
            // For existing nodes with higher cost, 
            next_Node.g = temp_g;
            next_Node.h = heuristic(nx, ny, goal_.first, goal_.second);
            next_Node.f = next_Node.g + next_Node.h;

            gCost[next_Node] = next_Node.g;
            cameFrom[next_Node] = current;
            auto it = openSet.find({next_Node.f, next_Node});
            if (it != openSet.end()) 
              openSet.erase(it);
            openSet.insert({next_Node.f, next_Node});

          }
        }
      }
    }

  }

};

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;
/*   bool run_sim = true;
  n.getParam("sim_mode",run_sim);
  Digit_MPC digit_mpc(run_sim); */
  ros::Rate loop_rate(1);

  Astar_Planner path_finder = Astar_Planner();

  std::string package_path; 
  try {
    package_path = ros::package::getPath("Digit_Ros");
    if (package_path.empty()) {
      throw 1;
    }
  } catch(...) {
    std::cerr << "package not found\n";
  }
  string map_file_name_ = package_path + "/Data_Process/map.txt";
  std::vector<std::vector<int>> grid = readGridFromFile(map_file_name_);
  path_finder.Update_Map(grid);
  path_finder.Update_Goal({40,60});

  auto pathfinder_time_start = std::chrono::system_clock::now();
  path_finder.Plan_Path();
  auto pathfinder_time_end = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - pathfinder_time_start);
  cout << "solving time: " << pathfinder_time_end.count() << endl;

  while (ros::ok()){
    auto mpc_time_start = std::chrono::system_clock::now();
    std::cout << "running" << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  //ros::Subscriber sub = n.subscribe("/digit_state", 100, chatterCallback);
  ros::spin();
  return 0;
  //ros::spin();
}


std::vector<std::vector<int>> readGridFromFile(const string& filename) {
    std::vector<std::vector<int>> grid;
    ifstream file(filename);

    if (!file) {
        cerr << "Error: Could not open file!" << endl;
        return grid;
    }

    string line;
    while (getline(file, line)) {
        vector<int> row;
        stringstream ss(line);
        int value;

        while (ss >> value) {
            row.push_back(value);
        }

        grid.push_back(row);
    }

    file.close();
    return grid;
}