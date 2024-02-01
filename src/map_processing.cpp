#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <bits/stdc++.h>
#include <fstream>
#include <iostream>

using namespace std;

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int> > pPair;

// A structure to hold the necessary parameters
struct cell {
 // Row and Column index of its parent
 // Note that 0 <= i <= height-1 & 0 <= j <= width-1
 int parent_i, parent_j;
 // f = g + h
 double f, g, h;
};

class MapProcessing {
public:
  MapProcessing() :  tfListener(tfBuffer){
    ros::NodeHandle nh_;
    
    // Attente du service
    ROS_INFO("Waiting for service");
    // Note: 2 services : static_map & dynamic_map
    ros::service::waitForService("/static_map");
    ROS_INFO("Service available");
    
    // Création d'un objet pour appeler le service
    client = nh_.serviceClient<nav_msgs::GetMap>("/static_map");

    map_pointer_sub_=nh_.subscribe<geometry_msgs::PointStamped>("clicked_point", 10, &MapProcessing::point_cb, this);

    path_pub_=nh_.advertise<nav_msgs::Path>("path_map", 10);


    RecupMap();
    //TODO Dans certains cas, l'algo de recherche de chemin ne trouve pas la destination. à voir pourquoi

    // AfficheMap();

  }

private:
  //*********************************** DÉCLARATIONS ***********************************//
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  ros::ServiceClient client;
  nav_msgs::MapMetaData metadata_msg;
  ros::Publisher path_pub_;
  ros::Subscriber map_pointer_sub_;


  int width = 0;
  int height = 0;
  vector<std::vector<int>> grid;
  vector<std::vector<int>> AstarGrid;


  cv::Point RobotPose;
  cv::Point Destination;
  
  stack<Pair> Path;
  cv::Mat image;

  //*********************************** RECUPERATION DE LA CARTE ET CALLBACKS ***********************************//
  void RecupMap(){
    // Création de la requête
    nav_msgs::GetMap srv;

    // Appel du service
    if (client.call(srv))
    {
      
      // La carte est disponible dans srv.response.map
      metadata_msg = srv.response.map.info;
      width = metadata_msg.width;
      height = metadata_msg.height;
      ROS_INFO("width: %d, height: %d", width, height);

      // Initialize 2D grid
      grid.resize(height, std::vector<int>(width, 0));
      AstarGrid.resize(height, std::vector<int>(width, 0));

      // Remplir la grille avec les données de la carte
      for (int row = 0; row < height; ++row) {
        for (int col = 1; col <= width; ++col) {
          int index = col + row * width;
          grid[row][width-col] = srv.response.map.data[index];
        }
      }

      applyOpening(grid, AstarGrid, 5);
    }
    else
    {
      ROS_ERROR("Echec de l'appel du service");
    }
  }

  void point_cb(const geometry_msgs::PointStamped::ConstPtr& pointmsg){
    geometry_msgs::PoseStamped Pose;
    Pose.pose.position.x = pointmsg->point.x;
    Pose.pose.position.y = pointmsg->point.y;
    ConvertPose(&Pose, false);
    Destination.x = Pose.pose.position.x;
    Destination.y = Pose.pose.position.y;

    GetRobotPose(&Pose);
    ConvertPose(&Pose, false);
    RobotPose.x = Pose.pose.position.x;
    RobotPose.y = Pose.pose.position.y;

    Pair src = make_pair(RobotPose.y, RobotPose.x);
    Pair dest = make_pair(Destination.y, Destination.x);

    AstarSearch(src, dest);
    PublishPath();

    return;
  }

  void GetRobotPose(geometry_msgs::PoseStamped *Pose){
    geometry_msgs::TransformStamped transform;
    
    try {
      transform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }

    catch (tf2::TransformException &ex){
      ROS_ERROR("%s",ex.what());
    }

    Pose->pose.position.x = transform.transform.translation.x;
    Pose->pose.position.y = transform.transform.translation.y;
    //ROS_INFO("Transform : %f , %f", Pose->pose.position.x, Pose->pose.position.y);
    return;
  }

  void PublishPath() {
    geometry_msgs::PoseStamped Pose;
    nav_msgs::Path pathmsg;
    pathmsg.header.frame_id = "map";

    int del = 0;
    while (!Path.empty()) {
      pair<int, int> p = Path.top();
      Path.pop();
      Pose.pose.position.x = p.first;
      Pose.pose.position.y = p.second;
      Pose.header.frame_id = "map";
      ConvertPose(&Pose, true);
      if ((del%5==0) || (Path.size()==1)) { //TODO: faire la sélection des waypoints dans Map Processing plutôt que ici
        pathmsg.poses.push_back(Pose);
      }
      del++;
    }
    path_pub_.publish(pathmsg);
    return;
  }

  //*********************************** AFFICHAGE DE LA CARTE (Not used anymore) ***********************************//

  static void mouse_cb(int event, int x, int y, int flags, void* param){
    if(event == CV_EVENT_LBUTTONDOWN){
      cv::Point* p = (cv::Point*)param;
      p->x = x;
      p->y = y;
      return;
    }
  }

  void AfficheMap(){
    // Afficher la grille avec OpenCV
    cv::Mat image(grid.size(), grid[0].size(), CV_8UC1);
    cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);

    int pixel = 0;

    while(cv::getWindowProperty("Map", cv::WND_PROP_AUTOSIZE) >= 0){
      geometry_msgs::PoseStamped Pose;
      nav_msgs::Path pathmsg;
      pathmsg.header.frame_id = "map";

      for (int row = 0; row < image.rows; ++row) {
        for (int col = 0; col < image.cols; ++col) {
          pixel = grid[row][col];
          if (pixel  == 0){
            image.at<uchar>(row, col) = 255;
          }
          if (pixel == 1){
            image.at<uchar>(row, col) = 0;
          }
          if (pixel == -1){
            image.at<uchar>(row, col) = 127;
          }
        }
      }
      

      cv::imshow("Map", image);

      cv::setMouseCallback("Map", mouse_cb, &RobotPose);
      ROS_INFO("Select the source point and press 'enter'");
      cv::waitKey(0);
      ROS_INFO("Source acquired");
      std::cout << RobotPose.x  << " " << RobotPose.y << std::endl;
      image.at<uchar>(RobotPose.y, RobotPose.x) = 0;
      cv::imshow("Map", image);

      cv::setMouseCallback("Map", mouse_cb, &Destination);
      ROS_INFO("Select the destination point and press 'enter'");
      cv::waitKey(0);
      ROS_INFO("Destination acquired");
      std::cout << Destination.x  << " " << Destination.y << std::endl;
      cv::imshow("Map", image);

      image.at<uchar>(Destination.y, Destination.x) = 0;
      
      Pair src = make_pair(RobotPose.y, RobotPose.x);
      Pair dest = make_pair(Destination.y, Destination.x);

      AstarSearch(src, dest);

      while (!Path.empty()) {
        // ROS_INFO("Displaying path");
        pair<int, int> p = Path.top();
        Path.pop();
        Pose.pose.position.x = p.first;
        Pose.pose.position.y = p.second;
        Pose.header.frame_id = "map";
        ConvertPose(&Pose, true);
        pathmsg.poses.push_back(Pose);
        image.at<uchar>(p.first, p.second) = 0;
        //ROS_INFO("Path displayed");
      }
      path_pub_.publish(pathmsg);

      cv::imshow("Map", image);
      ROS_INFO("Press 'enter'");
      cv::waitKey(0);
    }

    cv::destroyWindow("Map");
  }

  //*********************************** MAP PROCESSING ***********************************//

  void ConvertPose(geometry_msgs::PoseStamped *Pose, bool from_pixels_to_meters){

    /* Pose->pose.position.x = ((Pose->pose.position.x) * metadata_msg.resolution);
    Pose->pose.position.y = ((Pose->pose.position.y) * metadata_msg.resolution); */
    /* ROS_INFO("Origine - X: %f, Y: %f", metadata_msg.origin.position.x, metadata_msg.origin.position.y);
    ROS_INFO("Taille map - Width: %f, height: %f", width* metadata_msg.resolution, height* metadata_msg.resolution);
    ROS_INFO("Pose - X: %f, Y: %f", Pose->pose.position.x, Pose->pose.position.y); */
    if (from_pixels_to_meters){
      Pose->pose.position.z = Pose->pose.position.x;
      Pose->pose.position.x = Pose->pose.position.y;
      Pose->pose.position.y = Pose->pose.position.z;
      Pose->pose.position.z = 0;

      Pose->pose.position.x = ((width - Pose->pose.position.x) * metadata_msg.resolution) + metadata_msg.origin.position.x;
      Pose->pose.position.y = ((Pose->pose.position.y) * metadata_msg.resolution) + metadata_msg.origin.position.y;
    }
    else {
      Pose->pose.position.x = std::ceil(width - ((Pose->pose.position.x - metadata_msg.origin.position.x) / metadata_msg.resolution));
      Pose->pose.position.y = std::ceil((Pose->pose.position.y - metadata_msg.origin.position.y) / metadata_msg.resolution);
    }
    // ROS_INFO("Pose convertie - X: %f, Y: %f", Pose->pose.position.x, Pose->pose.position.y);
    
  }

  // Function to apply opening operation with a given radius
  void applyOpening(const std::vector<std::vector<int>>& inputGrid, std::vector<std::vector<int>>& outputGrid, int radius) {
    // Create a structuring element for morphological operations
    int size = 2 * radius + 1;
    std::vector<std::vector<int>> structuringElement(size, std::vector<int>(size, 1));

    // Erosion operation
    for (int row = 0; row < outputGrid.size(); ++row) {
      for (int col = 0; col < outputGrid[0].size(); ++col) {
        bool applyErosion = true;

        for (int i = 0; i < size; ++i) {
          for (int j = 0; j < size; ++j) {
            int r = row - radius + i;
            int c = col - radius + j;

            // Check if the pixel is outside the input grid bounds
            if (r < 0 || r >= inputGrid.size() || c < 0 || c >= inputGrid[0].size()) {
              continue;
            }

            // Check if the pixel in the structuring element is 1 and the corresponding pixel in the input grid is 0
            if (structuringElement[i][j] == 1 && inputGrid[r][c] == 0) {
              applyErosion = false;
              break;
            }
          }

          if (!applyErosion) {
            break;
          }
        }

        // Set the value in the output grid based on erosion result
        outputGrid[row][col] = applyErosion ? 1 : 0;
      }
    }

    // Dilatation operation on the result of erosion
    for (int row = 0; row < outputGrid.size(); ++row) {
      for (int col = 0; col < outputGrid[0].size(); ++col) {
        for (int i = -radius; i <= radius; ++i) {
          for (int j = -radius; j <= radius; ++j) {
            int r = row + i;
            int c = col + j;

            // Check if the pixel is outside the output grid bounds
            if (r < 0 || r >= outputGrid.size() || c < 0 || c >= outputGrid[0].size()) {
              continue;
            }
            // Set the value in the output grid based on dilation result
            outputGrid[row][col] = std::max(outputGrid[row][col], inputGrid[r][c]);
          }
        }
      }
    }
  }


  //*********************************** ALGO A STAR ***********************************//

  // A Utility Function to check whether given cell (row, col)
  // is a valid cell or not.
  bool isValid(int row, int col){
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < height) && (col >= 0) && (col < width);
  }

  // A Utility Function to check whether the given cell is
  // blocked or not
  bool isUnBlocked(int row, int col){
    // Returns true if the cell is not blocked else false
    if (AstarGrid[row][col] == 0)
      return (true);
    else
      return (false);
  }

  // A Utility Function to check whether destination cell has
  // been reached or not
  bool isDestination(int row, int col, Pair dest){
    if (row == dest.first && col == dest.second)
      return (true);
    else
      return (false);
  }

  // A Utility Function to calculate the 'h' heuristics.
  double calculateHValue(int row, int col, Pair dest) {
    // Return using the distance formula
    return ((double)sqrt((row - dest.first) * (row - dest.first) + (col - dest.second) * (col - dest.second)));
  }


  // Note: Nous avons adapté cet algorithme trouvé sur le site de GeeksForGeeks par Rachit Belwariar: https://www.geeksforgeeks.org/a-search-algorithm/

  // A Function to find the shortest path between
  // a given source cell to a destination cell according
  // to A* Search Algorithm
  void AstarSearch(Pair src, Pair dest) {
    // If the source is out of range
    if (isValid(src.first, src.second) == false) {
      Path.push(src);
      ROS_INFO("Source is invalid\n");
      return;
    }

    // If the destination is out of range
    if (isValid(dest.first, dest.second) == false) {
      ROS_INFO("Destination is invalid\n");
      Path.push(dest);
      return;
    }

    // Either the source or the destination is blocked
    if (isUnBlocked(src.first, src.second) == false || isUnBlocked(dest.first, dest.second) == false) {
      ROS_INFO("Source or the destination is blocked or too close to a wall\n");
      return;
    }

    // If the destination cell is the same as source cell
    if (isDestination(src.first, src.second, dest) == true) {
      ROS_INFO("We are already at the destination\n");
      return;
    }

    // Create a closed list and initialise it to false which
    // means that no cell has been included yet This closed
    // list is implemented as a boolean 2D array
    bool closedList[width][height];
    memset(closedList, false, sizeof(closedList));

    // Declare a 2D array of structure to hold the details
    // of that cell
    cell cellDetails[width][height];

    int i, j;

    for (i = 0; i < width; i++) {
      for (j = 0; j < height; j++) {
        cellDetails[i][j].f = FLT_MAX;
        cellDetails[i][j].g = FLT_MAX;
        cellDetails[i][j].h = FLT_MAX;
        cellDetails[i][j].parent_i = -1;
        cellDetails[i][j].parent_j = -1;
      }
    }

    // Initialising the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    /*
    Create an open list having information as-
    <f, <i, j>>
    where f = g + h,
    and i, j are the row and column index of that cell
    Note that 0 <= i <= width-1 & 0 <= j <= height-1
    This open list is implemented as a set of pair of
    pair.*/
    set<pPair> openList;

    // Put the starting cell on the open list and set its
    // 'f' as 0
    openList.insert(make_pair(0.0, make_pair(i, j)));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!openList.empty()) {
      pPair p = *openList.begin();

      // Remove this vertex from the open list
      openList.erase(openList.begin());

      // Add this vertex to the closed list
      i = p.second.first;
      j = p.second.second;
      closedList[i][j] = true;

      /*
      Generating all the 8 successor of this cell

      N.W   N     N.E
      \     |     /
        \   |   /
      W----Cell----E
        /   |   \
      /     |     \
      S.W   S     S.E

      Cell-->Popped Cell (i, j)
      N --> North (i-1, j)
      S --> South (i+1, j)
      E --> East (i, j+1)
      W --> West (i, j-1)
      N.E--> North-East (i-1, j+1)
      N.W--> North-West (i-1, j-1)
      S.E--> South-East (i+1, j+1)
      S.W--> South-West (i+1, j-1)*/

      // To store the 'g', 'h' and 'f' of the 8 successors
      double gNew, hNew, fNew;

      //----------- 1st Successor (North) ------------

      // Only process this cell if this is a valid one
      if (isValid(i - 1, j) == true) {
        // If the destination cell is the same as the
        // current successor
        if (isDestination(i - 1, j, dest) == true) {
          // Set the Parent of the destination cell
          cellDetails[i - 1][j].parent_i = i;
          cellDetails[i - 1][j].parent_j = j;
          ROS_INFO("The destination cell is found\n");

          int row = dest.first;
          int col = dest.second;

          while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
            Path.push(make_pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
          }

          Path.push(make_pair(row, col));

          foundDest = true;
          return;
        }
        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        else if (closedList[i - 1][j] == false && isUnBlocked(i - 1, j) == true) {
          gNew = cellDetails[i][j].g + 1.0;
          hNew = calculateHValue(i - 1, j, dest);
          fNew = gNew + hNew;

          // If it isn’t on the open list, add it to
          // the open list. Make the current square
          // the parent of this square. Record the
          // f, g, and h costs of the square cell
          // OR
          // If it is on the open list already, check
          // to see if this path to that square is
          // better, using 'f' cost as the measure.
          if (cellDetails[i - 1][j].f == FLT_MAX || cellDetails[i - 1][j].f > fNew) {
            openList.insert(make_pair(
            fNew, make_pair(i - 1, j)));

            // Update the details of this cell
            cellDetails[i - 1][j].f = fNew;
            cellDetails[i - 1][j].g = gNew;
            cellDetails[i - 1][j].h = hNew;
            cellDetails[i - 1][j].parent_i = i;
            cellDetails[i - 1][j].parent_j = j;
          }
        }
      }

      //----------- 2nd Successor (South) ------------

      // Only process this cell if this is a valid one
      if (isValid(i + 1, j) == true) {
        // If the destination cell is the same as the
        // current successor
        if (isDestination(i + 1, j, dest) == true) {
          // Set the Parent of the destination cell
          cellDetails[i + 1][j].parent_i = i;
          cellDetails[i + 1][j].parent_j = j;
          ROS_INFO("The destination cell is found\n");

          int row = dest.first;
          int col = dest.second;

          while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
            Path.push(make_pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
          }

          Path.push(make_pair(row, col));

          foundDest = true;
          return;
        }
        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        else if (closedList[i + 1][j] == false && isUnBlocked(i + 1, j) == true) {
          gNew = cellDetails[i][j].g + 1.0;
          hNew = calculateHValue(i + 1, j, dest);
          fNew = gNew + hNew;

          // If it isn’t on the open list, add it to
          // the open list. Make the current square
          // the parent of this square. Record the
          // f, g, and h costs of the square cell
          // OR
          // If it is on the open list already, check
          // to see if this path to that square is
          // better, using 'f' cost as the measure.
          if (cellDetails[i + 1][j].f == FLT_MAX || cellDetails[i + 1][j].f > fNew) {
            openList.insert(make_pair(
            fNew, make_pair(i + 1, j)));
            // Update the details of this cell
            cellDetails[i + 1][j].f = fNew;
            cellDetails[i + 1][j].g = gNew;
            cellDetails[i + 1][j].h = hNew;
            cellDetails[i + 1][j].parent_i = i;
            cellDetails[i + 1][j].parent_j = j;
          }
        }
      }

      //----------- 3rd Successor (East) ------------

      // Only process this cell if this is a valid one
      if (isValid(i, j + 1) == true) {
        // If the destination cell is the same as the
        // current successor
        if (isDestination(i, j + 1, dest) == true) {
          // Set the Parent of the destination cell
          cellDetails[i][j + 1].parent_i = i;
          cellDetails[i][j + 1].parent_j = j;
          ROS_INFO("The destination cell is found\n");

          int row = dest.first;
          int col = dest.second;

          while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
            Path.push(make_pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
          }

          Path.push(make_pair(row, col));

          foundDest = true;
          return;
        }

        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        else if (closedList[i][j + 1] == false && isUnBlocked(i, j + 1) == true) {
          gNew = cellDetails[i][j].g + 1.0;
          hNew = calculateHValue(i, j + 1, dest);
          fNew = gNew + hNew;

          // If it isn’t on the open list, add it to
          // the open list. Make the current square
          // the parent of this square. Record the
          // f, g, and h costs of the square cell
          // OR
          // If it is on the open list already, check
          // to see if this path to that square is
          // better, using 'f' cost as the measure.
          if (cellDetails[i][j + 1].f == FLT_MAX || cellDetails[i][j + 1].f > fNew) {
            openList.insert(make_pair(
            fNew, make_pair(i, j + 1)));

            // Update the details of this cell
            cellDetails[i][j + 1].f = fNew;
            cellDetails[i][j + 1].g = gNew;
            cellDetails[i][j + 1].h = hNew;
            cellDetails[i][j + 1].parent_i = i;
            cellDetails[i][j + 1].parent_j = j;
          }
        }
      }

      //----------- 4th Successor (West) ------------

      // Only process this cell if this is a valid one
      if (isValid(i, j - 1) == true) {
        // If the destination cell is the same as the
        // current successor
        if (isDestination(i, j - 1, dest) == true) {
          // Set the Parent of the destination cell
          cellDetails[i][j - 1].parent_i = i;
          cellDetails[i][j - 1].parent_j = j;
          ROS_INFO("The destination cell is found\n");

          int row = dest.first;
          int col = dest.second;

          while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
            Path.push(make_pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
          }

          Path.push(make_pair(row, col));

          foundDest = true;
          return;
        }

        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        else if (closedList[i][j - 1] == false && isUnBlocked(i, j - 1) == true) {
          gNew = cellDetails[i][j].g + 1.0;
          hNew = calculateHValue(i, j - 1, dest);
          fNew = gNew + hNew;

          // If it isn’t on the open list, add it to
          // the open list. Make the current square
          // the parent of this square. Record the
          // f, g, and h costs of the square cell
          // OR
          // If it is on the open list already, check
          // to see if this path to that square is
          // better, using 'f' cost as the measure.
          if (cellDetails[i][j - 1].f == FLT_MAX || cellDetails[i][j - 1].f > fNew) {
            openList.insert(make_pair(
            fNew, make_pair(i, j - 1)));

            // Update the details of this cell
            cellDetails[i][j - 1].f = fNew;
            cellDetails[i][j - 1].g = gNew;
            cellDetails[i][j - 1].h = hNew;
            cellDetails[i][j - 1].parent_i = i;
            cellDetails[i][j - 1].parent_j = j;
          }
        }
      }

      //----------- 5th Successor (North-East) ------------

      // Only process this cell if this is a valid one
      if (isValid(i - 1, j + 1) == true) {
        // If the destination cell is the same as the
        // current successor
        if (isDestination(i - 1, j + 1, dest) == true) {
          // Set the Parent of the destination cell
          cellDetails[i - 1][j + 1].parent_i = i;
          cellDetails[i - 1][j + 1].parent_j = j;
          ROS_INFO("The destination cell is found\n");

          int row = dest.first;
          int col = dest.second;

          while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
            Path.push(make_pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
          }

          Path.push(make_pair(row, col));

          foundDest = true;
          return;
        }

        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        else if (closedList[i - 1][j + 1] == false && isUnBlocked(i - 1, j + 1) == true) {
          gNew = cellDetails[i][j].g + 1.414;
          hNew = calculateHValue(i - 1, j + 1, dest);
          fNew = gNew + hNew;

          // If it isn’t on the open list, add it to
          // the open list. Make the current square
          // the parent of this square. Record the
          // f, g, and h costs of the square cell
          // OR
          // If it is on the open list already, check
          // to see if this path to that square is
          // better, using 'f' cost as the measure.
          if (cellDetails[i - 1][j + 1].f == FLT_MAX
          || cellDetails[i - 1][j + 1].f > fNew) {
            openList.insert(make_pair(
            fNew, make_pair(i - 1, j + 1)));

            // Update the details of this cell
            cellDetails[i - 1][j + 1].f = fNew;
            cellDetails[i - 1][j + 1].g = gNew;
            cellDetails[i - 1][j + 1].h = hNew;
            cellDetails[i - 1][j + 1].parent_i = i;
            cellDetails[i - 1][j + 1].parent_j = j;
          }
        }
      }

      //----------- 6th Successor (North-West) ------------

      // Only process this cell if this is a valid one
      if (isValid(i - 1, j - 1) == true) {
        // If the destination cell is the same as the
        // current successor
        if (isDestination(i - 1, j - 1, dest) == true) {
          // Set the Parent of the destination cell
          cellDetails[i - 1][j - 1].parent_i = i;
          cellDetails[i - 1][j - 1].parent_j = j;
          ROS_INFO("The destination cell is found\n");

          int row = dest.first;
          int col = dest.second;

          while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
            Path.push(make_pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
          }

          Path.push(make_pair(row, col));


          foundDest = true;
          return;
        }

        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        else if (closedList[i - 1][j - 1] == false && isUnBlocked(i - 1, j - 1) == true) {
          gNew = cellDetails[i][j].g + 1.414;
          hNew = calculateHValue(i - 1, j - 1, dest);
          fNew = gNew + hNew;

          // If it isn’t on the open list, add it to
          // the open list. Make the current square
          // the parent of this square. Record the
          // f, g, and h costs of the square cell
          // OR
          // If it is on the open list already, check
          // to see if this path to that square is
          // better, using 'f' cost as the measure.
          if (cellDetails[i - 1][j - 1].f == FLT_MAX || cellDetails[i - 1][j - 1].f > fNew) {
          openList.insert(make_pair(
          fNew, make_pair(i - 1, j - 1)));
          // Update the details of this cell
          cellDetails[i - 1][j - 1].f = fNew;
          cellDetails[i - 1][j - 1].g = gNew;
          cellDetails[i - 1][j - 1].h = hNew;
          cellDetails[i - 1][j - 1].parent_i = i;
          cellDetails[i - 1][j - 1].parent_j = j;
          }
        }
      }

      //----------- 7th Successor (South-East) ------------

      // Only process this cell if this is a valid one
      if (isValid(i + 1, j + 1) == true) {
        // If the destination cell is the same as the
        // current successor
        if (isDestination(i + 1, j + 1, dest) == true) {
          // Set the Parent of the destination cell
          cellDetails[i + 1][j + 1].parent_i = i;
          cellDetails[i + 1][j + 1].parent_j = j;
          ROS_INFO("The destination cell is found\n");

          int row = dest.first;
          int col = dest.second;

          while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
            Path.push(make_pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
          }

          Path.push(make_pair(row, col));

          foundDest = true;
          return;
        }

        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        else if (closedList[i + 1][j + 1] == false && isUnBlocked(i + 1, j + 1) == true) {
          gNew = cellDetails[i][j].g + 1.414;
          hNew = calculateHValue(i + 1, j + 1, dest);
          fNew = gNew + hNew;

          // If it isn’t on the open list, add it to
          // the open list. Make the current square
          // the parent of this square. Record the
          // f, g, and h costs of the square cell
          // OR
          // If it is on the open list already, check
          // to see if this path to that square is
          // better, using 'f' cost as the measure.
          if (cellDetails[i + 1][j + 1].f == FLT_MAX || cellDetails[i + 1][j + 1].f > fNew) {
            openList.insert(make_pair(
            fNew, make_pair(i + 1, j + 1)));

            // Update the details of this cell
            cellDetails[i + 1][j + 1].f = fNew;
            cellDetails[i + 1][j + 1].g = gNew;
            cellDetails[i + 1][j + 1].h = hNew;
            cellDetails[i + 1][j + 1].parent_i = i;
            cellDetails[i + 1][j + 1].parent_j = j;
          }
        }
      }

      //----------- 8th Successor (South-West) ------------

      // Only process this cell if this is a valid one
        if (isValid(i + 1, j - 1) == true) {
        // If the destination cell is the same as the
        // current successor
        if (isDestination(i + 1, j - 1, dest) == true) {
          // Set the Parent of the destination cell
          cellDetails[i + 1][j - 1].parent_i = i;
          cellDetails[i + 1][j - 1].parent_j = j;
          ROS_INFO("The destination cell is found\n");

          int row = dest.first;
          int col = dest.second;

          while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
            Path.push(make_pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
          }

          Path.push(make_pair(row, col));

          foundDest = true;
          return;
        }

        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        else if (closedList[i + 1][j - 1] == false && isUnBlocked(i + 1, j - 1) == true) {
          gNew = cellDetails[i][j].g + 1.414;
          hNew = calculateHValue(i + 1, j - 1, dest);
          fNew = gNew + hNew;

          // If it isn’t on the open list, add it to
          // the open list. Make the current square
          // the parent of this square. Record the
          // f, g, and h costs of the square cell
          // OR
          // If it is on the open list already, check
          // to see if this path to that square is
          // better, using 'f' cost as the measure.
          if (cellDetails[i + 1][j - 1].f == FLT_MAX || cellDetails[i + 1][j - 1].f > fNew) {
            openList.insert(make_pair(
            fNew, make_pair(i + 1, j - 1)));

            // Update the details of this cell
            cellDetails[i + 1][j - 1].f = fNew;
            cellDetails[i + 1][j - 1].g = gNew;
            cellDetails[i + 1][j - 1].h = hNew;
            cellDetails[i + 1][j - 1].parent_i = i;
            cellDetails[i + 1][j - 1].parent_j = j;
          }
        }
      }
    }

    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destination cell. This may happen when the
    // there is no way to destination cell (due to
    // blockages)
    if (foundDest == false)
    ROS_INFO("Failed to find the Destination Cell\n");

    return;
  }

};



//*********************************** MAIN ***********************************//

int main(int argc, char **argv)
  {
    ros::init(argc, argv, "map_process");
    MapProcessing mapproc;
    ros::spin();
    return 0;
  }