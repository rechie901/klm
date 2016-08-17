#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <lidar_tracker/centroids.h>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>
using namespace std;
using namespace Eigen;

//lidar_tracker::centroids track;
lidar_tracker::centroids predicted;

struct track
{
    float x, y, mean_x, mean_y, vel_x, vel_y, var_x, var_y; // track at time t
    float x_pred, y_pred, vel_x_pred, vel_y_pred; // predicted states at time t+1
    float z_x, z_y; // measurements observed
    float z_x_pred, z_y_pred; // predicted measurements at time t+1
    float v_x, v_y; // measurements residual
};
vector < track > tracks;
vector < track > MAP;

// Constant Matrix


/*void update_mean_var(float x, float y, float mean_x, float mean_y, float var_x, float var_y)
{
    cout<<" lalalala"<< endl;
    x=100;
    mean_x=(x+mean_x)/2;
    mean_y=(y+mean_y)/2;
     var_x=(x-mean_x)*(x-mean_x);
    var_y=(y-mean_y)*(y-mean_y);
}
*/

void chatterCallback(const lidar_tracker::centroidsConstPtr& input)
{
  float T = 0.1; // 100 ms
  //lidar_tracker::centroids centroids;
  int num_input = input->points.size();
  int num_tracks = tracks.size();
  std::cout<<"Observation Size: "<<num_input<<std::endl;
  std::cout<<"Track Size: "<<num_tracks<<std::endl;
     
  MatrixXf F(4,4);
  F(0,0),F(1,1),F(2,2),F(3,3) = 1.0;
  F(0,1),F(0,3),F(1,0),F(1,2),F(2,0),F(2,1),F(2,3),F(3,0),F(3,1),F(3,2) = 0.0;
  F(0,2),F(1,3) = T;
  //F()
  MatrixXf H(4,4);
  H(0,0),H(1,1),H(2,2),H(3,3)=1.0;
  H(0,1),H(0,2),H(0,3),H(1,0),H(1,2),H(1,3),H(2,0),H(2,1),H(2,3),H(3,0),H(3,1),H(3,2)=0.0;
  ////
  VectorXf X_pred(4);// state prediction vector at t+1
  VectorXf X(4); // state vector at t
  VectorXf Z_pred(4); // measurement prediction vector at t+1
  VectorXf Z(4); // measurement vector at t
  VectorXf r(4); // residual
  VectorXf u(4);// noise
  u(0),u(1),u(2),u(3) = 0;
  VectorXf v(4); // noise
  v(0),v(1),v(2),v(3) = 0;
 
  if(num_input > 0)
  {
         if(num_tracks == 0)  // Initializing the empty pool
         {
             tracks.resize(input->points.size());
           for(std::vector<int>::size_type i = 0; i != input->points.size(); i++)
            {
                   tracks[i].x=input->points[i].x;
                   tracks[i].y=input->points[i].y;
                   //tracks[i].mean_x=(tracks[i].x+tracks[i].mean_x)/2;
                   //tracks[i].z=0.0;
                //std::cout<<tracks[i].mean_x<<std::endl;
            }
         }
         else //(num_tracks < num_input)
         {
                 //state_prediction;
            for(std::vector<int>::size_type i = 0; i != tracks.size(); i++)
            {  
                 
                X_pred(0)=tracks[i].x_pred;
                X_pred(1)=tracks[i].y_pred;
                X_pred(2)=tracks[i].vel_x_pred;
                X_pred(3)=tracks[i].vel_y_pred;
               
               
                X(0)=tracks[i].x;
                X(1)=tracks[i].y;
                X(2)=tracks[i].vel_x;
                X(3)=tracks[i].vel_y;
               
               
                Z_pred(0)=tracks[i].z_x_pred;
                Z_pred(1)=tracks[i].z_y_pred;
                Z_pred(2)=0;
                Z_pred(3)=0;
               

                Z(0)=tracks[i].z_x;
                Z(1)=tracks[i].z_y;
                Z(2)=0;
                Z(3)=0;
               

                r(0) = tracks[i].v_x;
                r(1) = tracks[i].v_y;
                r(2),r(3) = 0;
                 
               
                cout<< tracks[i].mean_x << "  "<< tracks[i].mean_y<<"  "<<tracks[i].var_x<<"  "<<tracks[i].var_y << endl;
            }            
                X_pred = F*X + u;
                Z_pred = H*X_pred + v;
                // MAP
               
               
                vector < float > map_x;
                vector < float > map_y;
                map_y.resize(tracks.size());
                map_x.resize(tracks.size());
                for(std::vector<int>::size_type i = 0; i != input->points.size(); i++)
                {
                   
                       for(std::vector<int>::size_type j = 0; j != tracks.size(); j++)
                       {
                           map_x[j] = exp(-(pow((input->points[i].x - tracks[j].mean_x),2))/tracks[j].var_x);
                           map_y[j] = exp(-(pow((input->points[i].y - tracks[j].mean_y),2))/tracks[j].var_y);
                       }
                       auto index_x = max_element(map_x.begin(), map_x.end());
                       int k_x = distance(map_x.begin(), index_x);
                       auto index_y = max_element(map_y.begin(), map_y.end());
                       int k_y = distance(map_y.begin(), index_y);
                       if(k_x == k_y)
                       {
                           tracks[k_x].z_x=input->points[i].x;
                           tracks[k_y].z_y=input->points[i].y;

                    }               
                }
                r = Z - Z_pred;
                X = X_pred + r;   
                                   
                 //filter_tracks;
                 //MAP;
                 //update;
         }
        for(std::vector<int>::size_type i = 0; i != tracks.size(); i++)
        {  
            tracks[i].mean_x=(tracks[i].x+tracks[i].mean_x)/2;
            tracks[i].mean_y=(tracks[i].y+tracks[i].mean_y)/2;
             tracks[i].var_x=(tracks[i].x-tracks[i].mean_x)*(tracks[i].x-tracks[i].mean_x);
            tracks[i].var_y=(tracks[i].y-tracks[i].mean_y)*(tracks[i].y-tracks[i].mean_y);
            cout<< tracks[i].mean_x << "  "<< tracks[i].mean_y<<"  "<<tracks[i].var_x<<"  "<<tracks[i].var_y << endl;
        }//update_mean&Variance;
  }

     
 
 
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/cluster_centroids", 10, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
