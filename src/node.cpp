/*
 Copyright (C) 2015 Stefan Kohlbrecher, TU Darmstadt

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ros/ros.h"
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ps4_stereo_camera");

  // Shared parameters to be propagated to nodelet private namespaces
  nodelet::Loader manager(true); // Don't bring up the manager ROS API
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  manager.load(ros::this_node::getName(), "ps4_stereo_camera/StereoCameraNodelet", remappings, my_argv);

  ros::spin();
  return 0;
}
