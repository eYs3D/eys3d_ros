/*Copyright:
	This file copyright (C) 2017 by

	eYs3D an Etron company

	An unpublished work.  All rights reserved.

	This file is proprietary information, and may not be disclosed or
	copied without the prior permission of eYs3D an Etron company.
*/

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "dm_preview_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load(ros::this_node::getName(),
      "dm_preview/DMPreviewNodelet",
      remap, nargv);

  ros::spin();

  return 0;
}
