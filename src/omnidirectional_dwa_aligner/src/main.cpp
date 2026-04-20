#include "omnidirectional_dwa.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "omnidirectional_dwa_aligner_node");
  Omnidirectional_DWAPlanner planner;
  planner.process();
  return 0;
}
