// Copyright (c) 2018 Raphael Deimel
#include <array>
#include <cmath>
#include <iostream>
#include <iterator>



#include <pdcontroller.h>


int main(int argc, char** argv) {
  //init rosnode
  ros::init(argc, argv, "pdcontroller_realtime");
  int exitval = mainloop<PDController >(); //run the control main loop using PDController
  return exitval;
}


