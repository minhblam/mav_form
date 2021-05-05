// Navigation Manager
// Modifies given waypoints from the navigation manager and creates offset for any given number of drones to pass down to the tracjectory manager.

#include <math.h>
// #include <vector>
#include <iostream>
// #include <string>





void line_form(float x_off, float y_off, float psi_off, int k){ //Where off is original WP and k is the number of drones

// This function generates offset waypoints for each drone. Obviously needs more loops right now.
  if (drone_number_odd(k)){ //Left
    offset[k].x = heading*line; //Find the x coordinate in a line next to original WP
    offset[k].y = heading*line; //Find the y coordinate in a line next to original WP
    offset[k].z = heading*line; //Find the z coordinate in a line next to original WP
  }else{                    //Right
    offset[k].x = heading*line; //Find the x coordinate in a line next to original WP
    offset[k].y = heading*line; //Find the y coordinate in a line next to original WP
    offset[k].z = heading*line; //Find the z coordinate in a line next to original WP
  }
}

void vee_form(float x_off, float y_off, float psi_off, int k){ //Where off is original WP and k is the number of drones

// This function generates offset waypoints for each drone. Obviously needs more loops right now.
  if (drone_number_odd(k)){ //Left
    offset[k].x = heading*line; //Find the x coordinate in a line next to original WP
    offset[k].y = heading*line; //Find the y coordinate in a line next to original WP
    offset[k].z = heading*line; //Find the z coordinate in a line next to original WP
  }else{                    //Right
    offset[k].x = heading*line; //Find the x coordinate in a line next to original WP
    offset[k].y = heading*line; //Find the y coordinate in a line next to original WP
    offset[k].z = heading*line; //Find the z coordinate in a line next to original WP
  }
}

void run_form(std::string form){
  if (form=='line'){
    line_form(x,y,z,psi) // Replace with subscribed values from nav_manager
  }
  if (form=='vee'){
    vee_form(x,y,z,psi) // Replace with subscribed values from nav_manager
  }
}


int main(int argc, char **argv)
{
  run_form('v');
  return 0;
}
