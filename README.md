# car_detecter
Project Description
-----------------------------
Make a algorithm, detecting new objects compared to a known map.

Prerequisite 
-----------------------------
### amcl
### laser_scan_mathcer
  * transform scan data to LDP type data(laser_data of csm libaray)
  * With the current ldp data and the previous ldp data, it executes an ICP algorithm.
  * getPrediction -> calculate delta predicted position from some information, ex. velocity, odom, ...
  * sm_icp -> doing an ICP algorithm
  * I will use map data instead of the current ldp data.
  
