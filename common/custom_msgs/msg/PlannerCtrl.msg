Header header

int8  target_way_mode
# target_way_mode = 0  : load global map ,the target point is the finish point
# target_way_mode = 1 : load global map   and  the target point is get from cloud com
float64   TargetMap_x
float64   TargetMap_y
float64    TargetReflectColumn_x
float64    TargetReflectColumn_y

# target_way_mode = 2  : don'tload global map, and  the target points is get from cloud com
LaneArray Lane_array

int8 target_speed_mode
#0 : use the waypoints speed
#1 : use the gui speed --gui speed enable
#2 : use the cloud com speed ,temp is 8
float64 target_speed

int8 task_cmd 
#0: reserve
#1: location stop 
#2:cps
#3:lidar- location

int8 bridge_number 
# brdige or bay number

int16 bay  
string container_pose        
string container_size     
string yard_block          
string yard_num           

int8 bridge_in_right_bay
int8 qc_bridge_in_right_bay