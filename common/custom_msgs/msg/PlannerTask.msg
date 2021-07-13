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

int8 task_cmd 
#0:run
#1:match
#2:done
#3:second match forward
#4:second match backward
#5:lock/target point
#6:dec speed
#7:arrive wait
#8:first stop
#10:stop
#11:un stop
#12:stop

int8 bridge_number 
# brdige or bay number

int16 bay  
string container_pose        
string container_size     
string yard_block          
string yard_num           

int8 bridge_in_right_bay
int8 qc_bridge_in_right_bay

string lock_pose
int8 qc_lane_id
string head_orien
string isContainerOn
string msg_id
