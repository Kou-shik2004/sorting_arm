# One object observation, identical whether it comes from the fixed YAML source
# or from wrist perception (D14). No perception-internal fields belong here.

string id
string label
geometry_msgs/PoseStamped centre

uint8 primitive_type
float64[] dimensions
