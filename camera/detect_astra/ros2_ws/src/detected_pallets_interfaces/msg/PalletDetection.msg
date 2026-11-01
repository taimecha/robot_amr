# A semantic seed, never a live RGB+Depth measurement. All images share header.
std_msgs/Header header
bool valid
string status
string class_name
float32 confidence
float32[4] bbox_xyxy
sensor_msgs/Image rgb
# Full-size mono mask encoded as PNG; not resized or a predicted depth map.
sensor_msgs/CompressedImage mask
