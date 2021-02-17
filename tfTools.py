def get_transform_vector_from_pose(msg):
    return (msg.position.x,msg.position.y,msg.position.z)


def get_orientation_from_pose(msg):
    return (msg.orientation.x, msg.orientation.y,msg.orientation.z,msg.orientation.w)

def get_2d_point_moved_using_vector(vector,point):
    result_point=(point[0]+vector[0],point[1]+vector[1])
    return result_point