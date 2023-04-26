import rospy
import math
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from statistics import mean 



if __name__ == '__main__':
    #initialization
    rospy.init_node('walking_robot_listener')
    tfBuffer = tf2_ros.Buffer()
    shanks = ('LF_SHANK','LH_SHANK','RF_SHANK','RH_SHANK')
    base = 'base'

    # TF listener
    listener = tf2_ros.TransformListener(tfBuffer)

    # TF publisher
    shank_center = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    rate = rospy.Rate(10.0)
    #rospy.wait_for_service('tf')
    while not rospy.is_shutdown():
        rate.sleep()
        coords = []
        trans = None
        # read all transforms
        for shank in shanks:
            try:
                trans = tfBuffer.lookup_transform(shank, base, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            coords.append((
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ))
        
        if trans is None:
            print('No translation')
            continue

        # calculating the data
        average = [mean(coord) for coord in zip(*coords)]
        rotation = trans.transform.rotation

        # printing the data
        print(f"Position: {average}")
        print(f"Rotation:{rotation}")

        # creating tf message
        t = geometry_msgs.msg.TransformStamped()

        t.header.frame_id = base
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "shank_center"

        x,y,z = average
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        t.transform.rotation = rotation

        tfm = tf2_msgs.msg.TFMessage([t])

        # publishing the data
        shank_center.publish(tfm)
        
        tfBuffer.clear()

        
        