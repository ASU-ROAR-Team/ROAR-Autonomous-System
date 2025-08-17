# pylint: disable=all
# mypy: ignore-errors
import rospy
from std_msgs.msg import String


class TalkerNode:
    def __init__(self):
        rospy.init_node("talker_supervisor")
        self.heartbeatPub = rospy.Publisher("/system/heartbeat", String, queue_size=10)
        self.mainPub = rospy.Publisher("/topic", String, queue_size=10)
        self.rate = rospy.Rate(10)


if __name__ == "__main__":
    try:
        node = TalkerNode()
        while not rospy.is_shutdown():
            node.heartbeatPub.publish("talker:running")
            node.mainPub.publish("hello")
            node.rate.sleep()

    except rospy.ROSInterruptException:
        pass
