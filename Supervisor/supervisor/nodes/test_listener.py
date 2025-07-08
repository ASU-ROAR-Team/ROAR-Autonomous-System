# pylint: disable=all
# mypy: ignore-errors
import rospy
from std_msgs.msg import String


class ListenerNode:
    def __init__(self):
        rospy.init_node("listener_supervisor")
        self.heartbeatPub = rospy.Publisher("/system/heartbeat", String, queue_size=10)
        rospy.Subscriber("/topic", String, self.callback)
        self.rate = rospy.Rate(10)

    def callback(self, msg):
        rospy.loginfo(f"Received message: {msg.data}")


if __name__ == "__main__":
    try:
        node = ListenerNode()
        while not rospy.is_shutdown():
            node.heartbeatPub.publish("listener:running")
            node.rate.sleep()

    except rospy.ROSInterruptException:
        pass
