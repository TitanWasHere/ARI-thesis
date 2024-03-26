import rospy
from std_msgs.msg import String
from speech_multi_recognizer.msg import SpeechRecognizerFeedback, SpeechRecognizerResult, SpeechRecognizerActionResult, SpeechRecognizerActionGoal, SpeechRecognizerGoal

class listener:
    def __init__(self):
        ## Subscribe to "result"
        self.image_sub = rospy.Subscriber("speech_multi_recognizer/result", StringMultiRecognizerActionResult, self.callback)
        self.pub = rospy.Publisher('speech_multi_recognizer/goal', StringMultiRecognizerActionGoal, queue_size=1)
        self.pubres = rospy.Publisher('chatter', String, queue_size=1)
        rospy.sleep(5)

        self.send_asr_action()

    def send_asr_action(self):
        print("Sending ASR action")
        goal = StringMultiRecognizerActionGoal()
        goal.goal.language = "it_IT"

        now = rospy.get_rostime()
        goal.header.stamp.secs = now
        goal.goal_id.stamp = now
        self.pub.publish(goal)
        print("done")

    def callback(self, SpeechMultiRecognizerActionResult):
        recognized = ''
        if SpeechMultiRecognizerActionResult.result.recognition_result:
            recognized = SpeechMultiRecognizerActionResult.result.recognition_result[0]
        if recognized == '':
            rospy.loginfo("No speech recognized")
        else:
            rospy.loginfo(recognized)
            self.pubres.publish(recognized)
            rospy.wait_for_message("chiarastatus", String, timeout=10)
        self.send_asr_action()

def main():
    rospy.init_node('listener', anonymous=True)
    sub = listener()
    print("Listener node started")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass