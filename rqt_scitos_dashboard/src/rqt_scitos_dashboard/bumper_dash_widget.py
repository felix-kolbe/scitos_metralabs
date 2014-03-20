'''
Created on Sep 27, 2013

@author: felix
'''


import rospy
from rqt_robot_dashboard.icon_tool_button import IconToolButton

from metralabs_msgs.msg import ScitosG5Bumper
from std_msgs.msg import Empty



class BumperDashWidget(IconToolButton):

    def __init__(self, context):
        icons = [#['mode.png']
                 ['bg-grey.svg'], ['bg-green.svg'], ['bg-orange.svg'], ['bg-red.svg']
                 ]

        super(BumperDashWidget, self).__init__('Bumper', icons)

        self.context = context
        self.clicked.connect(self._reset_bumper)

        self.setToolTip('The bumper status')
        self.update_state(0)

        self._bumper_sub = rospy.Subscriber('/bumper', ScitosG5Bumper, self._bumper_callback)
        self._bumper_reset_pub = rospy.Publisher('/bumper_reset', Empty)

    def _bumper_callback(self, msg):
        if msg.bumper_pressed:
            self.update_state(3)
            self.setToolTip('bumper status: pressed')
        elif msg.motor_stop:
            self.update_state(2)
            self.setToolTip('bumper status: motor stop')
        else:
            self.update_state(1)
            self.setToolTip('bumper status: normal')

    def _reset_bumper(self):
        self._bumper_reset_pub.publish()
