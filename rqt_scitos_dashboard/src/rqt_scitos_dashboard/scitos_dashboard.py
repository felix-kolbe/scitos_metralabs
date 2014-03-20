'''
Created on Aug 27, 2013

@author: felix
'''

import thread

import rospy


from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget, BatteryDashWidget
from rqt_robot_dashboard.icon_tool_button import IconToolButton

from rqt_scitos_dashboard.bumper_dash_widget import BumperDashWidget


from std_msgs.msg import Bool



class SMACHEnableSwitchDashWidget(IconToolButton):

    def __init__(self, context):
        icons = [#['mode.png']
                 ['bg-grey.svg'], ['bg-orange.svg'], ['bg-green.svg']
                 ]

        super(SMACHEnableSwitchDashWidget, self).__init__('SMACH enable switch', icons)

        self.context = context
        self.clicked.connect(self._smach_button_click)

        self.setToolTip('Whether smach is enabled. click to switch')
        self.update_state(0)
        self.current_state = None

        self._sub = rospy.Subscriber('/enable_smach', Bool, self._enable_smach_callback)
        self._enable_smach_pub = rospy.Publisher('/enable_smach', Bool)
        thread.start_new_thread(self._smach_enabled_publisher_thread, ())

    def _enable_smach_callback(self, msg):
        self.current_state = msg.data
        if msg.data:
            self.update_state(2)
            self.setToolTip('smach is enabled. click to disable')
        else:
            self.update_state(1)
            self.setToolTip('smach is disabled. click to enable')

    def _smach_button_click(self):
        # send negated current state. also sends True if current is None at from beginning
        self._enable_smach_pub.publish(not self.current_state)
        
    def _smach_enabled_publisher_thread(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            self._enable_smach_pub.publish(self.current_state)






class ScitosDashboard(Dashboard):

    def get_widgets(self):
        self.monitor = MonitorDashWidget(self.context)
        self.console = ConsoleDashWidget(self.context)
#        self.battery = BatteryDashWidget(self.context)
        self.bumper = BumperDashWidget(self.context)
        self.smach_switch = SMACHEnableSwitchDashWidget(self.context)

        return [[self.monitor, self.console], #[self.battery],
                [self.bumper],
                [self.smach_switch]
                ]
