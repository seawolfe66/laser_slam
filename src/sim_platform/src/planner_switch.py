#!/usr/bin/env python

import roslib
#roslib.load_manifest("sim_platform")
import rospy
import tf
from std_msgs.msg import String
from asr_navfn.srv import *


if __name__ == '__main__':
   rospy.wait_for_service('/move_base/supPlanner/change_plan')
#   rospy.wait_for_service('/move_base/sup_l_Planner/change_plan')
#    cpl=gPlannerSwitcher()
#    #cpl.changePlanner = "switch on"
#    #cpl = 'asr_navfn/NavfnROS'
#    if cpl == 'asr_navfn/NavfnROS':
#        cpl = 'linear_global_planner/LinearGlobalPlanner'
#    elif cpl =='linear_global_planner/LinearGlobalPlanner':
#        cpl = 'asr_navfn/NavfnROS'
#    else:
#        cpl = 'asr_navfn/NavfnROS'
   #cpl.localName = "dwa_local_planner/DWAPlannerROS"
#    cp=String()
#    cp= 'switch on'
#    cpl.changePlanner = cp
#    cp = 'asr_navfn/NavfnROS'
#    cpl.globalName = cp
#    cp = 'dwa_local_planner/DWAPlannerROS'
#    cpl.localName = cp
   cg = String()
   cg = 'asr_navfn/NavfnROS'
#   cg = 'linear_global_planner/LinearGlobalPlanner'
   cl = String()
   if cg == 'asr_navfn/NavfnROS':
       cl = 'dwa_local_planner/DWAPlannerROS'
   elif cg == 'linear_global_planner/LinearGlobalPlanner':
       cl = 'ftc_local_planner/FTCPlanner'
   
   try:
       gplannerchange = rospy.ServiceProxy('/move_base/supPlanner/change_plan', gPlannerSwitcher)
	#resp1 = plannerchange("switch off","navfn/NavfnROS","dwa_local_planner/DWAPlannerROS")
       resp1 =  gplannerchange(cg)
       print "call change planner service %s "% (resp1.changeResult)
   except rospy.ServiceException, e:
       print "Service call failed: %s"%e
       
#   try:
#       lplannerchange = rospy.ServiceProxy('/move_base/sup_l_Planner/change_plan', lPlannerSwitcher)
    #resp1 = plannerchange("switch off","navfn/NavfnROS","dwa_local_planner/DWAPlannerROS")
#       resp2 =  lplannerchange(cl)
#       print "call change planner service %s "% (resp2.changeResult)
#   except rospy.ServiceException, e:
#       print "Service call failed: %s"%e

