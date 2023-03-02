#!/usr/bin/env python

import numpy as np
import roslib
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import sys
from rovi_industrial import rtde_sock as comm

Config={
  'robot_ip':'127.0.0.1',
  'joint_ids':['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
  'robot_recipe':'default.xml',
  'robot_port':30004,
  'copy':[
    {'param':'/dashboard/ind/rsocket/enable','input':'input_bit_register_64'},
    {'param':'/dashboard/ind/rovi/stat','input':'input_bit_register_65'},
    {'state':'payload_inertia[0]','input':'input_int_register_24','gain':10},
  ]
}

rospy.init_node('rclient_ur',anonymous=True)
joints=JointState()
joints.name=Config['joint_ids']
try:
  Config.update(rospy.get_param('/config/rsocket'))
except Exception as e:
  print("get_param exception:",e.args)
pub_js=rospy.Publisher('/joint_states',JointState,queue_size=1)
pub_conn=rospy.Publisher('/rsocket/enable',Bool,queue_size=1)
print("rclient_ur::",Config['robot_ip'])

mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False

def update(q):
  if len(q)>=len(joints.name):
    joints.header.stamp=rospy.Time.now()
    joints.position=q
    pub_js.publish(joints)
    pub_conn.publish(mTrue)

while True:
  if rospy.is_shutdown(): sys.exit()
  rospy.sleep(3)

  if not comm.connect(Config['robot_ip'],Config['robot_port'],Config['robot_recipe'],uplink=True):
    print("rclient_ur::comm connect falied",Config['robot_ip'])
    continue
  if not comm.start():
    print("rclient_ur::comm start falied")
    continue

  print("rclient_ur::comm connected",Config['robot_ip'])

###Build code for interpreter(exec)##############
  for obj in Config['copy']:
    lvar='comm.inregs.'+obj['input']
    exec(lvar+'=0')
    if 'param' in obj:
      obj['cstr']=lvar+'=rospy.get_param("'+obj['param']+'")'
    elif 'state' in obj:
      obj['cstr']=lvar+'=int(comm.state.'+obj['state']+'*'+str(obj['gain'])+')'

###Start Event Loop##############
  loop=True
  while True:
    rospy.sleep(0.05)
    if rospy.is_shutdown():
      loop=False
      break
    try:
      comm.receive()
      update(comm.state.actual_q)
      for obj in Config['copy']:
        exec(obj['cstr'])
      comm.send()
    except Exception as e:
      print('rclient_ur::rtde failed',e)
      break

  try:
    comm.pause()
  except Exception as e:
    print('rclient_ur::rtde stop1',e)

  try:
    comm.disconnect()
  except Exception as e:
    print('rclient_ur::rtde stop2',e)

  if not loop: sys.exit()

