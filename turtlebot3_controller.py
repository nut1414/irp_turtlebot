# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
from dis import dis
import math
from socket import TIPC_SUBSCR_TIMEOUT
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from typing import Callable, List, Union
import logging
import traceback

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from functools import partial
    
    
class Action:
  def __init__(self, handler, *args, **kwargs):
    self.handler = handler
    self.kwargs = kwargs
    self.args = args
    
  def execute(self):
    return self.handler(*self.args,**self.kwargs)
  
  def By(self, *args, **kwargs):
    return self.__class__(self.handler, *args, **kwargs)
  
  def To(self, *args, **kwargs):
    return self.By(*args, **kwargs)
  
class Move(Action):
  def __init__(self, handler: Callable[[int], None], *args: int, **kwargs: int):
    super().__init__(handler, *args, **kwargs)

class Turn(Action):
  def __init__(self, handler, *args: int, **kwargs: int):
     super().__init__(handler, *args, **kwargs)
     
class Checku(Action):
  def __init__(self, handler: Callable[[], None], *args: int, **kwargs: int):
    super().__init__(handler, *args, **kwargs)
  
class GoNextNode(Action):
  def __init__(self, handler: Callable[[int], None], *args: int, **kwargs: int):
     super().__init__(handler, *args, **kwargs)

class GtnnHandling:
  node_moved: int = 0
  node_goal: int = 0
class Turtlebot3Controller(Node):

    def __init__(self):
        super().__init__('turtlebot3_controller')   #node name
        self.cmdVelPublisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scanSubscriber = self.create_subscription(LaserScan, 'scan', self.scanCallback, qos_profile=qos_profile_sensor_data)
        self.batteryStateSubscriber = self.create_subscription(BatteryState, 'battery_state', self.batteryStateCallback, 1)
        self.odomSubscriber = self.create_subscription(Odometry, 'odom', self.odomCallback, 1)
        self.valueLaserRaw = {
            'range_min':0.0,
            'range_max':0.0,
            'ranges':[0.0]*360,
        }
        self.valueBatteryState = None
        self.valueOdometry = {
            'position':None,        #Datatype: geometry_msg/Point   (x,y,z)
            'orientation':None,     #Datatype: geometry_msg/Quaternion (x,y,z,w)
            'linearVelocity':None,  #Datatype: geometry_msg/Vector3 (x,y,z)
            'angularVelocity':None, #Datatype: geometry_msg/Vector3 (x,y,z)
        }
        self.valuePrevOdometry = self.valueOdometry.copy()
        self.valueLastNodeOdomentry = self.valueOdometry.copy()
        self.prevMoveIsComplete = True
        
        self.state = 0
        #Use this timer for the job that should be looping until interrupted
        self.timer = self.create_timer(1,self.timerCallback)
        self.staticMove = []
        
        
        self.gtnn: GtnnHandling = GtnnHandling()

        self.Move = Move(self.moveByCenti)
        self.Turn = Turn(self.rotateByDegreePerSec)
        self.SMove = Move(self.special_move_by_centi)
        self.Checku = Checku(self.check_lidar_to_stop)
        self.GoNextNode = GoNextNode(self.handling_gtnn)
        
        self.OdomAllowActions = (Move, Checku, GoNextNode) # TypeError: union can't be used with with isinstance()
        self.TimerAllowActions = (Turn)
        
        
        self.staticMove = [
          self.GoNextNode.To(2),
          self.Turn.By(-90),
          self.GoNextNode.To(1),
          self.Turn.By(-90),
          # self.Turn.By(90),
          self.Turn.By(-90),
          # self.Turn.By(-90),
          self.GoNextNode.To(1),
          self.Turn.By(90),
          self.GoNextNode.To(2),
          
        ]
        
        
      
        
    def publishVelocityCommand(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity
        self.cmdVelPublisher.publish(msg)
        #self.get_logger().info('Publishing cmd_vel: "%s", "%s"' % linearVelocity, angularVelocity)

    def degToRad(self, deg):
      return deg * 0.0174532925
    
    def radToDeg(self, rad):
      if type(rad) != float:
        print("radToDeg")
        print(rad)
        print(type(rad))
      return rad * 57.2957795
    

    def euler_from_quaternion(self,x, y, z, w):
      """
      Convert a quaternion into euler angles (roll, pitch, yaw)
      roll is rotation around x in radians (counterclockwise)
      pitch is rotation around y in radians (counterclockwise)
      yaw is rotation around z in radians (counterclockwise)
      """
      t0 = +2.0 * (w * x + y * z)
      t1 = +1.0 - 2.0 * (x * x + y * y)
      roll = math.atan2(t0, t1)
  
      t2 = +2.0 * (w * y - z * x)
      t2 = +1.0 if t2 > +1.0 else t2
      t2 = -1.0 if t2 < -1.0 else t2
      pitch = math.asin(t2)
  
      t3 = +2.0 * (w * z + x * y)
      t4 = +1.0 - 2.0 * (y * y + z * z)
      yaw = math.atan2(t3, t4)
  
      return (roll, pitch, yaw) # in radians
    
    def get_yaw(self, orientation):
      (roll, pitch, yaw) = self.euler_from_quaternion(orientation.x,orientation.y,orientation.z,orientation.w)
      return yaw

    '''
      this method will rotate the robot to the nearest object
    '''
    def normalizeAngle(self, angle):
      sign = 1 if angle > 0 else -1
      angle = abs(angle) % 360
      if angle > 180:
        return sign * (angle - 360)
      else:
        return sign * angle  

    def distCentiFromPoint(self, pointA, pointB):
      return math.sqrt(((pointA.x - pointB.x) ** 2) + ((pointA.y - pointB.y) ** 2))
    
    def rotateByDegreePerSec(self, angle) -> None:
      targetAngle = self.degToRad(self.normalizeAngle(angle))
      print("targetAngle Rad"+ str(targetAngle))
      self.publishVelocityCommand(0.0, targetAngle)
    
    def rotateByDegree(self, angle: int) -> None:
      ROTATE_SPEED = 0.5
      MIN_ERROR = 2.5
      
      MIN_SPEED_FACTOR = 0.3
      
      rotate_factor = -1.0 if angle > 0 else 1.0
      
      prevOrient = self.valuePrevOdometry['orientation']
      currentOrient  = self.valueOdometry['orientation']
      prevYawDeg = self.valuePrevOdometry['yawDegree']
      currentYawDeg  = self.valueOdometry['yawDegree']
      
      distGoalCurrentYawDeg = 0
      print("calc")
      
      
      goalYawDeg = prevYawDeg + (self.normalizeAngle(angle) * -1 * rotate_factor)
      
      if goalYawDeg > 180:
        goalYawDeg -= 360
      elif goalYawDeg < -180:
        goalYawDeg += 360
      
        
      # Distance between the goal and current position
      angleDiffGoalCurrent = abs(goalYawDeg - currentYawDeg)
      distGoalCurrentYawDeg = min(360 - angleDiffGoalCurrent,angleDiffGoalCurrent)
      
      direction = -1
      
      
      
      # if the closer side is inward or outward
      if 360 - angleDiffGoalCurrent < angleDiffGoalCurrent:
        # outward
        if goalYawDeg < 0:
          direction = -1 if (goalYawDeg - currentYawDeg) > 0 else 1 # goal on the right, move left
        else:
          direction = 1 if (goalYawDeg - currentYawDeg) > 0 else -1 # goal on the left, move right
      else:
        # inward
        if goalYawDeg < 0:
          direction = 1 if (goalYawDeg - currentYawDeg) > 0 else -1 # goal on the right, move left
        else:
          direction = -1 if (goalYawDeg - currentYawDeg) > 0 else 1 # goal on the left, move right

        
        
      
      # Distance between the goal and start position
      angleDiffGoalStart = abs(goalYawDeg - prevYawDeg)
      distGoalStartYawDeg = min(360 - angleDiffGoalStart,angleDiffGoalStart)
      
      
      print("distance between goal and current")
      print(distGoalCurrentYawDeg)
      print("distance between goal and start")
      print(distGoalStartYawDeg)
      
      # if ((prevYawDeg > 0) and (currentYawDeg < 0)):
      #   print("adding diff")
      #   distYawDeg = abs(prevYawDeg) + abs(currentYawDeg)
      # elif ((prevYawDeg < 0) and (currentYawDeg > 0)):
      #   print("abs then subtracting diff")
      #   distYawDeg = abs(prevYawDeg) - abs(currentYawDeg)
      # else:
      #   print("subtracting then abs diff")
      #   distYawDeg = abs(prevYawDeg - currentYawDeg)
        
      print("speedfac")
      speed_factor = distGoalCurrentYawDeg / (distGoalStartYawDeg+0.1)
      speed_factor = max(MIN_SPEED_FACTOR, min(speed_factor, 1.0)) # clamp min and max
      
      print("distYawDeg")
      print(distGoalCurrentYawDeg)
      print("current prev at the start deg")
      print(prevYawDeg)
      print("currentDeg")
      print(currentYawDeg)
      print("goalYaw")
      print(goalYawDeg)
      # print(self.normolizeAngle(angle))
      # print(distZdeg - abs(self.normolizeAngle(angle)))
      # broken * 2
      if not (currentYawDeg < (goalYawDeg + MIN_ERROR) and currentYawDeg > (goalYawDeg - MIN_ERROR)): # 
        print('rotate!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        print(distGoalCurrentYawDeg - self.normalizeAngle(angle))
        self.publishVelocityCommand(0.0, direction * ROTATE_SPEED * speed_factor )
        return False
      else:
        self.valuePrevOdometry = self.valueOdometry.copy()
        self.publishVelocityCommand(0.0, 0.0)
        return True
      
      # self.publishVelocityCommand(0.0, self.normolizeAngle(angle) )
      
      # print( self.normolizeAngle(angle))
      # self.publishVelocityCommand(0.0, self.normolizeAngle(angle) * 0.0174532925)
       
    def moveByCenti(self, length: int) -> bool:
      MOVE_SPEED = 0.15
      MIN_SPEED_FACTOR = 0.4
      
      prevPos = self.valuePrevOdometry['position']
      currentPos  = self.valueOdometry['position']
      
      distCenti = self.distCentiFromPoint(prevPos,currentPos) * 100.0
      
      
      print("distCenti")
      print(distCenti)
      
      # speed_factor = (length - distCenti) / length+0.1
      # speed_factor = MIN_SPEED_FACTOR if speed_factor < MIN_SPEED_FACTOR else speed_factor
      speed_factor = 1.0
      
      if distCenti < length:
        self.publishVelocityCommand(MOVE_SPEED * speed_factor, 0.0)
        return False
      else:
        self.valuePrevOdometry = self.valueOdometry.copy()
        self.publishVelocityCommand(0.0, 0.0)
        return True



    def rotateToNearestObj(self):
      lidar = self.valueLaserRaw["ranges"]
      
      first_half = lidar[:180]
      second_half = lidar[180:]
      second_half.reverse()
      

      
      first_half = [x if x != 0.0 else 100.0 for x in first_half]
      second_half = [x if x != 0.0 else 100.0 for x in second_half]
      # min_id_first_half = list(map(lambda x: x if x != 0.0 else x + 100.0, min_id_first_half))
      # min_id_second_half = list(map(lambda x: x if x != 0.0 else x + 100.0, min_id_second_half))

      
      min_id_first_half = first_half.index(min(first_half))
      min_id_second_half = second_half.index(min(second_half))
      
        
      if first_half[min_id_first_half] > second_half[min_id_second_half]:
        # rotate to the left
        rotate_factor = -1
      else:
        rotate_factor = 1

      print(f"Min First angle  {min_id_first_half}")
      print(f"Min Second angle {min_id_second_half}")
      print(f"Min First angle dist  {first_half[min_id_first_half]}")
      print(f"Min Second angledist {second_half[min_id_second_half]}")
      

      if  abs(first_half[min_id_first_half] - second_half[min_id_second_half]) > 0.001:
        print("ROTATE")
        self.publishVelocityCommand(0.0, (rotate_factor * 3.5 * min(min_id_first_half+1.7, min_id_second_half+1.7))/180)
      else:
        print("no rotate")
        self.publishVelocityCommand(0.0, 0.0)
    
    def ratioedAngle(self,  angle):
      ratio = len(self.valueLaserRaw.ranges) / 360
      
      return int(angle * ratio)
      

    def dumbwandererv2(self):
      print("immadebug")
      print(self.valueLaserRaw)
      
      vision_r = self.valueLaserRaw['ranges'][:20]
      vision_l = self.valueLaserRaw['ranges'][340:]
      print("immadebug2")
      
      LENGTH = 0.35
      
      for val in vision_r:
        if val < LENGTH and val > 0.0:
          return self.publishVelocityCommand(0.0, 0.0)
      
      for val in vision_l:
        if val < LENGTH and val > 0.0:
          return self.publishVelocityCommand(0.0, 0.0)
        
      
      print("immadebug3")
      
      return self.publishVelocityCommand(0.2, 0.0)
    
    
    def smartwanderer(self):
      laser = self.valueLaserRaw['ranges']
      front, frontright, right, br,  b, bl, left, frontleft = laser[0], laser[45], laser[90], laser[135], laser[180], laser[225], laser[270], laser[315]

      far_range = 0.3
      close_range = 0.2
      
      BASE_SPEED = 0.07

      # move 
      
      # front close
      if front > close_range and front < far_range and left > 0 and left < far_range:  
        return self.publishVelocityCommand(BASE_SPEED*0.25, self.degToRad(100))
      elif front > close_range and front < far_range and right > 0 and right < far_range:
        return self.publishVelocityCommand(BASE_SPEED*0.25, self.degToRad(-100))
      elif front > close_range and front < far_range and frontleft > 0 and frontleft < far_range:
        return self.publishVelocityCommand(BASE_SPEED*0.25, self.degToRad(100))
      elif front > close_range and front < far_range and frontright > 0 and frontright < far_range:
        return self.publishVelocityCommand(BASE_SPEED*0.25, self.degToRad(-100))
      # reverse
      elif front > close_range and front < far_range:
        return self.publishVelocityCommand(BASE_SPEED, self.degToRad(-100))
      elif front > far_range or front == 0 and left > far_range and right > far_range:
        return self.publishVelocityCommand(BASE_SPEED * 2.25, 0.0)   
      # left right
      elif (frontleft < close_range and frontleft > 0):
        print("fleft too close")
        return self.publishVelocityCommand(-BASE_SPEED * 0.2, self.degToRad(25))   
      elif (frontright < close_range and frontright > 0):
        print("fright too close")
        return self.publishVelocityCommand(-BASE_SPEED * 0.2, -self.degToRad(25))  
      elif (left < close_range and left > 0):
        print("left too close")
        return self.publishVelocityCommand(BASE_SPEED * 0.2, self.degToRad(25))   
      elif right < close_range and right > 0:
        print("right too close")
        return self.publishVelocityCommand(BASE_SPEED * 0.2, -self.degToRad(25))   
      
      elif front < close_range:
        return self.publishVelocityCommand(BASE_SPEED * -0.75, 0.0)
      pass
    
    def minDistofCone(self, cone) -> float:
      min_val = 999999999999.0
      # print("calc Min Dist")
      for val in cone:
        if val < min_val and val != 0.0:
          min_val = val
          
      return min_val
    
    def whatDoISee(self):
      print("what do i see!")
      ls = self.valueLaserRaw["ranges"]
      # print("check1")
      
      # Front, Left, Back, Right = self.minDistofCone(ls[330:] + ls[:30]), self.minDistofCone(ls[60:120]), self.minDistofCone(ls[150:210]), self.minDistofCone(ls[240:300])
      Front, Left, Back, Right = self.minDistofCone(ls[345:] + ls[:15]), self.minDistofCone(ls[75:105]), self.minDistofCone(ls[165:195]), self.minDistofCone(ls[255:285])


      textFront, textRight, textBack, textLeft = "\033[0;0m", "\033[0;0m", "\033[0;0m", "\033[0;0m"
      
      
      # DIST_CONE = 0.2
      DIST_CONE = 0.25
      
      
      if Front < DIST_CONE:
        textFront = "\033[0;31m"
        
        
      if Right < DIST_CONE:
        textRight = "\033[0;31m"
          
      if Back < DIST_CONE:
        textBack = "\033[0;31m"
        
      if Left < DIST_CONE:
        textLeft = "\033[0;31m"
        
      # print("check3")
      
      # print(f"{textFront}F {textRight}L {textBack}B {textLeft}R \033[0;0m")
      print(f'''
            {textFront}{"-"*6}\033[0;0m
            {textLeft}|{" "*4}{textRight}|\033[0;0m
            {textLeft}|{" "*1}\033[0;0m{"^"}{"^ "}{textRight}|\033[0;0m
            {textLeft}|{" "*4}{textRight}|\033[0;0m
            {textBack}{"-"*6}\033[0;0m
            
            ''')
      
      return Front < DIST_CONE, Left < DIST_CONE, Back < DIST_CONE, Right < DIST_CONE
    
    
    def corridorFollow(self):
      ls = self.valueLaserRaw["ranges"]
      Front, Left, Back , Right = self.whatDoISee()
      LeftDist, RightDist = self.minDistofCone(ls[60:120]), self.minDistofCone(ls[240:300])
      
      # for assignment 2
      # BASE_SPEED = 0.07
      # TURN_SPEED = self.degToRad(4.6)  
      
      # for go to next node
      BASE_SPEED = 0.07
      TURN_SPEED = self.degToRad(4.3)
      THRESHOLD = 0.04
      
      print("Left Dist",LeftDist)
      print("Right Dist",RightDist)
      
      
      turnSpecial = -TURN_SPEED if RightDist > LeftDist+THRESHOLD else TURN_SPEED
      
      if not Front:
        if RightDist > LeftDist+THRESHOLD and Left:
          print("Turn Right")
          self.publishVelocityCommand(BASE_SPEED, -TURN_SPEED *  (LeftDist/RightDist))
        elif LeftDist > RightDist+THRESHOLD and Right:
          print("Turn Left")
          self.publishVelocityCommand(BASE_SPEED, TURN_SPEED * (RightDist/LeftDist))
        else:
          print("Forward")
          self.publishVelocityCommand(BASE_SPEED, 0.0)
      else:
        self.publishVelocityCommand(0.002, turnSpecial*1.35)
        pass
      
    def check_lidar_to_stop(self):
      # what do I see to check bot should stop.
      Front, Left, Back , Right = self.whatDoISee()
      forceStop: bool = Front 
      print("check if should stop")
      
      if self.gtnn.node_moved >= self.gtnn.node_goal: # if the bot moved enough and found a wall in front
        print("reach goal")
        
        self.gtnn.node_goal = 0
      elif forceStop:
        print("force stop")
        self.staticMove.clear()
        # reset current goal
        self.gtnn.node_goal = 0 
      print(self.gtnn.node_moved)
      return True
    
    def handling_gtnn(self, n_node: int) -> bool:
      # print("started")
      # reset current moved and goal
      self.gtnn.node_moved = 0
      self.gtnn.node_goal = n_node
      inserted_script: List[Action] = n_node * [self.SMove.To(50), self.Checku.By()]
      # print("inserted")
      
      self.staticMove.pop(0) # pop GTNN out before adding new task
      
      self.staticMove = inserted_script + self.staticMove
      print(self.staticMove)
      
      return False
    
    def special_move_by_centi(self, length: int) -> bool:
      
      ls = self.valueLaserRaw["ranges"]
      
      state: bool = self.moveByCentiCorridorFollow(length)
      if state:
        self.gtnn.node_moved += 1
      
      return state
    
    def moveByCentiCorridorFollow(self, length: int) -> bool:
      '''
        Corridor follow center path but follow the lenght like moveByCenti
      '''
      ls = self.valueLaserRaw["ranges"]
      Front, Left, Back , Right = self.whatDoISee()
      LeftDist, RightDist = self.minDistofCone(ls[60:120]), self.minDistofCone(ls[240:300])
      
      prevPos = self.valuePrevOdometry['position']
      currentPos  = self.valueOdometry['position']
      
      distCenti = self.distCentiFromPoint(prevPos,currentPos) * 100.0
      
      
      BASE_SPEED = 0.04
      TURN_SPEED = self.degToRad(4.3)
      THRESHOLD = 0.04
      
      print("Left Dist",LeftDist)
      print("Right Dist",RightDist)
      
      if distCenti < length and not Front:
        if RightDist > LeftDist+THRESHOLD and Left:
          print("Turn Right")
          self.publishVelocityCommand(BASE_SPEED, -TURN_SPEED *  (LeftDist/RightDist))
        elif LeftDist > RightDist+THRESHOLD and Right:
          print("Turn Left")
          self.publishVelocityCommand(BASE_SPEED, TURN_SPEED * (RightDist/LeftDist))
        else:
          print("Forward")
          self.publishVelocityCommand(BASE_SPEED, 0.0)
        return False
      else:
        self.valuePrevOdometry = self.valueOdometry.copy()
        return True
      
      
      

    def scanCallback(self, msg):
        lidar_ranges = list(msg.ranges)
        # print(lidar_ranges)
        self.valueLaserRaw = {
            'range_min':msg.range_min,
            'range_max':msg.range_max,
            'ranges': lidar_ranges,
        }
        
        ## 0.1
        # self.dumbwandererv2()
        
        ## 0.2
        # self.rotateToNearestObj()
        
        # # smart
        # self.smartwanderer()
        
        
        
        # #
        # self.whatDoISee()
        # self.corridorFollow()

    def batteryStateCallback(self, msg):
        self.valueBatteryState = msg

    def odomCallback(self, msg):
        # print("ODOM callback---------------")
        self.valueOdometry = {
            'position':msg.pose.pose.position,
            'orientation':msg.pose.pose.orientation,
            'linearVelocity':msg.twist.twist.linear,
            'angularVelocity':msg.twist.twist.angular,
            'yaw': self.get_yaw(msg.pose.pose.orientation),
            'yawDegree': self.radToDeg(self.get_yaw(msg.pose.pose.orientation))
        }
        # print(self.valueOdometry)
        
        if self.valuePrevOdometry['position'] == None:
          print("INIT____________")
          print(self.valueOdometry)
          print("INIT____________")
          self.valuePrevOdometry = self.valueOdometry.copy()
          
          

        if (len(self.staticMove) != 0):
          current_move = self.staticMove[0]
          # print(current_move)
          if isinstance(current_move, self.OdomAllowActions):
            # print("odom_move")
            self.moveIsComplete = current_move.execute()
            print("odom_move_execute")
            if self.moveIsComplete:
              print("next_move")
              self.prevMoveIsComplete = True
              self.moveIsComplete = False
              if len(self.staticMove) > 0:
                self.staticMove.pop(0)
                print("pop")
        else:
          self.publishVelocityCommand(0.0,0.0)
          
            
          

    def timerCallback(self):
      # print("-----------------------------------------------------------")
      # print('timer triggered')
      # print("OdomValue")
      # print(self.valueOdometry)
      
      print(self.staticMove)
      if (len(self.staticMove) != 0):
        current_move = self.staticMove[0]
        # special case for turn -> robot cant turn that quickly
        if isinstance(current_move, self.Turn.__class__): 
          # unpacking the target angle from args
          ( targetAngle, ) =  current_move.args
          targetAngle = self.normalizeAngle(targetAngle)
          # pop the current move
          print("targetAngle is " + str(targetAngle))
          # if the target angle is greater than 0
          if targetAngle > 90 or targetAngle < -90:
            factor = 1 if targetAngle > 0 else -1
            targetAngle = abs(targetAngle)
            self.staticMove.pop(0)
            while targetAngle > 0:
              if targetAngle > 90:
                print("adding 90 degree")
                targetAngle -= 90
                self.staticMove.insert(0, self.Turn.To(90 * factor))
              else:
                print("adding " + str(targetAngle) + " degree")
                
                self.staticMove.insert(0, self.Turn.To(targetAngle * factor))
                targetAngle = 0
            # resetting to the first new move
            current_move = self.staticMove[0]
        print("current_move is " + str(type(current_move)))
        
        # execute the move
        if isinstance(current_move, self.TimerAllowActions):
          if self.prevMoveIsComplete == False:
            self.prevMoveIsComplete = True
            self.moveIsComplete = False
            self.publishVelocityCommand(0.0,0.0) 
            print("next_move")
            if len(self.staticMove) > 0:
              self.staticMove.pop(0)
              print("pop")
          else:
            self.prevMoveIsComplete = False
            self.moveIsComplete = current_move.execute()
      else:
        self.publishVelocityCommand(0.0,0.0)

              
            
        
          

def robotStop():
    node = rclpy.create_node('tb3Stop')
    publisher = node.create_publisher(Twist, 'cmd_vel', 1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    publisher.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)
    tb3ControllerNode = Turtlebot3Controller()
    print('tb3ControllerNode created')
    try:
        rclpy.spin(tb3ControllerNode)
    except Exception as e:
        # adding traceback
        logging.error(traceback.format_exc())
        tb3ControllerNode.publishVelocityCommand(0.0,0.0)
        
        KeyboardInterrupt
    print('Done')
    
    tb3ControllerNode.publishVelocityCommand(0.0,0.0)
    tb3ControllerNode.destroy_node()
    robotStop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
