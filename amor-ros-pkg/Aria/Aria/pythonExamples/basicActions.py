"""
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
"""
from AriaPy import *
import sys

# Wander around, avoiding obstacles, using some ArActions predefined in ARIA.

Aria.init()
robot = ArRobot()
conn = ArSimpleConnector(sys.argv)


# Every robot has sonar:
print "Creating sonar object..."
sonar = ArSonarDevice()
robot.addRangeDevice(sonar)

# Some robots have laser rangefinders:
print "Creating SICK laser object..."
laser = ArSick()

# Run laser processing thread in the background
laser.runAsync()

# Add laser as a possible range device for other components to use. 
# It won't provide any data until we connect to the laser though...
robot.addRangeDevice(laser)

# Connect to the robot:
print "Connecting to the robot..."
if (not conn.connectRobot(robot)):
    print "Could not connect to robot, exiting"
    Aria.exit(1)


# Try to connect to the laser
print "Connecting to the laser..."
if not laser.blockingConnect():
  print "Error connecting to laser range finder!"




# Add actions to ArRobot.  While running the robot's action resolver will
# determine motion commands by evaluating the actions in order from lowest
# number to highest (so lower order actions' desired motion commands can
# supercede higher)

print "Adding actions..."
print "   1. StallRecover"
stallRecover = ArActionStallRecover()
robot.addAction(stallRecover, 1)
print "   2. AvoidFront"
avoid = ArActionAvoidFront()
robot.addAction(avoid, 2)
print "   3. LimitFront"
limitFront = ArActionLimiterForwards("limitFront", 300, 600, 250)
robot.addAction(limitFront, 3)
print "   4. LimitBack"
limitBack = ArActionLimiterBackwards()
robot.addAction(limitBack, 4)
print "   5. ConstantVelocity"
constVel = ArActionConstantVelocity()
robot.addAction(constVel, 5)


# Run robot thread here in the main thread. 
# 1 (=true) makes the function exit if the robot connection goes away unexpectedly.
print "Running robot..."
robot.enableMotors()
robot.run(1)

print "goodbye."
