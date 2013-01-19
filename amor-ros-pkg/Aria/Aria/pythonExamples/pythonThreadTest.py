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
from threading import Thread
from AriaPy import *
import time

# A quick test to make sure that built in Python threads work when the AriaPy library is also loaded
# and initialized.

Aria.init()

# A subclass of Python's thread class, to contain a method that runs in a new thread 

class ExampleThread(Thread):

  def __init__(self):
    Thread.__init__(self)
    print "init"

  def run(self):
    while 1:
      print "thread!"
      time.sleep(2)

exampleThread = ExampleThread()

# Launch the new thread in the background. This thread (of main()) continues
# immediately.

exampleThread.start()

while 1:
  print "main!"
  time.sleep(1.5)

