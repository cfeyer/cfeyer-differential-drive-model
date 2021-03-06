#!/usr/bin/python
#
# MIT License
#
# Copyright (c) 2017 Chris Feyerchak
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import unittest
from math import pi, sin, cos, sqrt

class Position_Heading_Coordinates:

   def __init__(self, x0=0, y0=0, heading0_rads=0):
      self.x = float(x0)
      self.y = float(y0)
      self.hdg_rads = float(heading0_rads) # 0 = east

   def __eq__(self, other):
      dx = other.x - self.x
      dy = other.y - self.y
      dh = other.hdg_rads - self.hdg_rads
      #print "   self: " + str(self)
      #print "   other: " + str(other)
      #print "   dx=" + str(dx) + ", dy=" + str(dy) + ", dh=" + str(dh)
      tol = 1.0e-14
      return (abs(dx) <= tol) and (abs(dy) <= tol) and (abs(dh) <= tol)

   def __ne__(self, other):
      return not self.__eq__(other)

   def __str__(self):
      return "x=" + str(self.x) + ", y=" + str(self.y) + ", h=" + str(self.hdg_rads)


class Differential_Drive:

   def __init__(self, track_width):
      self.track_width = float(track_width)

   def drive(self, initial_coords, left_travel, right_travel):
      final_coords = initial_coords

      if(left_travel == right_travel): # stopped or driving straight
         final_coords.x = initial_coords.x + cos(initial_coords.hdg_rads) * float(left_travel)
         final_coords.y = initial_coords.y + sin(initial_coords.hdg_rads) * float(left_travel)

      elif(left_travel == -right_travel): # rotate in place
         final_coords.hdg_rads = right_travel / self.track_width + initial_coords.hdg_rads

      elif(left_travel == 0): # rotate about left wheel
         turn_angle_rads = right_travel/self.track_width
         lw = self.left_wheel_coords(initial_coords)
         lw_x = lw.x
         lw_y = lw.y
         phi = turn_angle_rads
         x1 = initial_coords.x - lw_x
         y1 = initial_coords.y - lw_y
         x2 = cos(phi)*x1-sin(phi)*y1
         y2 = sin(phi)*x1+cos(phi)*y1
         final_coords.x = x2 + lw_x
         final_coords.y = y2 + lw_y
         final_coords.hdg_rads = initial_coords.hdg_rads + turn_angle_rads

      elif(right_travel == 0): # rotate about right wheel
         turn_angle_rads = -left_travel/self.track_width
         rw = self.right_wheel_coords(initial_coords)
         rw_x = rw.x
         rw_y = rw.y
         phi = turn_angle_rads
         x1 = initial_coords.x - rw_x
         y1 = initial_coords.y - rw_y
         x2 = cos(phi)*x1-sin(phi)*y1
         y2 = sin(phi)*x1+cos(phi)*y1
         final_coords.x = x2 + rw_x
         final_coords.y = y2 + rw_y
         final_coords.hdg_rads = initial_coords.hdg_rads + turn_angle_rads

      else:
         raise Exception("Not implemented for this (left_travel, right_travel)")

      return final_coords

   def left_wheel_coords(self, p):
      x = -(self.track_width/2.0) * sin(p.hdg_rads) + p.x
      y = (self.track_width/2.0) * cos(p.hdg_rads) + p.y
      h = p.hdg_rads
      return Position_Heading_Coordinates(x,y,h)

   def right_wheel_coords(self, p):
      x = (self.track_width/2.0) * sin(p.hdg_rads) + p.x
      y = -(self.track_width/2.0) * cos(p.hdg_rads) + p.y
      h = p.hdg_rads
      return Position_Heading_Coordinates(x,y,h)


PHC = Position_Heading_Coordinates
DD = Differential_Drive


class Test_PHC(unittest.TestCase):

   def test_default_ctor(self):
      coords = PHC()
      self.assertEqual(coords.x, 0)
      self.assertEqual(coords.y, 0)
      self.assertEqual(coords.hdg_rads, 0)

   def test_custom_ctor(self):
      coords = PHC(1,2,3)
      self.assertEqual(coords.x, 1)
      self.assertEqual(coords.y, 2)
      self.assertEqual(coords.hdg_rads, 3)

   def test_default_objects_equal(self):
      coords_1 = PHC()
      coords_2 = PHC()
      self.assertEqual(coords_1, coords_2)

   def test_custom_objects_equal(self):
      coords_1 = PHC(1,2,3)
      coords_2 = PHC(1,2,3)
      self.assertEqual(coords_1, coords_2)

   def test_custom_objects_x_not_equal(self):
      coords_1 = PHC(0,0,0)
      coords_2 = PHC(1,0,0)
      self.assertNotEqual(coords_1, coords_2)
      self.assertNotEqual(coords_2, coords_1)

   def test_custom_objects_y_not_equal(self):
      coords_1 = PHC(0,0,0)
      coords_2 = PHC(0,1,0)
      self.assertNotEqual(coords_1, coords_2)
      self.assertNotEqual(coords_2, coords_1)
      self.assertNotEqual(PHC(0,0,0), PHC(0,0,pi/2))
      self.assertNotEqual(PHC(0,0,0), PHC(0,0,pi))
      self.assertNotEqual(PHC(0,0,pi/2), PHC(0,0,pi))

   def test_custom_objects_heading_not_equal(self):
      coords_1 = PHC(0,0,0)
      coords_2 = PHC(0,0,1)
      self.assertNotEqual(coords_1, coords_2)
      self.assertNotEqual(coords_2, coords_1)

class Position_Heading_Coordinates_Test_Case(unittest.TestCase):

   def assertEqualPHC(self, actual, expected):
      x_error = abs(actual.x - expected.x)
      y_error = abs(actual.y - expected.y)
      heading_error = abs(actual.hdg_rads - expected.hdg_rads)
      tol = 1.0e-14
      msg = "expected: " + str(expected) + "\n" + \
            "actual: " + str(actual)
      is_fail = False
      if x_error > tol:
         is_fail = True
         msg += "\nassertion fail: x"
      if y_error > tol:
         is_fail = True
         msg += "\nassertion fail: y"
      if heading_error > tol:
         is_fail = True
         msg += "\nassertion fail: hdg_rads"

      if is_fail:
	self.fail(msg)
      

class Test_DD(Position_Heading_Coordinates_Test_Case):

   def test_stopped(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(), 0, 0), PHC())
      self.assertEqual(dd.drive(PHC(1,2,3), 0, 0), PHC(1,2,3))
      pass

   def test_drive_forward_straight_east(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,0), 1, 1), PHC(1,0,0))
      self.assertEqual(dd.drive(PHC(1,2,0), 2, 2), PHC(3,2,0))
      self.assertEqual(dd.drive(PHC(-3,4,0), 4, 4), PHC(1,4,0))
      self.assertEqual(dd.drive(PHC(-5,-6,0), 8, 8), PHC(3,-6,0))
      self.assertEqual(dd.drive(PHC(7,-8,0), 16, 16), PHC(23,-8,0))

   def test_drive_backward_straight_east(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,pi), -1, -1), PHC(1,0,pi))
      self.assertEqual(dd.drive(PHC(1,2,pi), -2, -2), PHC(3,2,pi))
      self.assertEqual(dd.drive(PHC(-3,4,pi), -4, -4), PHC(1,4,pi))
      self.assertEqual(dd.drive(PHC(-5,-6,pi), -8, -8), PHC(3,-6,pi))
      self.assertEqual(dd.drive(PHC(7,-8,pi), -16, -16), PHC(23,-8,pi))

   def test_drive_forward_straight_west(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,pi), 1, 1), PHC(-1,0,pi))
      self.assertEqual(dd.drive(PHC(1,2,pi), 2, 2), PHC(-1,2,pi))
      self.assertEqual(dd.drive(PHC(-3,4,pi), 4, 4), PHC(-7,4,pi))
      self.assertEqual(dd.drive(PHC(-5,-6,pi), 8, 8), PHC(-13,-6,pi))
      self.assertEqual(dd.drive(PHC(7,-8,pi), 16, 16), PHC(-9,-8,pi))

   def test_drive_backward_straight_west(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,0), -1, -1), PHC(-1,0,0))
      self.assertEqual(dd.drive(PHC(1,2,0), -2, -2), PHC(-1,2,0))
      self.assertEqual(dd.drive(PHC(-3,4,0), -4, -4), PHC(-7,4,0))
      self.assertEqual(dd.drive(PHC(-5,-6,0), -8, -8), PHC(-13,-6,0))
      self.assertEqual(dd.drive(PHC(7,-8,0), -16, -16), PHC(-9,-8,0))

   def test_drive_forward_straight_north(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,pi/2), 1, 1), PHC(0,1,pi/2))
      self.assertEqual(dd.drive(PHC(1,2,pi/2), 2, 2), PHC(1,4,pi/2))
      self.assertEqual(dd.drive(PHC(-3,4,pi/2), 4, 4), PHC(-3,8,pi/2))
      self.assertEqual(dd.drive(PHC(-5,-6,pi/2), 8, 8), PHC(-5,2,pi/2))
      self.assertEqual(dd.drive(PHC(7,-8,pi/2), 16, 16), PHC(7,8,pi/2))

   def test_drive_backward_straight_north(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,-pi/2), -1, -1), PHC(0,1,-pi/2))
      self.assertEqual(dd.drive(PHC(1,2,-pi/2), -2, -2), PHC(1,4,-pi/2))
      self.assertEqual(dd.drive(PHC(-3,4,-pi/2), -4, -4), PHC(-3,8,-pi/2))
      self.assertEqual(dd.drive(PHC(-5,-6,-pi/2), -8, -8), PHC(-5,2,-pi/2))
      self.assertEqual(dd.drive(PHC(7,-8,-pi/2), -16, -16), PHC(7,8,-pi/2))

   def test_drive_forward_straight_south(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,-pi/2), 1, 1), PHC(0,-1,-pi/2))
      self.assertEqual(dd.drive(PHC(1,2,-pi/2), 2, 2), PHC(1,0,-pi/2))
      self.assertEqual(dd.drive(PHC(-3,4,-pi/2), 4, 4), PHC(-3,0,-pi/2))
      self.assertEqual(dd.drive(PHC(-5,-6,-pi/2), 8, 8), PHC(-5,-14,-pi/2))
      self.assertEqual(dd.drive(PHC(7,-8,-pi/2), 16, 16), PHC(7,-24,-pi/2))

   def test_drive_backward_straight_south(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,pi/2), -1, -1), PHC(0,-1,pi/2))
      self.assertEqual(dd.drive(PHC(1,2,pi/2), -2, -2), PHC(1,0,pi/2))
      self.assertEqual(dd.drive(PHC(-3,4,pi/2), -4, -4), PHC(-3,0,pi/2))
      self.assertEqual(dd.drive(PHC(-5,-6,pi/2), -8, -8), PHC(-5,-14,pi/2))
      self.assertEqual(dd.drive(PHC(7,-8,pi/2), -16, -16), PHC(7,-24,pi/2))

   def test_drive_forward_straight_northeast(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,pi/4), 1, 1), PHC(sqrt(2)/2,sqrt(2)/2,pi/4))
      self.assertEqual(dd.drive(PHC(0,0,pi/6), 1, 1), PHC(sqrt(3)/2,0.5,pi/6))
      self.assertEqual(dd.drive(PHC(0,0,pi/3), 1, 1), PHC(0.5,sqrt(3)/2,pi/3))
      self.assertEqual(dd.drive(PHC(1,2,pi/4), 2, 2), PHC(1+2*sqrt(2)/2,2+2*sqrt(2)/2,pi/4))
      self.assertEqual(dd.drive(PHC(-3,4,pi/4), 4, 4), PHC(-3+4*sqrt(2)/2,4+4*sqrt(2)/2,pi/4))
      self.assertEqual(dd.drive(PHC(-5,-6,pi/4), 8, 8), PHC(-5+8*sqrt(2)/2,-6+8*sqrt(2)/2,pi/4))
      self.assertEqual(dd.drive(PHC(7,-8,pi/4), 16, 16), PHC(7+16*sqrt(2)/2,-8+16*sqrt(2)/2,pi/4))

   def test_drive_backward_straight_northeast(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,5*pi/4), -1, -1), PHC(sqrt(2)/2,sqrt(2)/2,5*pi/4))
      self.assertEqual(dd.drive(PHC(0,0,7*pi/6), -1, -1), PHC(sqrt(3)/2,0.5,7*pi/6))
      self.assertEqual(dd.drive(PHC(0,0,4*pi/3), -1, -1), PHC(0.5,sqrt(3)/2,4*pi/3))
      self.assertEqual(dd.drive(PHC(1,2,5*pi/4), -2, -2), PHC(1+2*sqrt(2)/2,2+2*sqrt(2)/2,5*pi/4))
      self.assertEqual(dd.drive(PHC(-3,4,5*pi/4), -4, -4), PHC(-3+4*sqrt(2)/2,4+4*sqrt(2)/2,5*pi/4))
      self.assertEqual(dd.drive(PHC(-5,-6,5*pi/4), -8, -8), PHC(-5+8*sqrt(2)/2,-6+8*sqrt(2)/2,5*pi/4))
      self.assertEqual(dd.drive(PHC(7,-8,5*pi/4), -16, -16), PHC(7+16*sqrt(2)/2,-8+16*sqrt(2)/2,5*pi/4))

   def test_drive_forward_straight_northwest(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,3*pi/4), 1, 1), PHC(-sqrt(2)/2,sqrt(2)/2,3*pi/4))
      self.assertEqual(dd.drive(PHC(0,0,5*pi/6), 1, 1), PHC(-sqrt(3)/2,0.5,5*pi/6))
      self.assertEqual(dd.drive(PHC(0,0,2*pi/3), 1, 1), PHC(-0.5,sqrt(3)/2,2*pi/3))

   def test_drive_backward_straight_northwest(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,7*pi/4), -1, -1), PHC(-sqrt(2)/2,sqrt(2)/2,7*pi/4))
      self.assertEqual(dd.drive(PHC(0,0,11*pi/6), -1, -1), PHC(-sqrt(3)/2,0.5,11*pi/6))
      self.assertEqual(dd.drive(PHC(0,0,5*pi/3), -1, -1), PHC(-0.5,sqrt(3)/2,5*pi/3))

   def test_drive_forward_straight_southwest(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,5*pi/4), 1, 1), PHC(-sqrt(2)/2,-sqrt(2)/2,5*pi/4))

   def test_drive_backward_straight_southwest(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,pi/4), -1, -1), PHC(-sqrt(2)/2,-sqrt(2)/2,pi/4))

   def test_drive_forward_straight_southeast(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,-pi/4), 1, 1), PHC(sqrt(2)/2,-sqrt(2)/2,-pi/4))

   def test_drive_backward_straight_southeast(self):
      dd = DD(1)
      self.assertEqual(dd.drive(PHC(0,0,3*pi/4), -1, -1), PHC(sqrt(2)/2,-sqrt(2)/2,3*pi/4))

   def test_rotate_in_place_counterclockwise_from_zero_heading(self):
      dd1 = DD(1)
      self.assertEqual(dd1.drive(PHC(0,0,0), -pi/2, pi/2), PHC(0,0,pi/2))
      self.assertEqual(dd1.drive(PHC(1,2,0), -pi, pi), PHC(1,2,pi))
      self.assertEqual(dd1.drive(PHC(3,4,0), -3*pi/2, 3*pi/2), PHC(3,4,3*pi/2))

      dd2 = DD(2)
      self.assertEqual(dd2.drive(PHC(0,0,0), -2*pi/2,   2*pi/2  ), PHC(0,0,pi/2))
      self.assertEqual(dd2.drive(PHC(1,2,0), -2*pi,     2*pi    ), PHC(1,2,pi))
      self.assertEqual(dd2.drive(PHC(3,4,0), -2*3*pi/2, 2*3*pi/2), PHC(3,4,3*pi/2))

   def test_rotate_in_place_clockwise_from_zero_heading(self):
      dd1 = DD(1)
      self.assertEqual(dd1.drive(PHC(0,0,0), pi/2, -pi/2), PHC(0,0,-pi/2))
      self.assertEqual(dd1.drive(PHC(1,2,0), pi, -pi), PHC(1,2,-pi))
      self.assertEqual(dd1.drive(PHC(3,4,0), 3*pi/2, -3*pi/2), PHC(3,4,-3*pi/2))

      dd2 = DD(2)
      self.assertEqual(dd2.drive(PHC(0,0,0), 2*pi/2,   -2*pi/2  ), PHC(0,0,-pi/2))
      self.assertEqual(dd2.drive(PHC(1,2,0), 2*pi,     -2*pi    ), PHC(1,2,-pi))
      self.assertEqual(dd2.drive(PHC(3,4,0), 2*3*pi/2, -2*3*pi/2), PHC(3,4,-3*pi/2))

   def test_rotate_in_place_counterclockwise_from_nonzero_heading(self):
      dd1 = DD(1)
      self.assertEqual(dd1.drive(PHC(0,0,1), -pi/2, pi/2), PHC(0,0,pi/2+1))
      self.assertEqual(dd1.drive(PHC(1,2,2), -pi, pi), PHC(1,2,pi+2))
      self.assertEqual(dd1.drive(PHC(3,4,3), -3*pi/2, 3*pi/2), PHC(3,4,3*pi/2+3))

      dd2 = DD(2)
      self.assertEqual(dd2.drive(PHC(0,0,-1), -2*pi/2,   2*pi/2  ), PHC(0,0,pi/2-1))
      self.assertEqual(dd2.drive(PHC(1,2,-2), -2*pi,     2*pi    ), PHC(1,2,pi-2))
      self.assertEqual(dd2.drive(PHC(3,4,-3), -2*3*pi/2, 2*3*pi/2), PHC(3,4,3*pi/2-3))

   def test_rotate_in_place_clockwise_from_zero_nonheading(self):
      dd1 = DD(1)
      self.assertEqual(dd1.drive(PHC(0,0,1), pi/2, -pi/2), PHC(0,0,-pi/2+1))
      self.assertEqual(dd1.drive(PHC(1,2,2), pi, -pi), PHC(1,2,-pi+2))
      self.assertEqual(dd1.drive(PHC(3,4,3), 3*pi/2, -3*pi/2), PHC(3,4,-3*pi/2+3))

      dd2 = DD(2)
      self.assertEqual(dd2.drive(PHC(0,0,-1), 2*pi/2,   -2*pi/2  ), PHC(0,0,-pi/2-1))
      self.assertEqual(dd2.drive(PHC(1,2,-2), 2*pi,     -2*pi    ), PHC(1,2,-pi-2))
      self.assertEqual(dd2.drive(PHC(3,4,-3), 2*3*pi/2, -2*3*pi/2), PHC(3,4,-3*pi/2-3))

   def test_left_wheel_coords(self):
      dd1 = DD(1)
      self.assertEqual(dd1.left_wheel_coords(PHC(0,0,0)),           PHC(0,0.5,0))
      self.assertEqual(dd1.left_wheel_coords(PHC(0,0,pi/2)),   PHC(-0.5,0,pi/2))
      self.assertEqual(dd1.left_wheel_coords(PHC(0,0,pi)),     PHC(0,-0.5,pi))
      self.assertEqual(dd1.left_wheel_coords(PHC(0,0,3*pi/2)), PHC(0.5,0,3*pi/2))

      self.assertEqual(dd1.left_wheel_coords(PHC(1,2,0)),           PHC(0+1,0.5+2,0))
      self.assertEqual(dd1.left_wheel_coords(PHC(-3,4,pi/2)),   PHC(-0.5-3,0+4,pi/2))
      self.assertEqual(dd1.left_wheel_coords(PHC(-5,-6,pi)),     PHC(0-5,-0.5-6,pi))
      self.assertEqual(dd1.left_wheel_coords(PHC(7,-8,3*pi/2)), PHC(0.5+7,0-8,3*pi/2))

      self.assertEqualPHC(dd1.left_wheel_coords(PHC(1,-1.5,0)), PHC(1,-1.0,0))

      dd2 = DD(2)
      self.assertEqual(dd2.left_wheel_coords(PHC(0,0,0)),           PHC(0,1,0))
      self.assertEqual(dd2.left_wheel_coords(PHC(0,0,pi/2)),   PHC(-1,0,pi/2))
      self.assertEqual(dd2.left_wheel_coords(PHC(0,0,pi)),     PHC(0,-1,pi))
      self.assertEqual(dd2.left_wheel_coords(PHC(0,0,3*pi/2)), PHC(1,0,3*pi/2))

      self.assertEqual(dd2.left_wheel_coords(PHC(1,2,0)),           PHC(0+1,1+2,0))
      self.assertEqual(dd2.left_wheel_coords(PHC(-3,4,pi/2)),   PHC(-1-3,0+4,pi/2))
      self.assertEqual(dd2.left_wheel_coords(PHC(-5,-6,pi)),     PHC(0-5,-1-6,pi))
      self.assertEqual(dd2.left_wheel_coords(PHC(7,-8,3*pi/2)), PHC(1+7,0-8,3*pi/2))

   def test_right_wheel_coords(self):
      dd1 = DD(1)
      self.assertEqual(dd1.right_wheel_coords(PHC(0,0,0)),           PHC(0,-0.5,0))
      self.assertEqual(dd1.right_wheel_coords(PHC(0,0,pi/2)),   PHC(0.5,0,pi/2))
      self.assertEqual(dd1.right_wheel_coords(PHC(0,0,pi)),     PHC(0,0.5,pi))
      self.assertEqual(dd1.right_wheel_coords(PHC(0,0,3*pi/2)), PHC(-0.5,0,3*pi/2))

      self.assertEqual(dd1.right_wheel_coords(PHC(1,2,0)),           PHC(0+1,-0.5+2,0))
      self.assertEqual(dd1.right_wheel_coords(PHC(-3,4,pi/2)),   PHC(0.5-3,0+4,pi/2))
      self.assertEqual(dd1.right_wheel_coords(PHC(-5,-6,pi)),     PHC(0-5,0.5-6,pi))
      self.assertEqual(dd1.right_wheel_coords(PHC(7,-8,3*pi/2)), PHC(-0.5+7,0-8,3*pi/2))

      dd2 = DD(2)
      self.assertEqual(dd2.right_wheel_coords(PHC(0,0,0)),           PHC(0,-1,0))
      self.assertEqual(dd2.right_wheel_coords(PHC(0,0,pi/2)),   PHC(1,0,pi/2))
      self.assertEqual(dd2.right_wheel_coords(PHC(0,0,pi)),     PHC(0,1,pi))
      self.assertEqual(dd2.right_wheel_coords(PHC(0,0,3*pi/2)), PHC(-1,0,3*pi/2))

      self.assertEqual(dd2.right_wheel_coords(PHC(1,2,0)),           PHC(0+1,-1+2,0))
      self.assertEqual(dd2.right_wheel_coords(PHC(-3,4,pi/2)),   PHC(1-3,0+4,pi/2))
      self.assertEqual(dd2.right_wheel_coords(PHC(-5,-6,pi)),     PHC(0-5,1-6,pi))
      self.assertEqual(dd2.right_wheel_coords(PHC(7,-8,3*pi/2)), PHC(-1+7,0-8,3*pi/2))

   def test_rotate_about_left_wheel_counterclockwise(self):
      dd1 = DD(1)
      self.assertEqualPHC(dd1.drive(PHC(0,-0.5,0),0,pi/2), PHC(0.5,0,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5,0,pi/2),0,pi/2), PHC(0,0.5,pi))
      self.assertEqualPHC(dd1.drive(PHC(0,0.5,pi),0,pi/2), PHC(-0.5,0,3*pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5,0,3*pi/2),0,pi/2), PHC(0,-0.5,2*pi))
      self.assertEqualPHC(dd1.drive(PHC(0.5,0,pi/2),0,pi/4), PHC(sqrt(2)/4,sqrt(2)/4,3*pi/4))

      self.assertEqualPHC(dd1.drive(PHC(0+1,-0.5+2,0),0,pi/2), PHC(0.5+1,0+2,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5+3,0+4,pi/2),0,pi/2), PHC(0+3,0.5+4,pi))
      self.assertEqualPHC(dd1.drive(PHC(0+5,0.5+6,pi),0,pi/2), PHC(-0.5+5,0+6,3*pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5+7,0+8,3*pi/2),0,pi/2), PHC(0+7,-0.5+8,2*pi))
      self.assertEqualPHC(dd1.drive(PHC(0.5+9,0+10,pi/2),0,pi/4), PHC(sqrt(2)/4+9,sqrt(2)/4+10,3*pi/4))

      dd2 = DD(2)
      self.assertEqualPHC(dd2.drive(PHC(0,-1,0),0,pi), PHC(1,0,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1,0,pi/2),0,pi), PHC(0,1,pi))
      self.assertEqualPHC(dd2.drive(PHC(0,1,pi),0,pi), PHC(-1,0,3*pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1,0,3*pi/2),0,pi), PHC(0,-1,2*pi))
      self.assertEqualPHC(dd2.drive(PHC(1,0,pi/2),0,pi/2), PHC(sqrt(2)/2,sqrt(2)/2,3*pi/4))

      self.assertEqualPHC(dd2.drive(PHC(0+1,-1+2,0),0,pi), PHC(1+1,0+2,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1+3,0+4,pi/2),0,pi), PHC(0+3,1+4,pi))
      self.assertEqualPHC(dd2.drive(PHC(0+5,1+6,pi),0,pi), PHC(-1+5,0+6,3*pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1+7,0+8,3*pi/2),0,pi), PHC(0+7,-1+8,2*pi))
      self.assertEqualPHC(dd2.drive(PHC(1+9,0+10,pi/2),0,pi/2), PHC(sqrt(2)/2+9,sqrt(2)/2+10,3*pi/4))

   def test_rotate_about_left_wheel_clockwise(self):
      dd1 = DD(1)
      self.assertEqualPHC(dd1.drive(PHC(0,-0.5,0),0,-pi/2), PHC(-0.5,0,-pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5,0,pi/2),0,-pi/2), PHC(0,-0.5,0))
      self.assertEqualPHC(dd1.drive(PHC(0,0.5,pi),0,-pi/2), PHC(0.5,0,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5,0,3*pi/2),0,-pi/2), PHC(0,0.5,pi))
      self.assertEqualPHC(dd1.drive(PHC(0.5,0,pi/2),0,-pi/4), PHC(sqrt(2)/4,-sqrt(2)/4,pi/4))

      self.assertEqualPHC(dd1.drive(PHC(0+1,-0.5+2,0),0,-pi/2), PHC(-0.5+1,0+2,-pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5+3,0+4,pi/2),0,-pi/2), PHC(0+3,-0.5+4,0))
      self.assertEqualPHC(dd1.drive(PHC(0+5,0.5+6,pi),0,-pi/2), PHC(0.5+5,0+6,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5+7,0+8,3*pi/2),0,-pi/2), PHC(0+7,0.5+8,pi))
      self.assertEqualPHC(dd1.drive(PHC(0.5+9,0+10,pi/2),0,-pi/4), PHC(sqrt(2)/4+9,-sqrt(2)/4+10,pi/4))

      dd2 = DD(2)
      self.assertEqualPHC(dd2.drive(PHC(0,-1,0),0,-pi), PHC(-1,0,-pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1,0,pi/2),0,-pi), PHC(0,-1,0))
      self.assertEqualPHC(dd2.drive(PHC(0,1,pi),0,-pi), PHC(1,0,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1,0,3*pi/2),0,-pi), PHC(0,1,pi))
      self.assertEqualPHC(dd2.drive(PHC(1,0,pi/2),0,-pi/2), PHC(sqrt(2)/2,-sqrt(2)/2,pi/4))

      self.assertEqualPHC(dd2.drive(PHC(0+1,-1+2,0),0,-pi), PHC(-1+1,0+2,-pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1+3,0+4,pi/2),0,-pi), PHC(0+3,-1+4,0))
      self.assertEqualPHC(dd2.drive(PHC(0+5,1+6,pi),0,-pi), PHC(1+5,0+6,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1+7,0+8,3*pi/2),0,-pi), PHC(0+7,1+8,pi))
      self.assertEqualPHC(dd2.drive(PHC(1+9,0+10,pi/2),0,-pi/2), PHC(sqrt(2)/2+9,-sqrt(2)/2+10,pi/4))

   def test_rotate_about_right_wheel_clockwise(self):
      dd1 = DD(1)
      self.assertEqualPHC(dd1.drive(PHC(0,0.5,0),pi/2,0), PHC(0.5,0,-pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5,0,pi/2),pi/2,0), PHC(0,0.5,0))
      self.assertEqualPHC(dd1.drive(PHC(0,-0.5,pi),pi/2,0), PHC(-0.5,0,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5,0,-pi/2),pi/2,0), PHC(0,-0.5,-pi))
      self.assertEqualPHC(dd1.drive(PHC(-0.5,0,pi/2),pi/4,0), PHC(-sqrt(2)/4,sqrt(2)/4,pi/4))

      self.assertEqualPHC(dd1.drive(PHC(0+1,0+2.5,0),pi/2,0), PHC(0.5+1,0+2,-pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5+3,0+4,pi/2),pi/2,0), PHC(0+3,0+4.5,0))
      self.assertEqualPHC(dd1.drive(PHC(0+5,-0.5+6,pi),pi/2,0), PHC(-0.5+5,0+6,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5+7,0+8,-pi/2),pi/2,0), PHC(0+7,-0.5+8,-pi))
      self.assertEqualPHC(dd1.drive(PHC(-0.5+9,0+10,pi/2),pi/4,0), PHC(-sqrt(2)/4+9,sqrt(2)/4+10,pi/4))

      dd2 = DD(2)
      self.assertEqualPHC(dd2.drive(PHC(0,1,0),pi,0), PHC(1,0,-pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1,0,pi/2),pi,0), PHC(0,1,0))
      self.assertEqualPHC(dd2.drive(PHC(0,-1,pi),pi,0), PHC(-1,0,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1,0,-pi/2),pi,0), PHC(0,-1,-pi))
      self.assertEqualPHC(dd2.drive(PHC(-1,0,pi/2),pi/2,0), PHC(-sqrt(2)/2,sqrt(2)/2,pi/4))

      self.assertEqualPHC(dd2.drive(PHC(0+1,1+2,0),pi,0), PHC(1+1,0+2,-pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1+3,0+4,pi/2),pi,0), PHC(0+3,1+4,0))
      self.assertEqualPHC(dd2.drive(PHC(0+5,-1+6,pi),pi,0), PHC(-1+5,0+6,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1+7,0+8,-pi/2),pi,0), PHC(0+7,-1+8,-pi))
      self.assertEqualPHC(dd2.drive(PHC(-1+9,0+10,pi/2),pi/2,0), PHC(-sqrt(2)/2+9,sqrt(2)/2+10,pi/4))

   def test_rotate_about_right_wheel_counterclockwise(self):
      dd1 = DD(1)
      self.assertEqualPHC(dd1.drive(PHC(0,0.5,0),-pi/2,0), PHC(-0.5,0,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5,0,pi/2),-pi/2,0), PHC(0,-0.5,pi))
      self.assertEqualPHC(dd1.drive(PHC(0,-0.5,pi),-pi/2,0), PHC(0.5,0,3*pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5,0,-pi/2),-pi/2,0), PHC(0,0.5,0))
      self.assertEqualPHC(dd1.drive(PHC(-0.5,0,pi/2),-pi/4,0), PHC(-sqrt(2)/4,-sqrt(2)/4,3*pi/4))

      self.assertEqualPHC(dd1.drive(PHC(0+1,0.5+2,0),-pi/2,0), PHC(-0.5+1,0+2,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5+3,0+4,pi/2),-pi/2,0), PHC(0+3,-0.5+4,pi))
      self.assertEqualPHC(dd1.drive(PHC(0+5,-0.5+6,pi),-pi/2,0), PHC(0.5+5,0+6,3*pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5+7,0+8,-pi/2),-pi/2,0), PHC(0+7,0.5+8,0))
      self.assertEqualPHC(dd1.drive(PHC(-0.5+9,0+10,pi/2),-pi/4,0), PHC(-sqrt(2)/4+9,-sqrt(2)/4+10,3*pi/4))

      dd2 = DD(2)
      self.assertEqualPHC(dd2.drive(PHC(0,1,0),-pi,0), PHC(-1,0,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1,0,pi/2),-pi,0), PHC(0,-1,pi))
      self.assertEqualPHC(dd2.drive(PHC(0,-1,pi),-pi,0), PHC(1,0,3*pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1,0,-pi/2),-pi,0), PHC(0,1,0))
      self.assertEqualPHC(dd2.drive(PHC(-1,0,pi/2),-pi/2,0), PHC(-sqrt(2)/2,-sqrt(2)/2,3*pi/4))

      self.assertEqualPHC(dd2.drive(PHC(0+1,1+2,0),-pi,0), PHC(-1+1,0+2,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1+3,0+4,pi/2),-pi,0), PHC(0+3,-1+4,pi))
      self.assertEqualPHC(dd2.drive(PHC(0+5,-1+6,pi),-pi,0), PHC(1+5,0+6,3*pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1+7,0+8,-pi/2),-pi,0), PHC(0+7,1+8,0))
      self.assertEqualPHC(dd2.drive(PHC(-1+9,0+10,pi/2),-pi/2,0), PHC(-sqrt(2)/2+9,-sqrt(2)/2+10,3*pi/4))

if __name__ == '__main__':
   unittest.main()
