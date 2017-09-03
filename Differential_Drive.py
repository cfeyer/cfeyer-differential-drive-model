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

   def __init__(self, x0_m=0, y0_m=0, heading0_rads=0):
      self.x_m = float(x0_m)
      self.y_m = float(y0_m)
      self.heading_rads = float(heading0_rads) # 0 = east

   def __eq__(self, other):
      dx = other.x_m - self.x_m
      dy = other.y_m - self.y_m
      dh = other.heading_rads - self.heading_rads
      #print "   self: " + str(self)
      #print "   other: " + str(other)
      #print "   dx=" + str(dx) + ", dy=" + str(dy) + ", dh=" + str(dh)
      tol = 1.0e-14
      return (abs(dx) <= tol) and (abs(dy) <= tol) and (abs(dh) <= tol)

   def __ne__(self, other):
      return not self.__eq__(other)

   def __str__(self):
      return "x=" + str(self.x_m) + ", y=" + str(self.y_m) + ", h=" + str(self.heading_rads)


class Differential_Drive:

   def __init__(self, track_width_m):
      self.track_width_m = float(track_width_m)

   def drive(self, initial_coords, left_travel_m, right_travel_m):
      final_coords = initial_coords
      if(left_travel_m == right_travel_m): # stopped or driving straight
         #print "   initial: " + str(initial_coords)
         final_coords.x_m = initial_coords.x_m + cos(initial_coords.heading_rads) * float(left_travel_m)
         final_coords.y_m = initial_coords.y_m + sin(initial_coords.heading_rads) * float(left_travel_m)
         #print "   final: " + str(final_coords)
      elif(left_travel_m == -right_travel_m): # rotate in place
         final_coords.heading_rads = right_travel_m / self.track_width_m + initial_coords.heading_rads
      elif(left_travel_m == 0): # rotate about left wheel
         turn_angle_rads = right_travel_m/self.track_width_m
         lw = self.left_wheel_coords(initial_coords)
         lw_x = lw.x_m
         lw_y = lw.y_m
         phi = turn_angle_rads
         x1 = initial_coords.x_m - lw_x
         y1 = initial_coords.y_m - lw_y
         x2 = cos(phi)*x1-sin(phi)*y1
         y2 = sin(phi)*x1+cos(phi)*y1
         final_coords.x_m = x2 + lw_x
         final_coords.y_m = y2 + lw_y
         final_coords.heading_rads = initial_coords.heading_rads + turn_angle_rads
      else:
         raise Exception("Not implemented for this (left_travel_m, right_travel_m)")
      return final_coords

   def left_wheel_coords(self, p):
      x = -(self.track_width_m/2.0) * sin(p.heading_rads) + p.x_m
      y = (self.track_width_m/2.0) * cos(p.heading_rads) + p.y_m
      h = p.heading_rads
      return Position_Heading_Coordinates(x,y,h)

   def right_wheel_coords(self, p):
      x = (self.track_width_m/2.0) * sin(p.heading_rads) + p.x_m
      y = -(self.track_width_m/2.0) * cos(p.heading_rads) + p.y_m
      h = p.heading_rads
      return Position_Heading_Coordinates(x,y,h)


PHC = Position_Heading_Coordinates
DD = Differential_Drive


class Test_PHC(unittest.TestCase):

   def test_default_ctor(self):
      coords = PHC()
      self.assertEqual(coords.x_m, 0)
      self.assertEqual(coords.y_m, 0)
      self.assertEqual(coords.heading_rads, 0)

   def test_custom_ctor(self):
      coords = PHC(1,2,3)
      self.assertEqual(coords.x_m, 1)
      self.assertEqual(coords.y_m, 2)
      self.assertEqual(coords.heading_rads, 3)

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
      x_error = abs(actual.x_m - expected.x_m)
      y_error = abs(actual.y_m - expected.y_m)
      heading_error = abs(actual.heading_rads - expected.heading_rads)
      tol = 1.0e-14
      msg = "expected: " + str(expected) + "\n" + \
            "actual: " + str(actual)
      is_fail = False
      if x_error > tol:
         is_fail = True
         msg += "\nassertion fail: x_m"
      if y_error > tol:
         is_fail = True
         msg += "\nassertion fail: y_m"
      if heading_error > tol:
         is_fail = True
         msg += "\nassertion fail: heading_rads"

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

      self.assertEqualPHC(dd1.drive(PHC(0+1,-0.5+2,0),0,pi/2), PHC(0.5+1,0+2,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5+3,0+4,pi/2),0,pi/2), PHC(0+3,0.5+4,pi))
      self.assertEqualPHC(dd1.drive(PHC(0+5,0.5+6,pi),0,pi/2), PHC(-0.5+5,0+6,3*pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5+7,0+8,3*pi/2),0,pi/2), PHC(0+7,-0.5+8,2*pi))

      dd2 = DD(2)
      self.assertEqualPHC(dd2.drive(PHC(0,-1,0),0,pi), PHC(1,0,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1,0,pi/2),0,pi), PHC(0,1,pi))
      self.assertEqualPHC(dd2.drive(PHC(0,1,pi),0,pi), PHC(-1,0,3*pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1,0,3*pi/2),0,pi), PHC(0,-1,2*pi))

      self.assertEqualPHC(dd2.drive(PHC(0+1,-1+2,0),0,pi), PHC(1+1,0+2,pi/2))
      self.assertEqualPHC(dd2.drive(PHC(1+3,0+4,pi/2),0,pi), PHC(0+3,1+4,pi))
      self.assertEqualPHC(dd2.drive(PHC(0+5,1+6,pi),0,pi), PHC(-1+5,0+6,3*pi/2))
      self.assertEqualPHC(dd2.drive(PHC(-1+7,0+8,3*pi/2),0,pi), PHC(0+7,-1+8,2*pi))

   def test_rotate_about_left_wheel_clockwise(self):
      dd1 = DD(1)
      self.assertEqualPHC(dd1.drive(PHC(0,-0.5,0),0,-pi/2), PHC(-0.5,0,-pi/2))
      self.assertEqualPHC(dd1.drive(PHC(0.5,0,pi/2),0,-pi/2), PHC(0,-0.5,0))
      self.assertEqualPHC(dd1.drive(PHC(0,0.5,pi),0,-pi/2), PHC(0.5,0,pi/2))
      self.assertEqualPHC(dd1.drive(PHC(-0.5,0,3*pi/2),0,-pi/2), PHC(0,0.5,pi))

if __name__ == '__main__':
   unittest.main()
