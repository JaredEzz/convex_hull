from which_pyqt import PYQT_VER

if PYQT_VER == 'PYQT5':
    from PyQt5.QtCore import QLineF, QPointF, QObject
elif PYQT_VER == 'PYQT4':
    from PyQt4.QtCore import QLineF, QPointF, QObject
else:
    raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))

import time
import math
from functools import partial

# Some global color constants that might be useful
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Global variable that controls the speed of the recursion automation, in seconds
#
PAUSE = 0.25


#
# This is the class you have to complete.
#
def get_center(points):
    x = [p.x() for p in points]
    y = [p.y() for p in points]
    return sum(x) / len(points), sum(y) / len(points)


# returns the angle of a point
def calculate_angle(point, center_x, center_y):
    return math.atan2(point.x() - center_x, point.y() - center_y)


def get_closest_x(partial_hull, side_id=0):
    closest_index = 0
    if side_id == 0:
        # 	left side
        for i, point in enumerate(partial_hull):
            if point > partial_hull[closest_index]:
                closest_index = i
    elif side_id == 1:
        # 	right side
        for i, point in enumerate(partial_hull):
            if point < partial_hull[closest_index]:
                closest_index = i
    return closest_index


def calculate_slope(a, b):
    return (b.y() - a.y()) / (b.x() - a.x())


def get_upper_tangent(left_start, left_hull, right_start, right_hull):
    left_i = left_start
    right_i = right_start
    previous_tangent = tuple([left_i, right_i])
    while 1:
        slope = calculate_slope(left_hull[left_i], right_hull[right_i])
        left_slope_decreasing = True
        while left_slope_decreasing:
            next_slope = calculate_slope(left_hull[(left_i - 1) % len(left_hull)], right_hull[right_i])
            if next_slope < slope:
                slope = next_slope
                left_i -= 1
                if left_i < 0:
                    left_i = len(left_hull) - 1
                left_slope_decreasing = True
            else:
                left_slope_decreasing = False

        right_slope_increasing = True
        while right_slope_increasing:
            next_slope = calculate_slope(left_hull[left_i], right_hull[(right_i + 1) % len(right_hull)])
            if next_slope > slope:
                slope = next_slope
                right_i += 1
                if right_i >= len(right_hull):
                    right_i %= len(right_hull)
                right_slope_increasing = True
            else:
                right_slope_increasing = False

                current_tangent = tuple([left_i, right_i])
                # if tangent value is constant, checks haven't changed it, it's final
                if previous_tangent == current_tangent:
                    break

                previous_tangent = current_tangent

        return left_i, right_i


class ConvexHullSolver(QObject):

    # Class constructor
    def __init__(self):
        super().__init__()
        self.pause = False
        # 	add array to self to store lines
        self.lines = []
        self.points = []

    # Some helper methods that make calls to the GUI, allowing us to send updates
    # to be displayed.

    def showTangent(self, line, color):
        self.view.addLines(line, color)
        if self.pause:
            time.sleep(PAUSE)

    def eraseTangent(self, line):
        self.view.clearLines(line)

    def blinkTangent(self, line, color):
        self.showTangent(line, color)
        self.eraseTangent(line)

    def showHull(self, polygon, color):
        self.view.addLines(polygon, color)
        if self.pause:
            time.sleep(PAUSE)

    def eraseHull(self, polygon):
        self.view.clearLines(polygon)

    def showText(self, text):
        self.view.displayStatusText(text)

    # compare QPointF base on x-axis
    # comparison is O(nlogn), space complexity is O(n)
    def compare_QPointF(original, comparison):
        return original.x() < comparison.x()

    QPointF.__lt__ = compare_QPointF

    # divide and conquer algorithm
    def d_and_c(self, points):

        # base case
        if len(points) == 1:
            return points

        halfway_index = len(points) // 2
        left_hull = self.d_and_c(points[0:halfway_index])
        right_hull = self.d_and_c(points[halfway_index:])

        return self.combine_hulls(left_hull, right_hull)

    def combine_hulls(self, left_hull, right_hull):
        # 	find upper and lower tangents
        points = []

        if len(left_hull) + len(right_hull) < 4:
            combined_hulls = left_hull + right_hull
            center_x, center_y = get_center(combined_hulls)
            hull = sorted(combined_hulls,
                          key=partial(calculate_angle, center_x=center_x, center_y=center_y))
            return hull

        else:
            left_start = get_closest_x(left_hull, 0)
            right_start = get_closest_x(right_hull, 1)

            left_upper_tangent, right_upper_tangent = \
                get_upper_tangent(left_start, left_hull, right_start, right_hull)
            right_lower_tangent, left_lower_tangent = \
                get_upper_tangent(right_start, right_hull, left_start, left_hull)

            points.append(left_hull[left_upper_tangent])
            i = right_upper_tangent
            while right_hull[i % len(right_hull)] != right_hull[right_lower_tangent]:
                points.append(right_hull[i % len(right_hull)])
                i += 1
            points.append(right_hull[right_lower_tangent])
            i = left_lower_tangent
            while left_hull[i % len(left_hull)] != left_hull[left_upper_tangent]:
                points.append(left_hull[i % len(left_hull)])
                i += 1

            # self.plot_points(points)
            return points

    # This is the method that gets called by the GUI and actually executes
    # the finding of the hull
    def compute_hull(self, points, pause, view):
        self.pause = pause
        self.view = view
        assert (type(points) == list and type(points[0]) == QPointF)

        t1 = time.time()
        # jh - sort the points using defined compare_to function
        points.sort()
        t2 = time.time()

        # jh - calculate hull lines
        t3 = time.time()
        self.lines = []
        hull_pts = self.d_and_c(points)
        num_hull_pts = len(hull_pts)

        # this is a dummy polygon of the first 3 unsorted points
        polygon = [QLineF(hull_pts[i], hull_pts[(i + 1) % num_hull_pts]) for i in range(num_hull_pts)]
        # TODO: REPLACE THE LINE ABOVE WITH A CALL TO YOUR DIVIDE-AND-CONQUER CONVEX HULL SOLVER
        t4 = time.time()

        # when passing lines to the display, pass a list of QLineF objects.  Each QLineF
        # object can be created with two QPointF objects corresponding to the endpoints
        self.showHull(polygon, BLUE)
        self.showText('Time Elapsed (Convex Hull): {:3.3f} sec'.format(t4 - t3))
