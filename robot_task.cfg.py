#!/usr/bin/env python
PACKAGE = "robot_task"
from dynamic_reconfigure.parameter_generator_catkin import *

def generate(gen):
    gen.add("Kp", double_t, 0, "Proportional gain", 0.02, 0.0, 1.0)
    gen.add("Ki", double_t, 0, "Integral gain", 0.001, 0.0, 0.1)
    gen.add("Kd", double_t, 0, "Derivative gain", 0.005, 0.0, 0.1)
