import sys
import os
import json

import robot

def get_objects(robot, task):
    return


def get_world(world):
    return


def get_goal(task):
    return


def get_escort_problem(robot, task):
    with open('escort_template.txt', 'w+') as f:
        template = f.read()

        objects = get_objects(robot, task)
        world = get_world(x)
        goal = get_goal(task)

        return template.format(objects, world, goal)