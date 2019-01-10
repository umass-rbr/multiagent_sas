import json

from robot import Robot
from task import EscortTask

WORLDS_DIRECTORY = "worlds/"
TEMPLATES_DIRECTORY = "templates/"

INDENT_SIZE = 4
DELIMITER = "\n" + " " * INDENT_SIZE

OBJECT_DECLARATION_TEMPLATE = "{0} - {1}"
AT_PREDICATE_TEMPLATE = "(at {0} {1})"
CONNECTED_PREDICATE_TEMPLATE = "(connected {0} {1})"
DISTANCE_ASSIGNMENT_TEMPLATE = "(+ (distance {0} {1}) {2})"


def get_object_declaration(ids, type):
    objects = " ".join(ids)
    return OBJECT_DECLARATION_TEMPLATE.format(objects, type)


def get_at_predicate(id, location):
    return AT_PREDICATE_TEMPLATE.format(id, location)


def get_connected_predicate(start_location, end_location):
    return CONNECTED_PREDICATE_TEMPLATE.format(start_location, end_location)


def get_distance_assignment(start_location, end_location, distance):
    return DISTANCE_ASSIGNMENT_TEMPLATE.format(start_location, end_location, distance)


def get_objects(robot, task, world):
    robot_object = get_object_declaration(["cobot"], "robot")
    person_object = get_object_declaration([task.person], "person")
    location_objects = get_object_declaration([id for id in world["locations"].keys()], "location")
    return DELIMITER.join([robot_object, person_object, location_objects])


def get_locations(robot, task):
    robot_location = get_at_predicate(robot.id, robot.pos)
    person_location = get_at_predicate(task.person, task.start_location)
    return DELIMITER.join([robot_location, person_location])


def get_world(world):
    features = []

    for path in world["paths"]:
        features.append(get_connected_predicate(path['startLocation'], path["endLocation"]))
        features.append(get_distance_assignment(path['startLocation'], path["endLocation"], path["distance"]))

    return DELIMITER.join(features)


def get_goal(task):
    return get_at_predicate(task.person, task.end_location)


# TODO Fix the naming throughout this file. It's pretty bad.
def generate_escort_problem(robot, task, world):
    with open(TEMPLATES_DIRECTORY + "escort_problem.template", "r") as template_file:
        template = template_file.read()

        objects = get_objects(robot, task, world)
        locations = get_locations(robot, task)
        world = get_world(world)
        goal = get_goal(task)

        return template.format(objects, locations, world, goal)


def main():
    with open(WORLDS_DIRECTORY + "world.json", "r") as world_file:
        robot = Robot("cobot", 0, "amrl")
        task = EscortTask("shlomo", "shlomoOffice", "amrl", "10:30", "10:45")
        world = json.load(world_file)

        print(generate_escort_problem(robot, task, world))


if __name__ == "__main__":
    main()
