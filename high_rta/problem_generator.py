import json

import utils
from item import Item
from robot import Robot
from task import DeliveryTask, EscortTask

WORLDS_DIRECTORY = "worlds/"
TEMPLATES_DIRECTORY = "templates/"

INDENT_SIZE = 4
LINE_DELIMITER = "\n" + " " * INDENT_SIZE

OBJECT_DECLARATION_TEMPLATE = "{0} - {1}"
AT_PREDICATE_TEMPLATE = "(at {0} {1})"
CONNECTED_PREDICATE_TEMPLATE = "(connected {0} {1})"
DISTANCE_ASSIGNMENT_TEMPLATE = "(+ (distance {0} {1}) {2})"


def make_object_declaration(object_id, object_type):
    return OBJECT_DECLARATION_TEMPLATE.format(object_id, object_type)


def make_at_predicate(object_id, location_id):
    return AT_PREDICATE_TEMPLATE.format(object_id, location_id)


def make_connected_predicate(start_location_id, end_location_id):
    return CONNECTED_PREDICATE_TEMPLATE.format(start_location_id, end_location_id)


def make_distance_assignment(start_location_id, end_location_id, distance):
    return DISTANCE_ASSIGNMENT_TEMPLATE.format(start_location_id, end_location_id, distance)


def generate_objects(robot, items, world):
    robot_declaration = make_object_declaration(robot.id, "robot")
    item_declarations = [make_object_declaration(item.id, item.type) for item in items]
    location_declarations = [make_object_declaration(location_id, "location") for location_id in utils.get_location_ids(world)]
    return LINE_DELIMITER.join([robot_declaration] + item_declarations + location_declarations)


def generate_positions(robot, items):
    robot_predicate = make_at_predicate(robot.id, robot.pos)
    item_predicates = [make_at_predicate(item.id, item.position) for item in items]
    return LINE_DELIMITER.join([robot_predicate] + item_predicates)


def generate_world(world):
    features = []

    for path in world["paths"]:
        features.append(make_connected_predicate(path['startLocationId'], path["endLocationId"]))
        features.append(make_distance_assignment(path['startLocationId'], path["endLocationId"], path["distanceId"]))

    return LINE_DELIMITER.join(features)


def generate_goal(item_id, location_id):
    return make_at_predicate(item_id, location_id)


def generate_escort_problem(robot, task, world):
    with open(TEMPLATES_DIRECTORY + "escort_problem.template", "r") as template_file:
        template = template_file.read()

        items = [Item(task.person, "person", task.start_location)]

        objects = generate_objects(robot, items, world)
        positions = generate_positions(robot, items)
        world = generate_world(world)
        goal = generate_goal(task.person, task.end_location)

        return template.format(objects, positions, world, goal)


def generate_delivery_problem(robot, task, world):
    with open(TEMPLATES_DIRECTORY + "escort_problem.template", "r") as template_file:
        template = template_file.read()

        items = [Item(task.package, "deliverable", task.start_location)]

        objects = generate_objects(robot, items, world)
        positions = generate_positions(robot, items)
        world = generate_world(world)
        goal = generate_goal(task.package, task.end_location)

        return template.format(objects, positions, world, goal)


# TODO Fix the naming throughout this file. It's pretty bad.
# TODO Make sure to clean up this file as well.
def main():
    with open(WORLDS_DIRECTORY + "lgrc.json", "r") as world_file:
        world = json.load(world_file)
        robot = Robot("cobot", 0, "amrl")

        task = EscortTask("shlomo", "shlomoOffice", "amrl", "10:30", "10:45")
        print(generate_escort_problem(robot, task, world))

        task = DeliveryTask("mail", "mailroom", "amrl", "12:30", "12:45")
        print(generate_delivery_problem(robot, task, world))


if __name__ == "__main__":
    main()
