def get_location_ids(world):
    return [id for id in world["locations"].keys()]

def get_distance(world, start_location_id, end_location_id):
    for path in world["paths"]:
        if path["startLocationId"] == start_location_id and path["endLocationId"] == end_location_id:
            return path["distance"]
    return None
    