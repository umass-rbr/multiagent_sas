def generate_map():
    campus_map = dict()
    
    # keys represent nodes on directed graph to other connected nodes
    # values represent list of destination nodes and the cost for travel 
    # items in value are -- (destination, cost)
    # cost: 2 forward, 4 backward, 10 no-move
    campus_map[1] = [
        (1, 10), 
        (2, 2)
    ]
    campus_map[2] = [
        (2, 10), 
        (3, 2), 
        (1, 4)
    ]
    campus_map[3] = [
        (3, 10), 
        (3, 2), 
        (4, 4)
    ]
    campus_map[4] = [
        (4, 10), 
        (3, 4), 
    ]

    return campus_map