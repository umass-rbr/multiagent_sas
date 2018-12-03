def generate_map():
    campus_map = dict()
    
    '''
        keys represent nodes on directed graph to other connected nodes
        values represent list of destination nodes and the cost for travel 
        items in value are -- (destination, cost, obstacle)
        cost: 2 downhill, 4 level, 6 uphill
    '''
    
    campus_map[1] = [
        (2, 6),
        (3, 4)
    ]
    campus_map[2] = [
        (4, 4),
        (1, 2)
    ]
    campus_map[3] = [
        (5, 4), 
        (7, 2),
        (1, 4)
    ]
    campus_map[4] = [
        (5, 2),
        (2, 4) 
    ]
    campus_map[5] = [
        (4, 6),
        (8, 4),
        (6, 4),
        (3, 4)
    ]
    campus_map[6] = [
        (5, 4), 
        (8, 4),
        (7, 2)
    ]
    campus_map[7] = [
        (6, 6), 
        (3, 6)
    ]
    campus_map[8] = [
        (6, 4),
        (5, 4) 
    ]

    return campus_map