def generate_map():
    campus_map = dict()
    
    '''
        keys represent nodes on directed graph to other connected nodes
        values represent list of destination nodes and the cost for travel 
        items in value are -- (destination, cost)
        cost: 2 downhill, 4 level, 6 uphill, 10 stay 
    '''
    
    campus_map[0] = [
        (1, 6),
        (2, 4)
    ]
    campus_map[1] = [
        (3, 4),
        (0, 2)
    ]
    campus_map[2] = [
        (4, 4), 
        (6, 2),
        (0, 4)
    ]
    campus_map[3] = [
        (4, 2),
        (1, 4) 
    ]
    campus_map[4] = [
        (3, 6),
        (7, 4),
        (5, 4),
        (2, 4)
    ]
    campus_map[5] = [
        (4, 4), 
        (6, 2)
    ]
    campus_map[6] = [
        (5, 6), 
        (2, 6)
    ]
    campus_map[7] = [
        (4, 4) 
    ]

    return campus_map