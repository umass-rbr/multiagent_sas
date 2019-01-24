def generate_map(map_number):
    
    # initializing the different campus maps
    campus_maps = [dict() for index in range(3)]

    # initializing the different obstacles
    
    '''
        keys represent nodes on directed graph to other connected nodes
        values represent list of destination nodes and the cost for travel 
        items in value are -- (destination, cost, obstacle)
        cost: 2 downhill, 4 level, 6 uphill, 10 obstacle
        obstacles include: crosswalk, door
    '''
    
    # creating first map
    campus_maps[0][0] = [
        (1, 4, "none")
    ]
    campus_maps[0][1] = [
        (0, 4, "none"),
        (2, 10, "crosswalk")
    ]
    campus_maps[0][2] = [
        (1, 10, "crosswalk"),
        (3, 10, "door")
    ]
    campus_maps[0][3] = [
        (2, 10, "door"),
        (4, 4, "none")
        
    ]
    campus_maps[0][4] = [
        (3, 4, "none")
    ]


    # creating second map
    campus_maps[1][0] = [
        (1, 6, "none"),
        (2, 4, "none")
    ]
    campus_maps[1][1] = [
        (3, 4, "none"),
        (0, 2, "none")
    ]
    campus_maps[1][2] = [
        (4, 4, "none"), 
        (6, 2, "none"),
        (0, 4, "none")
    ]
    campus_maps[1][3] = [
        (4, 2, "none"),
        (1, 4, "none") 
    ]
    campus_maps[1][4] = [
        (3, 6, "none"),
        (7, 10, "crosswalk"),
        (5, 4, "none"),
        (2, 4, "none")
    ]
    campus_maps[1][5] = [
        (4, 4, "none"), 
        (6, 2, "none")
    ]
    campus_maps[1][6] = [
        (5, 6, "none"), 
        (2, 6, "none")
    ]
    campus_maps[1][7] = [
        (4, 10, "crosswalk") 
    ]


    # creating third map
    campus_maps[2][0] = [
        (1, 10, "door"),
        (2, 10, "crosswalk")
    ]
    campus_maps[2][1] = [
        (0, 10, "door"),
        (3, 10, "crosswalk")
    ]
    campus_maps[2][2] = [
        (0, 10, "crosswalk"),
        (3, 10, "door")
    ]
    campus_maps[2][3] = [
        (2, 10, "door"),
        (1, 10, "crosswalk")
        
    ]


    # returning selected map
    return campus_maps[map_number]