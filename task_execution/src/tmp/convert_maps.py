import json
import os

def convert_amrl_map_to_rbr_map():
    with open('LGRC3_map.json') as map_file:
        map_ = json.load(map_file)
        new_map_ = {'name':map_['name'],'locations':{},'paths':{}}

        for dict_ in map_['vertices']:
            new_map_['locations'][dict_['name']] = {}
            new_map_['locations'][dict_['name']]['name'] = dict_['name']
            new_map_['locations'][dict_['name']]['pose'] = dict_['pose']

        for dict_ in map_['edges']:
            if not dict_['from'] in new_map_['paths']:
                new_map_['paths'][dict_['from']] = {}
                new_map_['paths'][dict_['from']][dict_['to']] = {}
                new_map_['paths'][dict_['from']][dict_['to']]['cost'] = dict_['weight']
                if "Office" in dict_['from'] or "Office" in dict_['to']:
                    new_map_['paths'][dict_['from']][dict_['to']]['obstruction'] = 'door'
                else:
                    new_map_['paths'][dict_['from']][dict_['to']]['obstruction'] = 'none'

    new_map_as_json = json.dumps(new_map_, indent=4)
    ofile = open('LGRC3_plan_map.json','w+')
    ofile.write(new_map_as_json)
    ofile.close()

if __name__ == '__main__':
    convert_amrl_map_to_rbr_map()