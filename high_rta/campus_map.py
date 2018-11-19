V = [0,1,2,3]
E = [(0,1,5),
	 (1,0,5),
	 (1,2,10),
	 (2,1,10),
	 (2,3,4),
	 (3,2,4)]
V1 = [0,1]
V2 = [2,3]

def get_building(l):
	if l in V1: return 'lgrc'
	else: return 'cs'

def distance(l1,l2):
	for e in E:
		if l1 == e[0] and l2 == e[1]: return e[2]