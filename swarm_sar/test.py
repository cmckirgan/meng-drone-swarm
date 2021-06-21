x=1
y=5
waypoint = []
point = [(x,y+20),(x+20,y),(x,y-20),(x-20,y)]

for x in point:
	waypoint.append((x[0],x[1],70))
	waypoint.append((x[0],x[1],20))
	waypoint.append((x[0],x[1],70))

print(waypoint)
