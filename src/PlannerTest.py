from cgi import test
import math

def target_heading(loc1,loc2):
    long1 = float(loc1[0])
    lat1 = float(loc1[1])
    long2 = float(loc2[0])
    lat2 = float(loc2[1])

    #print(type(long1))
    y = math.sin(long2-long1)*math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1)
    θ = math.atan2(y, x)
    target_h = (θ*180/math.pi + 360) % 360

    return target_h

def true_bearing(loc1,loc2):
    startLat = math.radians(loc1[1])
    startLong = math.radians(loc1[0])
    endLat = math.radians(loc2[1])
    endLong = math.radians(loc2[0])

    dLong = endLong - startLong

    dPhi = math.log(math.tan(endLat/2.0+math.pi/4.0)/math.tan(startLat/2.0+math.pi/4.0))
    if abs(dLong) > math.pi:
        if dLong > 0.0:
            dLong = -(2.0 * math.pi - dLong)
        else:
            dLong = (2.0 * math.pi + dLong)

    bearing = (math.degrees(math.atan2(dLong, dPhi)) + 360.0) % 360.0

    return bearing
#actual points on the Meadows
loc1 = (-3.194489,55.940678)
loc2 = (-3.191724,55.942440)
loc3 = (-3.196675,55.942284)
loc4 = (-3.194489,55.943)

kansas = (-94.581213,39.099912)
city = (-90.200203,38.627089)

test1 = (0,0)
test2 = (1,1)

print("test2 : ", true_bearing(test1,test2))
print("test : " ,target_heading(test1,test2))
print("new" , target_heading(kansas,city))
#print(target_heading(loc1,loc4))
#should be northeast
print(target_heading(loc1,loc2))
#southwest
print(target_heading(loc2,loc1))
#northwest
print(target_heading(loc1,loc3))
#southeast
print(target_heading(loc3,loc1))

def heading_difference(c,t):
    heading_difference = (t - c) % 360
    return heading_difference

# target to the right
print("Right-hand target:")
print(heading_difference(10,350))
print(heading_difference(350,300))
print(heading_difference(180,10))
# target to the left
print("Left-hand target:")
print(heading_difference(350,10))
print(heading_difference(300,350))
print(heading_difference(10,180))