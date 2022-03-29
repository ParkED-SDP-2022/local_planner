from cgi import test
import math
from re import I

#doesn't work; disregard and use true_bearing
def target_heading(loc1,loc2):
    long1 = float(loc1[0])
    lat1 = float(loc1[1])
    long2 = float(loc2[0])
    lat2 = float(loc2[1])

    y = math.sin(long2-long1)*math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1)
    θ = math.atan2(y, x)
    target_h = (θ*180/math.pi + 360) % 360

    return target_h


# version 
# positive y-axis as 0 degrees, positive x-axis as 90 degrees
def true_bearing(loc1,loc2):

    # long x axis lat y axis
    startLong = math.radians(loc1[0])
    startLat = math.radians(loc1[1])
    
    endLong = math.radians(loc2[0])
    endLat = math.radians(loc2[1])
   
    dLong = endLong - startLong

    dPhi = math.log(math.tan(endLat/2.0+math.pi/4.0)/math.tan(startLat/2.0+math.pi/4.0))
    if abs(dLong) > math.pi:
        if dLong > 0.0:
            dLong = -(2.0 * math.pi - dLong)
        else:
            dLong = (2.0 * math.pi + dLong)

    bearing = (math.degrees(math.atan2(dLong, dPhi)) + 360.0) % 360.0

    return bearing

# version2 
# positive y axis as 0 deg, NEGATIVE x-axis as 90 deg
def true_bearing2(loc1,loc2):

    # long x axis lat y axis
    startLong = math.radians(loc1[0])
    startLat = math.radians(loc1[1])
    
    endLong = math.radians(loc2[0])
    endLat = math.radians(loc2[1])
   
    dLong = endLong - startLong

    dPhi = math.log(math.tan(endLat/2.0+math.pi/4.0)/math.tan(startLat/2.0+math.pi/4.0))
    if abs(dLong) > math.pi:
        if dLong > 0.0:
            dLong = -(2.0 * math.pi - dLong)
        else:
            dLong = (2.0 * math.pi + dLong)

    bearing = (math.degrees(math.atan2(dLong, dPhi)) + 360.0) % 360.0

    return 360 - bearing

# version3
# positive x axis as 0 deg, positive y-axis as 90 deg
def true_bearing3(loc1,loc2):

    # long x axis lat y axis
    startLong = math.radians(loc1[0])
    startLat = math.radians(loc1[1])
    
    endLong = math.radians(loc2[0])
    endLat = math.radians(loc2[1])
   
    dLong = endLong - startLong

    dPhi = math.log(math.tan(endLat/2.0+math.pi/4.0)/math.tan(startLat/2.0+math.pi/4.0))
    if abs(dLong) > math.pi:
        if dLong > 0.0:
            dLong = -(2.0 * math.pi - dLong)
        else:
            dLong = (2.0 * math.pi + dLong)

    bearing = (math.degrees(math.atan2(dLong, dPhi)) + 360.0) % 360.0

    # 4 quadrants
    if (bearing >= 0 and bearing <= 90):
        return 90 - bearing
    elif (bearing >= 90 and bearing <= 180):
        return 360 - (bearing-90)
    elif (bearing >= 180 and bearing <= 270):
        return (270-bearing) + 180
    else :
        return (360-bearing) + 90

# version4
# positive x axis as 0 deg,NEGATIVE y-axis as 90 deg
def true_bearing4(loc1,loc2):

    # long x axis lat y axis
    startLong = math.radians(loc1[0])
    startLat = math.radians(loc1[1])
    
    endLong = math.radians(loc2[0])
    endLat = math.radians(loc2[1])
   
    dLong = endLong - startLong

    dPhi = math.log(math.tan(endLat/2.0+math.pi/4.0)/math.tan(startLat/2.0+math.pi/4.0))
    if abs(dLong) > math.pi:
        if dLong > 0.0:
            dLong = -(2.0 * math.pi - dLong)
        else:
            dLong = (2.0 * math.pi + dLong)

    bearing = (math.degrees(math.atan2(dLong, dPhi)) + 360.0) % 360.0

    # 4 quadrants
    
    if (bearing >= 0 and bearing <= 90):
        bearing = 90 - bearing
    elif (bearing >= 90 and bearing <= 180):
        bearing = 360 - (bearing-90)
    elif (bearing >= 180 and bearing <= 270):
        bearing = (270-bearing) + 180
    else :
        bearing = (360-bearing) + 90

    return 360 - bearing

<<<<<<< HEAD
def true_bearing5(curLoc,target):

        # long x axis lat y axis
        a1 = curLoc[0]
        a2 = curLoc[1]
        
        b1 = target[0]
        b2 = target[1]

        if (a1 == b1 and a2 == b2):
            return 0
        
        theta = math.atan2(a1-b1,b2-a2)

        if (theta < 0.0):
            theta += math.pi*2

        return math.degrees(theta)

=======
>>>>>>> 304c4b97f192ef689496e5916589ad635f460e4c
#actual points on the Meadows
loc1 = (-3.194489,55.940678)
loc2 = (-3.191724,55.942440)
loc3 = (-3.196675,55.942284)
loc4 = (-3.194489,55.943)

kansas = (-94.581213,39.099912)
city = (-90.200203,38.627089)

test1 = (0,0)
test2 = (1,1)

test_origin = (0,0)
test_45 = (1,1)
test_135 = (1,-1)
test_225 = (-1,-1)
test_315 = (-1,1)

<<<<<<< HEAD

=======
>>>>>>> 304c4b97f192ef689496e5916589ad635f460e4c
#testing true_bearing v1
print("test1: ", true_bearing(test_origin,test_45))
print("test2: " ,true_bearing(test_origin,test_135))
print("test3: " ,true_bearing(test_origin,test_225))
print("test4: " ,true_bearing(test_origin,test_315))

assert round(true_bearing(test_origin,test_45)) == 45
assert round(true_bearing(test_origin,test_135)) == 135
assert round(true_bearing(test_origin,test_225)) == 225
assert round(true_bearing(test_origin,test_315)) == 315
print("assertion passed : v1 \n")

#testing true_bearing v2
print("test1_v2",true_bearing2(test_origin,test_45)) 
print("test2_v2",true_bearing2(test_origin,test_135)) 
print("test3_v2",true_bearing2(test_origin,test_225))
print("test4_v2",true_bearing2(test_origin,test_315)) # should be 45

assert round(true_bearing2(test_origin,test_45)) == 315
assert round(true_bearing2(test_origin,test_135)) == 225
assert round(true_bearing2(test_origin,test_225)) == 135
assert round(true_bearing2(test_origin,test_315)) == 45
print("assertion passed : v2 \n")

#testing true_bearing v3
print("test1_v3",true_bearing3(test_origin,test_45)) 
print("test2_v3",true_bearing3(test_origin,test_135)) 
print("test3_v3",true_bearing3(test_origin,test_225))
print("test4_v3",true_bearing3(test_origin,test_315)) # should be 45

assert round(true_bearing3(test_origin,test_45)) == 45
assert round(true_bearing3(test_origin,test_135)) == 315
assert round(true_bearing3(test_origin,test_225)) == 225
assert round(true_bearing3(test_origin,test_315)) == 135
print("assertion passed : v3 \n")

#testing true_bearing v2
print("test1_v4",true_bearing4(test_origin,test_45)) 
print("test2_v4",true_bearing4(test_origin,test_135)) 
print("test3_v4",true_bearing4(test_origin,test_225))
print("test4_v4",true_bearing4(test_origin,test_315)) # should be 45

assert round(true_bearing4(test_origin,test_45)) == 315
assert round(true_bearing4(test_origin,test_135)) == 45
assert round(true_bearing4(test_origin,test_225)) == 135
assert round(true_bearing4(test_origin,test_315)) == 225

print("assertion passed : v4 \n")
<<<<<<< HEAD

# first one long should be second lat
point1 = [730,150]
point2 = [730,350]
print("test local 1:", true_bearing5(point1,point2))

=======
>>>>>>> 304c4b97f192ef689496e5916589ad635f460e4c
#print("new" , true_bearing(kansas,city))
#should be northeast
#print(true_bearing(loc1,loc2))
#southwest
#print(true_bearing(loc2,loc1))
#northwest
#print(true_bearing(loc1,loc3))
#southeast
#print(true_bearing(loc3,loc1))

def heading_difference(c,t):
    heading_difference = (t - c) % 360

    if (heading_difference < 180) : return "RIGHT"
    else: return "LEFT"

# target to the right
#print("Right-hand target:")
#print(heading_difference(350,10))
#print(heading_difference(300,350))
#print(heading_difference(10,180))
# target to the left
#print("Left-hand target:")
#print(heading_difference(10,350))
#print(heading_difference(350,300))
#print(heading_difference(180,10))
#print(heading_difference(80,10))