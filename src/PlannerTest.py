import math

def target_heading(loc1,loc2):
    long1 = loc1[0]
    lat1 = loc1[1]
    long2 = loc2[0]
    lat2 = loc2[1]

    y = math.sin(long2-long1)*math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1)
    θ = math.atan2(y, x)
    target_h = (θ*180/math.pi + 360) % 360

    return target_h

#actual points on the Meadows
loc1 = (-3.194489,55.940678)
loc2 = (-3.191724,55.942440)
loc3 = (-3.196675,55.942284)

#should be northeast
print(target_heading(loc1,loc2))
#southwest
print(target_heading(loc2,loc1))
#northwest
print(target_heading(loc1,loc3))
#southeast
print(target_heading(loc3,loc1))