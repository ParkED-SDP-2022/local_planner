# import Contingency
# import MotorDriver
#
#
# m1 = MotorDriver
# c1 = Contingency.Contingency(40, m1)
#
# print(c1.obstacles)

obstacles = []
for i in range(180):
    obstacles.append(True)
for i in range(30):
    obstacles.append(False)
for i in range(150):
    obstacles.append(True)


# print(obstacles)

def consective_values(input_list):
    n = 0
    count = 0
    for _ in range(2 * len(input_list)):
        if not input_list[n]:
            count += 1
        else:
            count = 0
        if count == 40:
            return n - 39
        n = (n + 1) % len(input_list)
    return -1


print(consective_values((obstacles)))
