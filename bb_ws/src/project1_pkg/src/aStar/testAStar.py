from aStarSearch import aStar
import matplotlib.pyplot as plt
import pickle
import csv

# Test 2D Array
# space = [
#     [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
#     [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
#     [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
#     [0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
#     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
#     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
#     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
# ]

# Reading from Devon map
space = pickle.load(open("../ga/global_map.p", "rb"))
space[space != 0] = 1

#Test space coordinates
# start = (0, 12) 
# goal = (9,2)

# Examples coordinates in Devon space with specified coordinates
# No scaling
# start = [350,250]
# goal = [350, 480] 

# scaled down by 100 scale factor = 10
start = [30, 25] 
goal = [30,47]

# #scale factor = 5
# start = [60, 55] 
# goal = [60,95]

search = aStar(space, start, goal)

scale = 10 
search.convertSpace(space, scale)

path = search.grid_astar()
print(path)
path = search.convert(path)
print(path)
print("distance:", len(path))

# Used to write out path to file as back up
# with open('pathScale5.csv', 'w') as file:
#     wr = csv.writer(file, delimiter=",")
#     wr.writerow(["Start", start[0], start[1]])
#     wr.writerow(["Goal", goal[0], goal[1]])
#     wr.writerows(path)
#     file.close()

# Used to read path written to file to display on grid without running A* search again
# newPath = []
# with open('pathScale5.csv', 'r') as file:
#     rd = csv.reader(file, delimiter=",")
#     rd.next()
#     rd.next()
#     for row in rd:
#         newPath.append([int(row[0]), int(row[1])])
#     file.close()
# print("NEW PATH:", newPath)
# path = newPath

search.display(start, path)