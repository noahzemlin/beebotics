from aStarSearch import aStar
import matplotlib.pyplot as plt
import pickle

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

space = pickle.load(open("../ga/global_map.p", "rb"))
space[space != 0] = 1
print(space)

# for i in range(len(space)):
#     for j in range(len(space[0])):
#         if(space[i][j] == 0 and i  < 405 and i > 295 and j < 400):
#             print("valid:[", i, ",", j, "]")

search = aStar(space, [200, 400], [250, 300])
path = search.grid_astar()
print(path)
path = search.convert(path)
print(path)
print("distance:", len(path))

plt.clf()
plt.imshow(space, interpolation='none')
plt.ion()
for i in range(0, len(path) - 1):
    pt1 = [path[i][0], path[i + 1][0]]
    pt2 = [path[i][1], path[i + 1][1]]
    plt.plot(pt1, pt2, marker='o', color='green')
plt.draw()
plt.show()
plt.pause(100)


# best_path = searchy.search((0,12), (9, 0), iters = 10000)

# print("result: " + str(best_path.pts))
# print("score: " + str(searchy._sort_key(best_path, (9, 0))))
# print("distance: " + str(best_path.get_path_length()))