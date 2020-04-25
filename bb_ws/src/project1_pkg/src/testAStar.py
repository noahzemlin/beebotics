from aStarSearch import aStar
import matplotlib.pyplot as plt

space = [
    [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

search = aStar(space, [0, 12], [9, 0])
path = search.grid_astar()
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