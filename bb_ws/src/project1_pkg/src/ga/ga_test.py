from gasearch import GASearch
import numpy
import pickle
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

space2 = [
    [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
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

searchy = GASearch(space, population_size=1000)
best_path = searchy.search((0, 12), (9, 2), iters=1000)
gen = 0

while True:
    plt.imshow(searchy.world, interpolation='none')
    for i in range(0, len(best_path.pts) - 1):
        pt1 = [best_path.pts[i][0], best_path.pts[i + 1][0]]
        pt2 = [best_path.pts[i][1], best_path.pts[i + 1][1]]
        plt.plot(pt1, pt2, marker='o', color='green')
    plt.title("gen: " + str(gen + 1))
    plt.show()

    gen += 1

    searchy.world = space if gen % 30 < 15 else space2

    print("result: " + str(best_path.pts))
    print("score: " + str(searchy._sort_key(best_path, (9, 0))))
    print("distance: " + str(best_path.get_path_length()))

    best_path = searchy.search((0, 12), (9, 2), iters=1000, init_pop=False)  # Keep searching
