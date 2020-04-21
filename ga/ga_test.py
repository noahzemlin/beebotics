from gasearch import GASearch

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

searchy = GASearch(space, population_size=200)
best_path = searchy.search((0,12), (9, 0), iters = 10000)

print("result: " + str(best_path.pts))
print("score: " + str(searchy._sort_key(best_path, (9, 0))))
print("distance: " + str(best_path.get_path_length()))