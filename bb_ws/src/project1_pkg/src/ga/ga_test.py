from gasearch import GASearch
import pickle

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

space = pickle.load(open("global_map.p", "rb"))
space[space != 0] = 1
print(space)

searchy = GASearch(space, population_size=1000)
best_path = searchy.search((300,480), (400, 300), iters = 10000)

print("result: " + str(best_path.pts))
print("score: " + str(searchy._sort_key(best_path, (9, 0))))
print("distance: " + str(best_path.get_path_length()))