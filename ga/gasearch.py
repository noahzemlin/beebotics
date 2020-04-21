import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

class GAChromosome:

    def __init__(self):
        self.pts = []

    def breed_with(self, significant_other):
        baby1 = GAChromosome()
        baby2 = GAChromosome()

        # choose crossover pts randomly?????
        crossover_pt1 = random.randint(1, len(self.pts) - 1)
        crossover_pt2 = random.randint(1, len(significant_other.pts) - 1)

        baby1 = GAChromosome()
        baby1.pts = self.pts[0:crossover_pt1]
        baby1.pts.extend(significant_other.pts[crossover_pt2:])

        baby2 = GAChromosome()
        baby2.pts = significant_other.pts[0:crossover_pt2]
        baby2.pts.extend(self.pts[crossover_pt1:])

        return baby1, baby2

    # delicious cost function
    def get_path_length(self):
        total = 0

        for i in range(1, len(self.pts)):
            total += math.sqrt((self.pts[i][0] - self.pts[i-1][0])**2 + (self.pts[i][1] - self.pts[i-1][1])**2)

        return total
        

class GASearch:

    MUTATION_PROB = 0.2

    def __init__(self, world, population_size = 100):
        self.world = world
        self.height = len(world)
        self.width = len(world[0])
        self.population_size = population_size

    def rand_pt_in_world(self):
        return (random.randint(0, self.width - 1), random.randint(0, self.height - 1))

    def raycast(self, pos, dir):

        deltaDistX = 0 if dir[1] == 0 else (1 if dir[0] == 0 else abs(1. / dir[0]))
        deltaDistY = 0 if dir[0] == 0 else (1 if dir[1] == 0 else abs(1. / dir[1]))

        mapX = int(pos[0])
        mapY = int(pos[1])

        if dir[0] < 0:
            stepX = -1
            sideDistX = (pos[0] - mapX) * deltaDistX
        else:
            stepX = 1
            sideDistX = (mapX + 1.0 - pos[0]) * deltaDistX

        if dir[1] < 0:
            stepY = -1
            sideDistY = (pos[1] - mapY) * deltaDistY
        else:
            stepY = 1
            sideDistY = (mapY + 1.0 - pos[1]) * deltaDistY

        if dir[0]*dir[0] + dir[1]*dir[1] > 0:

            cumulative_movement = [0, 0]

            while mapX >= 0 and mapX < self.width and mapY >= 0 and mapY < self.height:
                if self.world_copy[mapY][mapX] > 0:
                    return True

                # here instead of in while condition to still do previous check
                if not (dir[0] != cumulative_movement[0] or dir[1] != cumulative_movement[1]):
                    break

                if sideDistX < sideDistY:
                    sideDistX += deltaDistX
                    mapX += stepX
                    cumulative_movement[0] += stepX
                    side = 0
                else:
                    sideDistY += deltaDistY
                    mapY += stepY
                    cumulative_movement[1] += stepY
                    side = 1

        else:
            if self.world_copy[mapY][mapX] > 0:
                return True

        return False

    def _sort_key(self, chromie, goal):
        final_pt = chromie.pts[-1:][0]
        summy = chromie.get_path_length()

        self.world_copy = copy.deepcopy(self.world)
        for i in range(0, len(chromie.pts) - 1):
            fp = chromie.pts[i]
            lp = chromie.pts[i + 1]

            # Make sure there are no collisisons
            if self.raycast(fp, (lp[0] - fp[0], lp[1] - fp[1])):
                return 100001

            # Make sure last point is not in obstruction
            # TODO: put in raycast
            if self.world_copy[lp[1]][lp[0]] == 1:
                return 100003

            # Make the first point an obstacle for the remaining lines
            # TODO: make entire lines obstacles
            self.world_copy[fp[1]][fp[0]] = 1

        # If path length is 0, bad score
        if summy == 0:
            return 100002

        if final_pt[0] == goal[0] and final_pt[1] == goal[1]:
            # if valid, reward it
            return 1 - (1 / (summy + len(chromie.pts)))
        else:
            # is this D* now?
            return 0.2 * len(chromie.pts) + (200. / summy) + 3.0 * math.sqrt((final_pt[0] - goal[0])**2 + (final_pt[1] - goal[1])**2)


    def search(self, start, goal, iters = 100000):

        self.population = [GAChromosome() for i in range(self.population_size)]

        # Initalize population
        for chromie in self.population:
            chromie.pts = [start, self.rand_pt_in_world()]

        self.population.sort(key = lambda x: self._sort_key(x, goal))

        last_best = 100000000
        for _ in range(iters):
            pop_best_path = self.population[0]

            # Set new best (only for debugging, can be removed eventually)
            new_best = self._sort_key(pop_best_path, goal)
            if new_best < last_best:
                print("score: ", new_best)
                print("length: ", pop_best_path.get_path_length())
                print(pop_best_path.pts)
                last_best = new_best

                plt.clf()
                plt.imshow(self.world, interpolation='none')
                plt.ion()
                for i in range(0, len(pop_best_path.pts) - 1):
                    pt1 = [pop_best_path.pts[i][0], pop_best_path.pts[i+1][0]]
                    pt2 = [pop_best_path.pts[i][1], pop_best_path.pts[i+1][1]]
                    plt.plot(pt1, pt2, marker = 'o', color='green')
                plt.draw()
                plt.show()
                plt.pause(0.001)

            # Create children
            for i in range(int(self.population_size//1.2), self.population_size-1, 2):
                mom = self.population[random.randint(0, int(self.population_size//1.2))]
                dad = self.population[random.randint(0, int(self.population_size//1.2))]
                self.population[i], self.population[i+1] = mom.breed_with(dad)

                # Mutation for child 1
                if random.random() < self.MUTATION_PROB:
                    rng = random.random()
                    n = len(self.population[i].pts)
                    if rng < 0.6:
                        self.population[i].pts[random.randint(1, n-1)] = self.rand_pt_in_world()
                    elif rng < 0.9 and n > 2:
                        self.population[i].pts.pop(random.randint(1, n-1))
                    else:
                        self.population[i].pts.append(self.rand_pt_in_world())

                # Mutation for child 2
                if random.random() < self.MUTATION_PROB:
                    rng = random.random()
                    n = len(self.population[i+1].pts)
                    if rng < 0.6:
                        self.population[i+1].pts[random.randint(1, n-1)] = self.rand_pt_in_world()
                    elif rng < 0.9 and n > 2:
                        self.population[i+1].pts.pop(random.randint(1, n-1))
                    else:
                        self.population[i+1].pts.append(self.rand_pt_in_world())
                    
            random.shuffle(self.population)
            self.population.sort(key = lambda x: self._sort_key(x, goal))

        return self.population[0]