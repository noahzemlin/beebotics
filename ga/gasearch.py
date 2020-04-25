import random
import math
import copy
import matplotlib.pyplot as plt


def clamp(x, low, high):
    if x < low:
        return low
    if x > high:
        return high
    return x


def rand_pt_near_pt(pt, w, h):
    return clamp(pt[0] + random.randrange(-2, 3), 0, w-1), clamp(pt[1] + random.randrange(-2, 3), 0, h-1)


class GAChromosome:

    def __init__(self):
        self.pts = []

    def breed_with(self, significant_other):
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

    def mutate(self, w, h):
        rng = random.random()
        n = len(self.pts)
        if rng < 0.4:
            # Move random pt in chromosome
            rand_pt_i = random.randint(1, n - 1)
            self.pts[rand_pt_i] = rand_pt_near_pt(self.pts[rand_pt_i], w, h)
        elif rng < 0.8 and n > 2:
            # Remove random pt in chromosome
            self.pts.pop(random.randint(1, n - 1))
        else:
            # Add random pt in chromosome
            rand_pt_i = random.randint(0, n - 1)
            self.pts.insert(rand_pt_i + 1, rand_pt_near_pt(self.pts[rand_pt_i], w, h))

    # delicious cost function
    def get_path_length(self):
        total = 0

        for i in range(1, len(self.pts)):
            total += math.sqrt((self.pts[i][0] - self.pts[i - 1][0]) ** 2 + (self.pts[i][1] - self.pts[i - 1][1]) ** 2)

        return total


class GASearch:
    MUTATION_PROB = 0.80

    def __init__(self, world, population_size=100):
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

        if dir[0] * dir[0] + dir[1] * dir[1] > 0:

            cumulative_movement = [0, 0]

            while mapX >= 0 and mapX < self.width and mapY >= 0 and mapY < self.height:
                if self.world_copy[mapY][mapX] > 0:
                    return True

                # here instead of in while condition to still do previous check
                if not (dir[0] != cumulative_movement[0] or dir[1] != cumulative_movement[1]):
                    break

                self.world_copy[mapY][mapX] = 1

                if sideDistX < sideDistY:
                    sideDistX += deltaDistX
                    mapX += stepX
                    cumulative_movement[0] += stepX
                else:
                    sideDistY += deltaDistY
                    mapY += stepY
                    cumulative_movement[1] += stepY

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

        # If path length is 0, bad score
        if summy == 0:
            return 100002

        # This is basically where the real fitness function is
        if final_pt[0] == goal[0] and final_pt[1] == goal[1]:
            # Fitness function if the chromosome ends at the goal
            return 1 - (1 / (summy * 10. + len(chromie.pts)))
        else:
            # Fitness function if the chromosome does not end at the goal
            # Is weighted sum of # pts, reciprocal of length of path, and distance
            # from end pt to goal. It is very bad
            return 0.2 * len(chromie.pts) + (300. / summy) + 1.0 * math.sqrt(
                (final_pt[0] - goal[0]) ** 2 + (final_pt[1] - goal[1]) ** 2)

    def search(self, start, goal, iters=100000):

        self.population = [GAChromosome() for i in range(self.population_size)]

        # Initalize population
        for chromie in self.population:
            chromie.pts = [start, self.rand_pt_in_world()]

        self.population.sort(key=lambda x: self._sort_key(x, goal))

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
                    pt1 = [pop_best_path.pts[i][0], pop_best_path.pts[i + 1][0]]
                    pt2 = [pop_best_path.pts[i][1], pop_best_path.pts[i + 1][1]]
                    plt.plot(pt1, pt2, marker='o', color='green')
                plt.draw()
                plt.show()
                plt.pause(0.001)

            # Replace bottom 2% with new chromosomes
            for i in range(int(self.population_size * 0.98), self.population_size - 1, 2):
                # Select parents from top 5%
                inds = random.sample(range(int(self.population_size * 0.05)), k=2)
                mom = self.population[inds[0]]
                dad = self.population[inds[1]]
                self.population[i], self.population[i + 1] = mom.breed_with(dad)

                # Mutation for child 1
                if random.random() < self.MUTATION_PROB:
                    self.population[i].mutate(self.width, self.height)

                # Mutation for child 2
                if random.random() < self.MUTATION_PROB:
                    self.population[i+1].mutate(self.width, self.height)

            random.shuffle(self.population)
            self.population.sort(key=lambda x: self._sort_key(x, goal))

        return self.population[0]
