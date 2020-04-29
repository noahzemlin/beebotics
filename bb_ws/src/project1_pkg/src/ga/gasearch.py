import random
import math
import copy
import matplotlib.pyplot as plt

DEBUG = False

def clamp(x, low, high):
    if x < low:
        return low
    if x > high:
        return high
    return x


def rand_pt_near_pt(pt, w, h):
    return clamp(pt[0] + random.randrange(-5, 6), 0, w - 1), clamp(pt[1] + random.randrange(-5, 6), 0, h - 1)


class GAChromosome:

    def __init__(self):
        self.pts = []
        self.score = -1

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
        self.score = -1  # No longer know score

        rng = random.random()
        n = len(self.pts)
        if rng < 0.5 and n > 2:
            # Remove random section from chromosomes
            left_end = random.randrange(2, n)
            right_end = random.randrange(left_end, n+1)
            self.pts = self.pts[:left_end] + self.pts[right_end:]

            # this is intentionally in the delete if
            if rng > 0.3:
                # Add random pt to end of chromosome
                self.pts.insert(left_end-1, rand_pt_near_pt(self.pts[left_end-1], w, h))
        elif rng < 0.9:
            # Add random pt to chromosome
            i = random.randrange(1, n)
            self.pts.insert(i, rand_pt_near_pt(self.pts[i], w, h))
        else:
            self.pts.append(rand_pt_near_pt(self.pts[-1], w, h))

    # delicious cost function
    def get_path_length(self):
        total = 0

        for i in range(1, len(self.pts)):
            total += math.sqrt((self.pts[i][0] - self.pts[i - 1][0]) ** 2 + (self.pts[i][1] - self.pts[i - 1][1]) ** 2)

        return total


class GASearch:
    MUTATION_PROB = 0.80

    def __init__(self, population_size=100):
        self.population = [GAChromosome() for _ in range(population_size)]
        self.population_size = population_size

        self.last_best = 100000000
        self.iterations_without_improvement = 0

    def set_world(self, world):
        self.world = world
        self.height = len(world)
        self.width = len(world[0])

    def rand_pt_in_world(self):
        return random.randint(0, self.width - 1), random.randint(0, self.height - 1)

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

            while 0 <= mapX < self.width and 0 <= mapY < self.height:
                if self.world_copy[mapY][mapX] > 0:
                    return True

                # here instead of in while condition to still do previous check
                if not (dir[0] != cumulative_movement[0] or dir[1] != cumulative_movement[1]):
                    break

               # self.world_copy[mapY][mapX] = 1

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
        if chromie.score != -1:
            return chromie.score

        first_pt = chromie.pts[0]
        final_pt = chromie.pts[-1:][0]
        summy = chromie.get_path_length()

        #self.world_copy = copy.deepcopy(self.world)
        self.world_copy = self.world
        for i in range(0, len(chromie.pts) - 1):
            fp = chromie.pts[i]
            lp = chromie.pts[i + 1]

            # Make sure there are no collisisons
            if self.raycast(fp, (lp[0] - fp[0], lp[1] - fp[1])):
                chromie.score = 100001
                return 100001

            # Make sure last point is not in obstruction
            # TODO: put in raycast
            # if self.world_copy[lp[1]][lp[0]] == 1:
            #     chromie.score = 100003
            #     return 100003

        # If path length is 0, bad score
        if summy == 0:
            chromie.score = 100002
            return 100002

        # This is basically where the real fitness function is
        if final_pt[0] == goal[0] and final_pt[1] == goal[1]:
            # Fitness function if the chromosome ends at the goal
            chromie.score = 1 - (1 / (summy * 10. + len(chromie.pts)))
            return chromie.score
        else:
            # Fitness function if the chromosome does not end at the goal
            # Is weighted sum of # pts, reciprocal of length of path, and distance
            # from end pt to goal. It is very bad

            dist_to_goal = math.sqrt((final_pt[0] - goal[0]) ** 2 + (final_pt[1] - goal[1]) ** 2)
            dist_from_start = math.sqrt((final_pt[0] - first_pt[0]) ** 2 + (final_pt[1] - first_pt[1]) ** 2)

            chromie.score = 10000 + 0.3 * len(chromie.pts) + 1 * dist_to_goal

            if dist_to_goal >= 120: # When far away, use distance from start
                chromie.score -= 0.95 * dist_from_start
            else:
                chromie.score -= 2000

            return chromie.score

    def search(self, start, goal, init_pop=True, iters=1000):

        if init_pop:
            # Initalize population
            for chromie in self.population:
                chromie.pts = [start, rand_pt_near_pt(start, self.width, self.height)]

        self.population.sort(key=lambda x: self._sort_key(x, goal))

        for iter_num in range(iters):
            pop_best_path = self.population[0]

            # Set new best
            new_best = self._sort_key(pop_best_path, goal)
            if new_best < self.last_best:
                self.last_best = new_best
                self.iterations_without_improvement = 0

                if DEBUG:
                    print("score: ", new_best)
                    print("length: ", pop_best_path.get_path_length())
                    print(pop_best_path.pts)
                    plt.clf()
                    plt.imshow(self.world, interpolation='none')
                    plt.ion()
                    for i in range(0, len(pop_best_path.pts) - 1):
                        pt1 = [pop_best_path.pts[i][0], pop_best_path.pts[i + 1][0]]
                        pt2 = [pop_best_path.pts[i][1], pop_best_path.pts[i + 1][1]]
                        plt.plot(pt1, pt2, marker='o', color='green')
                    plt.title("iter: " + str(iter_num + 1))
                    plt.draw()
                    plt.show()
                    plt.pause(0.00001)
            else:
                self.iterations_without_improvement += 1
                if self.iterations_without_improvement >= 250:
                    # If we go 100 iters without improvement, remove last pt from all chromosomes
                    for chromie in self.population:
                        if len(chromie.pts) > 3 and random.random() < 0.8:
                            chromie.pts.pop(len(chromie.pts)-1)
                            chromie.score = -1

                    self.last_best = 100000000

            # Replace bottom 3% with new chromosomes
            for i in range(int(self.population_size * 0.97), self.population_size - 1, 2):
                # Select parents from top 40%
                inds = random.sample(range(int(self.population_size * 0.40)), k=2)
                mom = self.population[inds[0]]
                dad = self.population[inds[1]]
                self.population[i], self.population[i + 1] = mom.breed_with(dad)

                # Mutation for child 1
                if random.random() < self.MUTATION_PROB:
                    self.population[i].mutate(self.width, self.height)

                # Mutation for child 2
                if random.random() < self.MUTATION_PROB:
                    self.population[i + 1].mutate(self.width, self.height)

            random.shuffle(self.population)
            self.population.sort(key=lambda x: self._sort_key(x, goal))

        return self.population[0]