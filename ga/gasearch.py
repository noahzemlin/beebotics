import random
import math

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
    def get_sqr_path_length(self):
        total = 0

        for i in range(1, len(self.pts)):
            total += math.sqrt((self.pts[i][0] - self.pts[i-1][0])**2 + (self.pts[i][1] - self.pts[i-1][1])**2)

        return total
        

class GASearch:

    MUTATION_PROB = 0.15

    def __init__(self, world, population_size = 100):
        self.world = world
        self.height = len(world)
        self.width = len(world[0])
        self.population_size = population_size

    def rand_pt_in_world(self):
        return (random.randint(0, self.width - 1), random.randint(0, self.height - 1))

    def raycast(self, pos, dir):

        deltaDistX = 0 if dir[1] == 0 else 1 if dir[0] == 0 else abs(1 / dir[0])
        deltaDistY = 0 if dir[0] == 0 else 1 if dir[1] == 0 else abs(1 / dir[1])

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
                if self.world[mapY][mapX] > 0:
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
            if self.world[mapY][mapX] > 0:
                return True

        return False

    def _sort_key(self, chromie, goal):
        final_pt = chromie.pts[-1:][0]
        summy = chromie.get_sqr_path_length()
        for i in range(0, len(chromie.pts) - 1):
            fp = chromie.pts[i]
            lp = chromie.pts[i + 1]
            if lp in chromie.pts[:i+1]:
                return 10000
            if self.raycast(fp, (lp[0] - fp[0], lp[1] - fp[1])):
                return 100000 # bad dont hit pls

        # pls move
        if summy == 0:
            return 100000

        if final_pt[0] == goal[0] and final_pt[1] == goal[1]:
            # if valid, reward it
            print("found goal")
            return 1 - (1 / summy)
        else:
            # is this D* now?
            return len(chromie.pts)**1.4 + math.sqrt((final_pt[0] - goal[0])**2 + (final_pt[1] - goal[1])**2) - summy


    def search(self, start, goal, iters = 100000):
        self.population = [GAChromosome() for i in range(self.population_size)]

        for chromie in self.population:
            chromie.pts = [start, self.rand_pt_in_world()]

        self.population.sort(key = lambda x: self._sort_key(x, goal))

        last_best = 1000

        for _ in range(iters):
            god = self.population[0]

            new_best = self._sort_key(god, goal)
            if new_best < last_best:
                print("score: ", new_best)
                print("length: ", god.get_sqr_path_length())
                print(god.pts)
                last_best = new_best

            for i in range(int(self.population_size//1.2), self.population_size-1, 2):
                mom = self.population[random.randint(0, int(self.population_size//1.2))]
                dad = self.population[random.randint(0, int(self.population_size//1.2))]
                self.population[i], self.population[i+1] = mom.breed_with(dad)

                if random.random() < self.MUTATION_PROB:
                    if random.random() < 0.5:
                        n = len(self.population[i].pts)
                        self.population[i].pts[random.randint(1, n-1)] = self.rand_pt_in_world()
                    else:
                        self.population[i].pts.append(self.rand_pt_in_world())

                if random.random() < self.MUTATION_PROB:
                    if random.random() < 0.5:
                        n = len(self.population[i+1].pts)
                        self.population[i+1].pts[random.randint(1, n-1)] = self.rand_pt_in_world()
                    else:
                        self.population[i+1].pts.append(self.rand_pt_in_world())
                    
            random.shuffle(self.population)
            self.population.sort(key = lambda x: self._sort_key(x, goal))

        return self.population[0]