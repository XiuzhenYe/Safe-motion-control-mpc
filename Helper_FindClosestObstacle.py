import numpy as np


def find_closest_obstacle(q, obstacles):
        min_d = np.inf
        closest = None
        for center, r in obstacles:
            d = np.linalg.norm(q - center)
            if d < min_d:
                min_d = d
                closest = (center, r)
        center, r = closest
        d = np.linalg.norm(q - center)
        return center, d-r, r 