import math
import numpy as np
from queue import Queue
import pygame


TRAVERSABLE_CELL = 0
BLOCKED_CELL = 1
PARTIALLY_BLOCKED_CELL = 2
UNEXPLORED_CELL = 3


class TrackingException(Exception):
    pass


class NoRouteException(Exception):
    pass


def array_of_bytes(bytearr, size=None):
    if not size:
        size = int(math.sqrt(len(bytearr)))
    return np.array(bytearr).reshape((size, size))


class OccupancyGrid:
    def __init__(self, map_bytes, map_meters, grid_density):
        self._map_size = int(math.sqrt(len(map_bytes)))
        self._map = array_of_bytes(map_bytes, self._map_size)
        self._meters = map_meters
        self._density = grid_density  # Density is map pixels per grid cell
        self.__init_grid()

    @property
    def size(self):
        return self._grid_size

    @property
    def density(self):
        return self._density

    def _grid_cell_state(self, cell):
        # values, counts = np.unique(cell, return_counts=True)
        m = np.median(cell)
        epsilon = 20

        if abs(127-m) < epsilon:
            return UNEXPLORED_CELL
        elif abs(255-m) < epsilon:
            return TRAVERSABLE_CELL
        else:
            return BLOCKED_CELL
        # if len(values) >= 1 and values[0] > 180:
        #     return TRAVERSABLE_CELL
        # elif abs(np.median(values)-127) < 3:
        #     return UNEXPLORED_CELL
        # else:
        #     return BLOCKED_CELL

    def __init_grid(self):
        self._grid_size = int(self._map_size/self._density+0.5)
        self._grid = np.zeros((self._grid_size, self._grid_size))

        for x in range(self._grid_size):
            for y in range(self._grid_size):
                msx = slice(x*self._density, (x+1)*self._density, None)
                msy = slice(y*self._density, (y+1)*self._density, None)
                map_slice = self._map[msy, msx]
                # print(f"Cell {x},{y}")
                self._grid[y, x] = self._grid_cell_state(map_slice)

    def __getitem__(self, key):
        assert isinstance(key, tuple) and len(key) == 2  # Only 2d indexing

        x, y = key
        if self.in_bounds(x, y):
            return self._grid[x, y]
        else:
            return BLOCKED_CELL

    def in_bounds(self, x, y):
        return (0 <= x < self.size) and (0 <= y < self.size)

    def is_traversable(self, x, y):
        return self[x, y] in (TRAVERSABLE_CELL, UNEXPLORED_CELL)

    def _neighbours(self, p):
        deltas = ((-1, 0), (1, 0), (0, 1), (0, -1))
        px, py = p
        ns = []

        for (dx, dy) in deltas:
            nx, ny = px+dx, py+dy
            if self.in_bounds(nx, ny) and self.is_traversable(nx, ny):
                ns.append((nx, ny))

        return ns

    def get_path(self, point_a, point_b):
        """
        Return a list of cell coordinates representing a continuouspath between
        point_a and point_b. Raises an exception if path can not be traced.
        Path can be traced only by adjacent traversable cells within bounds of the map.
        """
        ax, ay = point_a
        bx, by = point_b

        if not (self.in_bounds(ax, ay) and self.is_traversable(ax, ay)):
            raise TrackingException("Starting point is out of bounds or not traversable")

        if not (self.in_bounds(bx, by) and self.is_traversable(bx, by)):
            raise TrackingException("Ending point is out of bounds or not traversable")

        frontier = Queue()
        frontier.put(point_a)
        came_from = {}
        came_from[point_a] = None

        while not frontier.empty():
            current = frontier.get()
            for n in self._neighbours(current):
                if n not in came_from:
                    frontier.put(n)
                    came_from[n] = current

        if point_b not in came_from:
            print(came_from)
            raise NoRouteException("Can't find a traversable route")

        current = point_b
        path = []
        while current != point_a:
            path.append(current)
            current = came_from[current]

        path.reverse()  # optional
        return path

    def to_map_coords(self, p):
        pa = np.array(p)
        return tuple(pa*self.density+self.density/2)


# class GridView:
#     def __init__(self, grid, screen, traversable_color, blocked_color):
#         self._grid = grid

#         self._tc = traversable_color
#         self._bc = blocked_color
#         self.screen = screen
#         self.screen_size = screen.get_size()

#     def _cell_surface(self, rect_size, color):
#         s = pygame.Surface(rect_size)
#         s.fill(color)
#         return s

#     def display(self, scale_factor=1):
#         # display width and height
#         dw, dh = self.screen_size

#         map_size = self._grid._map_size

#         map_scale_x = dw/map_size
#         map_scale_y = dh/map_size

#         gs = self._grid.size

#         screen_cell_w = map_size*map_scale_x/gs
#         screen_cell_h = map_size*map_scale_y/gs

#         cell_rect = (round(screen_cell_w), round(screen_cell_h))

#         t_cell = self._cell_surface(cell_rect, self._tc)
#         b_cell = self._cell_surface(cell_rect, self._bc)

#         for y in range(gs):
#             for x in range(gs):
#                 if self._grid.is_traversable(x, y):
#                     marker = t_cell
#                 else:
#                     marker = b_cell

#                 at = x*screen_cell_w, y*screen_cell_h
#                 self.screen.blit(marker, at, None, pygame.BLEND_RGB_SUB)
