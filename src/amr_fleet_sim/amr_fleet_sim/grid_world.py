"""Deterministic 2D Grid World representation for MAPF evaluation."""

from typing import Iterable, List, Set, Tuple

Position = Tuple[int, int]


class GridWorld:
    """2D 4-connected Grid World with obstacle support."""

    def __init__(
        self,
        width: int,
        height: int,
        obstacles: Iterable[Position] = (),
        resolution: float = 1.0,
    ):
        """Initialize grid dimensions, obstacles, and metric resolution."""
        if width <= 0 or height <= 0:
            raise ValueError(f'Invalid dimensions: {width}x{height}')
        if resolution <= 0.0:
            raise ValueError(f'Invalid resolution: {resolution}')
        self.width = width
        self.height = height
        self.resolution = float(resolution)
        self.obstacles: Set[Position] = set()

        for ox, oy in obstacles:
            if 0 <= ox < width and 0 <= oy < height:
                self.obstacles.add((ox, oy))
            else:
                raise ValueError(f'Obstacle ({ox}, {oy}) is out of bounds for {width}x{height}')

    def to_grid(self, world_x: float, world_y: float) -> Position:
        """Convert continuous world coordinates in meters to discrete grid coordinates."""
        gx = int(world_x / self.resolution)
        gy = int(world_y / self.resolution)
        gx = max(0, min(self.width - 1, gx))
        gy = max(0, min(self.height - 1, gy))
        return (gx, gy)

    def to_world(self, grid_pos: Position) -> Tuple[float, float]:
        """Convert discrete grid coordinate to cell center in world coordinates."""
        wx = (grid_pos[0] + 0.5) * self.resolution
        wy = (grid_pos[1] + 0.5) * self.resolution
        return (round(wx, 3), round(wy, 3))

    def in_bounds(self, pos: Position) -> bool:
        """Check if position is inside grid bounds."""
        return 0 <= pos[0] < self.width and 0 <= pos[1] < self.height

    def is_free(self, pos: Position) -> bool:
        """Check if position is inside grid and not an obstacle."""
        return self.in_bounds(pos) and pos not in self.obstacles

    def add_obstacle(self, pos: Position) -> None:
        """Add an obstacle cell."""
        if self.in_bounds(pos):
            self.obstacles.add(pos)

    def remove_obstacle(self, pos: Position) -> None:
        """Remove an obstacle cell."""
        self.obstacles.discard(pos)

    def get_neighbors(self, pos: Position, allow_wait: bool = True) -> List[Position]:
        """Return valid 4-connected neighbor positions plus optional wait action."""
        if not self.is_free(pos):
            return []

        candidates = [
            (pos[0] + 1, pos[1]),
            (pos[0] - 1, pos[1]),
            (pos[0], pos[1] + 1),
            (pos[0], pos[1] - 1),
        ]
        valid = [c for c in candidates if self.is_free(c)]
        if allow_wait:
            valid.append(pos)
        return valid

    @staticmethod
    def manhattan_distance(p1: Position, p2: Position) -> int:
        """Compute Manhattan distance between two points."""
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    @staticmethod
    def has_vertex_conflict(pos_a: Position, pos_b: Position) -> bool:
        """Check if two agents occupy the exact same cell at the same timestep."""
        return pos_a == pos_b

    @staticmethod
    def has_edge_conflict(
        from_a: Position,
        to_a: Position,
        from_b: Position,
        to_b: Position,
    ) -> bool:
        """Check if two agents traverse the same edge in opposite directions (swap conflict)."""
        return from_a == to_b and to_a == from_b

    @classmethod
    def from_ascii(cls, ascii_map: str) -> 'GridWorld':
        """
        Parse a multiline ASCII grid.

        Format: '.' = free, '@' or 'T' or '#' = obstacle.
        """
        lines = [line.strip() for line in ascii_map.strip().splitlines() if line.strip()]
        if not lines:
            raise ValueError('Empty ASCII map')

        height = len(lines)
        width = len(lines[0])
        obstacles: Set[Position] = set()

        for y, line in enumerate(lines):
            if len(line) != width:
                msg = f'Inconsistent line length at line {y}: expected {width}, got {len(line)}'
                raise ValueError(msg)
            for x, char in enumerate(line):
                if char in ('@', 'T', '#'):
                    obstacles.add((x, y))

        return cls(width, height, obstacles)

    @classmethod
    def create_warehouse_grid(
        cls,
        resolution: float = 0.5,
        warehouse_size: float = 16.0,
    ) -> 'GridWorld':
        """
        Create a discrete GridWorld mapping the 16m x 16m warehouse environment.

        Captures outer perimeter boundary walls and industrial storage racks
        (rack_1 through rack_4) from warehouse_small.sdf.
        """
        width = int(round(warehouse_size / resolution))
        height = int(round(warehouse_size / resolution))
        obstacles: Set[Position] = set()

        # Storage racks from warehouse_small.sdf: (center_x, center_y, size_x, size_y)
        racks = [
            (4.5, 5.5, 1.2, 3.0),
            (4.5, 10.5, 1.2, 3.0),
            (11.5, 5.5, 1.2, 3.0),
            (11.5, 10.5, 1.2, 3.0),
        ]

        for gx in range(width):
            for gy in range(height):
                wx = (gx + 0.5) * resolution
                wy = (gy + 0.5) * resolution

                # Perimeter walls (0.4m border)
                if (
                    wx <= 0.4
                    or wx >= (warehouse_size - 0.4)
                    or wy <= 0.4
                    or wy >= (warehouse_size - 0.4)
                ):
                    obstacles.add((gx, gy))
                    continue

                # Storage racks
                for rx, ry, sx, sy in racks:
                    half_x = sx / 2.0
                    half_y = sy / 2.0
                    in_x = (rx - half_x) <= wx <= (rx + half_x)
                    in_y = (ry - half_y) <= wy <= (ry + half_y)
                    if in_x and in_y:
                        obstacles.add((gx, gy))
                        break

        return cls(width, height, obstacles, resolution=resolution)


create_warehouse_grid = GridWorld.create_warehouse_grid
