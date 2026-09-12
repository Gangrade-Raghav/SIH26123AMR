"""Deterministic 2D Grid World representation for MAPF evaluation."""

from typing import Iterable, List, Set, Tuple

Position = Tuple[int, int]


class GridWorld:
    """2D 4-connected Grid World with obstacle support."""

    def __init__(self, width: int, height: int, obstacles: Iterable[Position] = ()):
        """Initialize grid dimensions and obstacles."""
        if width <= 0 or height <= 0:
            raise ValueError(f'Invalid dimensions: {width}x{height}')
        self.width = width
        self.height = height
        self.obstacles: Set[Position] = set()

        for ox, oy in obstacles:
            if 0 <= ox < width and 0 <= oy < height:
                self.obstacles.add((ox, oy))
            else:
                raise ValueError(f'Obstacle ({ox}, {oy}) is out of bounds for {width}x{height}')

    def in_bounds(self, pos: Position) -> bool:
        """Check if position is inside grid bounds."""
        return 0 <= pos[0] < self.width and 0 <= pos[1] < self.height

    def is_free(self, pos: Position) -> bool:
        """Check if position is inside grid and not an obstacle."""
        return self.in_bounds(pos) and pos not in self.obstacles

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
