"""Unit tests for amr_fleet_sim GridWorld model."""

from amr_fleet_sim.grid_world import GridWorld
import pytest


def test_grid_initialization():
    grid = GridWorld(5, 5, obstacles=[(2, 2), (2, 3)])
    assert grid.width == 5
    assert grid.height == 5
    assert (2, 2) in grid.obstacles
    assert grid.is_free((0, 0))
    assert not grid.is_free((2, 2))
    assert not grid.is_free((10, 10))


def test_invalid_dimensions():
    with pytest.raises(ValueError):
        GridWorld(0, 5)
    with pytest.raises(ValueError):
        GridWorld(5, -1)


def test_out_of_bounds_obstacle():
    with pytest.raises(ValueError):
        GridWorld(5, 5, obstacles=[(5, 5)])


def test_neighbors_with_obstacles():
    grid = GridWorld(3, 3, obstacles=[(1, 0), (0, 1)])
    # At (0,0), neighbors are (0,0) [wait] since (1,0) and (0,1) are obstacles
    neighbors = grid.get_neighbors((0, 0), allow_wait=True)
    assert neighbors == [(0, 0)]

    neighbors_no_wait = grid.get_neighbors((0, 0), allow_wait=False)
    assert neighbors_no_wait == []


def test_manhattan_distance():
    assert GridWorld.manhattan_distance((0, 0), (3, 4)) == 7
    assert GridWorld.manhattan_distance((2, 2), (2, 2)) == 0


def test_vertex_and_edge_conflicts():
    # Vertex conflict: same spot at same time
    assert GridWorld.has_vertex_conflict((2, 3), (2, 3))
    assert not GridWorld.has_vertex_conflict((2, 3), (2, 4))

    # Edge conflict: swap collision (Agent A: (1,1)->(1,2), Agent B: (1,2)->(1,1))
    assert GridWorld.has_edge_conflict((1, 1), (1, 2), (1, 2), (1, 1))
    # Non-conflicting traverse in same direction
    assert not GridWorld.has_edge_conflict((1, 1), (1, 2), (1, 1), (1, 2))


def test_from_ascii():
    ascii_map = """
    ...
    .@.
    ...
    """
    grid = GridWorld.from_ascii(ascii_map)
    assert grid.width == 3
    assert grid.height == 3
    assert not grid.is_free((1, 1))
    assert grid.is_free((0, 0))
