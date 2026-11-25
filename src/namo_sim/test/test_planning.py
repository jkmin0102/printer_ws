"""Small tests exercising safety boundaries and the actual time decision rule."""
import math
import numpy as np
import pytest
from namo_sim.planning import Grid, PlanError, choose_action, path_length


def test_rotated_map_origin_and_exact_endpoints():
    grid = Grid(np.zeros((30, 30)), 0.1, (2.0, -3.0, math.pi/2))
    assert grid.cell_to_world((4, 6)) == pytest.approx((1.35, -2.55))
    assert grid.world_to_cell((1.35, -2.55)) == (4, 6)
    start, goal = grid.cell_to_world((5, 5)), grid.cell_to_world((23, 18))
    path = grid.astar(start, goal)
    assert path[0] == start
    assert path[-1] == goal
    assert path_length(path) >= math.dist(start, goal)
    assert grid.world_to_cell((2.1, -3.0))[1] < 0


def test_diagonal_cannot_cross_two_wall_corners():
    grid = Grid([[0, 100], [100, 0]], 1.0)
    with pytest.raises(PlanError) as error:
        grid.astar((0.5, 0.5), (1.5, 1.5), radius=0)
    assert error.value.code == 'NO_PATH'


def test_footprint_blocks_narrow_passage_and_unknown():
    data = np.zeros((25, 25))
    data[:, 10:15] = 100
    data[11:14, 10:15] = 0  # 0.3 m passage: point can pass, 0.56 m robot cannot.
    grid = Grid(data, 0.1)
    assert grid.astar((0.55, 1.25), (1.95, 1.25), radius=0)
    with pytest.raises(PlanError) as error:
        grid.astar((0.55, 1.25), (1.95, 1.25), radius=0.28)
    assert error.value.code == 'NO_PATH'
    unknown = Grid([[0, -1, 0]], 1.0)
    with pytest.raises(PlanError):
        unknown.astar((0.5, 0.5), (2.5, 0.5), radius=0)


def test_dynamic_overlay_does_not_mutate_static_map():
    original = np.zeros((25, 25))
    grid = Grid(original, 0.1)
    start, goal = (0.55, 1.25), (1.95, 1.25)
    baseline = grid.astar(start, goal)
    wall = [(1.25, (j+0.5)*0.1) for j in range(25)]
    with pytest.raises(PlanError) as error:
        grid.astar(start, goal, extra_points=wall)
    assert error.value.code == 'NO_PATH'
    np.testing.assert_array_equal(grid.data, original)
    assert grid.astar(start, goal) == baseline


def test_invalid_endpoint_and_map_boundary_are_not_clipped():
    grid = Grid(np.zeros((20, 20)), 0.1)
    with pytest.raises(PlanError) as error:
        grid.astar((-0.1, 0.8), (1., 1.))
    assert error.value.code == 'INVALID_START'
    with pytest.raises(PlanError) as error:
        grid.astar((1., 1.), (1.95, 1.))
    assert error.value.code == 'INVALID_GOAL'
    with pytest.raises(ValueError):
        path_length([])


def test_cost_rule_not_scenario_label():
    a = choose_action(6., 16., v_eff=.2, mock_remove=22., margin=2.)
    b = choose_action(6., 9., v_eff=.2, mock_remove=22., margin=2.)
    assert a.action == 'REMOVE'
    assert b.action == 'DETOUR'
    assert (a.t_remove, a.t_detour) == (52., 80.)
    assert choose_action(6., 10.8, mock_remove=22., margin=2.).action == 'DETOUR'
    assert choose_action(6., 10.8001, mock_remove=22., margin=2.).action == 'REMOVE'


def test_missing_data_is_not_a_missing_route():
    assert choose_action(6., None).action == 'REMOVE'
    assert choose_action(6., None, data_ready=False).action == 'STOP'
    assert choose_action(None, 9.).action == 'DETOUR'
    assert choose_action(6., 16., removable=False).action == 'DETOUR'
    assert choose_action(6., None, removable=False).action == 'STOP'
    assert choose_action(None, None).action == 'STOP'
    assert choose_action(6., math.inf).action == 'STOP'
    assert choose_action(6., 9., v_eff=0).action == 'STOP'
