import numpy as np

from namo_sim.perception import ScanDetector, clearance_evidence
from namo_sim.planning import Grid


def detector(**kwargs):
    data = np.zeros((80, 100), dtype=np.int8)
    data[0:4, :] = 100
    data[-4:, :] = 100
    data[:, 0:4] = 100
    data[:, -4:] = 100
    return ScanDetector(Grid(data, 0.05, (-1.0, -2.0, 0.0)), **kwargs)


PATH = [(0.0, 0.0), (3.0, 0.0)]


def object_points(x=1.5):
    return np.column_stack((np.full(15, x), np.linspace(-0.4, 0.4, 15)))


def test_static_wall_noise_invalid_and_single_point_are_not_targets():
    det = detector()
    walls = np.column_stack((np.linspace(0, 2, 30), np.full(30, -1.79)))
    observations = np.vstack((walls, [[1, 0], [np.nan, 0], [0, np.inf]]))
    for _ in range(5):
        result = det.update(observations, PATH)
        assert result['target'] is None
        assert len(result['novel_points']) == 1


def test_persistence_and_observed_location_shift():
    for x in (1.5, 1.8):
        det = detector()
        assert det.update(object_points(x), PATH)['target'] is None
        assert det.update(object_points(x), PATH)['target'] is None
        target = det.update(object_points(x), PATH)['target']
        assert target['visible']
        assert np.allclose(target['center'], (x, 0))
        lost = det.update([], PATH)['target']
        assert not lost['visible']
        assert lost['bounds'] == target['bounds']


def test_consecutive_observations_required_and_off_path_cluster_rejected():
    det = detector()
    det.update(object_points(), PATH)
    det.update([], PATH)
    assert det.update(object_points(), PATH)['target'] is None
    assert det.update(object_points(), PATH)['target'] is None
    assert det.update(object_points(), PATH)['target'] is not None
    det.reset()
    for _ in range(5):
        assert det.update(object_points() + [0, 1.0], PATH)['target'] is None


def test_separate_unexpected_obstacle_is_reported():
    det = detector(persistence=1)
    det.update(object_points(), PATH)
    result = det.update(np.vstack((object_points(), object_points(2.5))), PATH)
    assert len(result['other_clusters']) == 1
    assert np.allclose(result['other_clusters'][0]['center'], (2.5, 0))


def test_clearance_requires_fresh_geometry_not_absence_or_occlusion():
    bounds = (1.5, -0.4, 1.55, 0.4)
    assert not clearance_evidence((0, 0), [], bounds)['clear']
    assert not clearance_evidence((0, 0), [[np.nan, 0]], bounds)['clear']
    endpoints = np.column_stack((np.full(15, 3.0), np.linspace(-0.7, 0.7, 15)))
    result = clearance_evidence((0, 0), endpoints, bounds)
    assert result['clear'] and result['through_rays'] == 15
    assert not clearance_evidence((0, 0), endpoints[:4], bounds)['clear']
    endpoints[7] = [1.0, 0.0]
    result = clearance_evidence((0, 0), endpoints, bounds)
    assert not result['clear'] and result['blocked_rays'] == 1
