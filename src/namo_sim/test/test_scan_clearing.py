import math
import pytest
from namo_sim.scan_clearing import conservative_ranges


def test_continuous_neighbors_only_shorten_known_free_range():
    result=conservative_ranges([2.,2.1],.12,3.5)
    assert len(result)==5
    assert result==pytest.approx([1.98,1.98,1.98,1.98,2.08])


def test_edges_and_invalid_rays_not_bridged():
    for pair in ([1.,2.],[math.nan,2.],[math.inf,3.4],[-math.inf,2.],[4.,2.]):
        assert all(math.isnan(x) for x in conservative_ranges(pair,.12,3.5)[1:4])


def test_two_no_return_rays_clear_only_up_to_known_sensor_range():
    assert conservative_ranges([math.inf,math.inf],.12,3.5)==pytest.approx([3.48]*5)


def test_original_angles_retained_and_no_extension_at_near_limit():
    ranges=conservative_ranges([.12,1.,1.05],.12,3.5)
    assert len(ranges)==9 and math.isnan(ranges[0])
    assert ranges[4]==pytest.approx(.98) and ranges[8]==pytest.approx(1.03)
    assert conservative_ranges([],.12,3.5)==[]
