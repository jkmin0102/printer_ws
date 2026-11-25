"""Geometry and ROS image decoding tests; these do not validate trained YOLO."""
from types import SimpleNamespace

import numpy as np
import pytest

from namo_manipulation.can_perception import (
    center_from_surface, check_support_plane, fit_lid_circle, fit_side_circle, image_array, red_detection, rotation_matrix,
)


def test_partial_label_side_arc_recovers_center_without_bbox_center_assumption():
    angles=np.linspace(.2,1.9,100)
    center=np.array([.345,-.091])
    points=center+.025*np.column_stack((np.cos(angles),np.sin(angles)))
    fitted,quality=fit_side_circle(points)
    assert fitted==pytest.approx(center,abs=.0003)
    assert quality['side_radius_m']==pytest.approx(.025,abs=.0003)


def test_narrow_arc_cannot_authorize_a_grasp():
    angles=np.linspace(.2,.7,100)
    with pytest.raises(ValueError):
        fit_side_circle(.025*np.column_stack((np.cos(angles),np.sin(angles))))


def test_flat_surface_and_wrong_diameter_are_rejected():
    with pytest.raises(ValueError):
        fit_side_circle(np.column_stack((np.linspace(0,.05,100),np.ones(100))))
    angles=np.linspace(0,2,100)
    with pytest.raises(ValueError):
        fit_side_circle(.05*np.column_stack((np.cos(angles),np.sin(angles))))


def test_side_surface_is_shifted_one_radius_away_from_camera():
    center, method = center_from_surface([.30, 0., .0675], [-.20, 0., .20])
    np.testing.assert_allclose(center, [.325, 0., .0675])
    assert method == 'upright_side_plus_radius'


def test_lid_center_does_not_receive_wrong_horizontal_radius_shift():
    center, method = center_from_surface([.325, .02, .135], [.30, .02, .30])
    np.testing.assert_allclose(center, [.325, .02, .0675])
    assert method == 'upright_lid_center'


def test_support_plane_height_is_part_of_center_assumption():
    center, _ = center_from_surface([.30, 0., .3675], [0., 0., .60], support_z=.30)
    assert center[2] == pytest.approx(.3675)


@pytest.mark.parametrize('surface', [[.3, 0., 0.], [.3, 0., .25], [float('nan'), 0., .06]])
def test_reject_floor_background_impossible_height_and_nan(surface):
    with pytest.raises(ValueError):
        center_from_surface(surface, [0., 0., .2])


def test_depth_encoding_respects_big_endian_and_row_padding():
    raw = np.array([[100, 200, 999], [300, 400, 999]], dtype='>u2').tobytes()
    message = SimpleNamespace(encoding='16UC1', is_bigendian=1, width=2, height=2, step=6, data=raw)
    np.testing.assert_array_equal(image_array(message), [[100, 200], [300, 400]])


def test_rgb_decoding_handles_padded_row_and_returns_bgr():
    message = SimpleNamespace(encoding='rgb8', is_bigendian=0, width=1, height=1,
                              step=4, data=bytes([255, 10, 20, 99]))
    np.testing.assert_array_equal(image_array(message), [[[20, 10, 255]]])


def test_truncated_image_is_not_treated_as_a_valid_observation():
    message = SimpleNamespace(encoding='32FC1', is_bigendian=0, width=2, height=1,
                              step=8, data=b'\0'*4)
    with pytest.raises(ValueError, match='Truncated'):
        image_array(message)


def test_full_quaternion_rotation_for_optical_point():
    r = rotation_matrix([0., 0., np.sin(np.pi/4), np.cos(np.pi/4)])
    np.testing.assert_allclose(r@np.array([1., 0., 0.]), [0., 1., 0.], atol=1e-12)


def test_explicit_red_segmentation_has_no_fake_yolo_confidence():
    bgr = np.zeros((40, 60, 3), dtype=np.uint8)
    bgr[5:25, 20:30, 2] = 220
    detection, mask = red_detection(bgr)
    assert detection.class_name == 'SIM_RED_color_proxy'
    assert detection.confidence == 0.
    assert np.count_nonzero(mask) == 200
    missing, _ = red_detection(np.zeros_like(bgr))
    assert missing is None


def test_known_floor_depth_confirms_assumed_support_plane():
    depth = np.ones((120, 160))
    result = check_support_plane(depth, [100, 0, 80, 0, 100, 60, 0, 0, 1],
                                [65, 20, 95, 60], np.diag([1, -1, -1]), [0, 0, 1])
    assert result['support_plane_error_m'] == pytest.approx(0.)


def test_depth_tf_mismatch_cannot_authorize_plausible_can_pose():
    depth = np.ones((120, 160))*.95
    with pytest.raises(ValueError, match='Support-plane/TF mismatch'):
        check_support_plane(depth, [100, 0, 80, 0, 100, 60, 0, 0, 1],
                            [65, 20, 95, 60], np.diag([1, -1, -1]), [0, 0, 1])


def test_white_print_inside_observed_red_silhouette_retains_depth_support():
    bgr = np.zeros((40, 60, 3), dtype=np.uint8)
    bgr[5:30, 20:40, 2] = 220
    bgr[12:20, 25:33] = 255
    detection, mask = red_detection(bgr)
    assert detection is not None
    assert mask[15, 28] == 255
    assert mask[15, 5] == 0


def test_complete_measured_lid_boundary_recovers_center_without_object_pose():
    angle = np.linspace(0, 2*np.pi, 100, endpoint=False)
    points = np.array([.31, -.04])+.025*np.column_stack((np.cos(angle), np.sin(angle)))
    center, quality = fit_lid_circle(points)
    np.testing.assert_allclose(center, [.31, -.04], atol=1e-7)
    assert quality['lid_radius_m'] == pytest.approx(.025, abs=1e-7)


@pytest.mark.parametrize('fraction,radii', [(0.45, (.025, .025)), (1., (.04, .018))])
def test_partial_arc_or_wrong_shape_cannot_authorize_can_center(fraction, radii):
    angle = np.linspace(0, 2*np.pi*fraction, 100)
    points = np.column_stack((radii[0]*np.cos(angle), radii[1]*np.sin(angle)))
    with pytest.raises(ValueError, match='complete known-size'):
        fit_lid_circle(points)
