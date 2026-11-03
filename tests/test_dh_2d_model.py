"""Unit and regression tests for the 2D DH extension."""
from math import pi, sin, cos
import numpy as np
import pytest

from ui.dh_2d_model import (
    PlanarDH, standard_dh_transform, forward_kinematics,
    collision_distances, configuration_space, COLLISION_CLEARANCE,
)


def test_zero_angles_original_geometry():
    p = forward_kinematics(0, 0)
    np.testing.assert_allclose(p.joint, [1, 0], atol=1e-12)
    np.testing.assert_allclose(p.end_effector, [2, 0], atol=1e-12)


def test_first_joint_90_degrees():
    p = forward_kinematics(pi/2, 0)
    np.testing.assert_allclose(p.joint, [0, 1], atol=1e-12)
    np.testing.assert_allclose(p.end_effector, [0, 2], atol=1e-12)


def test_second_joint_90_degrees_unequal_lengths():
    p = forward_kinematics(0, pi/2, PlanarDH(2, 3))
    np.testing.assert_allclose(p.joint, [2, 0], atol=1e-12)
    np.testing.assert_allclose(p.end_effector, [2, 3], atol=1e-12)


@pytest.mark.parametrize("t1,t2,a1,a2", [
    (0, 0, 1, 1), (0.3, 2.1, 0.8, 1.5), (2*pi, 2*pi, 2, 1.25),
    (-pi/4, pi/3, 2.4, 0.65), (pi, -pi/2, 0.3, 3.2),
])
def test_standard_dh_matches_analytic_fk(t1, t2, a1, a2):
    p = forward_kinematics(t1, t2, PlanarDH(a1, a2))
    expected = [a1*cos(t1)+a2*cos(t1+t2), a1*sin(t1)+a2*sin(t1+t2)]
    np.testing.assert_allclose(p.end_effector, expected, atol=1e-12)
    np.testing.assert_allclose(p.transforms[-1][:2,3], expected, atol=1e-12)


def test_standard_dh_general_transform_pure_translation():
    t = standard_dh_transform(0, 2, 3, 0)
    np.testing.assert_allclose(t[:3,3], [3,0,2])
    np.testing.assert_allclose(t[:3,:3], np.eye(3))


def test_periodic_angles_produce_identical_position():
    p1 = forward_kinematics(0.32, -1.17, PlanarDH(1.4, 0.7))
    p2 = forward_kinematics(0.32+2*pi, -1.17-2*pi, PlanarDH(1.4, 0.7))
    np.testing.assert_allclose(p1.end_effector, p2.end_effector, atol=1e-12)


def test_collision_at_link_center():
    p = forward_kinematics(0, 0)
    d = collision_distances(p, [(0.5, 0.0), (10.0, 10.0)])
    assert d.shape == (2,2)
    assert d[0,0] < COLLISION_CLEARANCE
    assert (d[:,1] >= COLLISION_CLEARANCE).all()


def test_strict_boundary_noncollision():
    p = forward_kinematics(0, 0)
    d = collision_distances(p, [(0.5, COLLISION_CLEARANCE)])
    assert d[0,0] == pytest.approx(COLLISION_CLEARANCE)
    assert not bool(d[0,0] < COLLISION_CLEARANCE)


def test_cspace_far_obstacle_clear():
    ang, grid = configuration_space(PlanarDH(), [(10,10)], resolution=32)
    assert ang.shape == (32,)
    assert grid.shape == (32,32)
    assert grid.sum() == 0


def test_cspace_matches_individual_collision_at_grid_centers():
    geo = PlanarDH(0.7, 1.8)
    obs = [(1.2, 0.9), (0.25,-0.1)]
    angles, grid = configuration_space(geo, obs, resolution=32)
    for i, j in [(0,0),(5,16),(16,5),(30,31),(11,21)]:
        pose = forward_kinematics(np.deg2rad(angles[i]), np.deg2rad(angles[j]), geo)
        actual = bool((collision_distances(pose, obs) < COLLISION_CLEARANCE).any())
        assert bool(grid[j,i]) == actual


@pytest.mark.parametrize("a1,a2", [(0,1),(1,0),(-1,1),(1,float('inf')),(float('nan'),1)])
def test_invalid_lengths_rejected(a1,a2):
    with pytest.raises(ValueError):
        PlanarDH(a1,a2)


def test_invalid_resolution_rejected():
    with pytest.raises(ValueError):
        configuration_space(PlanarDH(), [(0,0)], resolution=True)


def test_nonplanar_transform_not_supported_by_two_dimensional_page():
    # General standard-DH transform is valid, but the page fixes d and alpha to 0.
    t = standard_dh_transform(0, 0, 1, pi/2)
    assert t[1,2] == pytest.approx(-1)
