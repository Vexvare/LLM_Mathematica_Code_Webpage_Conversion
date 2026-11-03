"""Python model regression checks for the interactive DH editor."""
from math import pi, sqrt
import numpy as np
import pytest

from ui.dh_2d_model import (
    PlanarDH, standard_dh_transform, forward_kinematics,
    collision_distances, configuration_space, COLLISION_CLEARANCE,
)


@pytest.mark.parametrize('theta1, theta2, a1, a2, tip', [
    (0, 0, 1, 1, (2,0)),
    (pi/2, 0, 1.5, 0.5, (0,2)),
    (0, pi, 1.5, 0.5, (1,0)),
    (pi/2, -pi/2, 1.5, 0.5, (0.5,1.5)),
    (0, pi/2, 2, 1, (2,1)),
    (pi, 0, 0.4, 0.7, (-1.1,0)),
    (pi/4, -pi/2, sqrt(2), sqrt(2), (2,0)),
])
def test_forward_kinematics(theta1,theta2,a1,a2,tip):
    pose=forward_kinematics(theta1,theta2,PlanarDH(a1,a2))
    np.testing.assert_allclose(pose.end_effector,tip,atol=1e-12)


def test_each_joint_uses_its_own_length():
    pose=forward_kinematics(0,pi/2,PlanarDH(1.5,0.5))
    np.testing.assert_allclose(pose.joint,[1.5,0],atol=1e-12)
    np.testing.assert_allclose(pose.end_effector,[1.5,0.5],atol=1e-12)


def test_two_homogeneous_transforms_compose():
    a1,a2=1.3,2.7
    t1,t2=0.31,2.13
    pose=forward_kinematics(t1,t2,PlanarDH(a1,a2))
    np.testing.assert_allclose(pose.transforms[1],
        standard_dh_transform(t1,0,a1,0)@standard_dh_transform(t2,0,a2,0))
    np.testing.assert_allclose(pose.transforms[1][:2,3],pose.end_effector)


def test_periodic_angles():
    geo=PlanarDH(1.2,2.4)
    a=forward_kinematics(-pi/2,3*pi/4,geo)
    b=forward_kinematics(3*pi/2,11*pi/4,geo)
    np.testing.assert_allclose(a.end_effector,b.end_effector,atol=1e-12)


def test_obstacle_at_first_link_is_collision():
    pose=forward_kinematics(0,0,PlanarDH(1.5,0.5))
    distances=collision_distances(pose,((0.5,0),(8,8)))
    assert distances[0,0] < COLLISION_CLEARANCE
    assert distances[1,1] > COLLISION_CLEARANCE


def test_obstacle_at_second_link_is_collision():
    pose=forward_kinematics(0,pi/2,PlanarDH(1.5,0.5))
    distances=collision_distances(pose,((1.5,0.3),(8,8)))
    assert distances[1,0] < COLLISION_CLEARANCE


@pytest.mark.parametrize('resolution', [32,64,160])
def test_cspace_grid_shape_and_binary(resolution):
    angles,blocked=configuration_space(PlanarDH(1.5,0.5),((1.1,1.1),(1.1,-1.1)),resolution)
    assert angles.shape==(resolution,)
    assert blocked.shape==(resolution,resolution)
    assert set(np.unique(blocked)).issubset({0,1})


def test_cspace_cells_match_forward_kinematics():
    geo=PlanarDH(1.5,0.5)
    obstacles=((1.1,1.1),(1.1,-1.1))
    angles,blocked=configuration_space(geo,obstacles,64)
    for i,j in ((0,0),(3,42),(42,3),(22,15),(60,60)):
        pose=forward_kinematics(np.radians(angles[i]),np.radians(angles[j]),geo)
        distances=collision_distances(pose,obstacles)
        assert bool(blocked[j,i])==bool(np.any(distances<COLLISION_CLEARANCE))


@pytest.mark.parametrize('a1,a2',[(0,1),(-1,1),(1,float('nan'))])
def test_invalid_lengths_rejected(a1,a2):
    with pytest.raises(ValueError):
        PlanarDH(a1,a2)
