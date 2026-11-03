from math import pi
import numpy as np
import pytest

from ui.dh_chain_model import (
    PlanarJoint as J, MAX_JOINTS, forward_chain, collision_distances,
    configuration_slice, rows_to_joints, COLLISION_CLEARANCE,
)


def test_default_2r_equals_old_fk():
    pose=forward_chain([J(), J()])
    np.testing.assert_allclose(pose.points, [[0,0],[1,0],[2,0]],atol=1e-12)


def test_two_unequal_link_lengths_gt_five():
    p=forward_chain([J(a=9,theta_deg=90),J(a=6,theta_deg=-90)])
    np.testing.assert_allclose(p.points, [[0,0],[0,9],[6,9]],atol=1e-12)


def test_prismatic_extension_and_fixed_heading():
    p=forward_chain([J(a=2,theta_deg=90),J(kind='P',a=1,theta_deg=-90,extension=4,stroke=6)])
    np.testing.assert_allclose(p.points, [[0,0],[0,2],[5,2]],atol=1e-12)


def test_three_revolute_joints():
    p=forward_chain([J(theta_deg=45),J(theta_deg=45),J(theta_deg=-90)])
    np.testing.assert_allclose(p.end_effector,[1+2**-.5,1+2**-.5],atol=1e-12)


def test_many_joints():
    joints=[J(a=6) for _ in range(MAX_JOINTS)]
    p=forward_chain(joints)
    assert len(p.points)==MAX_JOINTS+1
    np.testing.assert_allclose(p.end_effector,[6*MAX_JOINTS,0],atol=1e-12)


def test_too_many_rejected():
    with pytest.raises(ValueError):forward_chain([J() for _ in range(MAX_JOINTS+1)])


def test_p_stroke_rejected():
    with pytest.raises(ValueError):J(kind='P',extension=3,stroke=2)


def test_prismatic_slice_axis_range():
    joints=[J(a=2),J(kind='P',a=1,stroke=5,extension=1)]
    x,y,grid=configuration_slice(joints,[(9,9)],0,1,32)
    assert grid.shape==(32,32)
    assert x[0]>0 and x[-1]<360
    assert y[0]>0 and y[-1]<5
    assert not grid.any()


def test_slice_matches_forward_collision_for_mixed_chain():
    joints=[J(a=2,theta_deg=30), J(kind='P',a=1,theta_deg=20,stroke=2,extension=.5), J(a=1,theta_deg=-30)]
    obstacles=[(1,1),(3,1)]
    x,y,blocked=configuration_slice(joints,obstacles,0,1,32)
    for i,j in ((0,0),(4,9),(9,4),(17,20),(29,31)):
        scenario=[J(a=2,theta_deg=x[i]),J(kind='P',a=1,theta_deg=20,stroke=2,extension=y[j]),joints[2]]
        d=collision_distances(forward_chain(scenario),obstacles)
        assert bool(blocked[j,i])==bool(np.any(d<COLLISION_CLEARANCE))


def test_slice_holds_other_joints_fixed():
    joints=[J(a=1,theta_deg=15),J(a=1,theta_deg=40),J(a=1.5,theta_deg=30)]
    obstacles=[(1,0),(2,0)]
    x,y,grid=configuration_slice(joints,obstacles,0,2,32)
    for i,j in ((5,4),(9,23),(12,20)):
        case=[J(a=1,theta_deg=x[i]),joints[1],J(a=1.5,theta_deg=y[j])]
        assert bool(grid[j,i])==bool(np.any(collision_distances(forward_chain(case),obstacles)<COLLISION_CLEARANCE))


def test_invalid_axes():
    with pytest.raises(ValueError):configuration_slice([J(),J()],[(2,2)],1,1,32)


def test_rows_parser():
    rows=[{"Type":"R","a":10,"θ (deg)":90,"s":0,"Stroke":3},
          {"Type":"P","a":3,"θ (deg)":0,"s":2,"Stroke":5}]
    joints=rows_to_joints(rows)
    assert joints[0].current_length==10
    assert joints[1].current_length==5
