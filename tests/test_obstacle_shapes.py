"""Tests of new editable obstacles and arbitrary 2D C-space slices."""
import numpy as np
import pytest
from ui.obstacle_geometry import Obstacle, SHAPES, obstacle_gaps, segment_obstacle_gap, polygon_vertices
from ui.dh_chain_model import PlanarJoint as J, forward_chain, configuration_slice


def test_all_shapes_and_validation():
    for shape in SHAPES:
        o = Obstacle(shape=shape)
        assert o.shape == shape
    with pytest.raises(ValueError):
        Obstacle(shape="Cube")
    with pytest.raises(ValueError):
        Obstacle(shape="Circle", radius=-1)


def test_default_circle_equals_original_point_obstacle_collision():
    pose=forward_chain([J(), J()])
    ob=Obstacle(x=.5,y=.2,radius=.125)
    assert np.isclose(obstacle_gaps(pose.points,[ob])[0,0],.2-.125-.125)


def test_circle_radius_affects_collision():
    small=Obstacle(x=.5,y=.45,radius=.1)
    large=Obstacle(x=.5,y=.45,radius=.4)
    assert segment_obstacle_gap(0,0,1,0,small) > 0
    assert segment_obstacle_gap(0,0,1,0,large) < 0


def test_square_dimension_affects_collision():
    small=Obstacle(x=.5,y=.45,shape="Square",width=.25)
    large=Obstacle(x=.5,y=.45,shape="Square",width=.9)
    assert segment_obstacle_gap(0,0,1,0,small) > 0
    assert segment_obstacle_gap(0,0,1,0,large) < 0


def test_rotated_rectangle_collision_and_rotation_direction():
    flat=Obstacle(x=.5,y=.4,shape="Rectangle",width=1,height=.1,rotation_deg=0)
    upright=Obstacle(x=.5,y=.4,shape="Rectangle",width=1,height=.1,rotation_deg=90)
    assert segment_obstacle_gap(0,0,1,0,flat) > 0
    assert segment_obstacle_gap(0,0,1,0,upright) < 0


def test_triangle_filled_and_rotated():
    triangle=Obstacle(x=.5,y=0,shape="Triangle",width=.4,height=.6)
    assert segment_obstacle_gap(0,0,1,0,triangle) < 0
    far=Obstacle(x=.5,y=1.5,shape="Triangle",width=.4,height=.6)
    assert segment_obstacle_gap(0,0,1,0,far) > 0
    verts=polygon_vertices(Obstacle(shape="Triangle",width=2,height=3))
    assert len(verts)==3


def test_polygon_segment_crosses_without_endpoints_inside():
    o=Obstacle(x=0,y=0,shape="Square",width=.5)
    assert segment_obstacle_gap(-1,0,1,0,o) < 0
    assert segment_obstacle_gap(-1,1,1,1,o) > 0


def test_large_distant_obstacle_parameters():
    o=Obstacle(x=100,y=-300,shape="Rectangle",width=75,height=20,rotation_deg=17)
    assert np.isfinite(polygon_vertices(o)).all()
    assert o.extent > 0


@pytest.mark.parametrize("shape",["Circle","Square","Rectangle","Triangle"])
def test_cspace_grid_uses_same_obstacle_shape_as_per_pose(shape):
    joints=[J(a=1.0),J(a=1.5,theta_deg=60),J(a=.7,theta_deg=-45),J(a=.8,theta_deg=30)]
    ob=Obstacle(x=1.3,y=1.,shape=shape,radius=.4,width=.9,height=.7,rotation_deg=30)
    x,y,grid=configuration_slice(joints,[ob],1,3,24)
    assert grid.shape==(24,24)
    for i,j in [(1,2),(5,9),(12,6),(14,20),(23,23)]:
        scenario=[joints[0],J(a=1.5,theta_deg=float(x[i])),joints[2],
                  J(a=.8,theta_deg=float(y[j]))]
        hit=bool(np.any(obstacle_gaps(forward_chain(scenario).points,[ob])<0))
        assert bool(grid[j,i])==hit, (shape,i,j)


def test_prismatic_axis_with_polygon():
    joints=[J(a=1,theta_deg=45),J(kind="P",a=.5,stroke=2,extension=.2,theta_deg=-45),J(a=1)]
    obstacle=Obstacle(x=2,y=0,shape="Square",width=.6,rotation_deg=30)
    xs,ys,grid=configuration_slice(joints,[obstacle],1,2,16)
    assert xs[0] > 0 and xs[-1] < 2
    assert ys[0] > 0 and ys[-1] < 360
    for i,j in [(1,2),(6,8),(11,14)]:
        scenario=[joints[0],J(kind="P",a=.5,stroke=2,extension=float(xs[i]),theta_deg=-45),
                  J(a=1,theta_deg=float(ys[j]))]
        assert bool(grid[j,i])==bool(np.any(obstacle_gaps(forward_chain(scenario).points,[obstacle])<0))
