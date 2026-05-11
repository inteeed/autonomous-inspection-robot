from inspection_robot.waypoint_state_machine import (
    choose_avoid_direction,
    marker_id_from_label,
    should_skip_label,
)


def test_marker_id_from_label():
    assert marker_id_from_label('approach_marker_4') == 4
    assert marker_id_from_label('return_home') is None


def test_should_skip_label():
    assert should_skip_label('approach_marker_2', {1, 2})
    assert not should_skip_label('approach_marker_3', {1, 2})
    assert not should_skip_label('return_home', {1, 2})


def test_choose_avoid_direction_prefers_clearer_side():
    assert choose_avoid_direction(1.0, 0.4) == 1
    assert choose_avoid_direction(0.2, 0.9) == -1
