import numpy as np
import pytest

from franka_capture import bridge


@pytest.mark.parametrize(("opening", "closure"), [(1.0, 0.0), (0.75, 0.25), (0.0, 1.0)])
def test_fr3_startup_feedback_agrees_with_leader_closure(opening, closure):
    joints = [-0.1, 0.2, -0.3, -0.4, 0.5, 0.6, -0.7]
    raw_feedback = np.array([*joints, opening])

    startup_feedback = bridge.fr3_feedback_in_action_coordinates(raw_feedback)

    assert startup_feedback == [*joints, closure]
    startup_feedback[0] = 8.0
    np.testing.assert_array_equal(raw_feedback, [*joints, opening])


@pytest.mark.parametrize(
    "feedback",
    [
        [0.0] * 7,
        [0.0] * 9,
        [float("nan"), *([0.0] * 7)],
        [*([0.0] * 7), float("inf")],
        [*([0.0] * 7), -0.01],
        [*([0.0] * 7), 1.01],
        [*([0.0] * 7), "0.5"],
        np.zeros((8, 1)),
    ],
)
def test_fr3_startup_rejects_incompatible_feedback(feedback):
    with pytest.raises(ValueError, match="FR3 startup"):
        bridge.fr3_feedback_in_action_coordinates(feedback)
