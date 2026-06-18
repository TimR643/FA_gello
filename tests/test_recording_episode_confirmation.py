from io import StringIO
import sys
import types


class _FakeNumpy(types.SimpleNamespace):
    ndarray = object

    def asarray(self, value, dtype=None):
        return value


sys.modules.setdefault(
    "numpy",
    _FakeNumpy(
        float32="float32",
        uint8="uint8",
        zeros=lambda size: [0] * size,
        abs=abs,
        argmax=lambda values: max(range(len(values)), key=values.__getitem__),
        arange=lambda size: list(range(size)),
        concatenate=lambda arrays: sum((list(array) for array in arrays), []),
    ),
)

from gello.utils.control_utils import LeRobotDatasetWriter, confirm_episode_keep


def test_confirm_episode_keep_defaults_to_keep_without_tty():
    output = StringIO()

    assert confirm_episode_keep(12, input_stream=StringIO(), output_stream=output)

    assert "12 frames" in output.getvalue()
    assert "keeping episode by default" in output.getvalue()


def test_discard_episode_uses_lerobot_clear_episode_buffer_when_available():
    class Dataset:
        def __init__(self):
            self.cleared = False

        def clear_episode_buffer(self):
            self.cleared = True

    writer = object.__new__(LeRobotDatasetWriter)
    writer.dataset = Dataset()

    writer.discard_episode()

    assert writer.dataset.cleared


def test_discard_episode_clears_mutable_episode_buffer_fallback():
    class Dataset:
        def __init__(self):
            self.episode_buffer = {
                "observation.state": [1, 2],
                "custom": {"frame": 1},
                "tuple_like": (1, 2),
            }

    writer = object.__new__(LeRobotDatasetWriter)
    writer.dataset = Dataset()

    writer.discard_episode()

    assert writer.dataset.episode_buffer["observation.state"] == []
    assert writer.dataset.episode_buffer["custom"] == {}
    assert writer.dataset.episode_buffer["tuple_like"] == []
