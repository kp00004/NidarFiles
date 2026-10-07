import pytest

from nidar_autonomy.flight_command_interface import (
    FlightCommandError,
    FlightCommandInterface,
    InvalidCommandError,
    NotArmedError,
)


def test_interface_cannot_be_instantiated_directly():
    with pytest.raises(TypeError):
        FlightCommandInterface()  # abstract -- must fail loud, not silently


def test_exception_hierarchy():
    assert issubclass(NotArmedError, FlightCommandError)
    assert issubclass(InvalidCommandError, FlightCommandError)
    # the two concrete cases must stay distinguishable from each other
    assert not issubclass(NotArmedError, InvalidCommandError)
    assert not issubclass(InvalidCommandError, NotArmedError)


def test_incomplete_implementation_cannot_be_instantiated():
    class Incomplete(FlightCommandInterface):
        # deliberately missing every abstract method
        pass

    with pytest.raises(TypeError):
        Incomplete()
