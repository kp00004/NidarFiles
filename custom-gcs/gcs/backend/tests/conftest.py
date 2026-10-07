"""No test may open a real serial port: create_app() builds a RadioLink
by default, so swap it for the fake everywhere. Tests that need to
inspect radio traffic pass their own FakeRadioLink via create_app(radio=...)."""
import pytest

from .fakes import FakeRadioLink


@pytest.fixture(autouse=True)
def _no_real_radio(monkeypatch):
    import app.main

    monkeypatch.setattr(app.main, "RadioLink", lambda *args, **kwargs: FakeRadioLink())
