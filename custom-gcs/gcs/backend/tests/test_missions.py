"""Tests for app/missions.py -- the static mission list behind the GCS
Mission dropdown. Pure data/lookup tests, no FastAPI/TestClient needed."""

import pytest

from app.missions import MISSION_REGISTRY, MissionNotFoundError, get_mission


def test_missions_are_hover_and_motor_test():
    assert [m.id for m in MISSION_REGISTRY] == ["hover", "motor_test"]
    assert get_mission("hover").name == "Hover"
    assert get_mission("motor_test").name == "Motor Test"


def test_radio_codes_match_the_jetson_table():
    # onboard-autonomy telem_command_codec: MISSION_HOVER == 1, MISSION_MOTOR_TEST == 2
    assert get_mission("hover").radio_code == 1
    assert get_mission("motor_test").radio_code == 2


def test_radio_codes_are_unique_and_nonzero():
    codes = [m.radio_code for m in MISSION_REGISTRY]
    assert len(codes) == len(set(codes))
    assert all(0 < c < 2**24 for c in codes)


def test_unknown_mission_raises():
    with pytest.raises(MissionNotFoundError):
        get_mission("main_nidar")
