"""Tests for app/config.py's ROS-optional-mode plumbing -- _env_bool's
parsing logic and Settings.ros_enabled's default/override behavior. See
tests/test_ros_disabled.py for the API-level behavior this setting
drives."""

from __future__ import annotations

import pytest

from app.config import Settings, _env_bool


class TestEnvBool:
    def test_returns_default_when_unset(self, monkeypatch):
        monkeypatch.delenv("SOME_FLAG", raising=False)
        assert _env_bool("SOME_FLAG", True) is True
        assert _env_bool("SOME_FLAG", False) is False

    @pytest.mark.parametrize("value", ["true", "True", "TRUE", "1", "yes", "on", " true "])
    def test_recognizes_truthy_values_case_and_whitespace_insensitively(self, monkeypatch, value):
        monkeypatch.setenv("SOME_FLAG", value)
        assert _env_bool("SOME_FLAG", False) is True

    @pytest.mark.parametrize("value", ["false", "False", "0", "no", "off", "", "garbage"])
    def test_recognizes_falsy_and_unrecognized_values_as_false(self, monkeypatch, value):
        """An unrecognized value (typo, empty string, etc.) must degrade to
        False rather than raising -- see _env_bool's own docstring for why
        that's the safe direction for GCS_ROS_ENABLED specifically."""
        monkeypatch.setenv("SOME_FLAG", value)
        assert _env_bool("SOME_FLAG", True) is False


class TestSettingsRosEnabled:
    def test_defaults_to_true(self):
        # Matches the existing Jetson deployment's behavior exactly: ROS
        # connectivity is attempted unless explicitly opted out via
        # GCS_ROS_ENABLED=false. This is the "do not accidentally make
        # the Jetson deployment lose ROS connectivity" guarantee.
        assert Settings().ros_enabled is True

    def test_can_be_overridden_explicitly_for_ros_optional_local_development(self):
        assert Settings(ros_enabled=False).ros_enabled is False
