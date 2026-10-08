"""rclpy raises "Logger severity cannot be changed between calls" when one
logging call site is used with different severities -- e.g.
`log = logger.info if ok else logger.error; log(...)`. That crashed the
hover node right after a TAKEOFF reply on the bench (2026-10-07). This
scan keeps the pattern out of the Jetson code."""
import os
import re

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DIRS = [
    os.path.join(_ROOT, "missions"),
    os.path.join(_ROOT, "onboard-autonomy", "nidar_autonomy", "nidar_autonomy"),
]
_PATTERN = re.compile(r"=\s*[\w.()]*get_logger\(\)\.\w+\s+if\b|=\s*[\w.]*log(ger)?\.\w+\s+if\b.*\belse\b")


def test_no_logger_method_chosen_at_runtime():
    offenders = []
    for base in _DIRS:
        for folder, _, files in os.walk(base):
            for name in files:
                if not name.endswith(".py") or name == "test_logging_call_sites.py":
                    continue
                path = os.path.join(folder, name)
                with open(path, encoding="utf-8") as fh:
                    for number, line in enumerate(fh, 1):
                        if _PATTERN.search(line):
                            offenders.append(f"{path}:{number}: {line.strip()}")
    assert offenders == []


def test_no_undefined_names_in_jetson_code():
    """pyflakes over the mission and Jetson code: an undefined name in a
    rarely-taken branch (e.g. an in-flight safety check) only fails when
    that branch runs -- in the air. Found by review 2026-10-08."""
    import pytest

    api = pytest.importorskip("pyflakes.api")
    reporter_mod = pytest.importorskip("pyflakes.reporter")
    import io

    out, err = io.StringIO(), io.StringIO()
    reporter = reporter_mod.Reporter(out, err)
    for base in _DIRS:
        api.checkRecursive([base], reporter)
    problems = [line for line in out.getvalue().splitlines() if "undefined name" in line]
    assert problems == [], "\n".join(problems)
