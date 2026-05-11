"""Tests for pid_optimizer.params_io module."""

import pytest
from pid_optimizer.params_io import load_params, save_params
from pid_optimizer.gains import Gains


def test_save_creates_valid_file(tmp_path):
    """Test that save_params creates a file with correct header and format."""
    gains = Gains()
    path = tmp_path / "params.txt"

    save_params(gains, str(path))

    # Read back and check format
    with open(path, "r") as f:
        lines = f.readlines()

    # First line should be the header comment
    assert lines[0] == "# Exported by pid_optimizer\n"

    # Should have at least 23 lines (header + all params)
    assert len(lines) >= 24

    # At least one line should match the MC_ROLLRATE_P pattern
    found_rollrate_p = False
    for line in lines[1:]:
        if line.startswith("MC_ROLLRATE_P\t"):
            found_rollrate_p = True
            break
    assert found_rollrate_p


def test_roundtrip_default_gains(tmp_path):
    """Test that save and load preserve default Gains."""
    original = Gains()
    path = tmp_path / "params.txt"

    save_params(original, str(path))
    loaded = load_params(str(path))

    assert loaded == original


def test_roundtrip_custom_gains(tmp_path):
    """Test that save and load preserve custom Gains values."""
    original = Gains(rollrate_P=0.25, z_vel_I=0.15)
    path = tmp_path / "params.txt"

    save_params(original, str(path))
    loaded = load_params(str(path))

    assert loaded.rollrate_P == 0.25
    assert loaded.z_vel_I == 0.15
    assert loaded == original


def test_load_ignores_unknown_keys(tmp_path):
    """Test that load_params silently ignores unknown keys."""
    path = tmp_path / "params.txt"

    # Write a file with one valid key and one bogus key
    with open(path, "w") as f:
        f.write("# Exported by pid_optimizer\n")
        f.write("MC_ROLLRATE_P\t0.25\n")
        f.write("BOGUS_KEY\t1.0\n")

    loaded = load_params(str(path))

    # Should load successfully with the valid key set
    assert loaded.rollrate_P == 0.25
    # Other fields should have defaults
    assert loaded.rollrate_I == Gains().rollrate_I


def test_load_skips_comment_lines(tmp_path):
    """Test that load_params skips comment-only files and returns defaults."""
    path = tmp_path / "params.txt"

    # Write a file with only comments
    with open(path, "w") as f:
        f.write("# Exported by pid_optimizer\n")
        f.write("# This is a comment\n")
        f.write("# Another comment\n")

    loaded = load_params(str(path))

    # Should return defaults
    assert loaded == Gains()
