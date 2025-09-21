#!/usr/bin/env python
#
# Copyright (C) 2024-2025  Kristian Sloth Lauszus.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Contact information
# -------------------
# Kristian Sloth Lauszus
# Web      :  https://www.lauszus.com
# e-mail   :  lauszus@gmail.com

from __future__ import annotations

import pathlib
import sys

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def test_version() -> None:
    """Test that the version string is the same as the one in pyproject.toml."""
    from pybldc import __version__

    with open(pathlib.Path(__file__).parent / "../pyproject.toml", "rb") as f:
        toml_dict = tomllib.load(f)

    assert isinstance(__version__, str)
    assert __version__ == toml_dict["project"]["version"]
