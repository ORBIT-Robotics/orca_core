"""Compatibility shim for the moved HELIOS head config helpers.

New code should import from ``helios_core.utils.head_config``. This module keeps
older runtime nodes, docs, and operator habits working while the package layout
is cleaned up.
"""

from helios_core.utils.head_config import *  # noqa: F401,F403
