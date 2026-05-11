import sys
from pathlib import Path


# Make the inspection_robot Python package importable without installing it.
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
