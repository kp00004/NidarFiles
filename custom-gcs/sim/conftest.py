import pathlib
import sys

# Allow `import rosbridge_sim` from tests without installing the package.
sys.path.insert(0, str(pathlib.Path(__file__).parent))
