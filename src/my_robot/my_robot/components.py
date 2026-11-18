"""Locate sensor-owned code in source and installed package layouts."""
from functools import lru_cache
import importlib.util
from pathlib import Path


def asset_root():
    source = Path(__file__).resolve().parents[3]
    if (source/'sensors').is_dir() and (source/'rover').is_dir():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('my_robot'))


@lru_cache(maxsize=None)
def load_implementation(relative):
    root = asset_root()/'sensors'
    path = (root/relative).resolve()
    if not path.is_relative_to(root.resolve()): raise ValueError('Invalid sensor implementation path')
    spec = importlib.util.spec_from_file_location('roam_component_'+str(abs(hash(relative))),path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
