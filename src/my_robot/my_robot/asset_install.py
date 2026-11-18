"""Collect reusable assets preserving their layout in the ROS package share."""
from pathlib import Path


def asset_data_files(repository, package='my_robot'):
    repository = Path(repository)
    result = []
    for name in ('rover', 'sensors', 'worlds', 'calibration'):
        root = repository/name
        for directory in [root, *sorted(p for p in root.rglob('*') if p.is_dir() and '__pycache__' not in p.parts)]:
            files = [str(p) for p in sorted(directory.iterdir()) if p.is_file() and p.suffix not in ('.pyc','.pyo')]
            if files:
                result.append(('share/'+package+'/'+directory.relative_to(repository).as_posix(),files))
    return result
