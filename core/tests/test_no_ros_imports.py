import ast
from pathlib import Path

import pytest

CORE_DIR = Path(__file__).resolve().parents[1]
ROS_MODULES = {"rclpy", "rclpy_action", "launch", "launch_ros", "ament_index_python"}
SKIPPED_DIRS = {".venv", "venv", "build", "dist", ".eggs"}


def _core_sources() -> list[Path]:
    return sorted(
        path
        for path in CORE_DIR.rglob("*.py")
        if not SKIPPED_DIRS.intersection(path.relative_to(CORE_DIR).parts)
    )


def _imported_top_level_modules(source: str) -> set[str]:
    modules: set[str] = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            modules.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            modules.add(node.module.split(".")[0])
    return modules


@pytest.mark.parametrize("path", _core_sources(), ids=lambda p: p.relative_to(CORE_DIR).as_posix())
def test_core_never_imports_ros(path: Path):
    imported = _imported_top_level_modules(path.read_text(encoding="utf-8"))

    assert not imported & ROS_MODULES
