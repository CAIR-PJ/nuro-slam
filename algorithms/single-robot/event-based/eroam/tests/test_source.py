from pathlib import Path
p=Path(__file__).resolve().parents[1]
s=(p/'src/eroam.hpp').read_text()
assert 'AlignWithLocalSphere' in s
assert 'InitRegionalDensityManagement' in s
assert 'Add_Points' in s
assert 'ros/ros.h' not in s
assert 'catkin' not in (p/'CMakeLists.txt').read_text()
print('Source preservation checks passed (not a compile test)')
