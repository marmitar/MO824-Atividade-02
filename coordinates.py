import os
os.chdir(os.path.dirname(__file__))
with open('coordenadas.txt', 'r') as file:
	lines = [tuple(float(x) for x in line.strip().split()) for line in file if line.strip()]


print('/* AUTO GENERATED */')
print('#pragma once')
print('#include <array>')
print('#include <cstdlib>')
print('#include "vertex.hpp"')
print()
print(f'static const constexpr std::array<vertex, {len(lines)}> DEFAULT_VERTICES', '=', '{')
for idx, (x1, y1, x2, y2) in enumerate(lines):
	print(f'    vertex::with_id({idx}, {x1}, {y1}, {x2}, {y2}),')
print('};')
print('')
