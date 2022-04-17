"""Generate C++ array with vertices from 'coordenadas.txt'."""
import os

os.chdir(os.path.dirname(__file__))
with open('coordenadas.txt', 'r', encoding='ascii') as file:
    lines = [
        tuple(float(x) for x in line.strip().split())
        for line in file if line.strip()
    ]

NAME = 'DEFAULT_VERTICES'
TYPE = 'vertex'
N = len(lines)
D = len(f'{N}')
E = len(max(max(f'{n}' for n in ns) for ns in lines))

print('/* AUTO GENERATED */')
print('#pragma once')
print()
print('#include <array>')
print('#include "vertex.hpp"')
print()
print(f'static const constexpr std::array<{TYPE}, {N}> {NAME} =', '{')
for idx, coords in enumerate(lines, start=1):
    ns = ', '.join(f'{n:{E}}' for n in coords)
    print(f'    vertex::with_id<{idx:{D}}U>({ns}),')
print('};')
