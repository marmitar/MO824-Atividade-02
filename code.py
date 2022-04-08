from __future__ import annotations
import os.path
from dataclasses import dataclass, field
from itertools import combinations, count
from math import ceil, hypot
from random import Random
from time import time
from typing import Iterable, Iterator, TypeVar

import gurobipy as gp
from gurobipy import GRB, Model, Var, tupledict

rng = Random(0x0123456789ABCDEF)
vertex_id = count()

@dataclass(frozen=True)
class Vertex:
    x1: float
    y1: float
    x2: float
    y2: float
    id: int = field(default_factory=lambda: next(vertex_id), init=False)

    def cost1(self, other: Vertex) -> float:
        return ceil(hypot(self.x1 - other.x1, self.y1 - other.y1))

    def cost2(self, other: Vertex) -> float:
        return ceil(hypot(self.x2 - other.x2, self.y2 - other.y2))

    def __hash__(self) -> int:
        return self.id

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Vertex) and self.id == other.id

    def __str__(self) -> str:
        return f"v:{self.id}"

    def __repr__(self) -> str:
        return f"vertex:{self.id}"

    @staticmethod
    def read(filename: str) -> Iterator[Vertex]:
        with open(filename) as file:
            for line in file:
                line = line.strip()
                if line:
                    x1, y1, x2, y2 = (float(val) for val in line.split())
                    yield Vertex(x1, y1, x2, y2)

T = TypeVar('T')
Edge = tuple[Vertex, Vertex]
LinkDict = tupledict[Vertex, tupledict[Vertex, T]]
Tour = tuple[Vertex, ...]

COORDS_FILE = os.path.join(os.path.dirname(__file__), 'coordenadas.txt')

class Graph:
    def __init__(self, vertices: Iterable[Vertex]):
        self.vertices = tuple(vertices)

    @property
    def order(self) -> int:
        """Number of vertices"""
        return len(self.vertices)

    @property
    def size(self) -> int:
        """Number of edges"""
        return (self.order * (self.order + 1)) // 2

    def add_var(self, name: str, obj_coef: float | None = None) -> Var:
        return self.model.addVar(0, 1, obj=obj_coef, vtype=GRB.BINARY, name=name)

    def add_vars(self) -> LinkDict[Var]:
        self.vars: LinkDict[Var] = tupledict({ u: tupledict() for u in self.vertices })
        for u, v in combinations(self.vertices, 2):
            x_uv = self.add_var(f"x_{u.id}_{v.id}", obj_coef=u.cost1(v))
            self.vars[u][v] = x_uv
            self.vars[v][u] = x_uv
        return self.vars

    def add_constraint_deg_2(self):
        return self.model.addConstrs((
            2 == gp.quicksum(self.vars[u])
            for u in self.vertices
        ), name="degree_eq_2")

    def add_model(self, name="kSTSP") -> Model:
        self.model = Model(name=name)
        self.model.setParam(GRB.Param.LazyConstraints, 1)

        self.add_vars()
        self.add_constraint_deg_2()
        return self.model

    def find_sub_tour(self, solution: LinkDict[float]) -> Tour:
        tours = SubTours(self.vertices, solution)
        return min(tours, key=len)

    def get_solutions(self) -> LinkDict[float]:
        return tupledict({ u: self.model.cbGetSolution(self.vars[u]) for u in self.vars })

    def lazy_constraint_subtour_elimination(self, where: int):
        if where != GRB.Callback.MIPSOL:
            return

        tour = self.find_sub_tour(self.get_solutions())
        if len(tour) >= self.order:
            return

        expr = gp.quicksum(self.vars[u][v] for u, v in combinations(tour, 2))
        self.model.cbLazy(expr, GRB.LESS_EQUAL, len(tour)-1)

    @property
    def iterations(self) -> int:
        return int(self.model.IterCount)

    @property
    def solution_count(self) -> int:
        return self.model.SolCount

    def solve(self) -> Tour:
        start_time = time()
        self.add_model()
        self.model.optimize(lambda _, where: self.lazy_constraint_subtour_elimination(where))
        end_time = time()

        self.elapsed = end_time - start_time
        if self.solution_count <= 0:
            raise ValueError("no solution found")

        edges = tupledict({
            u: tupledict({v: x_uv.X for v, x_uv in adj.items()})
            for u, adj in self.vars.items()
        })
        return self.find_sub_tour(edges)

    @staticmethod
    def read(*, sample_size = 250, filename = COORDS_FILE) -> Graph:
        vertices = list(Vertex.read(filename))
        return Graph(rng.sample(vertices, k=sample_size))


class SubTours(Iterable[Tour]):
    def __init__(self, vertices: Tour, solution: LinkDict[float]) -> None:
        self.vertices = vertices
        self.seen: set[Vertex] = set()
        self.iter = iter(range(len(vertices)))
        self.solution = solution

    def new_node(self) -> Vertex | None:
        for node in self.vertices:
            if node not in self.seen:
                return node

    def best_next(self, u: Vertex) -> Vertex | None:
        for v in self.vertices:
            if self.solution[u].get(v, 0) > 0.5 and v not in self.seen:
                return v

    def next_tour(self, node: Vertex) -> Iterator[Vertex]:
        for _ in self.vertices:
            self.seen.add(node)
            yield node

            if (node := self.best_next(node)) is None:
                return

    def tours(self) -> Iterator[Tour]:
        while (node := self.new_node()) is not None:
            yield tuple(self.next_tour(node))

    def __iter__(self) -> Iterator[Tour]:
        return self.tours()

if __name__ == '__main__':
    g = Graph.read(sample_size=100)
    print([v.id for v in g.solve()])
    print(g.elapsed)
