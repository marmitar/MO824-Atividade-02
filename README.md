# Combinatorial Optimization (MO824) - Assignment 2

- [Original Brief](enunciado.pdf)
- [Report](relatorio.pdf)

---

### 1 Goal

Model an **integer-linear program using lazy constraints** and solve it with **Gurobi**.
Work in teams of **2 - 3 students** (pairs get a small bonus because they're smaller).

---

### 2 Problem description

#### 2.1  Travelling Salesman Problem (TSP) recap

Given an undirected complete graph $G = (V, E)$ with non-negative edge costs $c_e$, find a **minimum-cost Hamiltonian cycle**.
A classical ILP for the TSP is:

$$
\begin{aligned}
\min\; &\sum_{e\in E} c_e x_e \quad &(1)\\
\text{s.t.}\; &\sum_{e\in \delta(i)} x_e = 2 &&\forall i\in V \quad&(2)\\
              &\sum_{e\in E(S)} x_e \le |S| - 1 &&\forall S \subset V,\, S\neq\varnothing \quad&(3)\\
              &x_e \in \{0,1\} &&\forall e\in E \quad&(4)
\end{aligned}
$$

* $x_e = 1$ if edge $e$ is in the tour.
* $\delta(i)$ = edges incident to vertex $i$.
* $E(S)$ = edges with both endpoints in subset $S$.

Constraints (2) force degree 2; constraints (3) kill subtours.
Because (3) is **exponential**, we add them **on-the-fly** with lazy constraints: solve the model without (3); whenever Gurobi finds an integer solution, call a callback, detect subtours, and inject any violated (3). The built-in Gurobi TSP example already does this.

---

### 3 Assignment requirements

#### 3.1  The k-Similar Travelling Salesmen Problem (kSTSP)

Now extend TSP to **two tours** that share at least **k edges**.

* Same graph $G=(V,E)$.
* Two cost functions $c^1_e, c^2_e$ ($e\in E$).
* **Goal:** find two Hamiltonian cycles with **minimum total cost** $\sum c^1_e x_e + \sum c^2_e y_e$ such that at least **k** edges are common to both tours.

Special cases

* $k = |V|$ → the two tours must be identical.
* $k = 0$ → completely independent tours.

**Task:** Give a full ILP for the kSTSP.

#### 3.2  Instance generation

Generate **12 instances**:

* |V| ∈ {100, 150, 200, 250}
* k ∈ {0, |V| ⁄ 2, |V|}

A text file with 250 lines of integer coordinates is provided:

```
x1_1  y1_1  x2_1  y2_1
x1_2  y1_2  x2_2  y2_2
...
x1_250 y1_250 x2_250 y2_250
```

For each vertex pair $(i,j)$:

$$
\begin{aligned}
c^1_{ij} &= \Bigl\lceil \sqrt{(x^1_i - x^1_j)^2 + (y^1_i - y^1_j)^2}\; \Bigr\rceil \\
c^2_{ij} &= \Bigl\lceil \sqrt{(x^2_i - x^2_j)^2 + (y^2_i - y^2_j)^2}\; \Bigr\rceil
\end{aligned}
$$

For a |V|-vertex instance use only the **first |V| lines**.

#### 3.3  Experiments

Solve all 12 instances with a **30-minute time-limit** each.
Record **solution cost, optimality gap, run-time**.

#### 3.4  Submission

* **Source code**
* **Report** (\~3-5 pages) with

  * Mathematical model (variables, constraints, etc.)
  * Results table (cost, gap, time for each instance)
  * Brief analysis (costs vs. CPU time)

#### 3.5  Grading

| Aspect          | What we look for                        |
| --------------- | --------------------------------------- |
| **Writing**     | Clear, concise, well-structured         |
| **Model**       | Correct variables, parameters, domains  |
| **Experiments** | Reproducible setup, instance gen., code |
| **Analysis**    | Insight into quality vs. time           |

---

### 4 Reference

Gurobi TSP example (Java): [https://www.gurobi.com/documentation/9.0/examples/tsp\_java.html](https://www.gurobi.com/documentation/9.0/examples/tsp_java.html)
