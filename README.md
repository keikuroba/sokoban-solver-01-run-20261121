# Sokoban solver — make.fun sokoban-solver-01-run-20261121

Push-optimal Sokoban solver, written from scratch with standard techniques.

## Method

The generated levels decompose exactly: the player starts in a one-cell-wide
hallway, and every room is a single connected component of floor off the
hallway row, holding as many boxes as goals and opening onto the hallway
through exactly one door. A box pushed out through a door can never come
back, so in an optimal solution every box stays in its room, and the level
optimum P\* is the sum of the rooms' optima. The solver therefore:

1. splits the level into rooms,
2. solves each room push-optimally with A\* over pushes (macro moves),
   using a minimum-assignment heuristic (boxes to goals by BFS distance)
   and dead-square pruning by reverse ("pull") reachability from the goals,
3. stitches the rooms together with plain BFS walks through the hallway,
   expanding each macro push into the player's individual steps.

Deterministic: fixed iteration orders, no clock, no randomness — two
executions write the same bytes.

## Practice set (kit)

All 5 practice instances solved at exactly P\* pushes:

| instance | P\* | pushes | points |
|----------|-----|--------|--------|
| i001 | 13 | 13 | 1000 |
| i002 | 16 | 16 | 1000 |
| i003 | 29 | 29 | 1000 |
| i004 | 47 | 47 | 1000 |
| i005 | 60 | 60 | 1000 |

Score 1,000,000 / 1,000,000. Whole set runs in ~1.6 s (limit 60 s).

## Run

```
node main.ts
```

Reads `/input/<id>/level.txt`, writes `/output/<id>/solution.txt`.
Locally: `MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts`
