[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Josesoll/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Josesoll/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

# Sokoban Solver — push-level A* with room decomposition

A Sokoban solver for the make.fun `sokoban-solver-01-run-20261121` bounty. One program,
`main.ts` at the repository root (Node 22, no dependencies), answers every instance of a
set in one run: it reads `/input/<id>/level.txt` (XSB format) and writes the player's
moves to `/output/<id>/solution.txt`.

## How it works

The checker scores **pushes**, so the search runs over pushes, not steps:

1. **Push-level A\*.** A state is the sorted box squares plus the canonical name of the
   player's reachable region (its lowest square). Walking between pushes is solved
   exactly with a small BFS when a push is generated, so a state's cost is exactly its
   push count.
2. **Matching heuristic.** For each goal, one reverse BFS over open floor gives the
   minimum pushes to bring a lone box from any square to that goal. The heuristic is the
   minimum-cost bipartite matching (Hungarian algorithm) between boxes and goals under
   those distances — admissible, and strictly stronger than summing nearest-goal
   distances, since two boxes can never claim the same goal. A\* with this heuristic
   returns push-optimal solutions.
3. **Deadlock pruning.** Squares with infinite push distance to every goal are dead: a
   box pushed there kills the state. A frozen 2×2 block of walls/boxes containing a box
   off its goal also kills the state. Neither rule prunes a solvable state.
4. **Room decomposition.** Generated levels are a one-square hallway with independent
   rooms off it, and a box pushed into the hallway can never return — so each box stays
   in its room and the level's optimum is the sum of its rooms' optima. The solver cuts
   the hallway's dead squares out of the floor, flood-fills the remaining components,
   and solves each room (3–4 boxes) push-optimally with the other rooms' boxes standing
   as walls, stitching the move strings together. Levels that do not decompose fall back
   to a whole-level search with the same engine.
5. **Determinism by work, not by clock.** Every search has a fixed expansion budget and
   the priority queue breaks ties by (f, h, insertion order). No randomness, no wall
   clock: two executions write byte-identical output.

## Measured results (kit practice set)

Scored locally with the kit's own checker against the practice set's known optima P*:

| Instance | P* | This solver | House baseline (plain-astar 1.0.0) |
|---|---|---|---|
| i001 | 13 | 13 pushes — 1,000 pts | 13 pushes — 1,000 pts |
| i002 | 16 | 16 pushes — 1,000 pts | unsolved — 0 pts |
| i003 | 29 | 29 pushes — 1,000 pts | unsolved — 0 pts |
| i004 | 47 | 47 pushes — 1,000 pts | unsolved — 0 pts |
| i005 | 60 | 60 pushes — 1,000 pts | unsolved — 0 pts |
| **Total** | | **1,000,000 / 1,000,000** | **200,000 / 1,000,000** |

Runtime for the whole 5-level set: ~2 s locally, against the 60 s budget. Every
solution is push-optimal on the practice set, and repeat runs are byte-identical.

## License

MIT — see `LICENSE`.
