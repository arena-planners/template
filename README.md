# template

Template for a planner of the [arena_planners](https://github.com/arena-planners/arena_planners) bridge. The planner it ships turns toward the goal pose and drives at it, which is enough to run it in Arena and to keep its tests and CI green from the first commit.

## New planner

1. `gh repo create arena-planners/<name> --template arena-planners/template --public`.
2. Replace `template` with `<name>` in `package.xml`, `CMakeLists.txt`, `pyproject.toml` and this README.
3. Write `step()` in `planner.py` and declare its observations, action type and `config` in `planner.yaml`, following the contract in [planners/README.md](https://github.com/arena-planners/arena_planners/blob/jazzy/planners/README.md). Declare checkpoints in `weights.yaml` there too.
4. Code taken from an upstream repo keeps its `LICENSE` and gets an `ATTRIBUTION.md` naming the source and the commit.
5. Register it as a submodule of arena_planners with a `planner = <name>` tag, then `arena feature planners add <name>`.

## Run

```sh
arena launch robot.mobile:=drl robot.mobile.planner:=<name>
```
