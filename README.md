# ARC Reactor

This is the main repository for the Duke 2017 Amazon Robotics Challenge.
Please read below regarding the project organization from a software standpoint.

## Getting Started

If this is your first time using Gitlab, please update your profile to include your at least your first/given name.

## Version Control

We will be using `git` as our version control system of choice.
Atlassian has put together a great [tutorial](https://www.atlassian.com/git/tutorials/ "Become a git guru.") on `git` usage.
Check out the [Feature Branch Workflow](https://es.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow/ "Feature Branch Workflow") tutorial.

### Workflow 

When starting work on a new feature/topic/idea, create a separate feature branch for that development.
This is the feature branch workflow.

```bash
git checkout -b feature-abc develop
```

When you have finished developing and testing your feature, merge it back into the `develop` branch.

```bash
git checkout master
git merge feature-abc
```

All development should take place in the `develop` branch.
The subteam leads are responsible for merging into the `master` branch.

### Guidelines

- Prefer `git pull --rebase` to `git pull` to avoid unnecessary merge commits which are difficult to undo if botched.
- Prefer many small commits to few large commits.
Use `git add -p` to interactively add partial files to a commit.
- Only merge code that compiles into the `develop` branch.
Use separate feature branches work in progress.
- Do not commit generated, binary, or large files.
For Python, exclude bytecode (`.pyc`) files.
For C/C++, exclude object (`.o`) and output files.
- Do not `git push --force` unless your subteam lead specifically approves it.

## Languages

This project will make extensive use of Python 2.7 due to its flexibility as an integration language.
C/C++ with Python bindings is an option for performance-critical components (e.g., image processing or grasp planning).

## Coding Guidelines

We recommend adhering to the Style Guide for Python defined in [PEP8](https://www.python.org/dev/peps/pep-0008/ "Style Guide for Python").
This will make the code easier to read and integrate in the long run.
The most relevant style recommendations are below.

- **Indentation**: Use four spaces instead of tabs.
- **Imports**: Avoid wildcard modules imports (e.g., `from math import sin, cos` instead of `from math import *`).
- **Docstrings**: Start each module and function with a docstring using [Doxygen](http://www.doxygen.org/ "Doxygen") syntax.

## Logging

Extensive and structured logging will greatly facilitate debugging and diagnostics.
Create a logger instance at the start of each module.

```python
import logging, traceback
logger = logging.getLogger(__name__)
```

High-level code can then selectively enable/disable logging output for a given severity level for each module.

### Severity Levels

Use the following hints for deciding the severity level for a message.

- `debug`: Used during module development for debugging.
- `info`: Module users will generally want to know this.
- `warn`: Something went wrong but meaningful steps have been taken to correct the problem.
- `error`: Something went wrong and no further action is possible.
- `critical`: Something is very wrong.

### Exceptions

Log all exceptions with a traceback to facilitate debugging.
```python
try:
    ...
except Exception as e:
    logger.error('something is wrong: {}'.format(e))
    logger.error(traceback.format_exc())
```

## Directory Structure

The plan for organizing this repository is below.

- `cmake/`: CMake modules specific for this project (if needed).
- `data/`: Symbolic links (or directory junctions) to synced folders (e.g., Duke Box, Dropbox, Google Drive, etc.) with large and/or binary files.
- `doc/`: External text-based documentation not already added with Doxygen into Python docstrings or C/C++ comments.
- `src/`: Top-level directory for all source code, further subdivided into modules (e.g., `src/perception/`, `src/grasp/`, etc.).
