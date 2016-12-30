# ARC Reactor

This is the main repository for the Duke 2017 Amazon Robotics Challenge.
Please read below regarding the project organization from a software standpoint.

[![Build Status](http://gitlab.oit.duke.edu/ARC-2017/reactor/badges/master/build.svg)](http://gitlab.oit.duke.edu/ARC-2017/reactor/commits/master)

## Getting Started

If this is your first time using Gitlab, please update your profile to include your at least your first/given name.

## Version Control

We will be using `git` as our version control system of choice.
Atlassian has put together a great [tutorial](https://www.atlassian.com/git/tutorials/ "Become a git guru.") on `git` usage.
Check out the [Feature Branch Workflow](https://es.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow/ "Feature Branch Workflow") tutorial.

### Workflow

When starting work on a new feature/topic/idea, create a separate feature branch for that development with a descriptive name.
This is the feature branch workflow.

```bash
git checkout -b feature-descriptive-name-here develop
```

This will create a new branch `feature-descriptive-name-here` that originates at the `develop` branch head.
If you have already created the branch, you can simply perform a checkout.

```bash
git checkout feature-descriptive-name-here
```

You can now commit your code on the `feature-descriptive-name-here` in as many commits as required.
Once you have finished developing and testing your feature, merge it back into the `develop` branch.

```bash
git checkout develop
git merge feature-descriptive-name-here
```

All development should take place in the `develop` branch.
The subteam leads are responsible for merging into the `master` branch.

### Worked Example

This section contains an example of adding a new feature to the codebase.
Note that this process is most appropriate for significant new development that may interfere with others' work if done in the `develop` branch.
For small bugfixes, working directly in the `develop` branch is fine.

Start by cloning the repository and creating a feature branch.

```bash
git clone git@gitlab.oit.duke.edu:ARC-2017/reactor.git   # clone the repository
git checkout -b feature-descriptive-name-here develop    # create and switch to new branch
```
Work on the codebase, periodically committing changes.
These change remain local to your machine unless you push the branch.
Pushing the feature branch is not necessary, but you can if you want to share your work-in-progress with others on your subteam.
Add files to build your commit.

```bash
git add new-files-here                                   # add files for committing
git add other-new-files-here                             # add more files
git commit -m "working on descriptive-name-here"         # make commit
```

You can break up your work into as many commits as needed.
Generally, many smaller commits are better than fewer larger commits.

```bash
git add edited-files-here                                # add edited files again
git commit -m "done with descriptive-name-here"          # make commit
```

Once work on the feature is done and read for integration, merge it into the `develop` branch.
Upload the changes to the GitLab repository by pushing the `develop` branch.

```bash
git checkout develop                                     # switch back to develop branch
git merge descriptive-name-here                          # merge new branch into develop branch
git push origin develop                                  # push updated develop branch
```

At this point you can delete the `feature-descriptive-name-here` branch if desired.
Ensure that your changes have merged and pushed correctly before doing so.
The command below will not let you delete an unmerged branch.

```bash
git branch -d feature-descriptive-name-here
```

Repeat the process for additional features.

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
