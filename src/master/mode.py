'''
Classes for accessing the system run modes.

The system has three levels of run modes.

- `manual`:
The system requires explicit approval to activate each state.

- `semi-automatic`:
The system requires explicit approval to initiate each picking/stow cycle.

- `automatic`:
The system will automatically initiate picking/stow cycles until the order
is finished.  No checkpoints are hit.
'''

class Mode(object):
    pass
