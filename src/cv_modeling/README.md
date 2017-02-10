# Introduction
The packges provide a banch of tools to provide automatic rendering with blender python API and (Rutgers Amazon Picking Challenge Database)[https://www.google.com]. please change the directory paths before running the scripts.

#Usage
To run the script in terminal
'''
blender --background --python [myscript.py]
'''

To run the script on a blend file
'''
blender --background [shelf.blend] --python [myscript.py]
'''

To import other script as a module(assume all scripts are in the current working directory)

'''
import os
import sys
cwd = os.getcwd()
if cwd not in sys..path:
    sys.path.append(cwd)

import imp
import myscript
imp.reload(myscript)

from myscript import myfunction
'''