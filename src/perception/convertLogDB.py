#Converts the log and database servers from python 2 to python 3

import os
import sys
import subprocess

#change to the log directory
os.chdir('../log')
files = os.listdir('.')
cwd = os.getcwd()
for f in files:
    if f.endswith('.py') and '__init__' not in f:
        subprocess.run('2to3 -w ' + cwd + '/' + f, shell=True)

os.chdir('../pensive')
files = os.listdir('.')
cwd = os.getcwd()
for f in files:
    if f.endswith('.py') and '__init__' not in f:
        subprocess.run('2to3 -w ' + cwd + '/' + f, shell=True)

