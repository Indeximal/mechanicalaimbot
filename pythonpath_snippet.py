import os
import sys

# from https://docs.python-guide.org/writing/structure/, 30.07.2019

# Absolute path of the folder this file is in.
module_path = os.path.abspath(os.path.dirname(__file__))
# Adds that folder to the python search path.
sys.path.insert(0, module_path)

# Parent directory
# os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
