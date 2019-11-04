import os
import sys
# Adds the folder containing the module to the python search path.
# from https://docs.python-guide.org/writing/structure/, 30.07.2019
sys.path.insert(0, os.path.join(os.path.abspath(os.path.dirname(__file__)),
                                ".."))
import mechbot.app.app

if __name__ == '__main__':
    mechbot.app.app.run()
