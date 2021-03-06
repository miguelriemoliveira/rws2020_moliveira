#!/usr/bin/env python
"""Reads .
This is a long, multiline description
"""

#########################
##    IMPORT MODULES   ##
#########################
import sys
import glob
import os  # we use os.path
import subprocess

#########################
##      HEADER         ##
#########################
__author__ = "Miguel Riem de Oliveira"
__date__ = "February 2015"
__credits__ = ["Miguel Riem de Oliveira"]
__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Miguel Oliveira"
__email__ = "m.riem.oliveira@gmail.com"
__status__ = "Development"

#########################
## FUNCTION DEFINITION ##
#########################

#########################
##  GLOBAL VARIABLES   ##
#########################

# Taken from https://github.com/miguelriemoliveira/rws2016_moliveira/wiki/List-of-players-and-teams
urls = ["https://github.com/miguelriemoliveira/rws2020_moliveira",
"https://github.com/joaofigueiredo22/rws2020_jfigueiredo",
"https://github.com/rubendfcosta/rws2020_rdcosta",
"https://github.com/DiogoF9/rws2020_dfigueiredo",
"https://github.com/diegosaraivanunes/rws2020_dnunes",
"https://github.com/dalmeida97/rws2020_dalmeida",
"https://github.com/inaciose/rws_2020_sinacio",
"https://github.com/marthorthegrey/rws2020_mgomes",
"https://github.com/fcurado/rws2020_fcurado",
"https://github.com/ruipcosta/rws2020_rcosta",
"https://github.com/Tisila/rws2020_tavila"
]

# Can be changed with command line argument clone, i.e., ./pull_all.py clone
clone_mode = True

##########
## MAIN ##
##########

if __name__ == "__main__":

    # --------------------#
    ### Initialization ###
    # --------------------#

    if len(sys.argv) > 1:
        if sys.argv[1] == "clone":
            clone_mode = True
        else:
            clone_mode = False

    path = "../"

    # --------------------#
    ###   Clone mode   ###
    # --------------------#
    if clone_mode:
        print("*** Clone mode selected *** ")
        print("Cloning reps to path ../")

        for url in urls:

            print(url)
            name = url.split("/")[-1][:-4]

            files = sorted(glob.glob(path + name))

            if not len(files) == 0:
                print("Repository " + name + " already exists in " + path + " , will not clone")
            else:
                cmd = "cd " + path + " && git clone " + url
                print("git clone " + url)
                p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                for line in p.stdout.readlines():
                    # print(line, end=' ')
                    print(line)
                    retval = p.wait()


    else:

        print("pulling all reps from path ../")

        # List all reps, i.e., folders starting by rws2019
        rep_paths = sorted(
            [os.path.join(path, o) for o in os.listdir(path) if os.path.isdir(os.path.join(path, o)) and 'rws' in o])
        # print rep_paths

        for rep_path in rep_paths:
            cmd = "cd " + rep_path + " && git pull"
            print("git pull " + rep_path)
            p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            for line in p.stdout.readlines():
                # print(line, end=' ')
                print(line)
                retval = p.wait()

    sys.exit()
