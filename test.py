#!/bin/python2

import subprocess
import re

INSTANCES = [ ["g01.dat", 2, 46]
            , ["g01.dat", 5, 477]
            , ["g02.dat", 4, 373]
            , ["g02.dat", 10, 1390]
            , ["g03.dat", 10, 725]
            , ["g03.dat", 25, 3074]
            , ["g04.dat", 14, 909]
            , ["g04.dat", 35, 3292]
            , ["g05.dat", 20, 1235]
            , ["g05.dat", 50, 4898]
            , ["g06.dat", 40, 2068]
            , ["g06.dat", 100, 6705]
            , ["g07.dat", 60, 1335]
            , ["g07.dat", 150, 4534]
            , ["g08.dat", 80, 1620]
            , ["g08.dat", 200, 5787]
            ]

METHODS = ["mtz", "mcf", "scf"]

BIN = "./kmst"
DATADIR = "data/"

if __name__ == "__main__":
    pattern = re.compile("Objective value:\s*(\d+)")

    total = 0
    failed = 0
    for meth in METHODS:
        for inst in INSTANCES:
            output = subprocess.check_output([BIN, "-f", DATADIR + inst[0],
                                              "-m", meth, "-k", str(inst[1])])
            match = pattern.search(output)
            actual = int(match.group(1))
            total += 1
            if actual != inst[2]:
                print "'%s -f %s -m %s -k %d' failed. Expected %d, got %d" % (BIN, DATADIR + inst[0], meth, inst[1], inst[2], actual)
                failed += 1
            else:
                print "'%s -f %s -m %s -k %d' OK." % (BIN, DATADIR + inst[0], meth, inst[1])

    print "%d total, %d failed" % (total, failed)

