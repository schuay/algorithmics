#!/bin/python2

import subprocess
import re

from optparse import OptionParser

INSTANCES = [ ["g06.dat", 10, 281]
            , ["g06.dat", 20, 825]
	    , ["g06.dat", 30, 1438]
	    , ["g06.dat", 40, 2068]
            , ["g06.dat", 50, 2681]
	    , ["g06.dat", 60, 3325]
	    , ["g06.dat", 70, 4038]
	    , ["g06.dat", 80, 4822]
	    , ["g06.dat", 90, 5685]
	    , ["g06.dat", 100, 6705]            
            ]

METHODS = ["mtz", "mcf", "scf"]

BIN = "./kmst"
DATADIR = "data/"

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("-m", "--model", dest = "models",
            help = "One of ['mtz', 'mcf', 'scf']", action = "append")
    (options, args) = parser.parse_args()

    if options.models:
        for m in options.models:
            if m not in METHODS:
                parser.error("Invalid model passed")
    else:
        options.models = METHODS

    pattern = re.compile("Objective value:\s*(\d+)")
    duration_pattern = re.compile("CPU time:\s*(\d+(?:\.\d+)?)")

    total = 0
    failed = 0
    for meth in options.models:
        for inst in INSTANCES:
            output = subprocess.check_output([BIN, "-f", DATADIR + inst[0],
                                              "-m", meth, "-k", str(inst[1])])
            total += 1

            command = "%s -f %s -m %s -k %d" % (BIN, DATADIR + inst[0], meth, inst[1])
            match = pattern.search(output)
            if not match:
                print "'%s' failed. No result" % command
                failed += 1
                continue

            duration = float(duration_pattern.search(output).group(1))
            actual = int(match.group(1))
            if actual != inst[2]:
                print "'%s' failed. Expected %d, got %d" % (command, inst[2], actual)
                failed += 1
            else:
                print "'%s' OK in %.2f s." % (command, duration)

    print "%d total, %d failed" % (total, failed)

