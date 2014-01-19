Algorithmics VU, 2013W
======================

This repository contains the programming exercise for the Algorithmics
course at the Vienna University of Technology.

Results
-------
The report can be found at doc/report.tex
The source code can be found at src/kMST_ILP.cpp

Setup
-----

1. Install CPLEX Studio (for instructions, check the specification slides).
2. Check out this repository and switch to the working branch. The master branch
   is for stable code, working is for ongoing development.
3. Set up the build environment. Local preferences have been extracted into
   the vars.in file which should look something like this:

        CPLEX_DIR = /path/to/CPLEX_Studio125
        ARCH = x86-64 # 32bit: x86, 64bit: x86-64
        GPP = g++

4. Run make.
