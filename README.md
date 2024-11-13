# Sparse Iterative Closest Point (SparseICP)

C++ implementation for the paper: 

    "Sparse Iterative Closest Point"
    Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
    Symposium on Geometry Processing 2013
    Journal: Computer Graphics Forum.


Project webpage: http://lgg.epfl.ch/sparseicp 
You can download the single-file header only library here:
https://raw.githubusercontent.com/OpenGP/sparseicp/master/ICP.h

# Source Code License
This Source Code is subject to the terms of the Mozilla Public License v. 2.0.
If a copy of the MPL was not distributed with this file, you can obtain one at http://mozilla.org/MPL/2.0/.

# HOW TO RUN
```
# Debug 构建
mkdir build/debug
cd build/debug
cmake -DCMAKE_BUILD_TYPE=Debug ../../
make

# Release 构建
mkdir build/release
cd build/release
cmake -DCMAKE_BUILD_TYPE=Release ../../
make
```

# VSCode debug config
```
{
    "name": "(gdb) sparseicp",
    "type": "cppdbg",
    "request": "launch",
    "program": "${workspaceFolder}/build/sparseicp",
    "args": ["--source_file", "~/common-3d-test-models/data/stanford-bunny/bunny/data/bun000.ply",
             "--target_file", "~/common-3d-test-models/data/stanford-bunny/bunny/data/bun045.ply",
             "--output_file", "~/dataset/aligned.pcd"],
    "stopAtEntry": false,
    "cwd": "${fileDirname}",
    "environment": [],
    "externalConsole": false,
    "MIMode": "gdb",
    "setupCommands": [
        {
            "description": "Enable pretty-printing for gdb",
            "text": "-enable-pretty-printing",
            "ignoreFailures": true
        },
        {
            "description": "Set Disassembly Flavor to Intel",
            "text": "-gdb-set disassembly-flavor intel",
            "ignoreFailures": true
        }
    ]
}
```
