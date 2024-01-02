---
title: Block Library
layout: posts
permalink: /block_library/

collection: posts

classes: wide

sidebar:
  nav: "docs"
---

The latest release of Visual Circuit includes the option to import from a community block library. This block library is currently being stored at [Visualcircuit-resources](https://github.com/JdeRobot/VisualCircuit-resources).

In order to start using this block library simply do the following:
1. Install submodules in VisualCircuit
```
git submodule init
git submodule update
```

2. Now a folder will be downloaded in `frontend/src/components/blocks/Visualcircuit-resources`

3. In the `block-library` folder you can use the readymade `file_structure.json`

4. If you add your own blocks, simply run the `generate_file_structure.py` file.
```
python3 generate_file_structure.py
```

5. The newly generated `file_structure.json` file will have your changes.

## Example of Visual Circuit running with a Block Library

![alt_text]({{ "assets/images/block_library.png" | absolute_url }})