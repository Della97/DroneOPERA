
		NetAnim RELEASE NOTES

This file contains NetAnim release notes (most recent releases first).  

Please refer the page https://www.nsnam.org/wiki/NetAnim for detailed instructions

Contributors
============

- John Abraham <john.abraham.in@gmail.com>
- Eugene Kalishenko <ydginster@gmail.com> (Open Source and Linux Laboratory http://dev.osll.ru/)
- Emanuel Eichhammer (For QCustomPlot www.qcustomplot.com. Included in this package with GPLv2 with special permission from the author)
- George F Riley (Original Concept for packet animation)
- Dmitrii Shakshin <d.shakshin@gmail.com> (Open Source and Linux Laboratory http://dev.osll.ru/)
- Makhtar Diouf <makhtar.diouf@gmail.com>
- André Apitzsch <andre.apitzsch@etit.tu-chemnitz.de>

Availability
============
Please use the following to download using git:
git clone https://gitlab.com/nsnam/netanim.git

Support
=======

NetAnim is lightly supported; please use the
[issue tracker](https://gitlab.com/nsnam/netanim/-/issues) and
[merge request tracker](https://gitlab.com/nsnam/netanim/-/merge_requests) for improvements.

Supported Qt versions
---------------------
Currently, netanim-3.110 has been tested on Qt 5.15.13 and 6.4.2.

Older releases netanim-3.109 is compatible with Qt5, and netanim-3.108 is compatible with Qt4 and Qt5.  

For recent Ubuntu releases, the following qt6 packages are required:

    apt install cmake qt6-base-dev libqt6svg6 libglvnd-dev

Alternatively, the following qt5 packages can also be used needed:

    apt install cmake qtbase5-dev

For Ubuntu 20.10 and earlier, the following package is sufficient:

    apt install cmake qt5-default

Supported platforms
-------------------
NetAnim is intermittently tested on recent versions of Linux, macOS and Windows.

Release 3.110
=============

Changelog
---------
1. Add Qt6 support
2. Add CI jobs
3. Switch build system from QMake to CMake
4. Fix some compiler warnings and deprecations
5. Remove deprecated Qt4 checks

Release 3.109
=============

Changelog
---------
1. Remove Qt4 support
2. Always apply "Scale X/Y By" on the original background size and not on
the size received after previous scaling.
3. Fix some compiler warnings and deprecations

Release 3.108
=============

Changelog
---------
1. Bug 2370 - Fails to build from source with GCC 6
2. Initial support for Ipv6
3. Fixes ambiguous operator << overload on g++ 7


Release 3.107
=============

Changelog
---------
1. Fixes an issue where nodes with images get incorrectly centered with the description missing
2. Fixes an issue where CSMA packets were not being completely animated
3. Fixes incorrect RGB assignment on node's colors, when property browser is used for editing
4. Fixes an issue where node colors change if cliked


Release 3.106
=============

Changelog
---------
1. Adds support for Qt 5 with gcc. Tested upto Qt 5.4
2. Nodes which are images(pixmaps) are positioned with their centers aligned with the node's 
   x, y co-ordinate. Prior to this change, the node's x, y co-ordinate was aligned with the pixmap's
   top-left corner.
3. Enhancements to the early detection of invalid trace files.

Supported Qt versions
---------------------
- Qt 4.7.x to Qt 4.8.0
- Qt 5.4 (with gcc only)

Supported platforms
-------------------
- Fedora Core 15 
- Ubuntu 12.10
- OS X Mountain Lion (with Qt 4.x only)

