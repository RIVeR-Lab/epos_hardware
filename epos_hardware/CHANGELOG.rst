^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package epos_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2015-06-18)
------------------
* Fixed quickstop diagnostics status
* Added diagnostics information from statusword
* Added ability to set fault recovery option
* Added nominal current and max current to diagnostics
* Added diagnostics warning if nominal current is exceeded
* Added diagnostics for motor output
* Added torque constant parameter
* Added support for velocity halt command
* Show error message if not all faults were cleared
* Don't write value if command is nan
* Limit profile velocity to specified limit
* Cast parameters to integers because sometimes it causes the configuration to fail
* Contributors: Mitchell Wills

0.0.2 (2015-03-06)
------------------
* Fixed so that udev rules file and example launch files are actually installed
* Contributors: Mitchell Wills

0.0.1 (2015-01-30)
------------------
* Initial release of epos_hardware
* Contributors: Mitchell Wills
