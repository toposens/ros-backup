^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package toposens_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.2 (2019-03-01)
------------------

0.9.1 (2019-03-01)
------------------
* Added pcd saving function
* Removed dependency on custom octree
* Contributors: Adi Singh

0.9.3 (2019-03-25)
------------------
* Experimenting with changed cmake config.
* Experimenting with changed cmake config.
* Experimenting with changed cmake config.
* Put string to float function in cpp file.
* Enumerated command class.
* Added finishing touches to doxygen comments.
* Implemented save PCD on terminate.
* Replace std::stof by implementation of (float)stoi
* Finished marker docs.
* Added docs to marker implementation.
* Remove coordinate transformation from firmware to ROS coordinate system and remove second declaration of Sensor::_reconfig method
* Rearrange comment in Sensor::_reconfig
* Add documentation for Command::generate method
* Bug fix - refactoring of command.cpp filename also in CMakeList.txt
* Rename commands.h to command.h
* Rename commands.h to command.h
* Rename commands.cpp to command.cpp
* Hard code command buffer size.
* Add return type to Commands::generate method and make scope of firmware defined commands public
* Added docs for markers header.
* Fix tabbing in sensor.h file.
* Fix tabbing in sensor.h
* Add partial firmware command explanations.
* Resolve merge comments.
* Create Command class for command structure with overloaded constructors for singular and dimensional commands respectively.
* Unify calls to generate sensor control commands in Sensor class.
* Reformat tab indents.
* Outsource generation of commands to static function in commands.cpp
* Unscramble command generation into singular and dimensional format commands.
* Add command.cpp and command.h files
* Create commands.cpp and commands.h and outsource command generation from Sensor class
* Completed sensor documentation.
* Refactor transformation of TS coordinate frame into ROS coordinate frame into separate function.
* Documented _parse and _getCmd functions.
* Fixed merge conflict.
* Added documentation for sensor header.
* adapted topic and parameter names
* Updated package descriptions
* Reworded package file
* Testing version bumping
* Updated version tags
* Updated package descriptions
* Generated changelogs
* Added pcd saving function
* Removed dependency on custom octree
* fixed build
* Added native pcl integration
* Integrated auto calibration waiting period
* Added dynamic reconfiguring to markers visualization
* Implemented markers using rviz visual tools
* Refactored markers into separate class
* Refactored driver files
* Modified driver namespace
* Added booster params to settings
* Added dynamic reconfigure capability
* Update srv dependencies.
* Refactored to stanard ROS package template.
* Contributors: Adi Singh, Christopher Lang, n.seckel

0.9.0 (2019-02-27)
------------------
* Added native pcl integration
* Integrated auto calibration waiting period
* Contributors: Adi Singh

0.8.1 (2019-02-20)
------------------
* Added dynamic reconfiguring to markers visualization
* Contributors: Adi Singh

0.8.0 (2019-02-19)
------------------
* Implemented markers using rviz visual tools
* Refactored markers into separate class
* Contributors: Adi Singh

0.7.1 (2019-02-13)
------------------
* Refactored driver files
* Modified driver namespace
* Added booster params to settings
* Contributors: Adi Singh

0.7.0 (2019-02-08)
------------------
* Added dynamic reconfigure capability
* Update srv dependencies.
* Contributors: Adi Singh

0.5.0 (2019-02-07)
------------------
* Refactored to standard ROS package template.
* Contributors: Adi Singh
