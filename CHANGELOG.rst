^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vector_pursuit_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2026-09-14)
------------------
* Use actual odometry speed for closed-loop acceleration limiting (`#23 <https://github.com/blackcoffeerobotics/vector_pursuit_controller/issues/23>`_)
  The last_cmd_vel\_ member was being overwritten with the controller's own
  just-computed output instead of the real speed argument from
  controller_server, making acceleration limiting open-loop. Under motor
  saturation/lag this lets commanded speed climb unbounded regardless of
  the robot's real speed (issue `#20 <https://github.com/blackcoffeerobotics/vector_pursuit_controller/issues/20>`_).
  Nav2's own OdomSubscriber had a data race making this speed value
  occasionally unreliable, but that's fixed on the jazzy branch
  (ros-navigation/navigation2@46abdba34ee2495b3e6c06967f7e7edd06f4c194),
  so it's safe to trust it here.
  Adds a regression test proving the fix: reported speed held flat across
  cycles must not let commanded speed keep climbing.
  Co-authored-by: sambhav_bcr <sambhav@blackcoffeerobotics.com>
  Co-authored-by: Claude Sonnet 5 <noreply@anthropic.com>
* Contributors: Sambhav Jain

2.0.0 (2026-01-02)
------------------
* Prevent overshoot for final rotation
* Contributors: Tatsuro Sakaguchi

1.1.0 (2025-05-25)
-----------
* Added jazzy compliant params and launch file
* Fix include paths to comply with nav2 jazzy, improve variable naming in controller and fix linting issues
* Update install binary (`#3 <https://github.com/blackcoffeerobotics/vector_pursuit_controller/issues/3>`_)
* Contributors: Kostubh Khandelwal(exMachina316)

1.0.1 (2024-09-03)
------------------
* Fixed liniting issues
* package manifest updated
* updated readme
* docs: Update README with detailed parameter descriptions and feature parameters
* Updated parameter descriptions and values. Changed p_gain to k. Readme updates.
* new tutorial added with launch and config in repo, readme updated
* updated readme and code coverage report
* Dyanmic parameter test
* Update README.md to add video and fix typo
* Merge pull request `#1 <https://github.com/blackcoffeerobotics/vector_pursuit_controller/issues/1>`_ from blackcoffeerobotics/devel-testing
  Improved test coverage and minor bug fixes
* Improved test coverage, fixed turning radius calculation, updated optimal p_gain value in README
* Linter updates
* readme update, code coverage added, package manifest update
* Added Screw calculation
  Updated the algorithm theory and added relevant diagrams for better understanding
* initial commit
* Contributors: Arthur, Arthur Gomes, Kostubh Khandelwal, exMachina316
