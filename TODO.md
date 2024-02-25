## TODO List ##

- Migrate python scripts to python 3
- control_toolbox cpp nodes instead of python based.  
- Remove dependencies on uuv_simulator_msgs from control_msgs, change to mavros_msgs
- Test that SITL works in Noetic / Python 3
- Check all dependencies from package scipts are listed in package.xml and CMakeLists.txt
- Remove unused packages from package.xml and CMakeLists.txt

        <!-- Complete convenient_installer.sh -->
        <!-- App to configure controller for autopilot and send user input
                Use RQT Python Tutorial To Make an App that allows user to fill in form and select-->
        <!-- TODO Test Waterlinked, upgrade firmware if required.
                answer: waterlinked works -->
        <!-- TODO NMEA_GPS Driver, rules plus implementation
                answer: GPS integrated, need to include rules in udev/99-bluerov2.rules -->
        <!-- TODO publish nose_camera joint state: use old tf_manager.py code
                answer: Done -->
        <!-- TODO Remotely Start Nose_Camera Driver & Republish Image Topics to Local Machine
                answer: To test Monday -->
        <!-- TODO Ping Driver refactor for python 3 & ros
                answer: Possibly Sunday -->
        <!-- TODO Tune PIDs
                answer: Wednesday arvo -->
        <!-- TODO mapviz
                answer: Done -->
        <!-- TODO Publish flightdata annotated camera image
                answer: Needs tuning.-->
        <!-- TODO 6DoF
                answer: maybe before week 8-->
        <!-- TODO Should mavros/local_position publish a different frame_id (e.g. odom?)
                answer: No, in fact local_position should broadcast tf and global_position SHOULD NOT.-->
        <!-- TODO Test fake_waterlinked.py
                answer: tested-->
        <!-- TODO Divide out guidance related operations from Cascade4DoF into a separate package
                answer: actually did it-->