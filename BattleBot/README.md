## Arduino:
- Upload the code in arduino_code/battlebot into opencm

## ROS:
- Gmapping:
    - ```
        ## LAUNCH AT ROBOT ITSELF
        # launch base driver, camera, and mapping algorithm
        roslaunch atr_nav bringup.launch
        ```

    - ```
        ## LAUNCH AT REMOTE MASTER
        # open visualizer 
        # add image, map, laserscan tool and set the topic
        rosrun rviz rviz
        ```

    - ```
        ## LAUNCH AT REMOTE MASTER
        # remote control robot
        rosrun teleop_twist_keyboard teleop_twist_keyboard.py
        ```

    - ```
        ## LAUNCH AT ROBOT ITSELF
        # Save Map
        rosrun map_server map_saver -f ~/map
        ```

- Navigation:
    - ```
        ## Kill all Commands launched above
        ```
    
    - ```
        ## LAUNCH AT ROBOT ITSELF
        # launch navigation stack
        roslaunch atr_nav nav.launch
        ```


## Random Notes
- launch file break down testing for `bringup.launch`
    - `rosrun atr_nav ros_con.py`
    - `roslaunch atr_nav astra.launch`
    - `roslaunch atr_nav gmapping.launch`
    - `roslaunch atr_nav static_transform.launch`

