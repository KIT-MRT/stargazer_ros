# Project Name

This package provieds an GPS-like camera-based indoor-localization-solution. The localization is given as a transformation from the stargazer frame to the camera frame.


## Installation

TODO: Describe the installation process

## Usage

### Calibration

 1. Record rosbag of poses and landmarks using 
    roslaunch stargazer_ros_tool stargazer_nodelets.launch record:=true
 2. Start calibration
    roslaunch stargazer_ros_tool landmark_calibrator.launch

### Localization

Just launch 
    roslaunch stargazer_ros_tool stargazer_nodelets.launch

### Visualization

For visualization of the landmarks and the agent launch
    roslaunch stargazer_ros_tool landmark_visualizer.launch

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## History

TODO: Write history

## Credits

TODO: Write credits

## License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
