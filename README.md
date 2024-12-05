## Unofficial Python or ROS2 library for Manus Meta Gloves

While [Manus Meta](https://www.manus-meta.com/) natively supports [C++ SDK](https://docs.manus-meta.com/2.4.0/Plugins/SDK/), there is no Python or ROS2 bindings or retargeting code to robot hands such as LEAP Hand.  

This repo has tools which we use with the [LEAP Hand](https://leaphand.com/) in many projects and demos including [Bimanual Dexterity for Complex Tasks](https://bidex-teleop.github.io/)  Please see Bidex for more videos.

### Python + LEAP Hand
- This demo integrates MANUS gloves with the [LEAP Hand Python SDK](https://github.com/leap-hand/LEAP_Hand_API/tree/main/python).  Check the LEAP Hand for more details.
- First install LEAP Hand:
    - `pip install dynamixel_sdk numpy`
    - Copy the `leap_hand_utils` folder from the [LEAP Hand Python SDK](https://github.com/leap-hand/LEAP_Hand_API/tree/main/python) next to the LEAP_hand_example.py in this repository so that it can be imported correctly.
- Run our MANUS SDK, then connect and power your LEAP Hand and then run: `python LEAP_Hand_example.py`
- If successful, the LEAP Hand will come to life similar to our conference demo as seen on the bottom of [https://leaphand.com/](https://leaphand.com/)
- Note that this does not match the pinch grasps between the two hands but instead copies joint angles directly from the MANUS skeleton to LEAP Hand for simplicity.  The thumb will never be that good using this mode due to the differing kinematics of the two hands.

# ROS2 + LEAP Hand Retargeting Setup
It is useful to retarget the glove data to robot hands to perform similar fingertip grasping between the two hands.
- Inspired by [Robotic Telekinesis](https://robotic-telekinesis.github.io/) and [Dexcap](https://dex-cap.github.io/), this code retargets and solves for robot joint angles that matches the pinch grasps between the human and robot hands.
- This retargeter takes the Manus Glove fingertip data, runs [SDLS](https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/SdlsPaper.pdf) from [Pybullet](https://pybullet.org/wordpress/) and then returns joint angles for the robot.
- We provide examples using [LEAP Hand](https://leaphand.com/), but you can also import your own URDF/MJCF into Pybullet.
- All of these nodes are in Python so they are easy to use.

### ROS2 Setup
- If not already installed, follow [ROS2 Installation Instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install ROS2.
- The code we provide is two ROS2 Node/Packages:
    - `Telekinesis` performs the inverse kinematics

### ROS2 + LEAP Hand
- To run on the real [LEAP Hand](https://leaphand.com/), run our MANUS SDK first, then this ROS2 node and then the LEAP hand ROS2 node from the [LEAP Hand API](https://github.com/leap-hand/LEAP_Hand_API).


### Extensions and Licensing
- Feel free to fork this repository and add your own robot hands.  The code is compatible with many robot hands.  You will need the URDF/MJCF of the hand available if you are using IK.
- If you find issues/bugs please feel free to open a Github Issue.
- This is based off of the Manus Meta SDK.  Our tools are released under the MIT license.  See License.md for details.

## Citing
If you find this codebase or [LEAP Hand](https://leaphand.com/) useful in your research, please cite: 

```
@inproceedings{shaw2024bimanual,
    title={Bimanual Dexterity for Complex Tasks},
    author={Shaw, Kenneth and Li, Yulong and Yang, Jiahui and Srirama, Mohan Kumar and Liu, Ray and Xiong, Haoyu and Mendonca, Russell and Pathak, Deepak},
    booktitle={8th Annual Conference on Robot Learning},
    year={2024}
}

@article{shaw2023leaphand,
	title={LEAP Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning},
	author={Shaw, Kenneth and Agarwal, Ananye and Pathak, Deepak},
	journal={Robotics: Science and Systems (RSS)},
	year={2023}
}
```

Thank you to MANUS Meta, Maarten Witteveen and Sarah Shaban for all of the help with their gloves.


<p align="center">
    <a href="https://www.manus-meta.com/">
        <img width="320" height="120" src="./readme_media/powered_by_manus.png">
    </a>
</p>


