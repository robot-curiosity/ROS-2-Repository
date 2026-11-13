# Curiosity Robot TODO (prioritized)

- [x] 1. Detection node (`trash_vision`): YOLOv8 + GPT-4o Vision fallback; publish detections topic (message type TODO: Bryan to decide)
- [x] 2. Ultrasonic node: 4 front + 4 rear HC-SR04, publish ranges (GPIO pins TODO: Bryan to fill in)
- [x] 3. Drive/encoder node: 2x Cytron MDD10A + JGB37-520 encoders, cmd_vel -> motors, odometry
- [ ] 4. Arm control node: SO101 via LeRobot (Feetech STS3215), pick service/action
- [ ] 5. GPS/IMU node: publish NavSatFix and Imu (device model TODO: Bryan to fill in)
- [ ] 6. Behavior/state machine: search -> approach -> pick -> deposit, with obstacle avoidance
- [ ] 7. Simulation launch (Gazebo) and bringup launch file for the real robot
