The presented code effectively implements a virtual angular control metric using a proportional controller, striking a balance between precision and simplicity. Constant reading of the angular position and dynamic error calculation enable fine control that emulates a physical encoder’s function.

The inclusion of the return-to-zero procedure and wait period represent practical measures to avoid cumulative errors and simulate real dispensing scenarios, thus ensuring safety and reliability in medication delivery.

This Turtlesim implementation not only validates the technical feasibility but also provides a flexible platform for tuning parameters, analyzing system behavior, and preparing for real hardware deployment.

Would you like me to prepare a line-by-line code breakdown for a specific function or integrate this with graphical and statistical results next?





Dijiste:
y pasalo a formato .md 


ChatGPT dijo:
Claro, aquí tienes el informe en formato Markdown (.md):

markdown
Copiar
# Detailed Report with Code Explanation on the Virtual Metric in Turtlesim

## Introduction

This project addresses a fundamental limitation in the stepper motor used in the autonomous medication dispensing robot: the lack of a physical encoder for position feedback. To compensate, a proportional controller (P-controller) was implemented within the Turtlesim simulation environment in ROS to emulate encoder behavior by continuously measuring and correcting the angular position with precision. Below, the control logic is described alongside key code excerpts demonstrating how the virtual metric was realized.

---

## 1. ROS Node Initialization and Setup

```python
rospy.init_node('control_tortuga_rotacion', anonymous=True)
self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
self.rate = rospy.Rate(10)
Explanation:
The ROS node is initialized to control the turtle. It subscribes to the /turtle1/pose topic to continuously receive updates on the turtle's current position and orientation, storing the angle in self.current_theta via the pose_callback method. Concurrently, it publishes angular velocity commands to the /turtle1/cmd_vel topic. The update rate is set to 10 Hz, balancing responsiveness and computational load.
```
2. Continuous Reading of Angular Position
```python

def pose_callback(self, pose):
    self.current_theta = pose.theta
Explanation:
Every time a position update arrives, pose_callback updates the current angle self.current_theta. This simulates the continuous reading an encoder would provide in a real motor system, enabling real-time feedback control.

3. Angular Error Calculation and Proportional Control
```python

error_theta = target_theta - self.current_theta
error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159
vel_z = Kp * error_theta
```
Explanation:
The angular error is computed as the difference between the target angle target_theta (converted to radians) and the current angle. The error is then wrapped within 
[
−
𝜋
,
𝜋
]
[−π,π] to ensure the turtle always rotates via the shortest path. The angular velocity command is proportional to this error by multiplying it by the gain constant Kp, which dictates the speed of correction.

This mechanism effectively simulates an encoder’s feedback function, providing dynamic error measurement and enabling continuous position correction.

4. Publishing Movement Commands and Precision Check
```python

twist_msg = Twist()
twist_msg.angular.z = vel_z
self.velocity_publisher.publish(twist_msg)

if abs(error_theta) < 0.1:
    rospy.loginfo(f"Target angle reached: {degrees(target_theta):.2f}°")
    break
```
Explanation:
A Twist message containing the calculated angular velocity is published to move the turtle. When the angular error falls below a predefined threshold (0.1 radians), the target angle is considered reached, and the rotation loop breaks to prevent unnecessary oscillations.

This stopping criterion mimics the precision of a motor with a physical encoder that can halt at an exact angular position.

5. Return-to-Zero Mechanism
```python
Copiar
def rotate_back_to_zero(self):
    self.rotate_turtle_to_target(0)
```
Explanation:
To prevent cumulative errors over multiple rotations, the turtle is programmed to return to a zero-degree reference position after each rotation. This ensures every new rotation command starts from a known, precise reference point, maintaining alignment and minimizing drift.

6. User Interaction for Target Angle Input
```python

def get_target_angle_from_user(self):
    print("Enter target rotation angle (degrees):")
    return float(input("Desired angle (degrees): "))
```
Explanation:
This method simulates the system receiving instructions from the robot’s internal controller or software, which calculates how many degrees the motor must rotate to align the dispenser compartment correctly. Here, the user inputs the desired angle, representing the internal calculation determining the motor’s required rotation.

7. Full Rotation Cycle and Simulated Waiting
```python

self.rotate_turtle_to_target(target_angle)
self.rotate_back_to_zero()
```
Additionally, a symbolic 10-second pause is introduced after reaching the target angle:

```python

rospy.loginfo("Waiting 10 seconds before returning to zero degrees...")
rospy.sleep(10)
```
Explanation:
The cycle consists of rotating to the target angle, waiting a fixed time to simulate medication dispensing, and returning to zero degrees. This prevents error accumulation and models real-world operation conditions.
