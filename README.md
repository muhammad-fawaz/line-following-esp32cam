# Simple ESP32-CAM Line-Tracking Robot

This project runs a fast line-following robot using a single **ESP32-CAM** board. It captures images, calculates where the line is, and controls the motors all on one chip.

---

## 🛠️ How It Sees the Line

Instead of looking at the whole image, the camera checks **three specific horizontal rows**:
* **Top Row:** Looks ahead to see upcoming turns.
* **Middle Row:** Checks if the robot is centered right now.
* **Bottom Row:** Combined with the top row to find the track angle.

---

## 📐 The Sensor Readings

The robot calculates two numbers from the camera frame:

### 1. Position Offset (Where we are now)
This tells the robot if it is centered over the line using the middle row.
* **0:** Perfectly centered.
* **Positive (+):** The robot is too far left.
* **Negative (-):** The robot is too far right.

### 2. Angle Offset (Where we are heading)
By comparing the track position on the **Top Row** vs the **Bottom Row**, the robot calculates the actual angle of the line in degrees.
* **0°:** The line is straight.
* **Positive (+):** The track is curving to the **Right**.
* **Negative (-):** The track is curving to the **Left**.

---

## 🏎️ PID Motor Control

Most basic robots only use the *Position Offset*. This causes them to swing wildly back and forth. By combining **Position** and **Angle**, our PID controller drives much smoother (in theory atleast):

1. **P (Proportional):** Turns based on how far off-center the robot is **right now** + how sharp the coming **angle** is.
2. **I (Integral):** Fixes any constant pulling or drift over time.
3. **D (Derivative):** Acts like a shock absorber. It notices if the robot is turning too fast and slows it down to prevent overshooting.

### The Result (what I think we should do, test it out next week)
* **Left Motor Speed** = Base Speed + Turn Output
* **Right Motor Speed** = Base Speed - Turn Output

When a sharp turn appears, the look-ahead **Angle Offset** causes the robot to start turning *before* it actually slips off the line, keeping it perfectly stable at high speeds!


### TODO:
1. work on PID:
2. calculate kp, ki and kd
3. Implement motor control using PID output (turm_motors function)
4. test (should get till here next week)

5. create code for obstacle detection
6. test if both run simultaneously
7. test edge cases
8. test if robot can avoid false paths
9. test if the robot can go through the invisible line section without veering off