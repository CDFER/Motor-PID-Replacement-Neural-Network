**Neural Network PID Replacement for Brushed DC Motors**
=============================================================

**Introduction**
---------------
This repository presents a novel approach to controlling brushed DC motors using neural networks, overcoming many of the limitations of traditional PID controllers. The motivation behind this project stems from the challenges of driving model trains smoothly, particularly at slow speeds. The low-quality motors often used in model trains, combined with the high static friction and backlash of worm drives, make it difficult to achieve stable and smooth motion. Additionally, the varying loads and inclines encountered by model trains further complicate the control problem. After struggling to achieve anything near good enough results with hand-tuned PID loops, I embarked on this project, which explores the potential of neural networks to learn and adapt to the complexities of model train control.

**View The Code**
---------------

**Microcontroller Code**:
* **C++ Code**: Located in `src/main.cpp` and `include/model`

**Model Training**:
* **Python Notebook**: `/modelTraining.ipynb` contains the model training code

**Conversion to C++ Header Files**:
* **Python Notebook**: `/convertToMicro.ipynb` contains the conversion code


**My Approach to Creating the Model**
------------------------------------
The following diagram illustrates the overall flow of the neural network (more detail is in `/modelTraining.ipynb`): ![Flow chart of neural network](images/overview.png)

* **Event-driven Timing**: Instead of running the model at fixed intervals, I opted to trigger the neural network when a rotation is complete, as indicated by the hall effect sensor.
* **Swarm-optimized Model Structure**: Instead of guessing, I set up a swarm of particles to find the number of history steps and parameters that give the best results.

* **On-Device Training (not yet stable)**: The model can be trained (albeit much more slowly) on the ESP32 (see my on-device training demo). I hope to have this stable in a week or so, once my brain recovers from the adventure of setting up this code -> I really have come to dislike corporate open-source code and the current state of TensorFlow...


**The Physical Setup**
---------------------

![Sentinel 4wDH with Janky electronics blu-tacked on top](images/DSC05507.JPG)

To overcome the challenges of controlling model trains, I designed a custom setup to collect data and train the neural network. The key components of this setup are:

* **Rotational Position Sensor**: Instead of using a traditional rotational position sensor, I opted for a digital hall effect sensor with one pulse per motor revolution. This was due to the difficulty of fitting a traditional sensor to the back of the motor. A small magnet is attached to the motor shaft to trigger the hall effect sensor.
* **ESP32-based Control Board**: I utilized a custom-designed ESP32-based control board, which I had manufactured by JLCPCB, to test digital model train control (DCC). A future version of the board is in development, which will offer significant improvements.
![PCB Schematic](images/schematic.png)
* **Power Supply**: The motor is powered by a small lithium-ion battery, which is boosted to 12V using a ME2149-based boost converter. I really like the ME2149; it really is an impressive boost IC that seems to have come out of nowhere.

**Dataset Generation**
---------------------

Generating the dataset for training the neural network was simplified using an ESP32 microcontroller, which can connect over WiFi. This allowed me to quickly:

* **Software Updates**: Flash software updates over LAN using ArduinoOTA
* **Real-time Data Collection**: Collect data in real-time using telnet over LAN and PuTTY to log the telnet data directly to the training data CSV files.



**Why Not RNNs or Reinforcement Learning?**
------------------------------------------

In exploring alternative approaches, I considered using **Recurrent Neural Networks (RNNs)** to model the temporal relationships in the motor's behavior. However, I opted against RNNs due to the challenges associated with training them, particularly the need for careful warm-up periods to stabilize the gradients.

I also experimented with **Reinforcement Learning (RL)**, but found it to be overly reliant on meticulous parameter and reward function tuning to achieve decent results. The trial-and-error process of adjusting rewards, penalties, and exploration rates proved time-consuming and frustrating, leading me to seek a more simple approach.

Instead, I chose to focus on a more traditional **feedforward neural network**, K.I.S.S.


**Previous Work in this Space**
--------------------------------

#### Replacing PID Controllers with ANN Controllers for DC Motor Position Control

https://arxiv.org/pdf/1312.0148

* **Author:** Aamir, Muhammad
* **Date:** April 2013
* **Journal:** International Journal of Research Studies in Computing
* **Neural Network Implementation:** Feedforward neural network trained through supervised learning
* **Validation Environment:** MATLAB's Simulink model

**Key Findings:**

* The neural network controller showed acceptable results in terms of time delay factor and system dynamics.
* Increasing the network size can improve system performance, but may also increase cost and complexity.

**Limitations:**

* The accuracy of the neural network control may not be very high, but still comparable to PID control with acceptable results.

---------------------------------------------------

#### Comparison of PID and ANN Controllers in Robotic Arms

https://www.diva-portal.org/smash/get/diva2:1351191/FULLTEXT01.pdf

* **Authors:** Joseph Ariss, Salim Rabat
* **Date:** 2019
* **Institution:** KTH Royal Institute of Technology, Stockholm, Sweden
* **Neural Network Implementation:** Feedforward neural network trained through supervised learning
* **Validation Environment:** MATLAB's Simulink, with a custom robotic arm model

**Key Findings:**

* The neural network controller outperforms the traditional PID controller in noisy environments.
* The neural network controller is beneficial in scenarios with high friction and noise.

**Limitations:**

* The PID controller is more precise in non-noisy environments.

---------------------------------------------------

#### Optimization of Neural Network-Based Self-Tuning PID Controllers

https://www.mdpi.com/2076-3417/11/17/8002

* **Authors:** Yong-Seok Lee and Dong-Won Jang
* **Date:** 2021
* **Journal:** Applied Sciences
* **Neural Network Implementation:** Two neural networks: one for identifying the target system and one for recommending PID parameters; LSTM networks showed better performance than ANN
* **Validation Environment:** A simulator based on a mass-spring-damper model, with a second-order system for position control

**Key Findings:**

* LSTM networks showed approximately 98% success in tuning.
* LSTM network with 11 sampling data points and no response characteristics showed 53% better performance than ANN.

**Limitations:**

* ANN without response characteristics showed poor performance.
* PID setting failed for certain values.
* Noise in the system can result in failed tuning attempts.
* Complex approach requires lots of training data and GPU time.

**Interesting Aspects:**

* Use of two neural networks to reduce the number of tuning attempts.
* Ability to identify system information and recommend optimal PID parameters.
* Potential for application in conservative industries where PID controllers are widely used.
* Robustness against noise and changes in target position.

---------------------------------------------------
