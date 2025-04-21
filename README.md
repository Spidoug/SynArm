# SynArm – Robotic Arm Control Platform

## 1. Executive Summary
SynArm is a multimodal control framework for a 6‑DOF robotic arm that targets low‑cost hobby servos (TD8120MG) driven by a PCA9685 PWM expander.  The project integrates **Leap Motion** gestural input, **joystick/keyboard** fallback, real‑time 3‑D visualisation in Processing 4, and an **Arduino Uno**‑based firmware that also supports a stepper‑driven linear axis and on‑board inertial sensing (MPU6050).

![image](https://github.com/user-attachments/assets/630dd27b-5c37-4317-8ebc-3588b76aa993)

https://github.com/user-attachments/assets/0a86b27f-d8db-4aad-b719-9c595e23936b

A WebSocket bridge exposes the arm state to external clients; a lightweight HTML5 dashboard demonstrates live telemetry and command streaming.

---

## 2. System Goals
* Intuitive, contact‑free control with dual‑hand tracking.
* Deterministic servo timing (±2 µs) and stable positioning even under load.
* Collision‑aware rendering that caps angles when boxes intersect.
* Reconfigurable serial protocol to add axes or sensors without reflashing.
* Network‑ready architecture (Processing ↔ WebSocket ↔ JS front‑end).

---

## 3. Hardware Stack
| Sub‑system | Part | Notes |
|------------|------|-------|
| MCU & Power | **Arduino Uno R3** | 16 MHz ATmega328P, powered from USB 5 V (logic only) |
| PWM Driver | **PCA9685 @ 0×40** | I²C 400 kHz, OE held LOW; outputs @ 50.0 Hz |
| Servos | 6 × **TD8120MG** | 10–20 kg·cm, powered at 6 V 3 A via external BEC |
| Stepper | NEMA‑17 + A4988 | Linear Z‑axis, 1.8 °/step |
| IMU | **MPU6050** | Motion6 packet every 4 ms |
| Input | **Leap Motion v2**, Logitech F310 | USB HID |

### Power Topology
* 5 V logic isolated from 6 V servo rail.
* 470 µF + 0.1 µF decoupling at PCA9685 V+.
* Peak current ≈ 2.8 A when all servos stall.

---

## 4. Firmware Architecture (Arduino)
1. **safeMap()** – saturates PWM counts 1…4095 (no full‑off/on).
2. **Dead‑band** ±2 ° prevents jitter.
3. **OE_PIN = 7** held LOW → no tri‑state glitches.
4. **Serial protocol**  
   * `S<ch>P<deg>` – set servo angle  
   * `M<steps>` – move stepper  
   * `READY?` handshake
5. **Telemetry** every 4 ms  
   `#SENS|MPU:ax,ay,az,gx,gy,gz|AN:0,…5|EX1:…|EX2:…`

---

## 5. Host Software (Processing 4)
* **Input**: Leap Motion (voidplus), GameControlPlus, keyboard.
* **Physics Layer**: bounding boxes + tolerance collision test.
* **WebSocket Server**: `websockets.*` @ ws://localhost:9000/.
* **UI Panels**: servo sliders, sample recorder (JSON), status HUD.
* **Rate limiter**: Leap → servo updates max 33 fps.

### Data Flow
```text
Leap / Joystick / Keyboard
          │
          ▼           Web Dashboard
  Processing 4  ◄───────┤  HTML5 + JS
          │ WS JSON     │  (live control)
          ▼             │
      Serial 115200     │
          ▼             │
      Arduino Uno ──────┘
```

---

## 6. HTML Dashboard
* Pure client‑side JS, connects via WebSocket.
* Six range‑sliders push JSON `{servo,angle}`.
* Panels overwrite in‑place: latest angles, raw serial, decoded sensors.

---

## 7. Calibration & Safety
1. **SERVOMIN/SERVOMAX** – start with 120…490 counts; refine empirically.
2. **Brown‑out tests** – verify Vservo > 5.8 V under simultaneous stall.
3. **OE sanity** – confirm 0 V at OE during full load.
4. **Collision gate** – Processing blocks movements that keep boxes intersecting.

---

## 8. Troubleshooting Log
| Issue | Root Cause | Fix |
|-------|------------|-----|
| Sudden full‑range jump | PWM count 0 from overflow | safeMap + dead‑band |
| Servo tremor at dual‑hand input | Micro‑variations ±1° each frame | 30 ms rate‑limit + dead‑band |
| Random disable | OE floating on clone board | Hard‑wire OE to GND |

---

## 9. Future Work
* Add second stepper channel (gripper linear slide).
* Replace TD8120MG with bus‑powered smart servos (CAN / RS‑485).
* Implement inverse kinematics with target‑point dragging.
* Auto‑tune servo limits & centre via IMU feedback.
* Deploy Web dashboard as PWA for mobile control.

---

## 10. References
* Adafruit PCA9685 Library v2.5.0
* JRowberg I2Cdevlib – MPU6050
* Leap Motion SDK 2.3.1.31549

> **Author:** Douglas “spidoug” Santana — April 2025

