
# 🤖 RoboGames 2024 – University Category | Team BB-Alr-8

Welcome to the official repository of **Team BB-Alr-8**, the **2nd Runners-up** 🥉 in **RoboGames 2024 – University Category**. This repository includes all simulation files, controller logic, hardware code, and supporting documentation for the **Elimination**, **Semifinal**, and **Final** rounds.

---

## 🧠 Team Members

- **Team Name**: BB-Alr-8  
- **Achievement**: 🥉 2nd Runners-Up – RoboGames 2024  
- **Tools & Platforms**: Webots, Python, Raspberry Pi 5, Kobuki Base, Kinect Camera

---

## 📁 Repository Structure

```plaintext
├── .vscode/                   # VS Code IDE settings
├── Elimination/               # Webots files for Round 1
├── Semifinal/                 # Maze rescue simulation files
├── Final/                     # Object matching & pushing simulation
├── final_kobuki_codes/        # Real robot (Kobuki) implementation using Raspberry Pi 5 and Kinect
├── README.md                  # You are here!
```

---

## 🟢 Round 1 – Elimination: Maze Navigation by Color Sequence (Webots + E-puck)

### 🎯 Objective
Simulate an E-puck robot in a Webots environment to navigate a maze by following a **specific color wall sequence**:

> 🔴 Red → 🟡 Yellow → 💗 Pink → 🤎 Brown → 🟢 Green

The robot must:
- Begin from any arbitrary point.
- Identify and follow the color walls in sequence.
- Stop once the goal is achieved.

### 🌐 Arena
- Size: **2.5m x 2.5m**
- Wall spacing: **0.25m**
- Colored wall codes:
  - Red: `#FF0000`
  - Yellow: `#FFFF00`
  - Pink: `#FF00FF`
  - Brown: `#A5691E`
  - Green: `#00FF00`

---

## 🔥 Round 2 – Semifinal: Survivor Rescue in Fire Maze

### 🎯 Objective
Simulate a custom-built robot to navigate a dangerous maze, rescue **3 survivors**, and avoid damage zones.

### 🔥 Fire Pit Zones
- **Red Zone** (`#FF0000`) – 0.25m², Damage: 40
- **Orange Zone** (`#FF5500`) – 0.75m², Damage: 10
- **Yellow Zone** (`#FFFF7F`) – 1.25m², Damage: 0  

💡 Damage accumulates every 2 seconds while inside a zone.

### 🧍 Survivor Details
- Size: `0.1m x 0.1m`, Height: `0.01m`
- Color: `#55FF00` (Green)
- Placement: Near maze walls, requires entry and 3 seconds of presence

### 🛠 Robot Specs
- Built from scratch in Webots (no pre-built bots)
- Size limit: `0.25m x 0.25m x 0.25m`
- No overhead cameras or wall climbing

### 🧠 Rescue Strategy
- Dry-run allowed (robot must return to start)
- Real run starts post-dry-run
- Must rescue all 3 survivors and return to entry
- Points:  
  - Base: 100  
  - +20 per survivor  
  - Penalties for fire damage  
  - Disqualification if score ≤ 0

---

## 📦 Round 3 – Final: Real-World Color Matching & Box Pushing (Kobuki + Raspberry Pi 5 + Kinect)

### 🏆 Highlights
In the final round, we implemented a **real-time box sorting robot** using a **Kobuki base**, controlled by **Raspberry Pi 5**, and powered by a **Kinect camera** for RGB + depth sensing.

### 🎯 Task Overview
- Detect **colored boxes** (Blue, Red, Green, Yellow)
- Detect **matching large colored zones**
- Navigate toward and **push the box** into its respective zone
- Avoid white-colored **obstacles** in the arena
- Repeat this for all boxes in the sequence

### 💡 Features of Our Solution
- **Camera auto-detection** with multiple port trials
- **Color tracking** using HSV-based filtering
- **Kinect-based depth sensing** (goal-area detection via bounding box area)
- **Task Phases**:
  - Turn and Align
  - Find Box → Approach Box
  - Capture → Turn to Goal → Push → Retreat
- **Failsafe recovery** when object is lost
- **Multithreaded motor control** for responsiveness
- Configurable sequence via code
- Live visualization and mask debugging via OpenCV

### 📂 Code Location
> All code related to this round is in [`final_kobuki_codes/`](./final_kobuki_codes)

---

## ▶️ How to Run (Webots Rounds)

1. Clone the repository:
   ```bash
   git clone https://github.com/KiranGunathilaka/BB-Alr-8.git
   cd BB-Alr-8
   ```
2. Open Webots.
3. Load the appropriate world:
   - `Elimination/worlds/...`
   - `Semifinal/worlds/...`
   - `Final/worlds/...`
4. Build the controller from `controllers/`
5. Run simulation and observe task execution

---

## 🎥 Video Demonstrations

- Demonstration videos for each round are submitted via the official portal.
- Real-time footage included as per RoboGames requirements.

---

## 🏁 License & Credits

This repository is maintained by **Team BB-Alr-8** for academic and competition purposes only. All simulation and hardware implementation logic is original and aligns with RoboGames 2024 rules.

---

**Made with ❤️ by Team BB-Alr-8 – 2nd Runners-Up of RoboGames 2024 🏆**
```

Let me know if you'd like me to update the actual `README.md` file in your project too!
