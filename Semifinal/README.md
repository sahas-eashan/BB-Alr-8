# **BB-Alr-8 Robot - Semifinals Round**

## **Strategy**
The BB-Alr-8 robot explores the **20×20** arena using a **modified Depth-First Search (DFS) algorithm**, prioritizing turns based on the number of times a cell has been visited.

### **Sensors & Navigation**
- **Wall Detection**: Uses **4 IR sensors** (out of 6 available), as the **angled sensors were excluded** based on confirmation from the Organizing Committee (OC) that the entrance remains constant.
- **PID Adjustments**: IR sensors are also utilized for fine-tuning movement.
- **Floor Detection**: A **1×1 pixel floor-oriented camera** identifies floor colors.
- **Survivor Detection**: A **1-pixel-high wide-angle camera** detects green survivors.
- **Recording**: A **256×256 camera** captures simulation footage.

### **Software Architecture**
- Implemented in **C++** with a **modular design** for real-world compatibility with actual robots and sensors.

---

## **Arena Requirements**
The following conditions must be met for the robot to function correctly. These were confirmed through direct communication with the **Robogames Organizing Committee**:

1. **Consistent Entrance Position**  
   - The robot assumes the entrance is always at **cell (10, 0)**.

2. **Shadow-Free Arena**  
   - Shadows interfere with color detection, making it unreliable. Even humans struggle to differentiate colors in shadowed areas.

3. **No Survivors in Yellow Cells**  
   - Due to color detection limitations, survivors should not be placed in yellow cells. This was confirmed by an OC member so no extra effort have put.

---

## **Important Notes**

- **Compilation Issues**  
  If the project fails to compile, **delete the build files** manually (cleaning alone won't work). Remove the `build/` directory and any `bbAlr8` or `bbAlr8.exe` files before rebuilding.

- **Rescue Run Timing**  
  The displayed runtimes represent **real-world time differences**, not simulation time. Since different devices execute the simulation at varying speeds, **these times are not reliable for performance comparisons**.
