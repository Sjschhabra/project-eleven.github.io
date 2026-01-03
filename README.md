# 🤖 Project Eleven - Robotic Quadruped

<div align="center">
  <img src="media/Eleven_model.png" alt="Project Eleven - Final Vision" width="500"/>
  
  ### Building an Intelligent Quadruped Robot through Reinforcement Learning
  
  [![Website](https://img.shields.io/badge/Website-Live-blue?style=for-the-badge)](https://sjschhabra.github.io/project-eleven.github.io/)
  [![SolidWorks](https://img.shields.io/badge/SolidWorks-CAD-red?style=for-the-badge)](https://www.solidworks.com/)
  [![MuJoCo](https://img.shields.io/badge/MuJoCo-Simulation-green?style=for-the-badge)](https://mujoco.org/)
  [![Python](https://img.shields.io/badge/Python-3.8+-yellow?style=for-the-badge)](https://www.python.org/)
</div>

---

## 🎯 Project Overview

Project Eleven explores training quadruped locomotion using reinforcement learning in MuJoCo with Python. The goal is to develop an agile, intelligent robot through simulation-based learning, starting with a bipedal prototype and scaling to a full quadruped design.

<div align="center">
  <img src="media/motion_gif1.gif" alt="Robot Motion Demo 1" width="400"/>
  <img src="media/motion_gif2.gif" alt="Robot Motion Demo 2" width="400"/>
</div>

---

## 🔄 Complete Workflow Pipeline

This project follows a comprehensive workflow from CAD design to physics simulation:

```
SolidWorks CAD → URDF Format → MuJoCo XML → Physics Simulation
```

### **Step 1: CAD Design in SolidWorks**

<img src="media/model_cad.png" alt="SolidWorks CAD Assembly" width="600"/>

Designed individual components for the bipedal prototype:
- **Waist** (1x)
- **Hip joints** (2x - mirrored)
- **Upper legs** (2x - mirrored)
- **Lower legs** (2x - mirrored)

**Key Technique - Independent Mirror Parts:**
For mirrored components that need independent axes and references:
1. Open the original part
2. Select a face or plane for mirroring
3. Go to `Insert → Mirror Part`
4. Choose features/body to mirror
5. ✨ **Select "Break link to original part"** under Link options
6. Click OK to create a distinct, editable copy

This creates a new part file that can be modified independently without affecting the source part.

---

### **Step 2: URDF Conversion with SW2URDF**

<img src="media/model_urdf.png" alt="URDF Model" width="600"/>

Converted SolidWorks assembly to URDF format using the SW2URDF exporter:
- Define the base link (root of kinematic tree)
- Configure all links in hierarchical order
- Set up joints between connected links
- Verify coordinate frames and joint axes

**Output:** A working URDF file with complete robot description

---

### **Step 3: MuJoCo XML Compilation**

<img src="media/model_xml_mujoco.png" alt="MuJoCo XML Model" width="600"/>

Used MuJoCo's built-in compiler to convert URDF to MuJoCo XML format:

```bash
compile Assembly_2legs/urdf/Assembly_2legs.urdf Assembly_2legs/urdf/Assembly_2legs.xml
```

**Compiler Location:**
```
C:\Users\[username]\mujoco-3.3.2-windows-x86_64\bin\compile.exe
```

**Result:** A MuJoCo-ready XML file with the complete kinematic tree preserved

---

### **Step 4: MuJoCo Configuration & Simulation**

With the XML file ready, you can now customize the simulation parameters:
- ⚙️ **Actuators** - Apply motors to joints for control
- 🔧 **Joint Limits** - Set realistic motion constraints
- 🎚️ **Stiffness** - Configure joint stiffness values
- 📉 **Damping** - Add damping for realistic motion

**Status:** Kinematic tree complete and ready for physics simulation!

---

## 🎯 Project Phases

### **Phase 1: Design & Simulation** 🔴 *ACTIVE*
Designing robot components in SolidWorks and converting to MuJoCo format. Currently focusing on 2-leg analysis for learning and understanding the complete workflow.

### **Phase 2: Control Systems** ⚡ *UPCOMING*
Implementing control systems to achieve bipedal locomotion. Testing stability and gait patterns with the 2-leg prototype.

### **Phase 3: Reinforcement Learning** ⚡ *UPCOMING*
Training the robot using RL algorithms in MuJoCo. Optimizing for agile and intelligent movement patterns.

### **Phase 4: Final Assembly** ⚡ *UPCOMING*
Scaling to full quadruped (4-leg) version or optimizing the bipedal design based on results. Final integration and testing.

---

## 🛠️ Technology Stack

- **CAD Design:** SolidWorks
- **Robot Description:** URDF (Unified Robot Description Format)
- **Physics Simulation:** MuJoCo
- **Programming:** Python 3.8+
- **Machine Learning:** Reinforcement Learning algorithms
- **Control Systems:** PID controllers, trajectory planning
- **Tools:** SW2URDF exporter, Jupyter Notebook

---

## 📦 Project Files

The repository includes all necessary files to replicate the workflow:

- `files/Assembly_2legs.SLDASM` - Complete SolidWorks assembly
- `files/Assembly_2legs.urdf` - Robot description format
- `files/Assembly_2legs.xml` - MuJoCo simulation model
- `files/mujoco_simulation.ipynb` - Python simulation code

---

## 🚀 Getting Started

### Prerequisites

```bash
# Required Software
- SolidWorks 2020 or later
- MuJoCo 3.0+
- Python 3.8+
- Jupyter Notebook
```

### Installation

1. **Clone the repository:**
```bash
git clone https://github.com/Sjschhabra/project-eleven.github.io.git
cd project-eleven.github.io
```

2. **Install Python dependencies:**
```bash
pip install mujoco numpy matplotlib jupyter
```

3. **Open the simulation notebook:**
```bash
jupyter notebook files/mujoco_simulation.ipynb
```

### Running the Simulation

Load the MuJoCo XML model and run physics simulation:

```python
import mujoco
import mujoco.viewer

# Load the model
model = mujoco.MjModel.from_xml_path('files/Assembly_2legs.xml')
data = mujoco.MjData(model)

# Launch interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

---

## 📚 Resources & Documentation

- **MuJoCo Documentation:** [https://mujoco.readthedocs.io/](https://mujoco.readthedocs.io/)
- **SW2URDF Plugin:** [https://github.com/ros/solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter)
- **URDF Specification:** [http://wiki.ros.org/urdf](http://wiki.ros.org/urdf)
- **Project Website:** [https://sjschhabra.github.io/project-eleven.github.io/](https://sjschhabra.github.io/project-eleven.github.io/)

---

## 🎥 Demo

<div align="center">
  <img src="media/motion_gif1.gif" alt="Bipedal Locomotion Test 1" width="400"/>
  <img src="media/motion_gif2.gif" alt="Bipedal Locomotion Test 2" width="400"/>
  
  *Current bipedal prototype demonstrating basic physics simulation*
</div>

---

## 🗺️ Roadmap

- [x] Complete CAD design of 2-leg prototype
- [x] Convert SolidWorks assembly to URDF
- [x] Compile URDF to MuJoCo XML format
- [x] Basic physics simulation in MuJoCo
- [ ] Implement control systems for bipedal walking
- [ ] Apply reinforcement learning algorithms
- [ ] Optimize gait patterns and stability
- [ ] Scale to full quadruped (4-leg) design
- [ ] Physical prototype construction (if feasible)

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome! Feel free to check the [issues page](https://github.com/Sjschhabra/project-eleven.github.io/issues).

---

## 📝 License

This project is open source and available under the [MIT License](LICENSE).

---

## 👤 Author

**Sjschhabra**

- GitHub: [@Sjschhabra](https://github.com/Sjschhabra)
- Project Website: [project-eleven.github.io](https://sjschhabra.github.io/project-eleven.github.io/)

---

## ⭐ Show Your Support

Give a ⭐️ if this project helped you or if you find it interesting!

---

<div align="center">
  <img src="media/Eleven_model.png" alt="Project Eleven" width="300"/>
  
  **Project Eleven** - From Simulation to Reality 🚀
</div>
