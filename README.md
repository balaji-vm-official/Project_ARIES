# Project ARIES: Automatic Robotic Interchangeable End Effector System

  **Project ARIES** is a smart, universal connector system designed to transform single-purpose robotic arms into versatile, multi-purpose industrial tools. By automating the end-effector switching process, ARIES eliminates manual intervention and reduces operational downtime.

  The system features a unique **Passive Docking Mechanism**: it requires no electricity to dock/undock, yet includes a dedicated power passthrough section to provide electricity to the end-effectors once connected.

---

## 🛠 System Overview

The system consists of three primary layers:

1. **Host:** The robotic arm.
2. **Connector:** The bridge between the host and tool, featuring three distinct sections:
    * **Top:** A specialized cut section for host engagement.
    * **Body:** Dual spring-activated mechanisms for secure locking and load-bearing.
    * **Bottom:** The interface where the specific end-effector is mounted.


3. **End-Effectors:** Currently supports a Mechanical Gripper and a Magnetic Gripper.

### Control Stack

The robotic arm is controlled via **ROS2 Jazzy** and **MoveIt2** for path planning, with hardware execution handled by an **Arduino UNO R3**.

> [!NOTE]
> For deep-dive technical details, please view the explanation files in each directory:
> [3D_CAD_Models.Explanation](https://github.com/balaji-vm-official/Project_ARIES/blob/main/3D_CAD_Models/3D_CAD_Models.Explanation.md), [Circuit_Diagram.Explanation](https://github.com/balaji-vm-official/Project_ARIES/blob/main/Circuit_Diagram/Circuit_Diagram.Explanation.md), [ros-ws.Explanation](https://github.com/balaji-vm-official/Project_ARIES/blob/main/ros-ws/ros-ws.Explanation.md)

---

## 📄 Publication & Citation

This project has been peer-reviewed and published in the *EPJ Web of Conferences*.

* **Title:** Automatic Robotic Interchangeable End Effector System (ARIES)
* **Authors:** Balaji V. M., Ponnarasan S. A., Srinivasan K., K. K. Manivannan, K. Gobivel
* **Journal:** *EPJ Web of Conferences*, Vol. 363, 01005 (2026)
* **Link:** [Read the full paper here](https://www.epj-conferences.org/10.1051/epjconf/202636301005)

### Citation (BibTeX)

If you use this work or codebase in your research, please use the following citation:

```bibtex
@article{aries2026,
  author = {Balaji, V. M. and Ponnarasan, S. A. and Srinivasan, K. and Manivannan, K. K. and Gobivel, K.},
  title = {Automatic Robotic Interchangeable End Effector System (ARIES)},
  journal = {EPJ Web of Conferences},
  volume = {363},
  pages = {01005},
  year = {2026},
  doi = {10.1051/epjconf/202636301005}
}

```

---

## 📸 Media Gallery

|  Exploded  View  |
![Exploded view](Media/exploded_view.jpeg)

|  Dissected View  |
![Disected view](Media/disected_side_view.jpeg)

| Annotated System |
![Annoted image](Media/annoted_image.jpeg)

---

## 🚀 Setup & Installation

### Software Requirements

* **ROS2 Jazzy:** [Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
* **MoveIt2:** [Binary Installation](https://moveit.ai/install-moveit2/binary/)
* **Arduino IDE:** [Download](https://www.arduino.cc/en/software/)
* **KiCAD:** [PCB Design Suite](https://www.kicad.org/download/)
* **SW to URDF Exporter:** [Wiki Link](https://wiki.ros.org/sw_urdf_exporter)

### Documentation & Resources

* **Official ROS Documentation:** [ROS Wiki](https://wiki.ros.org/)
* **MoveIt2 Tutorials:** [Setup Assistant Guide](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)
* **CAD Models:** [GrabCAD Library](https://grabcad.com/library)

---

## 👥 Contributors

* **Balaji V. M.** ([@balaji-vm-official](https://www.google.com/search?q=https://github.com/balaji-vm-official))
* **Ponnarasan S. A.**
* **Srinivasan K.**
* **Supervisor:** Mr. K.K. Manivannan (Assistant Professor, Mechatronics Engineering)
