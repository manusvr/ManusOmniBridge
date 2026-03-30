# Unified Hand Tracking Mapping: Manus → MediaPipe → Agibot

This document provides a 1:1 mapping for retargeting data from Manus Quantum/X Sens gloves through MediaPipe landmarks to the Agibot URDF joint structure.

## Master Mapping Table

| Manus ID | Manus Joint Name | MediaPipe ID | MediaPipe Landmark | Agibot Index | Agibot Joint (URDF) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **0** | Wrist | **0** | WRIST | - | *Base Link* |
| **1** | Thumb Metacarpal | **1** | THUMB_CMC | **1 & 2** | `R_thumb_roll` / `abad` |
| **2** | Thumb Proximal | **2** | THUMB_MCP | **3** | `R_thumb_mcp_joint` |
| **3** | Thumb Distal | **3** | THUMB_IP | **4** | `R_thumb_pip_joint` |
| **4** | Thumb Tip | **4** | THUMB_TIP | - | N/A |
| **5** | Index Metacarpal | **5** | INDEX_FINGER_MCP | **5** | `R_index_abad_joint` |
| **6** | Index Proximal | **6** | INDEX_FINGER_PIP | **6** | `R_index_mcp_joint` |
| **7** | Index Intermediate | **7** | INDEX_FINGER_DIP | **7** | `R_index_pip_joint` |
| **8** | Index Distal | - | *N/A (Manus Extra)* | - | N/A |
| **9** | Index Tip | **8** | INDEX_FINGER_TIP | - | N/A |
| **10** | Middle Metacarpal | **9** | MIDDLE_FINGER_MCP | **8** | `R_middle_abad_joint` |
| **11** | Middle Proximal | **10** | MIDDLE_FINGER_PIP | **9** | `R_middle_mcp_joint` |
| **12** | Middle Intermediate | **11** | MIDDLE_FINGER_DIP | **10** | `R_middle_pip_joint` |
| **13** | Middle Distal | - | *N/A (Manus Extra)* | - | N/A |
| **14** | Middle Tip | **12** | MIDDLE_FINGER_TIP | - | N/A |
| **15** | Ring Metacarpal | **13** | RING_FINGER_MCP | - | N/A |
| **16** | Ring Proximal | **14** | RING_FINGER_PIP | **11** | `R_ring_mcp_joint` |
| **17** | Ring Intermediate | **15** | RING_FINGER_DIP | - | N/A |
| **18** | Ring Distal | - | *N/A (Manus Extra)* | - | N/A |
| **19** | Ring Tip | **16** | RING_FINGER_TIP | - | N/A |
| **20** | Pinky Metacarpal | **17** | PINKY_MCP | - | N/A |
| **21** | Pinky Proximal | **18** | PINKY_PIP | **12** | `R_pinky_mcp_joint` |
| **22** | Pinky Intermediate | **19** | PINKY_DIP | - | N/A |
| **23** | Pinky Distal | - | *N/A (Manus Extra)* | - | N/A |
| **24** | Pinky Tip | **20** | PINKY_TIP | - | N/A |

---

## Integration Notes

### 1. The "Extra Node" Logic
Manus provides 5 nodes per finger (Metacarpal, Proximal, Intermediate, Distal, Tip). MediaPipe and most Robot URDFs use a 4-point system. 
* To map to Agibot, we primarily use the **Proximal** and **Intermediate** nodes. 
* The **Distal** nodes (8, 13, 18, 23) in the Manus system should be ignored for Agibot control.

### 2. Finger Reduction
The Agibot hand is optimized for grasping:
* **Index & Middle:** Full control (Abduction + 2 Flexion joints).
* **Ring & Pinky:** Only the main MCP (Knuckle) is actuated. All other distal data from Manus/MediaPipe for these fingers should be discarded or averaged into the MCP movement.

### 3. Coordinate Systems
* **Manus/MediaPipe:** Usually provide coordinates in a camera or world frame.
* **Agibot URDF:** Uses local joint rotations (Radians). 
* Ensure you calculate the angle between vectors (e.g., Vector from Node 5 to 6 vs Node 6 to 7) to drive the Agibot joint angles.