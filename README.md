# surface_gui

Rviz 2 panel (`surface_gui/SurfacePanel`) that drives the surface pipeline without the command line. It has two tabs and a log shared by both:

- **Acquisition**: acquisition and surface reconstruction with `cloud_merger_node` ([`surface_reconstruction`](https://github.com/JRL-CARI-CNR-UNIBS/surface_reconstruction)).
- **Path & trajectory**: raster path ([`surface_raster_planner`](https://github.com/JRL-CARI-CNR-UNIBS/surface_raster_planner)), joint path ([`surface_path_planning`](https://github.com/JRL-CARI-CNR-UNIBS/surface_path_planning)), time parametrization (`time_parametrization`) and execution with MoveIt.

Service and action calls are asynchronous: Rviz never freezes, even during long computations.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy, Rviz 2 (Qt 5)
- The interface packages, in the same workspace: `surface_reconstruction_msgs`, `surface_raster_planner_msgs`, `surface_path_planning_msgs`, `time_parametrization_msgs`
- `moveit_msgs`

The panel only needs the interface packages to build. The nodes it drives can run anywhere on the ROS network.

```bash
cd [ROS2_WORKSPACE]
rosdep install --from-paths src/surface_gui --ignore-src -y
colcon build --packages-up-to surface_gui
source install/setup.bash
```

## Usage

Start Rviz 2, then **Panels → Add New Panel → surface_gui → SurfacePanel**. The panel settings (file names, row step, offsets, interface names, ...) are saved in the Rviz config file.

### Tab "Acquisition"

| Control | Call | Notes |
|---|---|---|
| Acquire | `<node>/acquire` (`std_srvs/Trigger`) | the acquisition count is shown below the buttons |
| Remove last | `<node>/remove_last` (`std_srvs/Trigger`) | |
| Reset | `<node>/reset` (`std_srvs/Trigger`) | asks for confirmation |
| Compute surface | `<node>/compute_surface` (`std_srvs/Trigger`) | timeout 10 min |
| Save mesh | `<node>/save` (`surface_reconstruction_msgs/SaveMesh`) | empty file name: node parameter `default_mesh_file`. After saving to an absolute path, the file is also set in the other tab ("Load from file") |

`<node>` is `/cloud_merger_node` by default (section **Interfaces**).

### Tab "Path & trajectory"

The steps run in order. Each step uses the result of the previous one, and a new result clears the results of the following steps.

| Step | Call | Inputs |
|---|---|---|
| 1. Mesh | `<raster>/load_mesh_from_topic` (`std_srvs/Trigger`) or `<raster>/load_mesh_from_file` (`surface_raster_planner_msgs/LoadMesh`) | file and frame (empty frame: node parameter `mesh_frame`) |
| 2. Raster path | `<raster>/compute_trajectory` (`surface_raster_planner_msgs/ComputeTrajectory`) | row step, keep tool orientation |
| 3. Joint path | `/compute_joint_path` action (`surface_path_planning_msgs/ComputeJointPath`) | planning frame (TCP), leave/approach offset in the tool frame. The progress bar shows the server feedback |
| 4. Trajectory | `/compute_joint_trajectory` action (`time_parametrization_msgs/ApplyTimeParametrization`) | planning group, velocity/acceleration scaling factor |
| Execute | `/execute_trajectory` action of move_group (`moveit_msgs/ExecuteTrajectory`) | asks for confirmation |

`<raster>` is `/raster_planner_node` by default. All the names can be changed in the **Interfaces** section.

**Cancel** cancels the running action (joint path, time parametrization or execution: during the execution it stops the robot).

As in `send_raster_to_joint_path.py`, step 4 prepends the current robot configuration (read from `/joint_states`) to the joint path, otherwise MoveIt rejects the trajectory. For this reason, after an execution the trajectory is cleared: compute it again from the new robot configuration.

### Topics published by the panel

The panel creates the node `surface_gui_panel`.

| Topic | Type | Description |
|---|---|---|
| `/surface_gui_panel/display_trajectory` | `moveit_msgs/DisplayTrajectory` (latched) | last timed trajectory; show it with the MoveIt *Trajectory* display |

The joint path (before time parametrization) is already published by the `compute_joint_path` server on `/compute_joint_path/display_path`.

## Package layout

```
surface_gui/
├── include/surface_gui/
│   ├── surface_panel.hpp        # Rviz panel: tabs, log, ROS node spun in its own thread
│   ├── tab_base.hpp             # asynchronous service/action calls (callbacks in the GUI thread)
│   ├── reconstruction_tab.hpp   # tab "Acquisition"
│   └── trajectory_tab.hpp       # tab "Path & trajectory"
├── src/                         # implementations
└── plugins_description.xml      # pluginlib export (rviz_common::Panel)
```

## Authors

Manuel Beschi (CARI-JRL), <manuel.beschi@unibs.it>

## License

Apache-2.0, see [LICENSE](LICENSE).
