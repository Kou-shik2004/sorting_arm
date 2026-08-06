# sorting_arm_perception

One-shot RGB-D cube detection, served over `DetectObjects`, with a live viewer
window for watching what the detector sees.

## Running it

Normally launched as part of `camera_validation.launch.xml` or `app.launch.xml`.
On its own:

```bash
ros2 launch sorting_arm_bringup perception.launch.xml show_viewer:=true viewer_scale:=1.0
```

With it running, ask for a detection:

```bash
ros2 service call /detect_objects sorting_arm_interfaces/srv/DetectObjects "{expected_count: 4}"
```

## Parameters you might change

| Parameter | Controls |
|---|---|
| `show_viewer` | Whether the live detection window opens at all. |
| `display.viewer_scale` | How large that window is. Only affects the display, detection runs on the full-resolution frame regardless of this value. Turn it down if the window is too large for your screen or the redraw is slow on integrated graphics. |
| `cube_dimensions` | The physical cube size the detector expects, in metres. |
| `source_area` | The world-frame box detections are accepted from. |
| `excluded_areas` | Regions (the trays) where a detection is ignored, since placed cubes shouldn't be picked again. |
| `colour` | HSV thresholds for telling red and blue cubes apart. |
| `detector` | Contour and depth-validity thresholds; tighten these if detection is picking up noise, loosen them if real cubes are being missed. |

All of these live in `config/perception.yaml`.
