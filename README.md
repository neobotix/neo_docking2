# neo_docking2

Please visit our [online documentation to know more! ](https://neobotix-docs.de/ros/packages/neo_docking2_contour_matching.html)

## Safety-mode test server

Run the mock `set_safety_mode` service when testing without the relay board:

```bash
ros2 run neo_docking2 safety_mode_test_server
```

The server returns success for `SM_APPROACHING` and `SM_DEPARTING`. All other
safety modes return failure.
