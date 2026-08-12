# neo_docking2

Please visit our [online documentation to know more! ](https://neobotix-docs.de/ros/packages/neo_docking2_contour_matching.html)

## Safety-field test server

Run the mock `set_safety_field` service when testing without the relay board:

```bash
ros2 run neo_docking2 safety_field_test_server
```

The server returns success for safety fields `0` and `4`. All other field IDs
return failure.
