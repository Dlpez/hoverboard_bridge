# hoverboard_bridge (exportado de MILO)

Puente ROS 2 (Python) cmd_vel -> hoverboard con firmware hack-FOC por USART. Publica `/odom`,
TF `odom -> base_footprint`, `/joint_states` y `/hoverboard/bateria`.

Incluye la configuración probada en el robot real el 10-11-2026:
- `config/hoverboard_bridge.yaml`: geometría (r = 0.08255 m, separación efectiva 0.346 m,
  90 ticks/vuelta), límites (1.67 m/s, 0.4 rad/s, max_command 400), rampas, PI apagado,
  timeouts, frames y batería 10S.
- `config/hoverboard_calibracion.yaml`: signos de RPM/ticks, matriz `mix` y `swap_wheels: true`.

## Uso
```bash
cp -r hoverboard_bridge ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select hoverboard_bridge && source install/setup.bash
ros2 launch hoverboard_bridge hoverboard_bridge.launch.py hoverboard_port:=/dev/ttyUSB0
# opcional: cmd_vel:=/cmd_vel  (por defecto escucha /diff_drive_controller/cmd_vel_unstamped)
```

Recalibrar (RUEDAS EN EL AIRE): `ros2 run hoverboard_bridge calibrar --port <puerto>` y copiar
`~/.ros/hoverboard_calibracion.yaml` a `config/`.

Dependencias: rclpy, tf2_ros, python3-serial, python3-numpy, python3-yaml.
