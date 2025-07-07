# mower_comms_mavros

Ce package ROS intègre un backend MAVROS pour OpenMower, en exposant les services internes via MAVLink et ROS topics.

## Contenu

- `mower_comms_MAVROS.cpp` : Point d’entrée du backend, crée tous les services ROS.
- `launch/mower_comms_mavros.launch` : Démarre MAVROS, le backend et le filtre EKF.
- `config/params.yaml` : Paramètres de tous les services.
- `config/ekf.yaml` : Paramètres pour `robot_localization`.

## Utilisation

```bash
roslaunch mower_comms_mavros mower_comms_mavros.launch fcu_url:=/dev/ttyUSB0:921600
```

## Paramètres à configurer

- `fcu_url`: Port série MAVLink vers Pixhawk (USB ou UART).
- `services/diff_drive/ticks_per_m`
- `services/diff_drive/wheel_distance_m`
- `services/gps/baud_rate`, `protocol`, `datum_lat`, etc.

## Remarques

- Tous les topics utilisent le préfixe `ll/`
- Le backend publie directement les topics nécessaires à `robot_localization`