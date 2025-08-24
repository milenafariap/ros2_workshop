#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
import time

def popen(cmd):
    print("\n$ " + " ".join(cmd))
    # new process group pra matar tudo com Ctrl+C
    return subprocess.Popen(cmd, preexec_fn=os.setsid)

def main():
    parser = argparse.ArgumentParser(
        description="Ignition (opcional) + ros_gz_bridge para cmd_vel/odom/camera/lidar/imu"
    )
    parser.add_argument("--world", "-w", help="Caminho do SDF/World (ex.: ~/moray/moray_assets/farm_inspection/world/explore_world.sdf)")
    parser.add_argument("--gui", action="store_true", help="Abrir GUI do Ignition (padrão: headless)")
    parser.add_argument("--ignition-only", action="store_true", help="Rodar só o Ignition (sem bridge)")
    parser.add_argument("--bridge-only", action="store_true", help="Rodar só a bridge (não abre Ignition)")
    # qualquer coisa depois (ex.: -r remaps) vai para o --ros-args da bridge
    args, unknown_ros_args = parser.parse_known_args()

    procs = []

    try:
        # 1) (Opcional) Ignition Gazebo
        if not args.bridge_only and args.world:
            ign_cmd = ["ign", "gazebo"]
            if not args.gui:
                ign_cmd += ["-s", "-v", "3"]  # server/headless
            ign_cmd += [args.world]
            procs.append(popen(ign_cmd))
            # dá um fôlego pro sim subir antes da bridge
            time.sleep(2)

        if args.ignition_only:
            # fica aqui até Ctrl+C
            while True:
                time.sleep(1)

        # 2) Bridge ROS ↔ GZ (tudo em um processo)
        if not args.ignition_only:
            mappings = [
                # ROS -> GZ (controlar o robozinho pelo /cmd_vel do ROS)
                "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                # GZ -> ROS (sensores e odom vindo do sim)
                "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            ]

            bridge_cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"] + mappings
            if unknown_ros_args:
                # tudo que você passar depois do script vai para o --ros-args da bridge
                bridge_cmd += ["--ros-args"] + unknown_ros_args

            procs.append(popen(bridge_cmd))

            # espera até Ctrl+C
            while True:
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n[INFO] Encerrando...")
    finally:
        # mata todos os processos em grupo
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass

if __name__ == "__main__":
    main()

