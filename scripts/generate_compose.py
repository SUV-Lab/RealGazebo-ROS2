#!/usr/bin/env python3
"""
Generate docker-compose.override.yml from existing RealGazebo YAML configuration.

################################################################################
#  DO NOT RUN THIS SCRIPT DIRECTLY!                                            #
#  Use start_simulation.sh instead:                                            #
#                                                                              #
#    ./scripts/start_simulation.sh src/realgazebo/yaml/example.yaml            #
#    ./scripts/start_simulation.sh src/realgazebo/yaml/example.yaml --gui      #
#                                                                              #
################################################################################

This script is called internally by start_simulation.sh.
It reads the existing vehicle YAML format (same as realgazebo.launch.py)
and generates docker-compose.override.yml with vehicle services and networks.
"""

import sys
import os
import ast
import yaml
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description='Generate docker-compose.override.yml from vehicle configuration'
    )
    parser.add_argument(
        'config_file',
        help='Path to vehicle YAML configuration file (same format as realgazebo.launch.py)'
    )
    parser.add_argument(
        'output_file',
        nargs='?',
        default=None,
        help='Output path for docker-compose.override.yml (default: project root)'
    )
    parser.add_argument(
        '--unreal-ip',
        default='host.docker.internal',
        help='Unreal Engine server IP (default: host.docker.internal)'
    )
    parser.add_argument(
        '--unreal-port',
        default='5005',
        help='Unreal Engine server port (default: 5005)'
    )
    parser.add_argument(
        '--world',
        default='c-track',
        choices=['c-track', 'urban', 'vils'],
        help='World type (default: c-track)'
    )
    parser.add_argument(
        '--headless',
        action='store_true',
        default=True,
        help='Run in headless mode (default: true)'
    )
    parser.add_argument(
        '--gui',
        action='store_true',
        help='Run with Gazebo GUI (disables headless)'
    )
    return parser.parse_args()


def load_config(config_path):
    """Load vehicle configuration from YAML file (existing format)"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def parse_spawnpoint(spawnpoint_str):
    """Parse spawnpoint from string tuple format: (x, y, z, yaw)"""
    try:
        # Handle string tuple format like "(18.846, 14.751, -1.3, -3.14)"
        if isinstance(spawnpoint_str, str):
            parsed = ast.literal_eval(spawnpoint_str)
            return list(parsed)
        elif isinstance(spawnpoint_str, (list, tuple)):
            return list(spawnpoint_str)
    except:
        pass
    raise ValueError(f"Invalid spawnpoint format: {spawnpoint_str}")


def generate_compose_override(config, unreal_ip='host.docker.internal', unreal_port='5005',
                              world='c-track', headless=True):
    """Generate docker-compose.override.yml content from existing YAML format"""

    compose = {
        'services': {}
    }

    # Parse existing format:
    # px4_target:
    #   0: /path/to/PX4
    # vehicles:
    #   0:
    #     type: x500
    #     build_target: 0
    #     spawnpoint: (x, y, z, yaw)

    px4_targets = config.get('px4_target', {})
    vehicles = config.get('vehicles', {})

    # Obstacles are not included in vehicle containers (they spawn in gazebo)
    support_obstacle = ['rock']

    for vid, vehicle in vehicles.items():
        vid = int(vid)
        vtype = vehicle.get('type')

        # Skip obstacles - they're handled by gazebo container
        if vtype in support_obstacle:
            continue

        build_target_key = vehicle.get('build_target')
        spawnpoint = parse_spawnpoint(vehicle.get('spawnpoint'))

        service_name = f'vehicle_{vid}'

        # Format spawnpoint as comma-separated string
        spawnpoint_str = ','.join(map(str, spawnpoint))

        # Vehicle IPs on each network
        vehicle_gazebo_ip = f'172.20.0.{10 + vid}'
        vehicle_network_ip = f'172.30.0.{10 + vid}'

        # Create vehicle service
        compose['services'][service_name] = {
            'image': 'aware4docker/realgazebo:1.2',
            'container_name': service_name,
            'hostname': service_name,
            'privileged': True,
            'environment': [
                'DISPLAY=${DISPLAY:-:0}',
                'QT_X11_NO_MITSHM=1',
                f'GZ_IP={vehicle_gazebo_ip}',
                'GZ_PARTITION=realgazebo',
                'PX4_GZ_STANDALONE=1',
                'LOCAL_USER_ID=${LOCAL_USER_ID:-1000}',
                'MAVLINK_GCS_IP=${MAVLINK_GCS_IP:-172.17.0.1}',  # PX4 sends MAVLink to host
                f'FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/dds_profiles/px4_participant_{vid}.xml',
            ],
            'volumes': [
                '/tmp/.X11-unix:/tmp/.X11-unix',
            ],
            'networks': {
                'gazebo-network': {
                    'ipv4_address': vehicle_gazebo_ip
                },
                'vehicle-network': {
                    'ipv4_address': vehicle_network_ip
                }
            },
            'extra_hosts': [
                'host.docker.internal:host-gateway'
            ],
            'ports': [
                f'{18570 + vid}:{18570 + vid}/udp'  # GCS MAVLink link for QGC
            ],
            'depends_on': {
                'gazebo': {
                    'condition': 'service_healthy'
                }
            },
            'command': f'bash -c "source /opt/ros/jazzy/setup.bash && source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash && ros2 launch realgazebo vehicle.launch.py instance_id:={vid} vehicle_type:={vtype} spawnpoint:={spawnpoint_str} px4_path:=/home/user/realgazebo/RealGazebo-PX4 unreal_ip:={unreal_ip} unreal_port:={unreal_port}"',
            'deploy': {
                'resources': {
                    'limits': {
                        'memory': '4G'
                    }
                }
            }
        }

    return compose


def main():
    args = parse_args()

    headless = not args.gui if args.gui else args.headless

    # Determine output path
    if args.output_file:
        output_path = args.output_file
    else:
        # Default: same directory as this script's parent (project root)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        output_path = os.path.join(project_root, 'docker-compose.override.yml')

    # Load configuration
    config = load_config(args.config_file)

    # Generate compose override
    compose = generate_compose_override(
        config,
        unreal_ip=args.unreal_ip,
        unreal_port=args.unreal_port,
        world=args.world,
        headless=headless
    )

    # Write output
    with open(output_path, 'w') as f:
        yaml.dump(compose, f, default_flow_style=False, sort_keys=False)

    # Print summary
    vehicles = config.get('vehicles', {})
    support_obstacle = ['rock']
    vehicle_count = sum(1 for v in vehicles.values() if v.get('type') not in support_obstacle)

    print(f"Generated {output_path}")
    print(f"  - {vehicle_count} vehicle(s) configured:")
    for vid, v in sorted(vehicles.items()):
        if v.get('type') not in support_obstacle:
            print(f"    - vehicle_{vid}: {v['type']} at {v['spawnpoint']}")
    print(f"\nTo start the simulation:")
    print(f"  cd {os.path.dirname(output_path)}")
    print(f"  docker compose up -d")


if __name__ == '__main__':
    main()
