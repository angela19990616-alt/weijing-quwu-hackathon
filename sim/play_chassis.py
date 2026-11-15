"""Local-only TA2 chassis demonstration. W/S A/D Q/E; Space stop; R reset."""
import argparse
import math
import queue
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from chassis import Chassis, Square, PickRoute


def load_model(folder):
    import mujoco
    root = ET.parse(folder / 'robot.xml').getroot()
    for mesh in root.findall('asset/mesh'):
        mesh.set('file', str((folder / mesh.get('file')).resolve()))
    shoulder = root.find("worldbody/body[@name='shoulder_base']")
    shoulder.set('mocap', 'true')
    scene = ET.parse(folder / 'scene.xml').getroot()
    for tag in ('visual',):
        node = scene.find(tag)
        if node is not None:
            root.append(node)
    for element in scene.find('asset'):
        root.find('asset').append(element)
    for element in scene.find('worldbody'):
        root.find('worldbody').append(element)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, required=True)
    routes = parser.add_mutually_exclusive_group()
    routes.add_argument('--square', action='store_true')
    routes.add_argument('--pick-route', action='store_true')
    parser.add_argument('--width', type=float)
    parser.add_argument('--length', type=float)
    parser.add_argument('--side', type=float, default=0.2)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float, default=60)
    args = parser.parse_args()
    if args.pick_route and (args.width is None or args.length is None):
        parser.error('--pick-route requires --width and --length in meters')
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be positive')
    import mujoco
    model = load_model(args.assets.resolve())
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    chassis = Chassis()
    square = Square(chassis, args.side) if args.square else None
    if args.pick_route:
        square = PickRoute(chassis, args.width, args.length)
    mocap_id = model.body('shoulder_base').mocapid[0]
    origin = data.mocap_pos[mocap_id].copy()
    keys = queue.SimpleQueue()

    def key(code):
        nonlocal square
        if code == 71 and isinstance(square, PickRoute):
            square.confirm_grasp()
            return
        commands = {87: [0.05, 0, 0], 83: [-0.05, 0, 0],
                    65: [0, 0.05, 0], 68: [0, -0.05, 0],
                    81: [0, 0, 0.15], 69: [0, 0, -0.15], 32: [0, 0, 0]}
        if code in commands:
            square = None
            chassis.command(commands[code])
        elif code == 82:
            square = None
            chassis.__init__()
            mujoco.mj_resetDataKeyframe(model, data, 0)

    def tick():
        while not keys.empty():
            key(keys.get_nowait())
        if square:
            square.tick(chassis)
        chassis.step(model.opt.timestep)
        data.mocap_pos[mocap_id] = origin + [chassis.x, chassis.y, 0]
        data.mocap_quat[mocap_id] = [math.cos(chassis.yaw / 2), 0, 0, math.sin(chassis.yaw / 2)]
        mujoco.mj_step(model, data)
        if not all(math.isfinite(v) for v in data.qpos):
            raise RuntimeError('Non-finite simulated joint state')

    if args.headless:
        while chassis.time < args.duration:
            tick()
        print(f'pose=({chassis.x:.4f}, {chassis.y:.4f}, {chassis.yaw:.4f}); '
              f'completed_sides={square.index if square else 0}; sim_time={chassis.time:.2f}')
        if square and square.index != len(square.points):
            raise RuntimeError('Route incomplete; pick route requires explicit G confirmation in viewer')
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model, data, key_callback=keys.put) as viewer:
            viewer.cam.lookat[:] = [0, 0, -0.5]
            viewer.cam.distance = 3
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
            while viewer.is_running():
                start = time.monotonic()
                with viewer.lock():
                    tick()
                viewer.sync()
                time.sleep(max(0, model.opt.timestep - (time.monotonic() - start)))


if __name__ == '__main__':
    main()
