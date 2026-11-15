"""Local-only TA2 chassis demonstration. W/S A/D Q/E; Space stop; R reset."""
import argparse
import math
import queue
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from chassis import Chassis, Square


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
    parser.add_argument('--square', action='store_true')
    parser.add_argument('--side', type=float, default=0.2)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float, default=60)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be positive')
    import mujoco
    model = load_model(args.assets.resolve())
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    chassis = Chassis()
    square = Square(chassis, args.side) if args.square else None
    mocap_id = model.body('shoulder_base').mocapid[0]
    origin = data.mocap_pos[mocap_id].copy()
    keys = queue.SimpleQueue()

    def key(code):
        nonlocal square
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
        if square and square.index != 4:
            raise RuntimeError('Square did not finish within duration')
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
