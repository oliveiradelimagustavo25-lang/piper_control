"""Load the MuJoCo scene and attach its camera to the robot optical frame."""

from pathlib import Path

import mujoco

from ..sensors.camera import add_camera


def build_scene(world_dir, description_dir, robot_model=None, scene_file=None, camera_config=None):
    """Compose a preview or controlled robot with the same world resources."""
    world_dir = Path(world_dir).resolve()
    robot_model = Path(robot_model or Path(description_dir) / 'mjcf/piper.xml').resolve()
    scene_file = Path(scene_file or world_dir / 'basic/preview.xml').resolve()
    # Resolve and flatten the robot's includes before crossing package boundaries.
    robot = mujoco.MjSpec.from_file(str(robot_model))
    mesh_dir = robot_model.parent / robot.meshdir
    for mesh in robot.meshes:
        mesh.file = str((mesh_dir / mesh.file).resolve())
    robot.compile()
    spec = mujoco.MjSpec.from_file(
        str(scene_file), include={'robot_description.xml': robot.to_xml().encode()},
    )
    # Exported runtime XML lives in a temporary directory; textures must remain accessible.
    for texture in spec.textures:
        if texture.file:
            texture.file = str((scene_file.parent / spec.texturedir / texture.file).resolve())
    add_camera(spec, camera_config or world_dir / 'config/camera.yaml')
    return spec


def load_scene(world_dir, description_dir):
    """Load the uncontrolled world used by the independent preview test."""
    return build_scene(world_dir, description_dir).compile()


def export_scene(output_file, world_dir, description_dir, **options):
    """Write flattened MJCF with absolute asset paths for the native plugin."""
    spec = build_scene(world_dir, description_dir, **options)
    spec.compile()
    Path(output_file).write_text(spec.to_xml())
