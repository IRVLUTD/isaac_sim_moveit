import argparse
import os

parser = argparse.ArgumentParser(description='Prepare root Scan-3D objects URDFs for Pybullet/IsaacSim',
    usage="\
    python root_prepare_urdf.py -f /PATH/TO/root_DATA_FOLDER/ -m model_list.txt' -o ../rendering-test/root-urdfs/")

parser.add_argument('-m', '--models_file', type=str, default='model_list.txt', help="List of object names to export to URDF")
parser.add_argument('-f', '--root_dir', type=str, default='/mnt/Data/rootScannedObjects/SampleObjects/', help='Path to rootScan3d Models Directory.')
parser.add_argument('-o', '--output_dir', type=str, default='./root-urdfs/', help="Output directory for the urdf files")
# parser.add_argument('-s', '--scale', type=int, default='1000', help='Scale in mm. default is 1000 (i.e 1.0 scale in urdf mesh attribute)')
# parser.add_argument('-t', '--mesh_type', type=str, default='root_16k', help='Which mesh types desired (root__16k, 64k etc...')


def get_urdf_file(ycb_model: str, mesh_path: str) -> str:
    URDF_TEMPLATE = '''<?xml version='1.0' encoding='ASCII'?>
<robot name="{ycb_model_name}">
    <link name="baseLink">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="{ycb_model_mesh_path}" scale="1.0 1.0 1.0"/>
            </geometry>
            <material name="texture">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="{ycb_model_mesh_path}" scale="1.0 1.0 1.0"/>
            </geometry>
        </collision>   
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
        </inertial>
    </link>
</robot>'''
    return URDF_TEMPLATE.format(ycb_model_name=ycb_model, ycb_model_mesh_path=mesh_path)


def main(args):

    if not args.root_dir:
        print(f"models list not specified")
        exit(0)

    if not os.path.isdir(args.root_dir):
        print(f"models directory (containing meshes): {args.root_dir} is incorrect")
        exit(0)

    model_names = ['003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', \
                '007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana', \
                '021_bleach_cleanser', '024_bowl', '025_mug', '037_scissors', '040_large_marker', \
                '052_extra_large_clamp', 'cafe_table_org']

    for model in model_names:
        if model == 'cafe_table_org':
            mesh = 'meshes/cafe_table.dae'
        else:
            mesh = 'textured_simple.obj'

        urdf_content = get_urdf_file(model, mesh)
        # sample fname: 021_bleach_cleanser.urdf
        urdf_fname = f"{model}.urdf"
        output_file = os.path.join(args.root_dir, model, urdf_fname)
        print(output_file)
        with open(output_file, "w") as outf:
            outf.write(urdf_content)


if __name__ == '__main__':
    main(parser.parse_args())
