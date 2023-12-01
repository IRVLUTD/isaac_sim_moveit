import os
import argparse

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.kit.commands


def import_urdf(urdf_path, dest_path):
    """ Import URDF from manager.grippers dictionary

    Args: None
    """
    _status, _config = omni.kit.commands.execute("URDFCreateImportConfig")
    _config.merge_fixed_joints = True
    _config.convex_decomp = True
    _config.import_inertia_tensor = True
    _config.fix_base = True
    _config.make_default_prim = True
    _config.self_collision = False
    _config.create_physics_scene = True
    _config.import_inertia_tensor = False
    _config.make_instanceable = True
    _config.instanceable_usd_path = dest_path

    result = omni.kit.commands.execute(
            "URDFParseAndImportFile", 
            urdf_path=urdf_path, 
            import_config=_config, 
            dest_path=dest_path
    )
    return


def make_parser():
    parser = argparse.ArgumentParser(description='Prepare root Scan-3D objects USDs for IsaacSim. Need to run it with IsaacSim Python Env',
    usage="\
    ./python.sh ~/PATH_TO_SCRIPT/save_object_usd.py -f Path_To_rootData -m model_list.txt'")

    parser.add_argument('-m', '--models_file', type=str, required=False, help="List of object names to export to URDF")
    parser.add_argument('-f', '--root_dir', type=str, required=True, help='Path to rootScan3d Models Directory.')
    parser.add_argument('-o', '--output_dir', type=str, required=False, help='Path to Output Directory with USDs.')
    return parser


if __name__ == "__main__":
    parser = make_parser()
    args = parser.parse_args()

    if not os.path.isdir(args.root_dir):
        print(f"models directory (containing meshes): {args.root_dir} is incorrect")
        exit(0)

    model_names = ['003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', \
                '007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana', \
                '021_bleach_cleanser', '024_bowl', '025_mug', '037_scissors', '040_large_marker', \
                '052_extra_large_clamp', 'cafe_table_org']
   
    for model in model_names:
        # print(model)
        # model = model_names[0]
        model_p = os.path.join(args.root_dir, model)
        urdf_p  = os.path.join(model_p, f"{model}.urdf")
        usd_p = os.path.join(model_p, f"{model}.usd")
        print(usd_p)
        if os.path.exists(usd_p):
            os.remove(usd_p)
        import_urdf(urdf_path=urdf_p, dest_path=usd_p)

    simulation_app.close()
