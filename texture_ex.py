import omni
import carb
from pxr import UsdShade, Sdf

# Change the server to your Nucleus install, default is set to localhost in omni.isaac.sim.base.kit
default_server = carb.settings.get_settings().get("/persistent/isaac/asset_root/default")
mtl_created_list = []
# Create a new material using OmniPBR.mdl
omni.kit.commands.execute(
    "CreateAndBindMdlMaterialFromLibrary",
    mdl_name="OmniPBR.mdl",
    mtl_name="OmniPBR",
    mtl_created_list=mtl_created_list,
)
stage = omni.usd.get_context().get_stage()
mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
# Set material inputs, these can be determined by looking at the .mdl file
# or by selecting the Shader attached to the Material in the stage window and looking at the details panel
omni.usd.create_material_input(
    mtl_prim,
    "diffuse_texture",
    "/home/student/Documents/tang/isaac_robotsim/frame_0001.jpg",
    #default_server + "/Isaac/Samples/DR/Materials/Textures/marble_tile.png",
    Sdf.ValueTypeNames.Asset,
)
# Create a prim to apply the material to
result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
# Get the path to the prim
cube_prim = stage.GetPrimAtPath(path)
# Bind the material to the prim
cube_mat_shade = UsdShade.Material(mtl_prim)
UsdShade.MaterialBindingAPI(cube_prim).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)
