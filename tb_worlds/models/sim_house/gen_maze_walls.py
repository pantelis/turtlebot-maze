import xml.etree.ElementTree as ET

# Define path to user's local SDF file
local_model_path = "./tb_worlds/models/sim_house/model.sdf"  # user will upload this file
output_path = "./tb_worlds/models/sim_house/maze_vslam.sdf"

# Define available textures to rotate through
texture_files = [
    "checker_blue.png",
    "checker_red.png",
    "checker_green.png",
    "noise.png",
    "bricks.png",
]
# Check if the file exists and parse it
try:
    tree = ET.parse(local_model_path)
    root = tree.getroot()

    # Rotate textures again
    texture_index = 0
    for visual in root.findall(".//visual"):
        material = visual.find("material")
        if material is None:
            material = ET.SubElement(visual, "material")
        script = material.find("script")
        if script is None:
            script = ET.SubElement(material, "script")
        uri = script.find("uri")
        if uri is None:
            uri = ET.SubElement(script, "uri")
        uri.text = "file://media/materials/scripts/gazebo.material"
        name = script.find("name")
        if name is None:
            name = ET.SubElement(script, "name")
        name.text = "Gazebo/FlatGrey"

        diffuse = material.find("diffuse")
        if diffuse is None:
            diffuse = ET.SubElement(material, "diffuse")
        texture_name = texture_files[texture_index % len(texture_files)]
        #diffuse.text = f"file://media/materials/textures/{texture_name}"
        #diffuse.text = f"__model__/materials/textures/{texture_name}"
        #diffuse.text = f"materials/textures/{texture_name}"
        diffuse.text = f"file://materials/textures/{texture_name}"

        texture_index += 1

    # Save the new SDF with textures
    tree.write(output_path)
    sdf_ready = True
except Exception as e:
    sdf_ready = False
    error_message = str(e)

sdf_ready, output_path if sdf_ready else error_message

