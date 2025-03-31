import xml.etree.ElementTree as ET

# Define path to user's local SDF file
local_model_path = (
    "./tb_worlds/models/sim_house/model.sdf"  # user will upload this file
)
output_path = "./tb_worlds/models/sim_house/maze_bricks.sdf"

# Define available textures to rotate through
texture_files = [
    "checker_blue.png",
    "checker_red.png",
    "checker_green.png",
    "noise.png",
    "bricks.png",
]

# Initialize variables
sdf_ready = False
error_message = "Unknown error"

# Check if the file exists and parse it
try:
    tree = ET.parse(local_model_path)
    root = tree.getroot()

    texture_index = 0
    for visual in root.findall(".//visual"):
        material = visual.find("material")
        if material is None:
            material = ET.SubElement(visual, "material")
        else:
            # Clear existing children for clean insertion
            for child in list(material):
                material.remove(child)

        # Script block
        script = ET.SubElement(material, "script")
        uri = ET.SubElement(script, "uri")
        uri.text = "file://media/materials/scripts/gazebo.material"
        name = ET.SubElement(script, "name")
        name.text = "Gazebo/Bricks"  # You can vary this if needed

        # Add PBR material
        pbr = ET.SubElement(material, "pbr")
        metal = ET.SubElement(pbr, "metal")
        texture_name = texture_files[texture_index % len(texture_files)]
        albedo_map = ET.SubElement(metal, "albedo_map")
        albedo_map.text = f"materials/textures/{texture_name}"

        # Standard material properties
        ambient = ET.SubElement(material, "ambient")
        ambient.text = "1 1 1 1"
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = "1 1 1 1"
        specular = ET.SubElement(material, "specular")
        specular.text = "0.2 0.2 0.2 1"
        emissive = ET.SubElement(material, "emissive")
        emissive.text = "0 0 0 1"

        texture_index += 1

    # Save the updated file
    tree.write(output_path)
    sdf_ready = True

except Exception as e:
    sdf_ready = False
    error_message = str(e)

sdf_ready, output_path if sdf_ready else error_message
