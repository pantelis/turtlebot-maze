from PIL import Image, ImageDraw
import os

# Output directory for textures
texture_dir = "./materials/textures"
os.makedirs(texture_dir, exist_ok=True)

# Texture generation functions
def create_checkerboard(name, size=256, block_size=32, color1=(255, 255, 255), color2=(0, 0, 255)):
    img = Image.new("RGB", (size, size), color1)
    draw = ImageDraw.Draw(img)
    for y in range(0, size, block_size):
        for x in range(0, size, block_size):
            if (x // block_size + y // block_size) % 2 == 0:
                draw.rectangle([x, y, x+block_size-1, y+block_size-1], fill=color2)
    img.save(os.path.join(texture_dir, name))

def create_noise_texture(name, size=256):
    import numpy as np
    arr = (np.random.rand(size, size, 3) * 255).astype("uint8")
    img = Image.fromarray(arr, "RGB")
    img.save(os.path.join(texture_dir, name))

def create_brick_pattern(name, size=256, brick_size=(64, 32), mortar=(200, 200, 200), brick=(150, 50, 50)):
    img = Image.new("RGB", (size, size), mortar)
    draw = ImageDraw.Draw(img)
    for y in range(0, size, brick_size[1]):
        offset = (brick_size[0] // 2) if (y // brick_size[1]) % 2 else 0
        for x in range(-offset, size, brick_size[0]):
            draw.rectangle([x, y, x + brick_size[0] - 2, y + brick_size[1] - 2], fill=brick)
    img.save(os.path.join(texture_dir, name))

# Generate texture files
create_checkerboard("checker_blue.png", color2=(0, 0, 255))
create_checkerboard("checker_red.png", color2=(255, 0, 0))
create_checkerboard("checker_green.png", color2=(0, 255, 0))
create_noise_texture("noise.png")
create_brick_pattern("bricks.png")

os.listdir(texture_dir)
