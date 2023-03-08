import numpy as np
from PIL import Image

# Read the data from array.txt into a 2D numpy array
with open("array.txt") as f:
    data = [line.split() for line in f]
data = np.array(data, dtype=np.uint8)

# Define the parameters of the bowtie slices and sectors
n_slices = 30
n_sectors = 29
sector_angle = 360.0 / n_sectors
inner_radius = 50
outer_radius = 500

# Create an empty 2D numpy array to store the reconstructed image
image_size = (outer_radius + inner_radius) * 2
image = np.zeros((image_size, image_size), dtype=np.uint8)

# Reconstruct the bowtie slices and sectors from the data
for i in range(n_slices):
    start_angle = i * 360.0 / n_slices
    end_angle = start_angle + sector_angle / 2
    for j in range(n_sectors):
        sector_start_angle = start_angle + j * sector_angle
        sector_end_angle = sector_start_angle + sector_angle
        slice_data = data[i, j]
        if slice_data != 0:
            sector_data = np.full((outer_radius - inner_radius, sector_angle), slice_data, dtype=np.uint8)
            sector_image = Image.fromarray(sector_data, mode='L')
            sector_image = sector_image.rotate(180 - sector_start_angle, resample=Image.BILINEAR, expand=True)
            sector_position = (image_size // 2 - outer_radius, 0)
            image.paste(sector_image, sector_position)

# Reconstruct the bowtie slices from the sectors and combine them radially
for i in range(n_slices):
    start_angle = i * 360.0 / n_slices
    end_angle = start_angle + sector_angle / 2
    slice_image = Image.new('L', (image_size, outer_radius - inner_radius), 0)
    for j in range(n_sectors):
        sector_start_angle = start_angle + j * sector_angle
        sector_end_angle = sector_start_angle + sector_angle
        sector_position = (image_size // 2 - outer_radius, 0)
        sector_image = image.crop((sector_position[0], sector_position[1], sector_position[0] + outer_radius * 2, sector_position[1] + outer_radius - inner_radius))
        sector_image = sector_image.rotate(sector_start_angle - 180, resample=Image.BILINEAR, expand=True)
        slice_image.paste(sector_image, (0, j * (outer_radius - inner_radius)))
    slice_position = (0, i * (outer_radius - inner_radius))
    image.paste(slice_image, slice_position)

# Save the reconstructed image as a file
reconstructed_image = Image.fromarray(image, mode='L')
reconstructed_image.save("reconstructed_image.png")
