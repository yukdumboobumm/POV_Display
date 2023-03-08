from PIL import Image, ImageDraw
import numpy as np

# Open the image file
im = np.array(Image.open("paidyn.png"))

# Get the width and height of the image
# width, height = im.size
height, width, channels = im.shape

# Calculate the center of the image
center_x = width / 2
center_y = height / 2

# Set the number of slices
num_slices = 60
num_sectors = 14
sector_thickness = (width / 2 ) // num_sectors 

# Calculate the angle between each slice
angle_per_slice = 360.0 / num_slices

# Create a blank image to hold the slices
slices = []
sectors = []
sector_avgs = []

# LEDs = [0] * (num_sectors*2 + 1)
# FRAME = [LEDs] * (int(num_slices/2))
# image_data = [['0x00000000'] * (num_sectors * 2 + 1) for i in range(num_slices//2)]
LEDs = []
frame_data = []

# Loop through the slices
for i in range(int(num_slices/2)):
    # First pieslice
    # draw.pieslice((0, 0, width, height), i * angle_per_slice, (i + 1) * angle_per_slice, fill=255)
    start_angle = i * angle_per_slice
    end_angle = (i + 1) * angle_per_slice
    
    # # Second pieslice
    # draw.pieslice((0, 0, width, height), (i * angle_per_slice) + 180, ((i + 1) * angle_per_slice) + 180, fill=255)
    for j in range(num_sectors):
        outer_radius = (width/2) - (sector_thickness * j)
        inner_radius = outer_radius - sector_thickness

        vertex1_x = int(center_x + inner_radius * np.cos(np.radians(start_angle)))
        vertex1_y = int(center_y - inner_radius * np.sin(np.radians(start_angle)))
        vertex2_x = int(center_x + outer_radius * np.cos(np.radians(start_angle)))
        vertex2_y = int(center_y - outer_radius * np.sin(np.radians(start_angle)))

        # Calculate the indices of the pixels along the sector
        sector_indices = np.array(ImageDraw.Draw(Image.new('L', (width, height), 0)).pie((0, 0, width, height), start_angle, end_angle, fill=255, outline=None).getdata()).nonzero()[0]

        # Get the intersection of the slice indices and the sector indices
        sector_indices = np.intersect1d(indices, sector_indices)

        # Get the RGB values of the pixels in the sector
        sector_pixels.append(im.flat[sector_indices * channels : (sector_indices + 1) * channels].reshape(-1, channels))

    # Calculate the average RGB value of the sector
    sector_avg = np.average(np.concatenate(sector_pixels), axis=0)


    # Add the slice to the list of slices
    slices.append(slice)

# Save each slice as a separate image file
for i, slice in enumerate(slices):
    slice.save(".\\images\\slice_{}.png".format(i))
    for j in range(num_sectors):
        sectors[(i*num_sectors)+j].save(".\\images\\slice_{}-sector_{}.png".format(i,j))
