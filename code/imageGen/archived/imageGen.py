from PIL import Image, ImageDraw

# Open the image file
im = Image.open("paidyn.png")

# Get the width and height of the image
width, height = im.size

# Calculate the center of the image
center_x = width / 2
center_y = height / 2

# Set the number of slices
num_slices = 60

# Calculate the angle between each slice
angle_per_slice = 360.0 / num_slices

# Create a blank image to hold the slices
slices = []

# Loop through the slices
for i in range(num_slices):
    # Create a new image for each slice
    slice = Image.new('RGBA', (width, height), (0, 0, 0, 0))

    # Create a mask for the slice
    mask = Image.new('L', (width, height), 0)
    draw = ImageDraw.Draw(mask)
    draw.pieslice((0, 0, width, height), i * angle_per_slice, (i + 1) * angle_per_slice, fill=255)

    # Use the mask to extract the slice from the original image
    slice.putalpha(mask)
    slice.paste(im, (0, 0), mask)

    # Add the slice to the list of slices
    slices.append(slice)

# Save each slice as a separate image file
for i, slice in enumerate(slices):
    slice.save("slice_{}.png".format(i))
