from PIL import Image, ImageDraw
import numpy as np
from pprint import pprint
import os

# Open the image file
# im = Image.open(".\\images\\Texas_flag_map.png")
# im = Image.open("paidyn.png")
imageFile = "Texas_flag_map.png"
im = Image.open(os.path.join(".", "images", imageFile))

# Get the width and height of the image
width, height = im.size

# Calculate the center of the image
center_x = width / 2
center_y = height / 2

NUM_LEDS = 29
NUM_SLICES = 60

# Set the number of slices
num_slices = 60
num_sectors = (NUM_LEDS // 2)
sector_thickness = (width / 2 ) // num_sectors 

# Calculate the angle between each slice
angle_per_slice = 360.0 / num_slices

# Create a blank image to hold the slices
slices = []
sectors = []

LEDs = [0] * (num_sectors*2)
image_data = []

# Loop through the slices
for i in range(int(num_slices/2)):
    # Create a new image for each slice
    for k in range(2):
        slice = Image.new('RGBA', (width, height), (0, 0, 0, 0))

        # Create a mask for the slice
        mask = Image.new('L', (width, height), 0)
        draw = ImageDraw.Draw(mask)

        # First pieslice
        # draw.pieslice((0, 0, width, height), i * angle_per_slice, (i + 1) * angle_per_slice, fill=255)
        
        # # Second pieslice
        draw.pieslice((0, 0, width, height), ((i * angle_per_slice) + (k*180)), (((i + 1) * angle_per_slice) + (k*180)), fill=255)

        # Use the mask to extract the slice from the original image
        slice.putalpha(mask)
        slice.paste(im, (0, 0), mask)
        # Add the slice to the list of slices
        slices.append(slice)
        
        for j in range(num_sectors):
            sector = Image.new('RGBA', (width,height), (0,0,0,0))
            sectorMask = Image.new('L', (width, height), 0)
            sectorDraw = ImageDraw.Draw(sectorMask)

            outer_radius = (width/2) - (sector_thickness * j)
            inner_radius = outer_radius - sector_thickness
            sectorDraw.pieslice((center_x - outer_radius, center_y - outer_radius, center_x + outer_radius, center_y + outer_radius), 0, 360, fill=255)
            sectorDraw.pieslice((center_x - inner_radius, center_y - inner_radius, center_x + inner_radius, center_y + inner_radius), 0, 360, fill=0)

            sector.putalpha(sectorMask)
            sector.paste(slice, (0,0), sectorMask)
            sectors.append(sector)

            sector_array = np.array(sector)
            mask = (sector_array != 0)
            # print(sector_array)
            # Get the RGB values from the array
            r = sector_array[:, :, 0]
            g = sector_array[:, :, 1]
            b = sector_array[:, :, 2]

            # Mask all the zeros in the array
            r_masked = np.ma.masked_equal(r, 0)
            g_masked = np.ma.masked_equal(g, 0)
            b_masked = np.ma.masked_equal(b, 0)
            # print(i,j,r_masked, g_masked, b_masked)  # Add this line

            # Calculate the average RGB values using NumPy's mean function
            # r_mean = int(np.mean(r_masked))
            # g_mean = int(np.mean(g_masked))
            # b_mean = int(np.mean(b_masked))

            if np.ma.count(r_masked) == 0:
                r_mean = 0
            else:
                r_mean = int(np.mean(r_masked))

            if np.ma.count(g_masked) == 0:
                g_mean = 0
            else:
                g_mean = int(np.mean(g_masked))

            if np.ma.count(b_masked) == 0:
                b_mean = 0
            else:
                b_mean = int(np.mean(b_masked))


            if(k):
                index=(num_sectors*2 - 1- j)
            else: 
                index=j   
            LEDs[index] = '0x'+hex(r_mean)[2:].zfill(2)+hex(g_mean)[2:].zfill(2)+hex(b_mean)[2:].zfill(2)
    # LEDs[num_sectors]='0x0'
    LEDs.insert(num_sectors,'0x000000')
    image_data.append(LEDs)
    print(LEDs)
    LEDs = [0] * (num_sectors*2)

# myArray = np.array(image_data)
# np.savetxt('image.txt',myArray)
outFileName = imageFile[:-4]+"_out.h"
with open(os.path.join(".", "images", outFileName), 'w') as f:
    f.write("#include \"Arduino.h\"\n\n")
    f.write("#ifndef "+ outFileName.upper().replace('.','_')+'\n')
    f.write("#define "+ outFileName.upper().replace('.','_')+'\n\n')
    f.write("#define NUM_LEDS " + str(NUM_LEDS) + "\n")
    f.write("#define SLICES " + str(num_slices) + "\n\n")

    for i, row in enumerate(image_data):
        f.write("const uint32_t LED_SLICE_"+str(i)+"[NUM_LEDS] PROGMEM = {\n")
        f.write(', '.join(row[:num_sectors+1]) + ',\n')
        f.write(', '.join(row[num_sectors+1:-1]) + ', ' + row[-1])
        f.write("\n};\n\n")
    f.write("const uint32_t* const FRAME_ARRAY[SLICES / 2] PROGMEM = {\n")
    for i in range(num_slices/2):
        f.write("LED_SLICE_" + str(i))
        if (i!=num_slices -1):
            f.write(', ')
        if not((i+1)%10):
            f.write('\n')
    f.write("\n};\n\n")
    f.write("#endif //"+outFileName.upper().replace('.','_')+'\n')

# Save each slice as a separate image file
# for i in range(len(slices)):
#     slices[i].save(".\\images\\slice_{}.png".format(i))
#     for j in range(num_sectors):
#         sectors[(i*num_sectors)+j].save(".\\images\\slice_{}-sector_{}.png".format(i,j))
# pprint(image_data, width=800, indent=4)
