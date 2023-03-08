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
NUM_SLICES = 80

# Set the number of slices
num_slices = NUM_SLICES
num_sectors = (NUM_LEDS // 2)
sector_thickness = (width / 2 ) // num_sectors 

# Calculate the angle between each slice
angle_per_slice = 360.0 / num_slices

# Create a blank image to hold the slices
slices = []
sectors = []

LEDs = [0] * (num_sectors*2)
image_data = []

def makeSlice(i,k):
    slice = Image.new('RGBA', (width, height), (0, 0, 0, 0))
    # Create a mask for the slice
    mask = Image.new('L', (width, height), 0)
    draw = ImageDraw.Draw(mask)

    # # Second pieslice
    draw.pieslice(((0, 0), (width, height)), ((i * angle_per_slice) + (k*180)), (((i + 1) * angle_per_slice) + (k*180)), fill=255)

    # Use the mask to extract the slice from the original image
    slice.putalpha(mask)
    slice.paste(im, (0, 0), mask)
    return slice

def makeSector(slice, j):
    sector = Image.new('RGBA', (width,height), (0,0,0,0))
    sectorMask = Image.new('L', (width, height), 0)
    sectorDraw = ImageDraw.Draw(sectorMask)

    outer_radius = (width/2) - (sector_thickness * j)
    inner_radius = outer_radius - sector_thickness

    x0_outer = center_x - outer_radius
    y0_outer = center_y - outer_radius
    x1_outer = center_x + outer_radius
    y1_outer = center_y + outer_radius

    x0_inner = center_x - inner_radius
    y0_inner = center_y - inner_radius
    x1_inner = center_x + inner_radius
    y1_inner = center_y + inner_radius

    sectorDraw.pieslice(((x0_outer, y0_outer), (x1_outer, y1_outer)), 0, 360, fill=255)
    sectorDraw.pieslice(((x0_inner, y0_inner), (x1_inner, y1_inner)), 0, 360, fill=0)

    sector.putalpha(sectorMask)
    sector.paste(slice, (0,0), sectorMask)
    # sector.show()
    return sector


def getRGB(sector) :
    sector_array = np.array(sector)
    # mask = (sector_array != 0)
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
    
    rgbVal = '0x'+hex(r_mean)[2:].zfill(2)+hex(g_mean)[2:].zfill(2)+hex(b_mean)[2:].zfill(2)
    return rgbVal

def makeHeaderFile():
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
        for i in range(num_slices//2):
            f.write("LED_SLICE_" + str(i))
            if (i!=num_slices -1):
                f.write(', ')
            if not((i+1)%10):
                f.write('\n')
        f.write("\n};\n\n")
        f.write("#endif //"+outFileName.upper().replace('.','_')+'\n')

def saveSlicedImages():
# Save each slice as a separate image file
    for i in range(len(slices)):
        slices[i].save(".\\images\\slice_{}.png".format(i))
        for j in range(num_sectors):
            sectors[(i*num_sectors)+j].save(".\\images\\slice_{}-sector_{}.png".format(i,j))
    pprint(image_data, width=800, indent=4)

def saveRawData():
    outFileName = imageFile[:-4]+"_raw.txt"
    with open(os.path.join(".", "images", outFileName), 'w') as f:
        for row in image_data:
            f.write(', '.join(row) + ',\n')

def reconstituteImage():
    output_image = Image.new('RGBA', (width, height), (0, 0, 0, 0))
    outFileName = imageFile[:-4]+"_recon.png"
    for i,row in enumerate(image_data):
        for k in range(2):
            if k==0:
                pixels = row[:num_sectors]
            else:
                pixels = row[len(row)-1::-1]
            for j,pixel in enumerate(pixels):
                outer_radius = (width/2) - (sector_thickness * j)
                inner_radius = outer_radius - sector_thickness

                x0_outer = center_x - outer_radius
                y0_outer = center_y - outer_radius
                x1_outer = center_x + outer_radius
                y1_outer = center_y + outer_radius

                x0_inner = center_x - inner_radius
                y0_inner = center_y - inner_radius
                x1_inner = center_x + inner_radius
                y1_inner = center_y + inner_radius

                start_angle = (i * angle_per_slice) + (k*180)
                end_angle = ((i + 1) * angle_per_slice) + (k*180)

                color_hex = int(pixel,16)
                r = (color_hex >> 16) & 0xff
                g = (color_hex >> 8) & 0xff
                b = color_hex & 0xff
                color = (r,g,b)
                pixel_mask = Image.new("L", im.size, 0)
                overlay = Image.new('RGB', (width, height), color)
                draw = ImageDraw.Draw(pixel_mask)
                draw.pieslice(((0, 0), (width, height)), start_angle, end_angle, fill=0)
                draw.pieslice(((x0_outer, y0_outer), (x1_outer, y1_outer)), start_angle, end_angle, fill=255)
                draw.pieslice(((x0_inner, y0_inner), (x1_inner, y1_inner)), start_angle, end_angle, fill=0)
                output_image.paste(overlay, (0,0), pixel_mask)
        # output_image.show()
    # Save the output image to a file
    output_image.save(os.path.join(".", "images", outFileName))







# Loop through the slices
for i in range(int(num_slices/2)):
    # Create a new image for each slice
    for k in range(2):
        # Add the slice to the list of slices
        slice = makeSlice(i,k)
        slices.append(slice)
        
        for j in range(num_sectors):
            sector = makeSector(slice, j)
            sectors.append(sector)
            if(k):
                index=(num_sectors*2 - 1- j)
            else: 
                index=j   
            LEDs[index] = getRGB(sector)

    # LEDs[num_sectors]='0x0'
    LEDs.insert(num_sectors,'0x000000')
    image_data.append(LEDs)
    print(LEDs)
    LEDs = [0] * (num_sectors*2)

# myArray = np.array(image_data)
# np.savetxt('image.txt',myArray)

##What actions do you want to take with the data?
makeHeaderFile()
# saveRawData()
# saveSlicedImages()
reconstituteImage()


