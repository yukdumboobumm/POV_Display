from PIL import Image, ImageDraw
import numpy as np
from pprint import pprint
import os

# Open the image file
# im = Image.open(".\\images\\Texas_flag_map.png")
# im = Image.open("paidyn.png")
# imageFile = "Texas_flag_map.png"
imageFile = "paidyn.png"
im = Image.open(os.path.join(".", "images", imageFile))

# Get the width and height of the image
width, height = im.size

# Calculate the center of the image
center_x = width / 2
center_y = height / 2

NUM_LEDS = 29
NUM_SLICES = 120

# Set the number of slices
num_slices = NUM_SLICES
num_sectors = (NUM_LEDS // 2)
sector_thickness = (width / 2 ) // num_sectors 

# Calculate the angle between each slice
angle_per_slice = 360.0 / num_slices

# Create lists to hold the slices and sectors
slices = []
sectors = []

#pre initialize an LED strip, create list for the raw data
LEDs = [0] * (num_sectors*2)
image_data = []

#get a slice from an image, will work on the arguments here...a bit lazy
def makeSlice(i,k):
    slice = Image.new('RGBA', (width, height), (0, 0, 0, 0))
    # Create a mask for the slice
    mask = Image.new('L', (width, height), 0)
    draw = ImageDraw.Draw(mask)

    sliceMirror = k * 180
    startAngle = (i * angle_per_slice) + sliceMirror
    endAngle = ((i + 1) * angle_per_slice) + sliceMirror
    boundingBox = ((0, 0), (width, height))

    # # Second pieslice
    draw.pieslice(boundingBox, startAngle, endAngle, fill=255)
    
    # Use the mask to extract the slice from the original image
    slice.putalpha(mask)
    slice.paste(im, (0, 0), mask)
    return slice

#make a sector from a slice, lazy with arguments here too
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

#get the average RGB color within a sector
#that numpy mean process is pretty expensive, might look at upgrading this soon
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
    with open(os.path.join(".", "output", outFileName), 'w') as f:
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
        sliceName = os.path.join(".", "output", "slicedImages", "slice_{}.png".format(i))
        slices[i].save(sliceName)
        for j in range(num_sectors):
            sectorName = os.path.join(".", "output", "slicedImages", "slice_{}-sector_{}.png".format(i,j))
            sectors[(i*num_sectors)+j].save(sectorName)
    # pprint(image_data, width=800, indent=4)

def saveRawData():
    outFileName = imageFile[:-4]+"_raw.txt"
    with open(os.path.join(".", "output", outFileName), 'w') as f:
        for row in image_data:
            f.write(', '.join(row) + ',\n')

#kind of the opposite of makeSlice --> makeSector but using a different process. Might actually be slower though :(
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



#main loop where all the magic happens
def main():
    # loop to create image slices
    #since I have a full rotor that extends across the entire image I only need half of the slices.
    #could update to make portalbe for use cases with only half a rotor (many POV displays use a half)
    for i in range(int(num_slices/2)):
        # i'm creating bow tie slices here, so i do the same thing twice, just mirrored 180deg on k=1
        for k in range(2):
            # makeSlice does it what it says it does
            slice = makeSlice(i,k)
            slices.append(slice)
            
            #after I make a slice I create two concentric circles to wind up with a sectors
            #these sectors have a span set by the number of slices and a thickness set by the number of LEDs
            for j in range(num_sectors):
                sector = makeSector(slice, j)
                sectors.append(sector)
                #sectors are created from the outside converging on the center
                #but the leds count from 0 - NUMLEDS from edge to edge
                #if k is zero, the count is normal, but for the mirrored sector (k==1) I need to set the LEDS from NUM_LEDS to the center
                #hence the need to initialize the list
                if(k):
                    index=(num_sectors*2 - 1- j)
                else: 
                    index=j   
                LEDs[index] = getRGB(sector)

        # LEDs[num_sectors]='0x0'
        #insert a blank pixel into the middle of the strip
        #that pixel doesn't actually move so having any different colors just looks like a flicker
        #3 options: 1)turn it off, 2)pick an accent color or 3)average all of the middle+1 and middle-1 pixels and it to that
        LEDs.insert(num_sectors,'0x000000')
        #add the slice to the image_data
        image_data.append(LEDs)
        #debug info to keep the terminal updated
        print(i,LEDs)
        #initialize the list. not neccesary but why not.
        LEDs = [0] * (num_sectors*2)

    ##What actions do you want to take with the data?
    #generate a header file for an arduino
    makeHeaderFile()
    #save the raw hex values
    # saveRawData()
    #save the sliced images (useful for debugging)
    # saveSlicedImages()
    #get a glimpse at what the result will look like
    reconstituteImage()

#make sure I'm not a module
if __name__ == "__main__":
    main()
