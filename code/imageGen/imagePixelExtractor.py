from PIL import Image, ImageDraw
import numpy as np
from pprint import pprint
import os

##Global defs

# Open the image file
IMAGE_FILE = "Texas_flag_map.png"
# IMAGE_FILE = "paidyn.png"
IM = Image.open(os.path.join(".", "images", IMAGE_FILE))

NUM_LEDS = 29
NUM_SLICES = 120

# Get the width and height of the image
WIDTH, HEIGHT = IM.size

# Calculate the center of the image
CENTER_X = WIDTH / 2
CENTER_Y = HEIGHT / 2

MIN_DIM = min(WIDTH, HEIGHT)

# Set the number of slices
NUM_SECTORS = (NUM_LEDS // 2)
SECTOR_THICKNESS = (MIN_DIM / 2 ) // NUM_SECTORS 

# Calculate the angle between each slice
SLICE_ANGLE = 360.0 / NUM_SLICES

###FUNCTION DEFS

#get a slice from an image, will work on the arguments here...a bit lazy
def makeSlice(i,k):
    slice = Image.new('RGBA', (WIDTH, HEIGHT), (0, 0, 0, 0))
    # Create a mask for the slice
    mask = Image.new('L', (WIDTH, HEIGHT), 0)
    draw = ImageDraw.Draw(mask)

    sliceMirror = k * 180
    startAngle = (i * SLICE_ANGLE) + sliceMirror
    endAngle = ((i + 1) * SLICE_ANGLE) + sliceMirror
    boundingBox = ((0, 0), (WIDTH, HEIGHT))

    ## draw pieslice
    draw.pieslice(boundingBox, startAngle, endAngle, fill=255)
    
    # Use the mask to extract the slice from the original image
    slice.putalpha(mask)
    slice.paste(IM, (0, 0), mask)
    return slice

#make a sector from a slice, lazy with arguments here too
def makeSector(slice, j):
    sector = Image.new('RGBA', (WIDTH,HEIGHT), (0,0,0,0))
    sectorMask = Image.new('L', (WIDTH, HEIGHT), 0)
    sectorDraw = ImageDraw.Draw(sectorMask)

    outer_radius = (WIDTH/2) - (SECTOR_THICKNESS * j)
    inner_radius = outer_radius - SECTOR_THICKNESS

    x0_outer = CENTER_X - outer_radius
    y0_outer = CENTER_Y - outer_radius
    x1_outer = CENTER_X + outer_radius
    y1_outer = CENTER_Y + outer_radius

    x0_inner = CENTER_X - inner_radius
    y0_inner = CENTER_Y - inner_radius
    x1_inner = CENTER_X + inner_radius
    y1_inner = CENTER_Y + inner_radius

    sectorDraw.pieslice(((x0_outer, y0_outer), (x1_outer, y1_outer)), 0, 360, fill=255)
    sectorDraw.pieslice(((x0_inner, y0_inner), (x1_inner, y1_inner)), 0, 360, fill=0)

    sector.putalpha(sectorMask)
    sector.paste(slice, (0,0), sectorMask)
    # sector.show()
    return sector

#get the average RGB color within a sector
#that numpy mean process is pretty expensive, might look at upgrading this soon
def getRGB(sector) :
    # create a numpy array from the sector canvas
    #each element in the array represents a pixel
    #and has 3-dimensions (height, width, (RGBA))
    sector_array = np.array(sector)
    #create a masked 3D array of T/F based on the alpha channel (preserves any black in the image)
    alphaMask = sector_array[...,3]!=0
    #create a new 1D array containing only the values with non-zero alpha
    sector_channels = sector_array[alphaMask]

    #check that there are actually some values to average
    if (sector_channels.size):
        # Get the RGB values from the array
        #[x,y,(r,g,b)] so slice all columns and all row and return the appropriate channel
        r = sector_channels[:, 0]
        g = sector_channels[:, 1]
        b = sector_channels[:, 2]
        #calculate the mean. though perhaps r.sum() / len(r) would be faster? Need to learn more about np.mean
        r_mean = int(np.mean(r))
        g_mean = int(np.mean(g))
        b_mean = int(np.mean(b))
        rgbVal = '0x'+hex(r_mean)[2:].zfill(2)+hex(g_mean)[2:].zfill(2)+hex(b_mean)[2:].zfill(2)
    #if there aren't then we need to assign some value. I've chosen black but it can be any image-background that we prefer
    else:
        rgbVal = '0x000000'
    return rgbVal

def makeHeaderFile(imData):
    outFileName = IMAGE_FILE[:-4]+"_out.h"
    with open(os.path.join(".", "output", outFileName), 'w') as f:
        f.write("#include \"Arduino.h\"\n\n")
        f.write("#ifndef "+ outFileName.upper().replace('.','_')+'\n')
        f.write("#define "+ outFileName.upper().replace('.','_')+'\n\n')
        f.write("#define NUM_LEDS " + str(NUM_LEDS) + "\n")
        f.write("#define SLICES " + str(NUM_SLICES) + "\n\n")

        for i, row in enumerate(imData):
            f.write("const uint32_t LED_SLICE_"+str(i)+"[NUM_LEDS] PROGMEM = {\n")
            f.write(', '.join(row[:NUM_SECTORS+1]) + ',\n')
            f.write(', '.join(row[NUM_SECTORS+1:-1]) + ', ' + row[-1])
            f.write("\n};\n\n")
        f.write("const uint32_t* const FRAME_ARRAY[SLICES / 2] PROGMEM = {\n")
        for i in range(NUM_SLICES//2):
            f.write("LED_SLICE_" + str(i))
            if (i!=NUM_SLICES -1):
                f.write(', ')
            if not((i+1)%10):
                f.write('\n')
        f.write("\n};\n\n")
        f.write("#endif //"+outFileName.upper().replace('.','_')+'\n')

def saveSlicedImages(sliceList, sectorList):
# Save each slice as a separate image file
    for i in range(len(sliceList)):
        sliceName = os.path.join(".", "output", "slicedImages", "slice_{}.png".format(i))
        sliceList[i].save(sliceName)
        for j in range(NUM_SECTORS):
            sectorName = os.path.join(".", "output", "slicedImages", "slice_{}-sector_{}.png".format(i,j))
            sectorList[(i*NUM_SECTORS)+j].save(sectorName)
    # pprint(image_data, WIDTH=800, indent=4)

def saveRawData(imData):
    outFileName = IMAGE_FILE[:-4]+"_raw.txt"
    with open(os.path.join(".", "output", outFileName), 'w') as f:
        for row in imData:
            f.write(', '.join(row) + ',\n')

#kind of the opposite of makeSlice --> makeSector but using a different process. Might actually be slower though :(
def reconstituteImage(imData):
    output_image = Image.new('RGBA', (WIDTH, HEIGHT), (0, 0, 0, 0))
    outFileName = IMAGE_FILE[:-4]+"_recon.png"
    for i,row in enumerate(imData):
        for k in range(2):
            if k==0:
                pixels = row[:NUM_SECTORS]
            else:
                pixels = row[len(row)-1::-1]
            for j,pixel in enumerate(pixels):
                outer_radius = (WIDTH/2) - (SECTOR_THICKNESS * j)
                inner_radius = outer_radius - SECTOR_THICKNESS

                x0_outer = CENTER_X - outer_radius
                y0_outer = CENTER_Y - outer_radius
                x1_outer = CENTER_X + outer_radius
                y1_outer = CENTER_Y + outer_radius

                x0_inner = CENTER_X - inner_radius
                y0_inner = CENTER_Y - inner_radius
                x1_inner = CENTER_X + inner_radius
                y1_inner = CENTER_Y + inner_radius

                start_angle = (i * SLICE_ANGLE) + (k*180)
                end_angle = ((i + 1) * SLICE_ANGLE) + (k*180)

                color_hex = int(pixel,16)
                r = (color_hex >> 16) & 0xff
                g = (color_hex >> 8) & 0xff
                b = color_hex & 0xff
                color = (r,g,b)
                pixel_mask = Image.new("L", IM.size, 0)
                overlay = Image.new('RGB', (WIDTH, HEIGHT), color)
                draw = ImageDraw.Draw(pixel_mask)
                draw.pieslice(((0, 0), (WIDTH, HEIGHT)), start_angle, end_angle, fill=0)
                draw.pieslice(((x0_outer, y0_outer), (x1_outer, y1_outer)), start_angle, end_angle, fill=255)
                draw.pieslice(((x0_inner, y0_inner), (x1_inner, y1_inner)), start_angle, end_angle, fill=0)
                output_image.paste(overlay, (0,0), pixel_mask)
        # print(i, end=", ")
        # output_image.show()
    # Save the output image to a file
    output_image.save(os.path.join(".", 'output', 'images', outFileName))


###MAIN

#main loop where all the magic happens
def main():
    #pre initialize an LED strip, create list for the raw data
    LEDs = [0] * (NUM_SECTORS*2)
    image_data = []
    # Create lists to hold the slices and sectors
    slices = []
    sectors = []
    # loop to create image slices
    #since I have a full rotor that extends across the entire image I only need half of the slices.
    #could update to make portalbe for use cases with only half a rotor (many POV displays use a half)
    for i in range(NUM_SLICES//2):
        # i'm creating bow tie slices here, so i do the same thing twice, just mirrored 180deg on k=1
        for k in range(2):
            # makeSlice does it what it says it does
            slice = makeSlice(i,k)
            slices.append(slice)
            
            #after I make a slice I create two concentric circles to wind up with a sectors
            #these sectors have a span set by the number of slices and a thickness set by the number of LEDs
            for j in range(NUM_SECTORS):
                sector = makeSector(slice, j)
                sectors.append(sector)
                #sectors are created from the outside converging on the center
                #but the leds count from 0 - NUMLEDS from edge to edge
                #if k is zero, the count is normal, but for the mirrored sector (k==1) I need to set the LEDS from NUM_LEDS to the center
                #hence the need to initialize the list
                if(k==1):
                    index=(NUM_SECTORS*2 - 1- j)
                else: 
                    index=j   
                LEDs[index] = getRGB(sector)

        # LEDs[NUM_SECTORS]='0x0'
        #insert a blank pixel into the middle of the strip
        #that pixel doesn't actually move so having any different colors just looks like a flicker
        #3 options: 1)turn it off, 2)pick an accent color or 3)average all of the middle+1 and middle-1 pixels and it to that
        LEDs.insert(NUM_SECTORS,'0x000000')
        #add the slice to the image_data
        image_data.append(LEDs)
        #debug info to keep the terminal updated
        print(i,LEDs)
        #initialize the list. not neccesary but why not.
        LEDs = [0] * (NUM_SECTORS*2)

    ##What actions do you want to take with the data?
    #generate a header file for an arduino
    makeHeaderFile(image_data)
    #save the raw hex values
    # saveRawData(image_data)
    #save the sliced images (useful for debugging)
    # saveSlicedImages(slices,sectors)
    #get a glimpse at what the result will look like
    reconstituteImage(image_data)

#make sure I'm not a module
if __name__ == "__main__":
    main()
