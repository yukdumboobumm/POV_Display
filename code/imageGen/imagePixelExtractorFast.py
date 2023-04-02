from PIL import Image, ImageDraw
import numpy as np
import math
from pprint import pprint
import os
import cv2

##Global defs

# Open the image file
# IMAGE_FILE = "Texas_flag_map.png"
# IMAGE_FILE = "paidyn.png"
# IMAGE_FILE = "clockTemplate.png"
# IMAGE_FILE = "hollow_mech_1.png"
# IMAGE_FILE = "hollow_knight_simple.png"
IMAGE_FILE = "elsa.png"
IM = Image.open(os.path.join(".", "images", IMAGE_FILE))
# IM = IM.quantize(colors=256)
# IM_PALETTE = IM.palette
IM_ARRAY = np.array(IM)

NUM_LEDS_G1 = 30
NUM_LEDS_G2 = 28
NUM_LEDS = NUM_LEDS_G1 + NUM_LEDS_G2
NUM_SLICES = 60


ROTOR_WIDTH = 203.5 #in mm
STRIP_GAP = 6.8 #in mm
LED_WIDTH = 5 #in mm
LED_GAP = 2 #in mm


LOW_RAM = True

# Get the width and height of the image
# WIDTH, HEIGHT = IM.size

#Get the width and height of the image array
HEIGHT,WIDTH,CHANNELS = IM_ARRAY.shape
# CHANNELS = IM_ARRAY.shape[2]
# WIDTH = IM_ARRAY.shape[1]
MIN_DIM = min(WIDTH, HEIGHT)


PIX_PER_MM = MIN_DIM / ROTOR_WIDTH
# LED_WIDTH_PIXELS = round(LED_WIDTH * PIX_PER_MM)
# LED_GAP_PIXELS = round(LED_GAP * PIX_PER_MM)
# STRIP_GAP_PIXELS = round(STRIP_GAP * PIX_PER_MM)
LED_WIDTH_PIXELS = LED_WIDTH * PIX_PER_MM
LED_GAP_PIXELS = LED_GAP * PIX_PER_MM
LED_OFFSET = (LED_WIDTH_PIXELS + LED_GAP_PIXELS) / 2
STRIP_GAP_PIXELS = STRIP_GAP * PIX_PER_MM
STRIP_HEIGHT = LED_WIDTH_PIXELS + STRIP_GAP_PIXELS + LED_WIDTH_PIXELS

# Calculate the center of the image
CENTER_X = WIDTH / 2 - 1
CENTER_Y = HEIGHT / 2 - 1
CENTER = (CENTER_X, CENTER_Y)

ROTOR_STARTROW = CENTER_Y - STRIP_HEIGHT / 2
ROTOR_ENDROW = ROTOR_STARTROW + STRIP_HEIGHT
UPPER_STRIP_ENDROW = ROTOR_STARTROW + LED_WIDTH_PIXELS
LOWER_STRIP_STARTROW = ROTOR_ENDROW - LED_WIDTH_PIXELS


# Set the number of slices
NUM_SECTORS = (NUM_LEDS // 2)
SECTOR_THICKNESS = (MIN_DIM / 2 ) // NUM_SECTORS 

# Calculate the angle between each slice
SLICE_ANGLE = 360.0 / NUM_SLICES

# Create 2D grid of coordinates and compute distance and angle to center
X_VALS, Y_VALS = np.meshgrid(np.arange(HEIGHT), np.arange(WIDTH))
CENTER_DISTANCE = np.sqrt((X_VALS - (WIDTH-1)/2)**2 + (Y_VALS - (HEIGHT-1)/2)**2)
THETA_XAXIS =np.arctan2((HEIGHT-1)/2 - Y_VALS, X_VALS - (WIDTH-1)/2)
THETA_XAXIS[THETA_XAXIS < 0] += 2*np.pi
THETA_XAXIS = np.rad2deg(THETA_XAXIS)

###FUNCTION DEFS

def rotateImageArray(imageArray, angle):
    rotationMatrix = cv2.getRotationMatrix2D(CENTER, angle, 1.0)
    rotatedArray = cv2.warpAffine(imageArray, rotationMatrix, (WIDTH, HEIGHT), flags=cv2.INTER_LINEAR)
    return rotatedArray

def rotateImage(imageArray, angle):
    rotationMatrix = cv2.getRotationMatrix2D(CENTER, angle, 1.0)
    rotatedArray = cv2.warpAffine(imageArray, rotationMatrix, (WIDTH, HEIGHT), flags=cv2.INTER_LINEAR)
    rotatedImage = Image.fromarray((rotatedArray).astype(np.uint8))
    # outputImage.show()
    return rotatedImage

#get a slice from an image, will work on the arguments here...a bit lazy
def makeSlice(i,k):
    slice = Image.new('RGBA', (WIDTH, HEIGHT), (0, 0, 0, 0))
    # Create a mask for the slice
    mask = Image.new('L', (WIDTH, HEIGHT), 0)
    draw = ImageDraw.Draw(mask)

    sliceMirror = k * 180
    startAngle = ((i * SLICE_ANGLE) - SLICE_ANGLE / 2) + sliceMirror
    # endAngle = ((i + 1) * SLICE_ANGLE) + sliceMirror
    endAngle = startAngle + SLICE_ANGLE
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
def getRGB(ledPixel) :
    #each element in the array represents a pixel
    #and has 3-dimensions (height, width, (RGBA))

    #create a masked 3D array of T/F based on the alpha channel (preserves any black in the image)
    alphaMask = ledPixel[...,3]!=0
    #create a new 1D array containing only the values with non-zero alpha
    sector_channels = ledPixel[alphaMask]

    #check that there are actually some values to average
    if (sector_channels.size):
        # Get the RGB values from the array
        #[x,y,(r,g,b)] create a vector of only the desired channel 
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
        for i in range(len(imData)):
            f.write("LED_SLICE_" + str(i))
            if (i!=len(imData)-1):
                f.write(', ')
            if ((i+1)%10==0):
                f.write('\n')
        f.write("\n};\n\n")
        f.write("#endif //"+outFileName.upper().replace('.','_')+'\n')

def makeHeaderFile_speed(imData):
    outFileName = IMAGE_FILE[:-4]+"_out.h"
    with open(os.path.join(".", "output", outFileName), 'w') as f:
        f.write("#include \"Arduino.h\"\n\n")
        f.write("#ifndef "+ outFileName.upper().replace('.','_')+'\n')
        f.write("#define "+ outFileName.upper().replace('.','_')+'\n\n')
        f.write("#define NUM_LEDS " + str(NUM_LEDS) + "\n")
        f.write("#define SLICES " + str(NUM_SLICES) + "\n\n")

        f.write("const uint32_t FRAME[(SLICES / 2) * NUM_LEDS] PROGMEM = {\n")
        for imageSlice in imData:
            for led in imageSlice:
                f.write(led + ',')
            f.write('\n') 
        f.write("\n};\n\n")
        f.write("#endif //"+outFileName.upper().replace('.','_')+'\n')

def saveSlicedImages(sliceList, sectorList):
# Save each slice as a separate image file
    if LOW_RAM:
        return
    for i in range(len(sliceList)):
        sliceName = os.path.join(".", "output", "slicedImages", "slice_{}.png".format(i))
        sliceList[i].save(sliceName)
        for j in range(NUM_SECTORS):
            sectorName = os.path.join(".", "output", "slicedImages", "slice_{}-sector_{}.png".format(i,j))
            sectorList[(i*NUM_SECTORS)+j].save(sectorName)
    # pprint(LED_image, WIDTH=800, indent=4)

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

def recomposeImage(led_data):
    reImageArray = np.zeros((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8)
    for i in range(NUM_SLICES // 2):
        reImageArray = rotateImageArray(reImageArray, i*SLICE_ANGLE)
        for j in range(NUM_LEDS // 2):
            ledR = int(led_data[i][j], 16) >> 16 & 0xff
            ledG = int(led_data[i][j], 16) >> 8 & 0xff
            ledB = int(led_data[i][j], 16) & 0xff
            ledAlpha = 255
            pixelData = [ledR, ledG, ledB, ledAlpha]
            ledEnd = WIDTH - ((LED_WIDTH_PIXELS + LED_GAP_PIXELS) * j)
            ledStart = ledEnd - LED_WIDTH_PIXELS
            pixelVector = [[pixelData] * (ledEnd - ledStart)] * (UPPER_STRIP_ENDROW - ROTOR_STARTROW)
            reImageArray[ROTOR_STARTROW:UPPER_STRIP_ENDROW,ledStart:ledEnd,:] = pixelVector
        for k in range(NUM_LEDS // 2):
            ledR = int(led_data[i][k+NUM_LEDS//2], 16) >> 16 & 0xff
            ledG = int(led_data[i][k+NUM_LEDS//2], 16) >> 8 & 0xff
            ledB = int(led_data[i][k+NUM_LEDS//2], 16) & 0xff
            ledAlpha = 255
            pixelData = [ledR, ledG, ledB, ledAlpha]
            ledStart = (LED_WIDTH_PIXELS + LED_GAP_PIXELS) * k
            ledEnd = ledStart + LED_WIDTH_PIXELS
            pixelVector = [[pixelData] * (ledEnd - ledStart)] * (ROTOR_ENDROW - LOWER_STRIP_STARTROW)
            reImageArray[LOWER_STRIP_STARTROW:ROTOR_ENDROW, ledStart:ledEnd, :] = pixelVector
    slice = Image.fromarray((reImageArray).astype(np.uint8))
    slice.show()

def getMask(x0, y0):
    midPointX = x0 + LED_WIDTH_PIXELS / 2
    midPointY = y0 + LED_WIDTH_PIXELS / 2
    midPointRadius = math.sqrt((midPointX - (WIDTH - 1) / 2)**2 + (midPointY - (HEIGHT - 1) / 2)**2)
    midPointTheta = np.arctan2(((HEIGHT-1) / 2) - midPointY, midPointX - (WIDTH-1) / 2) 
    midPointTheta = np.degrees(midPointTheta)
    alpha = (midPointTheta - SLICE_ANGLE / 2) % 360
    beta = (midPointTheta + SLICE_ANGLE / 2) % 360
    innerR = midPointRadius - LED_WIDTH_PIXELS / 2
    outerR = midPointRadius + LED_WIDTH_PIXELS /2
    if alpha>beta:
        pieSlice = np.logical_or(THETA_XAXIS >=alpha, THETA_XAXIS<=beta)
    else:
        pieSlice = np.logical_and(THETA_XAXIS >=alpha, THETA_XAXIS<=beta)
    mask = (CENTER_DISTANCE >= innerR) & (CENTER_DISTANCE <= outerR) & pieSlice
    return mask

def getPixelData(led):
    ledR = int(led, 16) >> 16 & 0xff
    ledG = int(led, 16) >> 8 & 0xff
    ledB = int(led, 16) & 0xff
    ledAlpha = 255
    return [ledR, ledG, ledB, ledAlpha]


###MAIN

#main loop where all the magic happens
def main():
    #pre initialize an LED strip, create list for the raw data
    LEDs = [0] * (NUM_LEDS)
    LED_image = []
    finalImageArrayUpper = np.zeros((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8)
    finalImageArrayLower = np.zeros((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8)
    
    # loop to create image slices
    #since I have a full rotor that extends across the entire image I only need half of the slices.
    #could update to make portable for use cases with only half a rotor (many POV displays use a half)
    for i in range(NUM_SLICES//2):
        # sliceArrayUpper = np.zeros((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8)
        sliceArrayUpperRC = np.zeros((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8)
        # sliceArrayLower = np.zeros((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8)
        sliceArrayLowerRC = np.zeros((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8)
        rotatedImageArray = rotateImageArray(IM_ARRAY, i*SLICE_ANGLE)

        for j in range(NUM_LEDS_G1):
            if j < NUM_LEDS_G1 // 2:
                ledEnd = WIDTH - (LED_WIDTH_PIXELS + LED_GAP_PIXELS) * j
                ledStart = ledEnd - LED_WIDTH_PIXELS
                startRow = ROTOR_STARTROW
                idx = j
            else:
                ledStart = (LED_WIDTH_PIXELS + LED_GAP_PIXELS) * (j - NUM_LEDS_G1 // 2)
                startRow = LOWER_STRIP_STARTROW
                idx = NUM_LEDS_G1//2 - 1 + j
            ledMask = getMask(ledStart, startRow)
            ledPixels = rotatedImageArray[ledMask, :]
            # sliceArrayUpper[ledMask, :] = ledPixels
            LEDs[idx] = getRGB(ledPixels)
            sliceArrayUpperRC[ledMask, :] = getPixelData(LEDs[idx])

        sliceArrayUpperRC = rotateImageArray(sliceArrayUpperRC, i * -SLICE_ANGLE)
        finalImageArrayUpper += sliceArrayUpperRC
        # imagePreview = Image.fromarray((finalImageArrayUpper).astype(np.uint8))
        # imagePreview.show()
        
        for k in range(NUM_LEDS_G2, 0, -1):
            if k > NUM_LEDS_G2 // 2:
                ledStart = LED_OFFSET + (LED_WIDTH_PIXELS + LED_GAP_PIXELS) * (NUM_LEDS_G2 - k)
                startRow = ROTOR_STARTROW
                idx = k
            else:
                ledEnd = (WIDTH - LED_OFFSET) - ((LED_WIDTH_PIXELS + LED_GAP_PIXELS) * (NUM_LEDS_G2 // 2 - k))
                ledStart = ledEnd - LED_WIDTH_PIXELS
                startRow = LOWER_STRIP_STARTROW
                idx = NUM_LEDS//2 + NUM_LEDS_G2//2 + k
            
            ledMask = getMask(ledStart, startRow)

            ledPixels = rotatedImageArray[ledMask, :]
            LEDs[idx] = getRGB(ledPixels)
            sliceArrayLowerRC[ledMask, :] = getPixelData(LEDs[idx])
        
        sliceArrayLowerRC = rotateImageArray(sliceArrayLowerRC, i * -SLICE_ANGLE)
        finalImageArrayLower += sliceArrayLowerRC
        # imagePreview = Image.fromarray((finalImageArrayLower).astype(np.uint8))
        # imagePreview.show()

        LED_image.append(LEDs)
        #debug info to keep the terminal updated
        print(i,i*-SLICE_ANGLE,LEDs)
        #initialize the list. not neccesary but why not.
        LEDs = [0] * (NUM_LEDS)
    print("finished loop")

    imageU = Image.fromarray((finalImageArrayUpper).astype(np.uint8))
    imageL = Image.fromarray((finalImageArrayLower).astype(np.uint8))
    imageU.show()
    imageL.show()
    blendedImage = Image.alpha_composite(imageU, imageL)
    blendedImage.show()


    ##What actions do you want to take with the data?
    #generate a header file for an arduino
    # makeHeaderFile_speed(LED_image)
    makeHeaderFile(LED_image)
    #save the raw hex values
    # saveRawData(LED_image)
    #save the sliced images (useful for debugging)
    # saveSlicedImages(slices,sectors)
    #get a glimpse at what the result will look like
    # reconstituteImage(LED_image)
    # recomposeImage(LED_image)

#make sure I'm not a module
if __name__ == "__main__":
    main()
