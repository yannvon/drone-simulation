import time
import random
import drawSample
import math
import _tkinter
import sys
import loader
import sys
import cPickle
import os.path
import Tkinter as tk
import sys
import Tkinter as tk
from sets import Set
import tkFont

from PIL import ImageFilter
import TileServer
import geoclass

debug = 0  # debug 1 or 2 means using a very simplified setup
verbose=0  # print info (can be set on command line)
versionNumber = 1.0
zoomLevel = 18
loadStateFile = 'classifier.state'  # default file for classifier data

documentation = \
"""
  This program is a stub for your COMP 417 robotics assignment.
"""




##########################################################################################
#########  Do non-stardard imports and print helpful diagnostics if necessary ############
#########  Look  for "real code" below to see where the real code goes        ############
##########################################################################################


missing = []
fix = ""

try: 
    import scipy
    from scipy import signal
except ImportError: 
    missing.append( " scipy" )
    fix = fix +  \
        """
        On Ubuntu linux you can try: sudo apt-get install python-numpy python-scipy python-matplotlib 
        On OS X you can try:  
              sudo easy_install pip
              sudo pip install  scipy
        """
try: 
    import matplotlib
    import matplotlib.pyplot as plt
except ImportError: 
    missing.append( " matplotlib" )
    fix = fix +  \
        """
        On Ubuntu linux you can try: sudo apt-get install python-matplotlib
        On OS X you can try:  
              sudo easy_install pip
              sudo pip install matplotlib
        """

try: 
    import numpy as np
except ImportError: 
     missing.append( " numpy " )
     fix = fix + \
        """
          sudo easy_install pip
          sudo pip install numpy
        """
try: 
    from sklearn.decomposition import PCA
    from sklearn.svm import SVC
except ImportError: 
     missing.append( " scikit-learn " )
     fix = fix + \
        """
          sudo easy_install pip
          sudo pip install scikit-learn
        """
try: 
    from PIL import Image
    from PIL import ImageTk
    from PIL import ImageDraw
except ImportError: 
     missing.append( " PIL (more recently known as pillow) " )
     fix = fix + \
        """
          sudo easy_install pip
          sudo pip install pillow
        """

if missing:
     print "*"*60
     print "Cannot run due to missing required libraries of modules."
     print "Missing modules: "
     for i in missing: print "    ",i
     print "*"*60
     print fix
     sys.exit(1)

#FIXME
version = "Yann's drone v%.1f  $HGdate: Fri, 2 Dec 2017 09:38:37 -0500 $ $Revision: f330eb3280c9 Local rev 2 $" % versionNumber

print version
print " ".join(sys.argv)
##########################################################################################
#########     Parse command-line arguments   #############################################
##########################################################################################
while len(sys.argv)>1:
    if len(sys.argv)>1 and sys.argv[1]=="-v":
        verbose = verbose+1
        del sys.argv[1]
    elif len(sys.argv)>1 and sys.argv[1]=="-load":
        if len(sys.argv)>2 and not sys.argv[2].startswith("-"):
            loadStateFile = sys.argv[2]
            del sys.argv[2]
        else:
            loadStateFile = 'classifier.state'
        del sys.argv[1]
    elif len(sys.argv)>1 and sys.argv[1] in ["-h", "-help", "--help"]: # help
        print documentation
        sys.argv[1] = "-forceusagemesasge"
    else:
        print "Unknown argument:",sys.argv[1]
        print "Usage: python ",sys.argv[0]," [-h (help)][-v]    [-f TRAININGDATADIR] [-t TESTDIR] [-load [STATEFILE]]"
        sys.exit(1)


##########################################################################################
#########  "real code" is here, at last!                                      ############
##########################################################################################
# my position
tx,ty = 0.5,0.5 # This is the translation to use to move the drone
oldp = [tx,ty]  # Last point visited
max_path_length = 50000

fill = "white"
image_storage = [ ] # list of image objects to avoid memory being disposed of

def autodraw():
    """ Automatic draw. """
    draw_objects()
    tkwindow.canvas.after(100, autodraw)

def draw_objects():
    """ Draw target balls or stuff on the screen. """
    global tx, ty, maxdx, maxdy, unmoved
    global oldp
    global objectId
    global ts # tileServer
    global actual_pX, actual_pY
    global fill
    global scalex, scaley  # scale factor between out picture and the tileServer
    #FIXME added variables for different Methods
    global theta
    global ax, ay, vx, vy
    global backtrack
    global old_tile_x, old_tile_y
    global totalTileChanges
    global allTiles
    global total_path_length
    global goRight
    global wall_found
    global tmp_tiles, visited_tiles
    global first_tile_of_circuit

    #tkwindow.canvas.move( objectId, int(tx-MYRADIUS)-oldp[0],int(ty-MYRADIUS)-oldp[1] )
    if unmoved:
        # initialize on first time we get here
        unmoved=0
        tx,ty = 5,0 #FIXME for algo 5 set to speed
        theta = math.pi / 2
        ax, ay, vx, vy = 0,0,0,0
        backtrack = 0
        old_tile_x, old_tile_y = -1,-1
        allTiles = Set()
        totalTileChanges = 0
        total_path_length = 0
        goRight = 0
        wall_found = 0
        tmp_tiles = set()
        visited_tiles = set()
        first_tile_of_circuit = (-1,-1)
    else:
        # draw the line showing the path
        tkwindow.polyline([oldp,[oldp[0]+tx,oldp[1]+ty]], style=5, tags=["path"]  )
        tkwindow.canvas.move( objectId, tx,ty )

    # update the drone position
    oldp = [oldp[0]+tx,oldp[1]+ty]

    # map drone location back to lat, lon
    # This transforms pixels to WSG84 mapping, to lat,lon
    lat,lon = ts.imagePixelsToLL( actual_pX, actual_pY, zoomLevel,  oldp[0]/(256/scalex), oldp[1]/(256/scaley) )

    # get the image tile for our position, using the lat long we just recovered
    im, foox, fooy, fname = ts.tiles_as_image_from_corr(lat, lon, zoomLevel, 1, 1, 0, 0)

    # Use the classifier here on the image "im"
    # FIXME
    class_index, class_str = geoclass.classifyOne(pca, clf, np.asarray(im, dtype=np.float32).flatten(), classnames)

    # print text to show the classification of the tile
    # original array ['arable' 'desert' 'urban' 'water']
    #text = classnames[class_index]
    text = ("A", "D", "U", "W")[class_index]
    color = ("white", "yellow", "red", "blue")[class_index]

    tkwindow.canvas.create_text(256/scalex*int(oldp[0]/(256/scalex))+10, 256/scalex*int(oldp[1]/(256/scalex))+10, fill=color, text=text)

    # This is the drone, let's move it around
    tkwindow.canvas.itemconfig(objectId, tag='userball', fill=fill)
    tkwindow.canvas.drawn = objectId

    #  Take the tile and shrink it to go in the right place
    im = im.resize((int(im.size[0]/scalex),int(im.size[1]/scaley)))
    im.save("/tmp/locationtile.gif")
    photo = tk.PhotoImage(file="/tmp/locationtile.gif" )

    tkwindow.image = tkwindow.canvas.create_image( 256/scalex*int(oldp[0]/(256/scalex)), 256/scalex*int(oldp[1]/(256/scalex)), anchor=tk.NW, image=photo, tags=["tile"] )
    image_storage.append( photo ) # need to save to avoid garbage collection

    # This arrenges the stuff being shown
    tkwindow.canvas.lift( objectId )
    tkwindow.canvas.tag_lower( "tile" )
    tkwindow.canvas.tag_lower( "background" )
    tkwindow.canvas.pack()


    # Code to move the drone can go here
    # Move a small amount by changing tx,ty

    # Part1: Check if we are on a different tile
    new_tile_x = 256/scalex*int(oldp[0]/(256/scalex))
    new_tile_y = 256/scalex*int(oldp[1]/(256/scalex))
    tile_change = new_tile_x != old_tile_x or new_tile_y != old_tile_y
    if tile_change:
        allTiles.add((new_tile_x, new_tile_y))
        totalTileChanges += 1

    #Method1: Brownian Algorithm
    '''tx = random.uniform(8,-8)
    ty = random.uniform(8,-8)

    ax = min(max(ax + random.uniform(5,-5), -5),5)
    ay = min(max(ay + random.uniform(5,-5), -5),5)
    vx = min(max(vx + ax, -5), 5)
    vy = min(max(vy + ay, -5), 5)
    tx = vx
    ty = vy
    '''


    #Method2: Simple Boustrophedon sweeping
    '''
    tx = -1
    ty = 25*math.cos(theta)
    theta = theta + 0.1;
    '''

    #Method3: Stupid Lawn Mover Copy
    #Idea: pick a direction and go straight, once you detect urban area, or the border of the map
    #      pick a new direction and go straight again.
    '''
    cruising_speed = 5
    backtrack_duration = 5;

    #Change cruising direction in case we hit border or city
    city = class_index == 2
    out_of_bounds = oldp[0] >= myImageSize or oldp[0] <= 0 or oldp[1] >= myImageSize or oldp[1] <= 0
    takeoff = theta == 0
    end_off_backtrack = backtrack == 1

    if backtrack > 1:
       backtrack -= 1
    elif takeoff or end_off_backtrack:
        backtrack = 0
        theta = random.uniform(0, 2 * math.pi)
    elif city or out_of_bounds:
        backtrack = backtrack_duration
        theta += math.pi  # go into opposite direction

    tx = cruising_speed * math.sin(theta)
    ty = cruising_speed * math.cos(theta)

    '''
    #Method4: Same as above without backtracking
    '''
    cruising_speed = 5

    # Change cruising direction in case we hit border or city
    city = class_index == 2
    out_of_bounds = oldp[0] >= myImageSize or oldp[0] <= 0 or oldp[1] >= myImageSize or oldp[1] <= 0
    takeoff = theta == 0

    # Here we suppose small movements (i.e it is not possible to enter a tile from two directions.
    # This is not quite true as this situation arrises occasionaly however it doesn't affect the outcome greatly

    if takeoff:
        theta = random.uniform(0, 2 * math.pi)

    elif city or out_of_bounds:

        #came from left
        if old_tile_x < new_tile_x or oldp[0] >= myImageSize:
            theta = random.uniform(math.pi, 2.0 * math.pi)
            print('from left')
        # came from right
        elif old_tile_x > new_tile_x or oldp[0] <= 0 :
            print('from right')
            theta = random.uniform(0, math.pi)
        # came from top
        elif old_tile_y < new_tile_y or oldp[1] >= myImageSize:
            theta = random.uniform(math.pi / 2, 3.0/2.0 *math.pi)
            print('from top')
        # came from bottom
        elif old_tile_y > new_tile_y or oldp[1] <= 0:
            theta = random.uniform(-math.pi / 2, math.pi / 2)
            print('from bot')

    if(out_of_bounds and not tile_change):
        print('woah no tile change to out of bounds')

    if(out_of_bounds):
        print('out of bounds')

    tx = cruising_speed * math.sin(theta)
    ty = cruising_speed * math.cos(theta)
    
    '''
    # Method 5: deterministic "hand on wall " lawnmover " algorithm (spiral shape)
    # Note: This algorithm corresponds to what I would personally do when moving the lawn,
    #       the other option would be a boustrophedon back and forth coverage.
    #       The running time can be exponentially worse than optimal,
    #       for example when two small patches are connected by a long one tile wide path.
    #       Nonetheless since in most cases it avoids passing twice at the same place it should
    #       not be too bad in practice.

    city = class_index == 2
    out_of_bounds = oldp[0] >= 1020 or oldp[0] <= 0 or oldp[1] >= 1020 or oldp[1] <= 0

    #Backtrack one step if out_of bounds
    print(new_tile_x, new_tile_y)
    if (city or out_of_bounds or (new_tile_x, new_tile_y) in visited_tiles) and not backtrack:
        if not wall_found:
            first_tile_of_circuit = (old_tile_x, old_tile_y)#fixme
            print("FIRST TILE OF CIRCUIT", first_tile_of_circuit)

        tx = -tx
        ty = -ty
        goRight = 40
        wall_found = 1
        backtrack = 3


    #Try to go right all the time
    if wall_found and (backtrack == 1 or goRight < 0) :
        old_ty = ty
        ty = tx
        tx = -old_ty
        goRight = 51.2
        backtrack = 0
    else:
        goRight -= math.sqrt(tx**2 + ty**2)
        backtrack = max(0, backtrack - 1)

    if not backtrack and tile_change and (new_tile_x, new_tile_y) == first_tile_of_circuit:
        visited_tiles = visited_tiles.union(tmp_tiles)
        visited_tiles.remove((old_tile_x, old_tile_y))
        tmp_tiles = set()
        #first_tile_of_circuit = (old_tile_x, old_tile_y) FIXME
        print("back to the start!")
        tx = -tx
        ty = -ty
        backtrack = 3

    if wall_found and tile_change:
        tmp_tiles.add((new_tile_x, new_tile_y))


    # COMMON PART TO ALL ALGORITHMS: Limit path length to a certain distance for comparison and output stats
    old_tile_x = new_tile_x
    old_tile_y = new_tile_y
    if total_path_length > max_path_length:
        tx, ty = 0,0
        font = tkFont.Font(size='20')
        tkwindow.canvas.create_text(200,
                                    100, fill='white',
                                    font = font,
                                    text="Simulation over.\nUnique tiles visited: %d\nTotal tiles visited: %d\nRatio: %f"
                                         % (len(allTiles), totalTileChanges, len(allTiles) / float(totalTileChanges)))
    else:
        total_path_length += math.sqrt(tx ** 2 + ty ** 2)


# MAIN CODE. NO REAL NEED TO CHANGE THIS

ts = TileServer.TileServer()

# Top-left corner of region we can see

lat, lon = 45.44203, -73.602995    # verdun
#lat, lon = 45.554925, -73.701590  # Boulevard Cartier O, Laval, H7N


# Size of region we can see, measure in 256-goepixel tiles.  Geopixel tiles are what
# Google maps, bing, etc use to represent the earth.  They make up the atlas.
#
tilesX = 20
tilesY = 20
tilesOffsetX = 0
tilesOffsetY = 0

# Get tiles to cover the whole map (do not really need these at this point, be we cache everything 
# at the biginning this way, and can draw it all.
# using 1,1 instead of tilesX, tilesY to see just the top left image as a check
#
#actual, actual_pX, actual_pY, fname = ts.tiles_as_image_from_corr(lat, lon, zoomLevel, 1, 1, tilesOffsetX, tilesOffsetY)
actual, actual_pX, actual_pY, fname = ts.tiles_as_image_from_corr(lat, lon, zoomLevel, tilesX, tilesY, tilesOffsetX, tilesOffsetY)


# Rather than draw the real data, we can use a white map to see what is unexplored.
bigpic = Image.new("RGB", (256*tilesX, 256*tilesY), "white")
bigpic.paste(actual, (0,0))  # paste the actual map over the pic.

# How to draw a rectangle.
# You should delete or comment out the next 3 lines.    #FIXME
#draw = ImageDraw.Draw(bigpic)
#xt,yt = 0,0
#draw.rectangle(((xt*256-1, yt*256-1), (xt*256+256+1, yt*256+256+1)), fill="red")

# put in image

# Size of our on-screen drawing is arbitrarily small
myImageSize = 1024
scalex = bigpic.size[0]/myImageSize  # scale factor between our picture and the tileServer
scaley = bigpic.size[1]/myImageSize  # scale factor between our picture and the tileServer
im = bigpic.resize((myImageSize,myImageSize))
im = im.filter(ImageFilter.BLUR)
im = im.filter(ImageFilter.BLUR)

im.save("mytemp.gif") # save the image as a GIF and re-load it does to fragile nature of Tk.PhotoImage
tkwindow  = drawSample.SelectRect(xmin=0,ymin=0,xmax=1024 ,ymax=1024, nrects=0, keepcontrol=0 )#, rescale=800/1800.)
root = tkwindow.root
root.title("Drone simulation")

# Full background image
photo = tk.PhotoImage(file="mytemp.gif")
tkwindow.imageid = tkwindow.canvas.create_image( 0, 0, anchor=tk.NW, image=photo, tags=["background"] )
image_storage.append( photo )
tkwindow.canvas.pack()

tkwindow.canvas.pack(side = "bottom", fill = "both",expand="yes")


MYRADIUS = 7
MARK="mark"

# Place our simulated drone on the map
sx,sy=600,640 # over the river
#sx,sy = 220,280 # over the canal in Verdun, mixed environment
oldp = [sx,sy]
objectId = tkwindow.canvas.create_oval(int(sx-MYRADIUS),int(sy-MYRADIUS), int(sx+MYRADIUS),int(sy+MYRADIUS),tag=MARK)
unmoved =  1

# initialize the classifier
# We can use it later using these global variables.
#
pca, clf, classnames = geoclass.loadState( loadStateFile, 1.0)

# launch the drawing thread
autodraw()

#Start the GUI
root.mainloop()
