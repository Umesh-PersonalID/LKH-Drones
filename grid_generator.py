import math, argparse, time

UPDATE_REF = False # update reference point based on the first waypoint
EARTH_R = 6371000.0 # radius of earth for calculations
INITIAL_LONGITUDE = -78.77810 # this may or may not be used within graph script
INITIAL_LATITUDE = 42.99849   # this may or may not be used within graph script
OFFSET_X = 0.0
OFFSET_Y = 0.0
GRID_COLS = 1
GRID_ROWS = 1
THRESHOLD = 0.01     #this is used in adjusting area and obstacles, later it will be multiplied by RESOLUTION

GRAPH_FORMAT='png'   #png, pdf, ps, eps, svg.
GRAPH_DPI=300
GRAPH_FIG_WIDTH = 25
GRAPH_FIG_HEIGHT= 10
GRAPH_IMAGE_SCALE = 10#7.725
GRAPH_MARKER_SIZE = 12
GRAPH_LINE_WIDTH = 3
TOTAL_WP = 0 #We will collect WP from Input File

global DUMP_FILE # file to store results
global OUT_FILE
global VERBOSE
global VERBOSE_STR #
class DUMP: # level of verbose
    ERROR=0
    WARNING=1
    INFO=2
    DEBUG=3
VERBOSE_STR=["ERROR", "WARNING", "INFO", "DEBUG"]

class MAV_CMD:
    WP=16 # waypoints
    RTL=20 # agent assignment
    LAND=21 # landing 
    TAKEOFF=22
MAV_CMD_ACCEPTABLE=[MAV_CMD.WP, MAV_CMD.RTL, MAV_CMD.LAND, MAV_CMD.TAKEOFF]

# setting up the parser
parser = argparse.ArgumentParser(description="Get travel distance our of waypoint file. As of now it only takes on mission file, consiss only of waypoints")
# input mission file
parser.add_argument("input_file", help="The input waypoints file which includes grid information")
parser.add_argument("-r", "--resolution", help="The resolution that grid is intended to made at. This overrides resolution in file.")
parser.add_argument("-g", "--graph", help="Draws the output grid file. requires matplotlib and pillow (from PIL import Image rather than import Image)", action="store_true")
parser.add_argument("-v", "--verbose", help="level of verbosity (1 WARNINGS only, 2 INFO, 3 DEBUG level")

args = parser.parse_args()

VERBOSE = int(args.verbose) if args.verbose else DUMP.INFO

# Set resolution from CLI argument, or default to -1 for fallback in input file
RESOLUTION = float(args.resolution) if args.resolution else -1.0


DUMP_FILE=open("grid_log.txt","a")
print(DUMP_FILE)
###################################
# calculates latitude for a given grid row (left bottom corner) based on resolution
def calc_lat(y):
    global INITIAL_LATITUDE
    global INITIAL_LONGITUDE
    global RESOLUTION
    global OFFSET_X
    global OFFSET_Y

    # return INITIAL_LATITUDE + math.degrees((OFFSET_Y + y * RESOLUTION) / EARTH_R)
    return y/1e5
###################################
# calculates longitude for a given grid column (left bottom corner) based on resolution
def calc_lon(x):
    global INITIAL_LATITUDE
    global INITIAL_LONGITUDE
    global RESOLUTION
    global OFFSET_X
    global OFFSET_Y

    # return INITIAL_LONGITUDE + math.degrees((OFFSET_X + x * RESOLUTION) / EARTH_R / math.cos(math.radians(INITIAL_LATITUDE)))
    return x/1e5
###################################
# generates one waypoint with the cooridneate of the center of a cell in grid
def generate_cell(i, j):
    global INITIAL_LATITUDE
    global INITIAL_LONGITUDE
    global RESOLUTION
    global OFFSET_X
    global OFFSET_Y
    global OUT_FILE

    x_coord = i/1e5 # calc_lat(GRID_ROWS - 1 - i + 0.5)
    y_coord = j/1e5 #calc_lon(j + 0.5)

    cell_lats.append(x_coord)
    cell_lons.append(y_coord)
    print(x_coord, GRID_ROWS, i)
    OUT_FILE.write(gen_wp(MAV_CMD.WP, x_coord, y_coord))
###################################
# generates an obstacle of line block, grid position and size is given
def generate_obstacle(i, j, size):
    global INITIAL_LATITUDE
    global INITIAL_LONGITUDE
    global RESOLUTION
    global OFFSET_X
    global OFFSET_Y
    global OUT_FILE
    OUT_FILE.write(gen_wp(MAV_CMD.TAKEOFF,  calc_lat(GRID_ROWS - 1 - i + THRESHOLD),        calc_lon(j + THRESHOLD)))   # one cell row/coloumn is invalidated
    OUT_FILE.write(gen_wp(MAV_CMD.WP,       calc_lat(GRID_ROWS - 1 - i + THRESHOLD),        calc_lon(j + size - THRESHOLD))) # one cell row/coloumn is invalidated
    OUT_FILE.write(gen_wp(MAV_CMD.WP,       calc_lat(GRID_ROWS - i - THRESHOLD),    calc_lon(j + size - THRESHOLD)))
    OUT_FILE.write(gen_wp(MAV_CMD.LAND,     calc_lat(GRID_ROWS - i - THRESHOLD),    calc_lon(j + THRESHOLD)))
###################################
def gen_wp(cmd, latitude, longitude):
    gen_wp.index += 1   # this is function attribute, something like static variable
# index     current     frame       command     param1 ... param7   auto_continue
    wp = "{0}\t0\t3\t{1}\t0\t0\t0\t0\t{2:.16f}\t{3:.16f}\t5\t1\n".format(gen_wp.index, cmd, latitude, longitude)
    return wp
gen_wp.index = -1
###################################
def dump(verb, myStr):
    global DUMP_FILE
    global VERBOSE

    if (verb<=VERBOSE):
        print(myStr)

    now = time.time()
    myStr = "{0:8s}\t{1:14s}\t{2:s}".format(VERBOSE_STR[verb],time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime(now)),myStr)
    DUMP_FILE.write(myStr+"\n")
    return
####################################
# reading mission waypoint from input file
def read_input(input_file):
    global INITIAL_LATITUDE
    global INITIAL_LONGITUDE
    global RESOLUTION
    global OFFSET_X
    global OFFSET_Y
    global GRID_ROWS
    global GRID_COLS
    global TOTAL_WP

    # readin the input file and validate it
    in_file=open(input_file, "r")

    # skipping the header comments
    line = in_file.readline()
    while (not line.strip()) or line.strip().startswith('#'):
        line = in_file.readline()

    INITIAL_LATITUDE = float(line)
    INITIAL_LONGITUDE = float(in_file.readline())

    line = in_file.readline()
    if (RESOLUTION < 0):
        RESOLUTION = float(line)

    OFFSET_X = float(in_file.readline())
    OFFSET_Y = float(in_file.readline())
    TOTAL_WP = int(in_file.readline())
    GRID_COLS = 5
    GRID_ROWS = 5
    dump(DUMP.DEBUG, "read inputs: {0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}".format(INITIAL_LATITUDE, INITIAL_LONGITUDE, RESOLUTION, OFFSET_X, OFFSET_Y, GRID_COLS, GRID_ROWS))

    input_grid = []
    for i in range(TOTAL_WP):
        line = in_file.readline().strip()
        print(line)
        x, y = line.split(', ')
        input_grid.append([x,y])
        
        

    return input_grid
####################################
# drawing a path and graph it
# using projection in https://en.wikipedia.org/wiki/Equirectangular_projection
def draw_grid():
    dump(DUMP.DEBUG, "******* drawing path *******")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    LON_RATIO = math.cos(math.radians(INITIAL_LATITUDE))
    
    X=[]
    Y=[]
    dump(DUMP.DEBUG, "ID \t    X(m) \t    Y(m)")
    for i in range(len(area_lats)):
        Y.append(EARTH_R * math.radians(area_lats[i]-INITIAL_LATITUDE))
        X.append(EARTH_R * math.radians(area_lons[i]-INITIAL_LONGITUDE) * LON_RATIO)
        dump(DUMP.DEBUG, "{0:3d}\t{1:8.2f}\t{2:8.2f}".format(i, X[-1], Y[-1]))

    # from meters to pixel, based on the background map size
    X = [x * GRAPH_IMAGE_SCALE for x in X]
    Y = [y * GRAPH_IMAGE_SCALE for y in Y]

    min_x = int(min(X) - RESOLUTION * GRAPH_IMAGE_SCALE)
    max_x = int(max(X) + RESOLUTION * GRAPH_IMAGE_SCALE)
    min_y = int(min(Y) - RESOLUTION * GRAPH_IMAGE_SCALE)
    max_y = int(max(Y) + RESOLUTION * GRAPH_IMAGE_SCALE)

    img=plt.imread("map.png")
    IMG_LEN = len(img)
    img2 = img[IMG_LEN-max_y:IMG_LEN-min_y, min_x:max_x]
    ax.imshow(img2, extent=[min_x, max_x, min_y, max_y])
    ax.plot(X, Y, '-or', markersize=GRAPH_MARKER_SIZE, linewidth=GRAPH_LINE_WIDTH)

    dump(DUMP.DEBUG, "ID \t    X(m) \t    Y(m)")
    for i in range(len(cell_lats)):
        y = GRAPH_IMAGE_SCALE * EARTH_R * math.radians(cell_lats[i]-INITIAL_LATITUDE)
        x = GRAPH_IMAGE_SCALE * EARTH_R * math.radians(cell_lons[i]-INITIAL_LONGITUDE) * LON_RATIO
        dump(DUMP.DEBUG, "{0:3d}\t{1:8.2f}\t{2:8.2f}".format(i, x, y))
        ax.plot([x], [y], '-ob', markersize=GRAPH_MARKER_SIZE, linewidth=GRAPH_LINE_WIDTH)


    #plt.gca().set_aspect('equal', adjustable='box')
    #plt.show()
    plt.savefig("grid.%s"%(GRAPH_FORMAT), dpi=GRAPH_DPI, format=GRAPH_FORMAT)
    return
####################################
###### start of the main script
####################################

input_grid = read_input(args.input_file)
##### input file generator 
#OUT_FILE = open("/home/farshad/apmplanner2/missions/"+args.input_file+"_area.txt","w")
#OUT_FILE.write("QGC WPL 110\n")

# main coverage rectangle
# OUT_FILE.write(gen_wp(MAV_CMD.WP,       INITIAL_LATITUDE,           INITIAL_LONGITUDE))

# OUT_FILE.write(gen_wp(MAV_CMD.TAKEOFF,  calc_lat(-1 - THRESHOLD),            calc_lon(-1 - THRESHOLD)))   # one cell row/coloumn is invalidated
# OUT_FILE.write(gen_wp(MAV_CMD.WP,       calc_lat(-1),                        calc_lon(GRID_COLS + 1))) # one cell row/coloumn is invalidated
# OUT_FILE.write(gen_wp(MAV_CMD.WP,       calc_lat(GRID_ROWS+1 + THRESHOLD),   calc_lon(GRID_COLS+1 + THRESHOLD)))
# OUT_FILE.write(gen_wp(MAV_CMD.LAND,     calc_lat(GRID_ROWS+1),               calc_lon(-1)))

# obstacles
# for i in range(GRID_ROWS):
#     j = 0
#     while j < GRID_COLS:
#         if (input_grid[i][j]==True):
#             k = 0   # k will be the size of block line
#             while (j+k<GRID_COLS and input_grid[i][j+k]==True):
#                 k+=1
#             generate_obstacle(i,j,k)           
#             j += k - 1   #moving j forward, so it can continue
#         j+=1
# agent position
# OUT_FILE.write(gen_wp(MAV_CMD.RTL,      INITIAL_LATITUDE,           INITIAL_LONGITUDE))
# exit(0)
# calling the graph
#if args.graph:
#    draw_path(wp_latitude, wp_longitude)

##### grid points generator for hardcode input

OUT_FILE = open("grid_output.txt","w")
OUT_FILE.write("QGC WPL 110\n")

area_lats = [calc_lat(GRID_ROWS), calc_lat(0), calc_lat(0), calc_lat(GRID_ROWS), calc_lat(GRID_ROWS)]
area_lons = [calc_lon(0), calc_lon(0), calc_lon(GRID_COLS), calc_lon(GRID_COLS), calc_lon(0)]
print(area_lats,area_lons)
cell_lats = []
cell_lons = []

for i in range(len(area_lats)):
    OUT_FILE.write(gen_wp(MAV_CMD.LAND,     area_lats[i],   area_lons[i]))

for i in range(TOTAL_WP):
    x,y = float(input_grid[i][0])//1,float(input_grid[i][1])//1
    generate_cell(x,y)
    # elif z == 1:
    #     generate_obstacle(x,y,1)
    # if (i%2==0):
    #     j = 0
    # else:
    #     j = GRID_COLS - 1
    # while (j>=0 and j<GRID_COLS):
    #     if (input_grid[i][j]==False):
    #         generate_cell(i,j)

    #     if (i%2==0):
    #         j += 1
    #     else:
    #         j -= 1

# OUT_FILE.write(gen_wp(MAV_CMD.RTL,      INITIAL_LATITUDE,           INITIAL_LONGITUDE))

if args.graph:
    draw_grid()