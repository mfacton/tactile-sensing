import math
import random
import time

import cv2
import numpy as np

animate = True

color_white = (255, 255, 255)
color_black = (0, 0, 0)

directions = [
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
]

width = 8
height = 8
maze = [[[] for y in range(height)] for x in range(width)]
origin = (0, 0)

iterations = width*height*10

for x in range(1, width):
    for y in range(height):
        maze[x][y].append((-1, 0))

for y in range(1, height):
    maze[0][y].append((0, -1))

grid_size = 100
canvas_width = width * grid_size
canvas_height = height * grid_size

canvas = np.full((canvas_height, canvas_width, 3), color_black, dtype=np.uint8)

def show_maze():
    cv2.imshow("maze", canvas)
    cv2.waitKey(1)

def clear_canvas():
    global canvas
    canvas = np.full((canvas_height, canvas_width, 3), color_black, dtype=np.uint8)

def draw_circle(pos, color):
    cv2.circle(canvas, (int((pos[0]+0.5)*grid_size), canvas_height - int((pos[1]+0.5)*grid_size)), 2, color, 2)

def draw_grid():
    for x in range(width):
        for y in range(height):
            draw_circle((x, y), color_white)

def draw_iterations(current, extra):
    cv2.putText(canvas, f"{current}/{iterations} + {extra}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_white, 2)

# currently only works at 90 degree angles
def draw_arrow(pos1, pos2, color):
    x1 = (pos1[0]+0.5)*grid_size
    y1 = canvas_height - (pos1[1]+0.5)*grid_size
    x2 = (pos2[0]+0.5)*grid_size
    y2 = canvas_height - (pos2[1]+0.5)*grid_size

    dx = x2 - x1
    dy = y2 - y1

    cv2.line(canvas, (int(x1 + dx/10), int(y1 + dy/10)), (int(x2 - dx/10), int(y2 - dy/10)), color, 2)
    cv2.line(canvas, (int(x2 - dx/5 + (dy/5-dy/10)), int(y2 - dy/5 + (dx/5-dx/10))), (int(x2 - dx/10), int(y2 - dy/10)), color, 2)
    cv2.line(canvas, (int(x2 - dx/5 - (dy/5-dy/10)), int(y2 - dy/5 - (dx/5-dx/10))), (int(x2 - dx/10), int(y2 - dy/10)), color, 2)

def draw_maze():
    clear_canvas()
    draw_iterations(steps, extra)
    draw_grid()
    draw_circle(new_origin, (0, 0, 255))

    for x in range(width):
        for y in range(height):
            for dir in maze[x][y]:
                draw_arrow((x, y), (x+dir[0], y+dir[1]), color_white)

def origin_on_edge():
    if origin[0] == 0 or origin[0] == width-1:
        return True
    if origin[1] == 0 or origin[1] == height-1:
        return True
    return False

def next_ccw_edege():
    if origin[0] == 0:
        if origin[1] == 0:
            return (1, 0)
        else:
            return (0, -1)
    if origin[1] == 0:
        if origin[0] == width-1:
            return (0, 1)
        else:
            return (1, 0)
    if origin[0] == width-1:
        if origin[1] == height-1:
            return (-1, 0)
        else:
            return (0, 1)
    if origin[1] == height-1:
        if origin[0] == 0:
            return (0, -1)
        else:
            return (-1, 0)

show_maze()
time.sleep(0.01)

steps = 0
extra = 0

# random generation
while steps <= iterations or not origin_on_edge():
    new_dir = None
    new_origin = None
    while True:
        new_dir = directions[random.randint(0, 3)]
        new_origin = (origin[0]+new_dir[0], origin[1]+new_dir[1])
        if new_origin[0] >= 0 and new_origin[0] < width:
            if new_origin[1] >= 0 and new_origin[1] < height:
                break
        
    if new_dir not in maze[origin[0]][origin[1]]:
        maze[origin[0]][origin[1]].append(new_dir)
    
    maze[new_origin[0]][new_origin[1]] = []

    if animate:
        draw_maze()
        show_maze()
        time.sleep(0.01)

    origin = new_origin
    steps += 1

if animate:
    time.sleep(0.5)

# circle perimeter
for s in range(2 * (width+height-2) - 1):
    new_dir = next_ccw_edege()
    new_origin = (origin[0]+new_dir[0], origin[1]+new_dir[1])
        
    if new_dir not in maze[origin[0]][origin[1]]:
        maze[origin[0]][origin[1]].append(new_dir)
    
    maze[new_origin[0]][new_origin[1]] = []

    if animate:
        draw_maze()
        show_maze()
        time.sleep(0.1)

    origin = new_origin
    extra += 1

draw_maze()
show_maze()

while True:
    time.sleep(1)
