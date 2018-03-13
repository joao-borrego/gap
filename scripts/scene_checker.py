#!/usr/bin/python3

'''
    Copyright (C) 2018 João Borrego

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
    
        http://www.apache.org/licenses/LICENSE-2.0
        
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
'''

# GUI
import tkinter as tk
# Command line args
import sys, getopt
# XML parsing
import xml.etree.ElementTree as ET

# Title
APP_TITLE = "Scene dataset verification tool"
# Usage
USAGE = 'options: -i <image output directory>\n' +                      \
        '         -d <dataset output directory>\n' +                    \
        '         -s <number of scenes> [optional, default=1]\n' +      \
        '         -n <index of the first scene> [optional, default=0]\n'
# Image file extension
EXT_IMG = '.png'
# Dataset file extension
EXT_DATA = '.xml'

def parseArgs(argv):
    '''
    Parses command-line options.
    '''

    # Parameters
    data_dir = ''
    img_dir = ''
    first = 0
    scenes = 1

    usage = 'usage:   ' + argv[0] + ' [options]\n' + USAGE

    try:
        opts, args = getopt.getopt(argv[1:],
            "hi:d:s:n:",["img_dir=","data_dir=","scenes=","first="])
    except getopt.GetoptError:
        print (usage)
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print (usage)
            sys.exit()
        elif opt in ("-i", "--img_dir"):
            img_dir = arg
        elif opt in ("-d", "--data_dir"):
            data_dir = arg
        elif opt in ("-s", "--scenes"):
            scenes = int(arg)
        elif opt in ("-n", "--first"):
            first = int(arg)
    
    if not data_dir or not img_dir:
        print (usage)
        sys.exit(2)

    print ('Image directory      ', img_dir)
    print ('Dataset directory    ', data_dir)
    print ('Number of scenes     ', scenes)
    print ('Index of first scene ', first)

    return [data_dir, img_dir, scenes, first]

class ImageViewer(tk.Frame):
    '''
    Displays scene image with bounding box and additional info overlays.
    '''

    def __init__(self, parent, data_dir, img_dir, scenes, first):
        '''
        Initializes attributes
        '''
        tk.Frame.__init__(self, parent)
        self.parent = parent
        self.data_dir = data_dir
        self.img_dir = img_dir
        self.scenes = scenes
        self.first = first
        self.cur = first

        self.setupWidgets()

    def setupWidgets(self):
        '''
        Sets up GUI elements and event callback funtions
        '''

        # Configure window
        self.parent.maxsize(
            width = self.parent.winfo_screenwidth(),
            height= self.parent.winfo_screenheight())
        self.parent.title(APP_TITLE)

        # Create canvas in main window
        self.canvas = tk.Canvas(self.parent,
            width =self.parent.winfo_screenwidth(),
            height=self.parent.winfo_screenheight())
        self.canvas.pack()

        # Bind spacebar to keypress event
        self.parent.bind("<space>", self.onKeypress)

        # Update image
        self.onUpdate()

    
    def onUpdate(self):
        '''
        Shows image and object bounding box overlays.
        '''

        # Filenames
        img_file = self.img_dir + '/' + str(int(self.cur / 100)) + \
            '00/' + str(self.cur) + EXT_IMG
        data_file = self.data_dir + '/' + str(self.cur) + EXT_DATA
        print(img_file, data_file)

        # Clean canvas
        self.canvas.delete("all")

        # Update scene image
        image = tk.PhotoImage(file=img_file)
        self.canvas.image = image
        self.canvas.create_image(0, 0, image=image, anchor=tk.NW)

        # Open XML dataset file
        tree = ET.parse(data_file)
        annotation = tree.getroot()

        # Draw label with cur / total scene indicator
        label = str(self.cur) + "/" + str(self.scenes)
        self.canvas.create_text(40, 40, text=label, fill="black",
            font=('arial', '18'), anchor=tk.NW)
        # Draw help label
        label = "Press SPACEBAR to move on to next image."
        self.canvas.create_text(40, 80, text=label, fill="black",
            font=('arial', '18'), anchor=tk.NW)

        # Draw bounding boxes
        for obj in annotation.findall('object'):
            name = obj.find('name').text
            color = "red"
            if   (name == "sphere"):   color = "blue"
            elif (name == "cylinder"): color = "#32cd32" # green
            
            for bnd_box in obj.findall('bndbox'):
                x_min = int(bnd_box.find('xmin').text)
                y_min = int(bnd_box.find('ymin').text)
                x_max = int(bnd_box.find('xmax').text)
                y_max = int(bnd_box.find('ymax').text)
                self.canvas.create_rectangle(x_min, y_min, x_max, y_max,
                    outline=color, width=2)
                self.canvas.create_text(x_min, y_min-25, text=name, fill=color,
                    font=('arial', '16'), anchor=tk.NW)

    def onKeypress(self, event):
        '''
        Updates counter and calls update function.
        '''
        self.cur = self.cur + 1
        if (self.cur >= self.scenes): sys.exit(0) 
        self.onUpdate()

def main(argv):
    '''
    Simple tool to open image and overlay bounding box data to check
    whether or not a scene dataset is correct.
    '''
    
    # Obtain command-line arguments
    [data_dir, img_dir, scenes, first] = parseArgs(argv)

    # Open root window
    root = tk.Tk()
    # Create app object
    app = ImageViewer(root, data_dir, img_dir, scenes, first)
    # Main loop
    root.mainloop()

if __name__ == "__main__":
   main(sys.argv)
