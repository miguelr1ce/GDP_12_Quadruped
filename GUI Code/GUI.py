# -*- coding: utf-8 -*-
"""
Created on Mon Mar  6 12:32:36 2023

@author: K
"""

# F / B -> Foward / Backward
# L / R -> Left / Right


# Step 1: Define Variables / Inputs
# =============================================================================
# Libraries
import math
import tkinter as tk
# from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Inputs from Joy Sticks
A = 1 # Forward / Backward Multiplier
B = 1 # Left / Right Multiplier
C = 0 # Clockwise / Counter Clockwise Rotation Multiplier
Mode = 3 # Points of contact etc...

# Variables
L = 0.4 # Up / Down Multiplier
Max_Lift = 125 # Maximum height of foot step in mm
Lift = L * Max_Lift ## Set it to 50 for now
No_Iteration = 8 # Reference of the total iterations
No_Legs = 4
Step = 60 # Maximum translation distance of a step in mm

# Offsets to motors, taking center of robot body as origin
Offset_x = 122 # Left / Right
Offset_y = 165 # Front / Back
Offset_z = 0 # Up / Down

# =============================================================================
# Step 1: Define Functions
# =============================================================================

# Function that convert input into correct format of variable 
# before input into function xyz
def transform(component):
    return [[[item] for item in component[0]], component[1]]

# Function that rearrange a literation, from input to x y z coordinates format    
def xyz(component_list):
    final_list = []
    component = component_list[0]
    axis = component_list[1]
    for i in range(len(component)): 
        zeros = [0] * 3 # Create a 1x3 list for the inputting of x y z values
        for j in component[i]:
            count = 0
            for k in axis: # Find out which axis is the value influencing
                if k == 'x':
                    zeros[0] = j[count]
                    count += 1
                if k == 'y':
                    zeros[1] = j[count]
                    count += 1
                if k == 'z':
                    zeros[2] = j[count]
                    count += 1
        final_list.append(zeros)
    return final_list
 
# Function that move first element in a list to the end of the list   
def rearrange(index, component_list):
    local = component_list.copy()
    for i in range(index):
        last = local.pop()
        local.insert(0, last)
    return local

# Function that compile everything into the large list
def combine(No_Iteration, No_Legs, components, mode): # Mode defines point of contact
    
    end_list = [[[] for _ in range(No_Iteration)] for _ in range(len(components))]
    for i in range(len(components) - 1):
        iterations = xyz(components[i])
        for k in range(No_Iteration):
            for j in range(No_Legs):
                if mode == 1: # Crawl
                    Relative_offsets = No_Legs
                    rearranged = rearrange(int(j * (No_Iteration / Relative_offsets)), iterations)
                    end_list[i][k].append(rearranged[k])
                if mode == 2: # Trot
                    Relative_offsets = No_Legs / 2
                    if j < 2:
                        end_list[i][k].append(iterations[k])
                    else:
                        rearranged = rearrange(int((No_Iteration / Relative_offsets)), iterations)
                        end_list[i][k].append(rearranged[k])
    
    r1_iter = xyz(components[3][0]) # Leg 1
    r2_iter = xyz(components[3][1]) # Leg 2
    r3_iter = xyz(components[3][2]) # Leg 3
    r4_iter = xyz(components[3][3]) # Leg 4
    if mode == 1: # Crawl
        for i in range(No_Iteration):
            Relative_offsets = No_Legs
            end_list[3][i].append(r1_iter[i])
            end_list[3][i].append(rearrange(int(No_Iteration / Relative_offsets), r2_iter)[i])
            end_list[3][i].append(rearrange(int(No_Iteration / Relative_offsets * 2), r3_iter)[i])
            end_list[3][i].append(rearrange(int(No_Iteration / Relative_offsets * 3), r4_iter)[i])
    if mode == 2: # Trot
        for i in range(No_Iteration):
            Relative_offsets = No_Legs / 2
            end_list[3][i].append(r1_iter[i])
            end_list[3][i].append(r2_iter[i])
            end_list[3][i].append(rearrange(int(No_Iteration / Relative_offsets), r3_iter)[i])
            end_list[3][i].append(rearrange(int(No_Iteration / Relative_offsets), r4_iter)[i])
    return end_list

# Function that introduce multiplier for testing purposes
def multiply(component_list, multiplier):
    M_list = []
    M_list.append([x * multiplier for x in component_list[0]])
    M_list.append(component_list[1])
    return M_list

def replace_brackets(input_list):
    str_list = str(input_list)
    str_list = str_list.replace('[', '{').replace(']', '}')
    return str_list

# =============================================================================
# Step X: Define Gait Coordinates, For F / B, L / R and Rotation

# Example: Local coordinates of the rectangle movement of triangle

Angle = 20
r = math.sqrt(Offset_x ** 2 + Offset_y ** 2)
theta_origin = math.atan(Offset_y / Offset_x)

    # Global Coordinates calculation for rotation
def position(theta, r1, r2):
    y = round(r1 * math.sin(theta), 2)
    x = round(r2 * math.cos(theta), 2)
    return [x, y]
  
    # Change the global coordinates into local coordinates
def subtract(r, offset_x, offset_y):
    if r[0] >= 0:
        rx = round(r[0] - offset_x, 2)
    elif r[0] < 0:
        rx = round(r[0] + offset_x, 2)
    if r[1] >= 0:
        ry = round(r[1] - offset_y, 2)
    elif r[1] < 0:
        ry = round(r[1] + offset_y, 2)
    return [rx, ry]
    
    # Function that create rotation coorinates for each leg
def create_rotation(No_iterations, theta_o, angle, swing, stance, r1, r2):
    variables = []
    theta_outlet = theta_o - (math.radians(angle) / 2) # theta_origin is in radian
    if swing + stance == No_iterations:
        for i in range(No_iterations):
            var_name = f"var{i}"
            if i == 0:
                
                theta = theta_outlet + math.radians(angle)
                exec(f"{var_name} = subtract(position(theta, r1, r2), Offset_x, Offset_y) ")
            elif i < swing:
                theta = theta_outlet + math.radians(angle / swing * (swing - i)) 
                exec(f"{var_name} = subtract(position(theta, r1, r2), Offset_x, Offset_y) ")
            elif i == swing:
                theta = theta_outlet
                exec(f"{var_name} = subtract(position(theta, r1, r2), Offset_x, Offset_y) ")
            elif i > swing:
                theta = theta_outlet + math.radians(angle / stance * (i - swing)) 
                exec(f"{var_name} = subtract(position(theta, r1, r2), Offset_x, Offset_y) ")
            variables.append(eval(var_name))
        return variables
    else:
        print("Error: sum of swing and stance is not equal to number of iterations")
        

# Rotation coordinates for each leg in complete iteration

# =============================================================================

## GUI?


# =============================================================================
# Step X: Output Text File
# =============================================================================

    
# =============================================================================
# Step X: GUI
# =============================================================================

row1 = 100
distance = 50
offset = 50
component_label = 30

# Create first window for user to decide the number of iterations
class InputWindow(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Input Window")
        
        tk.Label(self, text="Number of Iterations:", anchor = "w").grid(row=0, column=0)
        self.num_sets_entry = tk.Entry(self, width = 5)
        self.num_sets_entry.grid(row=0, column=1)
        self.num_sets_entry.insert(0, 8)
        
        tk.Label(self, text="Number of legs:", anchor="w").grid(row=1, column=0)
        self.legs_var = tk.StringVar(self)
        self.legs_var.set("4")
        self.legs_dropdown = tk.OptionMenu(self, self.legs_var, "2", "4")
        self.legs_dropdown.config(bg="white")
        self.legs_dropdown.grid(row=1, column=1)
                
        tk.Button(self, text="OK", command=self.open_window).grid(row=0, column=2)

    def open_window(self):
        if float(self.num_sets_entry.get()) % float(self.legs_var.get()) != 0:
            tk.messagebox.showwarning("Warning", "The number of iterations should be a multiple of the number of legs.")
        else:    
            num_sets = int(self.num_sets_entry.get())
            num_legs = int(self.legs_var.get())
            self.destroy()
            self.window2 = InputSetsWindow(num_sets, num_legs)

        
class InputSetsWindow(tk.Tk):
    def __init__(self, num_sets, num_legs):
        super().__init__()
        # Info of the window
        self.title("Input Sets Window")
        self.geometry("1200x750+-10+0")
        
        # Variables
        self.entries = []
        self.num_sets = num_sets
        self.num_legs = num_legs
        
        # Create Gait Mode Drop Down
        self.gait_mode = tk.StringVar(self)
        self.gait_mode.set("Crawl")
        self.gait_dropdown = tk.OptionMenu(self, self.gait_mode, "Crawl", "Trot")
        self.gait_dropdown.place(x = 270, y = 25)
        
        # Create entries for gait components
        self.create_entry(num_sets, ['z'], "Marching", 1) # Marching
        self.create_entry(num_sets, ['y'], "Forward / Backward", 2) # Forward / Backward
        self.create_entry(num_sets, ['x'], "Left / Right", 3) # Left / Right
        self.create_rotation(num_sets, "Degree of Rotation")

        # Create checkbox for 3D plots
        self.var_list = [tk.BooleanVar(value=True) for i in range(3)]
        self.checkbutton1 = tk.Checkbutton(self, text="Forward / Backward", variable=self.var_list[0])
        self.checkbutton2 = tk.Checkbutton(self, text="Left / Right", variable=self.var_list[1])
        self.checkbutton3 = tk.Checkbutton(self, text="Rotation", variable=self.var_list[2])
        self.checkbutton1.place(x = 270, y = 100)
        self.checkbutton2.place(x = 270, y = 150)
        self.checkbutton3.place(x = 270, y = 200)
        
        # Create phase diagram
        self.canvas_table = tk.Canvas(self, width=520, height=250)
        self.canvas_table.place(x = 450, y = 450)
        self.create_table_labels(self.num_sets, self.num_legs)
        self.canvas_legend = tk.Canvas(self, width = 60, height = 45) 
        self.canvas_legend.place(x = 970, y = 450)
        self.canvas_legend.create_rectangle(2, 2, 58, 20, fill = "deepskyblue")
        self.canvas_legend.create_rectangle(2, 25, 58, 43, fill = "lightgrey")
        self.canvas_legend.create_text(30, 11, text = "SWING", fill = "white", font = ('TkDefaultFont', 9, 'bold'))
        self.canvas_legend.create_text(30, 34, text = "STANCE", font = ('TkDefaultFont', 9, 'bold'))

        
        tk.Button(self, text="Plot", command=self.plot_coordinates).place(x = 270, y = 300)
        tk.Button(self, text="Generate", command=self.generate).place(x = 320, y = 300)

        self.default_values(self.num_legs)
        self.plot_coordinates()

    
    def create_entry(self, num_sets, influenced_axis, component, order):
        displacement = (order - 1) * (num_sets * 20 + 50)
        tk.Label(self, text = component, font = 15).place(x = 50, y = offset - component_label + displacement)
        for i in range(num_sets):
            tk.Label(self, text="Set {}:".format(i+1)).place(x = 50, y = i * 20 + offset + displacement)
            x_entry = tk.Entry(self, width = 5)  
            if 'x' in influenced_axis:
                # x_entry.insert(0, random.randint(-20, 20))
                x_entry.insert(0, 0)
            else:
                x_entry.insert(0, 0)
                x_entry.config(state='disabled')
            x_entry.place(x = row1, y = i * 20 + offset + displacement)
            
            y_entry = tk.Entry(self, width = 5)
            if 'y' in influenced_axis:
                # y_entry.insert(0, random.randint(-20, 20))
                y_entry.insert(0, 0)
            else:
                y_entry.insert(0, 0)
                y_entry.config(state='disabled')
            y_entry.place(x = row1 + distance, y = i * 20 + offset + displacement)
            
            z_entry = tk.Entry(self, width = 5)
            if 'z' in influenced_axis:
                # z_entry.insert(0, random.randint(0, 30))
                z_entry.insert(0, 0)
            else:
                z_entry.insert(0, 0)
                z_entry.config(state='disabled')
            z_entry.place(x = row1 + distance * 2, y = i * 20 + offset + displacement)
            self.entries.append((x_entry, y_entry, z_entry))

        
    def create_rotation(self, num_sets, component):
        displacement = 3 * (num_sets * 20 + 50)
        tk.Label(self, text = component, font = 15).place(x = 50, y = offset - component_label + displacement)
        tk.Label(self, text=component).place(x = 50, y = offset + displacement)
        rot_entry = tk.Entry(self, width = 8)
        rot_entry.insert(0, 0)
        rot_entry.place(x = row1 + 65, y = offset + displacement)
        print(len(self.entries))
        self.entries.append(rot_entry)
        
        
    def plot_coordinates(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        x_data = []
        y_data = []
        z_data = []
        
        for entry in self.entries[:-1]:
            x = float(entry[0].get())
            y = float(entry[1].get())
            z = float(entry[2].get())
            x_data.append(x)
            y_data.append(y)
            z_data.append(z)
        
        x_data1 = [0] * (self.num_sets + 1)
        y_data1 = y_data[self.num_sets: self.num_sets * 2]
        z_data1 = z_data[0: self.num_sets]
        y_data1.append(y_data1[0])
        z_data1.append(z_data1[0])
        
        if self.var_list[0].get() == True:
            ax.plot(x_data1, y_data1, z_data1, color = "blue")
            ax.scatter(x_data1, y_data1, z_data1, marker='o', s=100, color = "blue")
            canvas = FigureCanvasTkAgg(fig, master=self)
            canvas.draw()

        for entry in self.entries[self.num_sets * 2: self.num_sets * 3]:
            x = float(entry[0].get())
            x_data.append(x)
        for entry in self.entries[0: self.num_sets]:
            z = float(entry[2].get())
            z_data.append(z)
        y_data2 = [0] * (self.num_sets + 1)
        x_data2 = x_data[self.num_sets * 2: self.num_sets * 3]
        z_data2 = z_data[0: self.num_sets]
        x_data2.append(x_data2[0])
        z_data2.append(z_data2[0])
        if self.var_list[1].get() == True:
            ax.plot(x_data2, y_data2, z_data2, color = "darkorange")
            ax.scatter(x_data2, y_data2, z_data2, marker='o', s=100, color = "darkorange")
            canvas = FigureCanvasTkAgg(fig, master=self)
            canvas.draw()
        ax.set_xlabel('L/R')
        ax.set_ylabel('F/B')
        ax.set_zlabel('Height')
              
        code, self.swing_state = self.Swing_Stance()
        degrees = float(self.entries[-1].get())
        rotation_data = xyz(transform([create_rotation(self.num_sets, theta_origin, degrees, self.swing_state, self.num_sets - self.swing_state, r, r), ['x', 'y']]))
        
        index_r = 1
        re_code = code.copy()
        print(re_code)
        while re_code[0] != 1 and re_code[-1] != 0:
            re_code = rearrange(index_r, code)
            index_r += 1
            print(re_code)
            print("yes")
        
        x_data3 = []
        y_data3 = []
        z_data3 = z_data[0: self.num_sets]
        
        for data in rotation_data:
            x_data3.append(data[0])
            y_data3.append(data[1])
            x_data.append(data[0])
            y_data.append(data[1])
        
        x_data3.append(x_data3[0])
        y_data3.append(y_data3[0])
        z_data3.append(z_data3[0])
        if self.var_list[2].get() == True:
            ax.plot(x_data3, y_data3, z_data3, color = "green")
            ax.scatter(x_data3, y_data3, z_data3, marker='o', s=100, color = "green")
            
        
        x = x_data
        y = y_data
        z = z_data
        max_range = max(max(x)-min(x), max(y)-min(y), max(z)-min(z))
        x_center = (max(x) + min(x)) / 2
        y_center = (max(y) + min(y)) / 2
        z_center = (max(z) + min(z)) / 2
        ax.set_xlim([x_center - max_range/2, x_center + max_range/2])
        ax.set_ylim([y_center - max_range/2, y_center + max_range/2])
        ax.set_zlim([0, z_center + max_range/2])
        ax.set_title("Gait Trajectories in 3D")
        
        canvas = FigureCanvasTkAgg(fig, master=self)
        canvas.draw()
        canvas.get_tk_widget().place(x = 450, y = offset - component_label)
        canvas.get_tk_widget().config(width=500, height=400)
        
        self.reset_table(self.num_legs)
        
    def reset_table(self, num_legs):
        self.canvas_table.delete("all")
        code, swing = self.Swing_Stance()
        if self.gait_mode.get() == "Crawl":
            for j in range(self.num_legs):
                self.create_big_rectangle(self.num_sets, rearrange(int(self.num_sets / self.num_legs) * j, code), j)
        elif self.gait_mode.get() == "Trot":
            for j in range(self.num_legs):
                if j < 2:
                    self.create_big_rectangle(self.num_sets, code, j)
                else:
                    self.create_big_rectangle(self.num_sets, rearrange(int(self.num_sets / 2), code), j)
        arrow_length = 500
        number_segment = 4
        segment_size = arrow_length / number_segment
        self.canvas_table.create_line(0, (50 + 30 * num_legs), arrow_length, (50 + 30 * num_legs), width = 4, arrow=tk.LAST)
        for k in range(number_segment - 1):
            self.canvas_table.create_line(segment_size * (k + 1), (50 + 30 * num_legs - 10), segment_size * (k + 1), (50 + 30 * num_legs + 10), width = 4)
    
    def create_table_labels(self, num_rectangles, num_legs):
        for i in range(num_rectangles):
            tk.Label(self, text = "Iter" + str(i + 1), font = ("Helvetica", 10)).place(x = 454 + (i * 500 / num_rectangles) + (500 / num_rectangles / 2), y = 437, anchor = 'center')
        for j in range(num_legs):
            tk.Label(self, text = "Leg " + str(j + 1), font = ("Helvetica", 10)).place(x = 410, y = j * (40) + 454)    
            
        
    def create_rectangle(self, x, y, width, height, color):
        self.canvas_table.create_rectangle(x, y, x + width, y + height, fill=color)

    def create_big_rectangle(self, num_rectangles, color_code, y):
        rectangle_width = 500 / num_rectangles
        rectangle_height = 30
        offset = 40
        
        for i in range(num_rectangles):
            if color_code[i] == 0:
                self.create_rectangle(i * rectangle_width + 2, 2 + y * offset, rectangle_width, rectangle_height - 2, "lightgrey")
            else:
                self.create_rectangle(i * rectangle_width + 2, 2 + y * offset, rectangle_width, rectangle_height - 2, "deepskyblue")
                
    def Swing_Stance(self):
        M = []
        for i in range(self.num_sets):
            M.append(float(self.entries[i][2].get()))
        
        list_ = M.copy()
        list_.append(M[0])
        code = []
        swing = 0
        
        for i in range(len(M)):
            if list_[i] != 0 or list_[i + 1] != 0:
                code.append(1)
                swing += 1
            else:
                code.append(0)
        
        return code, swing
    
    def generate(self):
        M_list = []
        FB_list = []
        LR_list = []
        
        for i in range(len(self.entries)):
            if i < self.num_sets:
                Mx = float(self.entries[i][0].get())
                My = float(self.entries[i][1].get())
                Mz = float(self.entries[i][2].get())
                M_list.append([Mz])
            elif i < self.num_sets * 2:
                FBx = float(self.entries[i][0].get())
                FBy = float(self.entries[i][1].get())
                FBz = float(self.entries[i][2].get())
                FB_list.append([FBy])
            elif i < self.num_sets * 3:
                LRx = float(self.entries[i][0].get())
                LRy = float(self.entries[i][1].get())
                LRz = float(self.entries[i][2].get())
                LR_list.append([LRx])
            
            
        Marching = transform([M_list, ['z']])
        F_B = transform([FB_list, ['y']])
        L_R = transform([LR_list, ['x']])
        angle = self.entries[-1].get()
        swing = self.swing_state
        stance = self.num_sets - swing
        r_leg1 = create_rotation(self.num_sets, theta_origin, float(angle), swing, stance, r, r) # Front Right Leg
        r_leg2 = create_rotation(self.num_sets, theta_origin, float(angle), swing, stance, -r, -r) # Back Left Leg
        r_leg3 = create_rotation(self.num_sets, theta_origin, float(angle), swing, stance, -r, r) # Front Left Leg
        r_leg4 = create_rotation(self.num_sets, theta_origin, float(angle), swing, stance, r, -r) # Back Right Leg
        Rot = [transform([r_leg1, ['x', 'y']]), transform([r_leg2, ['x', 'y']]), transform([r_leg3, ['x', 'y']]), transform([r_leg4, ['x', 'y']])]
        
        Gait = 0
        if self.gait_mode.get() == "Crawl":
            Gait = 1
        if self.gait_mode.get() == "Trot":
            Gait = 2       
        Compiled = combine(self.num_sets, 4, [Marching, F_B, L_R, Rot], Gait)
        
        for c in Compiled:
            print(replace_brackets(c), "\n")
            
        def confirm():
            file_name = entry.get() + ".txt"
            with open(file_name, "w") as f:
                f.write(f"struct {entry.get()}{{\n")
                f.write("    double March[itr][legn][coor]=" + str(replace_brackets(Compiled [0])) + ";\n")
                f.write("    double FB[itr][legn][coor]=" + str(replace_brackets(Compiled [1])) + ";\n")
                f.write("    double LR[itr][legn][coor]=" + str(replace_brackets(Compiled [2])) + ";\n")
                f.write("    double RT[itr][legn][coor]=" + str(replace_brackets(Compiled [3])) + ";\n")
                f.write("};\n")
            top.destroy()
    
        top = tk.Toplevel()
        top.title("Enter file name")
        label = tk.Label(top, text="Enter file name:")
        label.pack(side="left")
        entry = tk.Entry(top)
        entry.pack(side="left")
        button = tk.Button(top, text="Confirm", command=confirm)
        button.pack(side="left")
        
        
    def default_values(self, num_legs):
        if self.num_sets == 8:
            # M = [0, 50, 0, 0, 0, 0, 0, 0]
            # FB = [-60, 0, 60, 40, 20, 0, -20, -40]
            # LR = [-30, 0, 30, 20, 10, 0, -10, -20]
            
            M = [0, 50, 50, 50, 0, 0, 0, 0]
            FB = [-60, -30, 0, 30, 60, 30, 0, -30]
            LR = [-30, -15, 0, 15, 30, 15, 0, -15]
    
            for i in range(len(self.entries)):
                if i < 8:
                    self.entries[i][2].delete(0, tk.END)
                    self.entries[i][2].insert(0, M[i])
                elif i < 16:
                    self.entries[i][1].delete(0, tk.END)
                    self.entries[i][1].insert(0, FB[i-8])
                elif i < 24:
                    self.entries[i][0].delete(0, tk.END)
                    self.entries[i][0].insert(0, LR[i-16])
            
            # Rotation entry
            self.entries[24].delete(0, tk.END)
            self.entries[24].insert(0, 20)
                    
                    
        swing = int(self.num_sets / self.num_legs)
        stance = self.num_sets - swing
        code = []
        
        for i in range(self.num_sets):
            if i < swing:
                code.append(1)
            else:
                code.append(0)
        
        for j in range(self.num_legs):
            self.create_big_rectangle(self.num_sets, rearrange(swing * j, code), j)
        
        
        arrow_length = 500
        number_segment = 4
        segment_size = arrow_length / number_segment
        self.canvas_table.create_line(0, (50 + 30 * num_legs), arrow_length, (50 + 30 * num_legs), width = 4, arrow=tk.LAST)
        for k in range(number_segment - 1):
            self.canvas_table.create_line(segment_size * (k + 1), (50 + 30 * num_legs - 10), segment_size * (k + 1), (50 + 30 * num_legs + 10), width = 4)
            percentage = int((k + 1) * (100 / number_segment))
            tk.Label(self, text = str(percentage) + "%", font = ("Helvetica", 12, "bold")).place(x = 450 + segment_size * (k + 1), y = 500 + 20 + 30 * num_legs, anchor = 'center')
        
        tk.Label(self, text = "0%", font = ("Helvetica", 12, "bold")).place(x = 450, y = 500 + 20 + 30 * num_legs, anchor = 'center')
        tk.Label(self, text = "100%", font = ("Helvetica", 12, "bold")).place(x = 450 + arrow_length, y = 500 + 20 + 30 * num_legs, anchor = 'center')


window1 = InputWindow()
window1.mainloop()    

