import os
from PIL import Image
import numpy as np

import yaml


from copy import copy

from queue import PriorityQueue



class Queue():
    def __init__(self, init_queue = []):
        self.queue = copy(init_queue)
        self.start = 0
        self.end = len(self.queue)-1
    
    def __len__(self):
        numel = len(self.queue)
        return numel
    
    def __repr__(self):
        q = self.queue
        tmpstr = ""
        for i in range(len(self.queue)):
            flag = False
            if(i == self.start):
                tmpstr += "<"
                flag = True
            if(i == self.end):
                tmpstr += ">"
                flag = True
            
            if(flag):
                tmpstr += '| ' + str(q[i]) + '|\n'
            else:
                tmpstr += ' | ' + str(q[i]) + '|\n'
            
        return tmpstr
    
    def __call__(self):
        return self.queue
    
    def initialize_queue(self,init_queue = []):
        self.queue = copy(init_queue)
    
    def sort(self,key=str.lower):
        self.queue = sorted(self.queue,key=key)
        
    def push(self,data):
        self.queue.append(data)
        self.end += 1
    
    def pop(self):
        p = self.queue.pop(self.start)
        self.end = len(self.queue)-1
        return p
    
class Node():
    def __init__(self,name):
        self.name = name
        self.children = []
        self.weight = []
        
    def __repr__(self):
        return self.name
        
    def add_children(self,node,w=None):
        if w == None:
            w = [1]*len(node)
        self.children.extend(node)
        self.weight.extend(w)
    
class Tree():
    def __init__(self,name):
        self.name = name
        self.root = 0
        self.end = 0
        self.g = {}
    #     self.g_visual = Graph('G')
    
    # def __call__(self):
    #     for name,node in self.g.items():
    #         if(self.root == name):
    #             self.g_visual.node(name,name,color='red')
    #         elif(self.end == name):
    #             self.g_visual.node(name,name,color='blue')
    #         else:
    #             self.g_visual.node(name,name)
    #         for i in range(len(node.children)):
    #             c = node.children[i]
    #             w = node.weight[i]
    #             #print('%s -> %s'%(name,c.name))
    #             if w == 0:
    #                 self.g_visual.edge(name,c.name)
    #             else:
    #                 self.g_visual.edge(name,c.name,label=str(w))
        return #self.g_visual
    
    def add_node(self, node, start = False, end = False):
        self.g[node.name] = node
        if(start):
            self.root = node.name
        elif(end):
            self.end = node.name
            
    def set_as_root(self,node):
        # These are exclusive conditions
        self.root = True
        self.end = False
    
    def set_as_end(self,node):
        # These are exclusive conditions
        self.root = False
        self.end = True   
        
class AStar():
    def __init__(self,in_tree):
        self.in_tree = in_tree
        self.q = Queue()
        self.dist = {name:np.inf for name,node in in_tree.g.items()}
        self.h = {name:0 for name,node in in_tree.g.items()}
        
        self.via = {name: None for name in in_tree.g}
        for __,node in in_tree.g.items():
            self.q.push(node)
     
    def __get_f_score(self,node):
        return self.dist[node] + self.h[node]
    
    def solve(self, sn, en):

        open_set = PriorityQueue()

        entry_count = 0
        self.dist[sn.name] = 0
        start_f_score = self.__get_f_score(sn.name)
        
        open_set.put((start_f_score, entry_count, sn))

        entry_count += 1

        while not open_set.empty():
            #  3rd position (index 2) !!!
            current_node = open_set.get()[2]

            #check if note i Goal
            if current_node.name == en.name:
                return self.reconstruct_path(sn.name, en.name)

            # Explore childs 
            for i, child_node in enumerate(current_node.children):
                weight = current_node.weight[i]
                tentative_g_score = self.dist[current_node.name] + weight

                if tentative_g_score < self.dist[child_node.name]:
                    self.via[child_node.name] = current_node.name
                    self.dist[child_node.name] = tentative_g_score
                    #steps + disstance
                    #f_score = tentative_g_score + self.h[child_node.name]
                    f_score = self.__get_f_score(child_node.name) 
                    
                    #add to set
                    open_set.put((f_score, entry_count, child_node))
                    entry_count += 1

        return [], np.inf
    
    def reconstruct_path(self,sn,en):
        end_name = en.name if hasattr(en, 'name') else en
        #go back on path in dir start to fin the path
        path = []
        dist = self.dist[end_name]

        current = end_name
        while current is not None:
            path.append(current)
            current = self.via[current]
        
        # path is reverse, so flip it 
        return path[::-1], dist
    
class Map():

    def __init__(self, name):
        with open(name, 'r') as f:
            self.map_yaml = yaml.safe_load(f)

        map_directory = os.path.dirname(name)
        image_filename = self.map_yaml['image']
        self.image_file_name = os.path.join(map_directory, image_filename)
        
        self.resolution = self.map_yaml['resolution']
        self.origin = self.map_yaml['origin']
        self.negate = self.map_yaml['negate']
        self.occupied_thresh = self.map_yaml['occupied_thresh']
        self.free_thresh = self.map_yaml['free_thresh']
        
        map_image = Image.open(self.image_file_name)
        raw_image_array = np.array(map_image)
        
        self.height = raw_image_array.shape[0]

        free_pixel_threshold = int(255 * (1 - self.free_thresh))

        # Start with a grid where everything is an obstacle (value = 1).
        grid = np.ones_like(raw_image_array, dtype=int)

        grid[raw_image_array > free_pixel_threshold] = 0
        
        self.image_array = grid



class MapProcessor():
    def __init__(self,name):
        self.map = Map(name)
        self.inf_map_img_array = np.copy(self.map.image_array) 
        self.map_graph = Tree(name)

    # Add this method inside your MapProcessor class
    def visualize_map_array(self, map_array, title="Map"):
        """
        Displays the given map array using matplotlib.
        Obstacles (1) will be black, Free space (0) will be white.
        """
        try:
            import matplotlib.pyplot as plt
            plt.imshow(map_array, cmap='gray_r') # gray_r makes 0=white, 1=black
            plt.title(title)
            plt.show()
        except ImportError:
            print("Matplotlib is not installed. Please run 'pip install matplotlib' to visualize the map.")
    
    def __modify_map_pixel(self,map_array,i,j,value,absolute):
        if( (i >= 0) and 
            (i < map_array.shape[0]) and 
            (j >= 0) and
            (j < map_array.shape[1]) ):
            if absolute:
                map_array[i][j] = value
            else:
                map_array[i][j] += value 
    
    def __inflate_obstacle(self,kernel,map_array,i,j,absolute):
        dx = int(kernel.shape[0]//2)
        dy = int(kernel.shape[1]//2)
        if (dx == 0) and (dy == 0):
            self.__modify_map_pixel(map_array,i,j,kernel[0][0],absolute)
        else:
            for k in range(i-dx,i+dx):
                for l in range(j-dy,j+dy):
                    self.__modify_map_pixel(map_array,k,l,kernel[k-i+dx][l-j+dy],absolute)
        
    def inflate_map(self,kernel,absolute=True):
        # Perform an operation like dilation, such that the small wall found during the mapping process
        # are increased in size, thus forcing a safer path.
        self.inf_map_img_array = np.copy(self.map.image_array)
        
        # We need to find the locations of the original obstacles (value=1)
        obstacle_indices = np.where(self.map.image_array == 1)
        
        # Now, for each original obstacle, inflate it onto our new map
        for i, j in zip(*obstacle_indices):
            self.__inflate_obstacle(kernel, self.inf_map_img_array, i, j, absolute)
            
        self.inf_map_img_array[self.inf_map_img_array > 0] = 1
                
    def get_graph_from_map(self):
        # Create the nodes that will be part of the graph, considering only valid nodes or the free space
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    node = Node('%d,%d'%(i,j))
                    self.map_graph.add_node(node)
        # Connect the nodes through edges
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:                    
                    if (i > 0):
                        if self.inf_map_img_array[i-1][j] == 0:
                            # add an edge up
                            child_up = self.map_graph.g['%d,%d'%(i-1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up],[1])
                    if (i < (self.map.image_array.shape[0] - 1)):
                        if self.inf_map_img_array[i+1][j] == 0:
                            # add an edge down
                            child_dw = self.map_graph.g['%d,%d'%(i+1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw],[1])
                    if (j > 0):
                        if self.inf_map_img_array[i][j-1] == 0:
                            # add an edge to the left
                            child_lf = self.map_graph.g['%d,%d'%(i,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_lf],[1])
                    if (j < (self.map.image_array.shape[1] - 1)):
                        if self.inf_map_img_array[i][j+1] == 0:
                            # add an edge to the right
                            child_rg = self.map_graph.g['%d,%d'%(i,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_rg],[1])
                    if ((i > 0) and (j > 0)):
                        if self.inf_map_img_array[i-1][j-1] == 0:
                            # add an edge up-left 
                            child_up_lf = self.map_graph.g['%d,%d'%(i-1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_lf],[np.sqrt(2)])
                    if ((i > 0) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i-1][j+1] == 0:
                            # add an edge up-right
                            child_up_rg = self.map_graph.g['%d,%d'%(i-1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_rg],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j > 0)):
                        if self.inf_map_img_array[i+1][j-1] == 0:
                            # add an edge down-left 
                            child_dw_lf = self.map_graph.g['%d,%d'%(i+1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_lf],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i+1][j+1] == 0:
                            # add an edge down-right
                            child_dw_rg = self.map_graph.g['%d,%d'%(i+1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_rg],[np.sqrt(2)])                    
        
    def gaussian_kernel(self, size, sigma=1):
        size = int(size) // 2
        x, y = np.mgrid[-size:size+1, -size:size+1]
        normal = 1 / (2.0 * np.pi * sigma**2)
        g =  np.exp(-((x**2 + y**2) / (2.0*sigma**2))) * normal
        r = np.max(g)-np.min(g)
        sm = (g - np.min(g))*1/r
        return sm
    
    def rect_kernel(self, size, value):
        m = np.ones(shape=(size,size))
        return m
    
    def draw_path(self,path):
        path_tuple_list = []
        path_array = copy(self.inf_map_img_array)
        for idx in path:
            tup = tuple(map(int, idx.split(',')))
            path_tuple_list.append(tup)
            path_array[tup] = 0.5
        return path_array


if __name__ == '__main__':
    # === ROBUST PATH FIX START ===
    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_file_path = os.path.join(os.path.dirname(script_dir), 'maps', 'classroom_map.yaml')
    # === ROBUST PATH FIX END ===

    processor = MapProcessor(map_file_path)

    print(f"Successfully loaded map from: {map_file_path}")

    print("\nDisplaying initial map from Map class...")
    print("Obstacles should appear as BLACK pixels.")
    processor.visualize_map_array(processor.map.image_array, title="1. Initial Loaded Map")

    unique_vals = np.unique(processor.map.image_array)
    print(f"Unique values in initial map array: {unique_vals}")
    if len(unique_vals) < 2:
        print("!! ERROR: Obstacles are not being detected correctly in the Map class.")

    print("\nInflating map...")
    kernel = processor.rect_kernel(size=5, value=1)
    processor.inflate_map(kernel)

    print("Displaying inflated map...")
    print("Obstacles (walls) should be thicker now.")
    processor.visualize_map_array(processor.inf_map_img_array, title="2. Inflated Map")