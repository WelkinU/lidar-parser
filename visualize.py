import numpy as np
import matplotlib.pyplot as plt #for plotting point cloud scatter plots

def plot_cloud(pcl: np.ndarray, pcl2: np.ndarray = None, title:str = None, plot_3d:bool = True, **kwargs):
    ''' Helper function: 3D scatter plots a point cloud. kwargs go into scatter() function'''
    
    fig = plt.figure(figsize = (7,7), dpi = 100)

    if plot_3d:
        ax = fig.add_subplot(projection='3d')
        ax.set_box_aspect([1,1,1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.scatter(pcl[:,0], pcl[:,1], pcl[:,2], **kwargs)
        if pcl2 is not None:
            ax.scatter(pcl2[:,0], pcl2[:,1], pcl2[:,2], marker = "*", s = 900, c = 'red', alpha = 0.9)

        ax.view_init(elev=90, azim=-95)
    else:
        ax = fig.add_subplot()
        ax.scatter(pcl[:,0], pcl[:,1], s = 1, **kwargs)
        if pcl2 is not None:
            ax.scatter(pcl2[:,0], pcl2[:,1], marker = "*", s = 1, c = 'red', alpha = 0.9)

        ax.grid()
        ax.set_aspect('equal', adjustable='box')

    if title is not None:
        ax.set_title(title)

    plt.tight_layout(pad=0)
    plt.show()