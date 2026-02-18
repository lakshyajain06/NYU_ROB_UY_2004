# External libraries
import pickle
import math
import matplotlib.pyplot as plt
import numpy as np
            
# Utility for loading saved data
class DataLoader:

    # Constructor
    def __init__(self, filename):
        self.filename = filename
        
    # Load a dictionary from file.
    def load(self):
        with open(self.filename, 'rb') as file_handle:
            loaded_dict = pickle.load(file_handle)
        return loaded_dict
        
def plot_leg_path(data_dictionary):
    time_stamp_list = data_dictionary['time_stamp']
    theta1_f_list = data_dictionary['theta1_f']
    theta2_f_list = data_dictionary['theta2_f']
    theta3_f_list = data_dictionary['theta3_f']
    theta1_b_list = data_dictionary['theta1_b']
    theta2_b_list = data_dictionary['theta2_b']
    theta3_b_list = data_dictionary['theta3_b']
    end_effector_position_f = np.array(data_dictionary['end_effector_position_f'])
    end_effector_position_b = data_dictionary['end_effector_position_b']
    x_ee_f =end_effector_position_f[:,0]
    y_ee_f =end_effector_position_f[:,1]
    z_ee_f =end_effector_position_f[:,2]


    # plt.plot(x_ee_f,z_ee_f)
    # plt.title('End Effector trajectory')
    # plt.xlabel('EE X(m)')
    # plt.ylabel('EE Z(m)')
    # plt.show()
    
#     plt.plot(x, y1, label='Sine Wave')
# plt.plot(x, y2, label='Cosine Wave')

# # 2. Add plt.legend() to display it
# plt.legend(loc='best'

    plt.plot(time_stamp_list,z_ee_f, label="Z")
    plt.plot(time_stamp_list,y_ee_f, label="Y")
    plt.plot(time_stamp_list,x_ee_f, label="X")
    plt.title('End Effector Coords vs Time')
    plt.legend(loc='best')
    plt.xlabel('Time(s)')
    plt.ylabel('Distance (m)')
    plt.savefig('my_plot.png')
    plt.show()



##### MAIN ######
data_loader = DataLoader('./lab_2_x_movement.pkl')
data_dictionary = data_loader.load()
plot_leg_path(data_dictionary)
