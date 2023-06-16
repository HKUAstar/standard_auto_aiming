import numpy as np

# Load the .npy file
data = np.load('./camera.npy')

# Save the data to a .txt file
np.savetxt('./camera.txt', data, delimiter=',')