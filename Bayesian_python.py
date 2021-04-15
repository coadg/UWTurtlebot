import numpy as np 
import math as m
from matplotlib import pyplot as plt
from scipy.io import loadmat

def bay_filter(z, sigma = 0.0011, d_min = 0.1, d_max = 1,step= 0.01 ):
    """
    This function applies a bayesian filter for the vl53l1x TOF sensor.
    
    Input:
    z = The current sensor reading
    sigma = standard deviation
    d_min = minimum range value for the sensor in meters
    d_max = maximum range value for the sensor in meters
    step = The accuracy in meters
    
    Output:
    z_filter = the filtered sensor reading value
    
    """
    # Initialization
    x_range =np.arange(d_min,d_max,step)
    zmax = 1
    
    #range of possible values from the sensor
    d_range = np.arange(d_min,d_max,step)
    p = np.zeros((1,np.size(x_range)))
    
    for i in range(np.size(x_range)):
        
        #gaussian distribution for the sensor data over all of the values of x
        fx = 1/(sigma*np.sqrt(2*np.pi))*np.exp(-.5*np.square((x_range[i]-d_range)/sigma))
        
        #Limit the gaussian distribution to be within the range of the sensors
        fx = fx * (d_range <= zmax) * (d_range >= 0)
        
        #This normalizes the function
        eta1 = np.sum(fx*step)
        fx = fx/eta1
        
        """Failure component Both this and the random measurement I added just to 
        add variation to the probablility and weights. I am not sure what they are for"""
        #p3 = (d_range == zmax)
        #eta3 = np.sum(p3 * step)
        #p3 = p3/eta3
        
        #Random Measurement
        #p4 = 1/zmax * (d_range < zmax) * (d_range >= 0)
        
        #Probability weights
        w1 = 1; #0.95
        w3 = 0; #0.005
        w4 = 0; #0.045
        
        #Finds the index of the value in the range and then calculates the likelihood of the value
        Index =  np.argwhere(abs(d_range - z) == np.amin(abs(d_range - z)))
        
        prob = w1 * fx #+ w3 * p3 + w4 * p4
        p[0,i] = prob[Index[0]]
    
    #normalize to ensure it is normalized
    p = p/np.sum(p)

    #This is the original probability distribution for x0
    prior = np.ones((1,np.size(x_range)))*(1 - 0)
    
    #The final belief of the function
    belief = p * prior/np.sum(p*prior*step)
    
    #x_mean should be the filtered distance of the bot.
    x_mean = np.sum(belief*d_range)*step
    z_filter = x_mean
    return z_filter


#z_filter = bay_filter(.365)
#print('the answer is', z_filter)

#change this to test different values
sample_data = np.array((.304,.305,.305,.306,.306,.307,.305,.304,.306,.304,.305,.305,.306,.306,.307,.304,.306,.304,.305,.305,.305,.304,.306,.306))


filter_array = np.zeros((1,np.size(sample_data)))
time = np.linspace(0,10,np.size(sample_data))

for i in range(np.size(sample_data)):
    data = sample_data[i]
    z_filter =bay_filter(data)
    filter_array[0,i] = z_filter
    print('input',data,'output',z_filter)

print(filter_array)


plt.plot(time,sample_data,'b')
plt.plot(time,filter_array[0,:],'r--')
plt.legend(('Sample data','Filtered data'))
plt.show()

