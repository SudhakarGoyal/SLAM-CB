# For each cylinder in the scan, find its ray and depth.
# 03_c_find_cylinders
# Claus Brenner, 09 NOV 2012
from pylab import *
from lego_robot import *

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
    jumps = [ 0 ]
    for i in range(1, len(scan) - 1):
        l = scan[i-1]
        r = scan[i+1]
        if l > min_dist and r > min_dist:
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    return jumps

# For each area between a left falling edge and a right rising edge,
# determine the average ray number and the average depth.
def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0
    
    i = 0
    while (i < len(scan_derivative)):
        
    #for i in range(len(scan_derivative)):
        # --->>> Insert your cylinder code here.
        # Whenever you find a cylinder, add a tuple
        # (average_ray, average_depth) to the cylinder_list.
        #print(scan_derivative[i], i)
        if scan_derivative [i] >= jump:
            scan_derivative [i] = jump
        elif scan_derivative[i] <= -jump:
            scan_derivative[i] = -jump
        else:
            scan_derivative[i] = 0
            
        sum_ray = 0
        
        if (scan_derivative[i] == -jump):
            #print(i)
            on_cylinder = True
        else:
            on_cylinder = False
            
        
        while(on_cylinder and i < len(scan_derivative)-1):
            sum_ray += i
            sum_depth += scan[i]
            rays += 1
            prev_scan_der = False
            if scan_derivative[i] >= jump:
                on_cylinder = False
                
            if on_cylinder and scan_derivative[i] <= -jump :
                rays = 0
                sum_ray= 0
                sum_depth = 0
                
            i+=1
         
        
        if rays > 0 and not on_cylinder:
            #print(i)
            avg_ray = sum_ray / rays
            avg_depth = sum_depth / rays
            # Just for fun, I'll output some cylinders.
            # Replace this by your code.
           
            cylinder_list.append( (avg_ray, avg_depth) )
        rays = 0
        sum_ray= 0
        sum_depth = 0
        i+=1
    #print(len(cylinder_list))
    return cylinder_list


if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan = logfile.scan_data[6]

    # Find cylinders.
    der = compute_derivative(scan, minimum_valid_distance)
    
    cylinders = find_cylinders(scan, der, depth_jump,
                               minimum_valid_distance)

    # Plot results.
    plot(scan)
    plot(der)
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
        c='r', s=200)
    show()
