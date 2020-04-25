# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)



# --->>> Copy your convolve(a, b) and multiply(a, b) functions here.
def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
        # --->>> Put your code here.

    c = [0]*( len(a.values) + len(b.values) - 1)
    print("len of c", len(c))
    for i in range(len(a.values)):
        for j in range(len(b.values)):
            c[i+j] += a.values[i] * b.values[j] 
    
    result  = Distribution()
    result.values = c
    result.offset = a.offset + b.offset

    return result # Replace this by your own result.


def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""

    # --->>> Put your code here.
    offset = 0
    if len(a.values) < len(b.values):
        offset = a.offset
    else:
        offset = b.offset
    
    stop = 0
    if a.offset + len(a.values) < b.offset + len(b.values):
        stop = a.offset + len(a.values)
    else:
        stop = b.offset + len(b.values)
        
    result = [0] * (stop - offset + 1)    
    for i in range(offset, stop):
        result[i - offset] = a.values[i - a.offset] * b.values[i - b.offset]
        
    
    c = Distribution(offset, result)
    c.normalize()
    return c  # Modify this to return your result.


if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in range(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        print(position.values)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        print(position.values)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()
