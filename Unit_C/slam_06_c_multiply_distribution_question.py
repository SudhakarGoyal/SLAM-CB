# Multiply a distribution by another distribution.
# 06_c_multiply_distribution
# Claus Brenner, 26 NOV 2012
from pylab import plot, show
from distribution import *

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
    arena = (0,1000)

    # Here is our assumed position. Plotted in blue.
    position_value = 400
    position_error = 100
    position = Distribution.triangle(position_value, position_error)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         color='b', linestyle='steps')

    # Here is our measurement. Plotted in green.
    # That is what we read from the instrument.
    measured_value = 510
    measurement_error = 200
    measurement = Distribution.triangle(measured_value, measurement_error)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='g', linestyle='steps')

    # Now, we integrate our sensor measurement. Result is plotted in red.
    position_after_measurement = multiply(position, measurement)
    plot(position_after_measurement.plotlists(*arena)[0],
         position_after_measurement.plotlists(*arena)[1],
         color='r', linestyle='steps')

    show()
