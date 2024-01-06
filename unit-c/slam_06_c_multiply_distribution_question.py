# Multiply a distribution by another distribution.
# 06_c_multiply_distribution
# Claus Brenner, 26 NOV 2012
from venv import create
from pylab import plot, show, step
from distribution import *

def multiply(a, b):
     """Multiply two distributions and return the resulting distribution."""
     if a.offset > b.offset:
          left = b
          right = a
     else:
          left = a
          right = b

     new_values = []
     left_idx = right.offset - left.offset
     prod = 1
     idx = 0
     while prod != 0 and idx < len(left.values) and idx < len(right.values):
          prod = right.values[idx] * left.values[idx + left_idx]
          new_values.append(prod)
          idx += 1

     created_dist = Distribution(right.offset, new_values)
     created_dist.normalize()
     return created_dist


if __name__ == '__main__':
    arena = (0,1000)

    # Here is our assumed position. Plotted in blue.
    position_value = 400
    position_error = 100
    position = Distribution.triangle(position_value, position_error)
    step(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         color='b')

    # Here is our measurement. Plotted in green.
    # That is what we read from the instrument.
    measured_value = 410
    measurement_error = 200
    measurement = Distribution.triangle(measured_value, measurement_error)
    step(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='g')

    # Now, we integrate our sensor measurement. Result is plotted in red.
    position_after_measurement = multiply(position, measurement)
    step(position_after_measurement.plotlists(*arena)[0],
         position_after_measurement.plotlists(*arena)[1],
         color='r')

    show()
