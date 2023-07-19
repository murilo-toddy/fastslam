# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim, step
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)


def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
    new_dists = []
    for idx, val in enumerate(a.values):
        local_dist = []
        for err in b.values:
            local_dist.append(val * err)
        new_dists.append(Distribution(a.offset + b.offset + idx, local_dist))
    return Distribution.sum(new_dists)


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
     while prod != 0 and idx + left_idx < len(left.values) and idx < len(right.values):
          prod = right.values[idx] * left.values[idx + left_idx]
          new_values.append(prod)
          idx += 1

     created_dist = Distribution(right.offset, new_values)
     created_dist.normalize()
     return created_dist


if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    step(position.plotlists(*arena)[0], position.plotlists(*arena)[1])

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
        step(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        step(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r')

    show()
