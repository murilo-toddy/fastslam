# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# Claus Brenner, 26 NOV 2012
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


if __name__ == '__main__':
    arena = (0,100)

    # Move 3 times by 20.
    moves = [20] * 3

    # Start with a known position: probability 1.0 at position 10.
    position = Distribution.unit_pulse(10)
    step(position.plotlists(*arena)[0], position.plotlists(*arena)[1])
    
    # Now move and plot.
    for m in moves:
        move_distribution = Distribution.triangle(m, 2)
        position = convolve(position, move_distribution)
        step(position.plotlists(*arena)[0], position.plotlists(*arena)[1])

    ylim(0.0, 1.1)
    show()
