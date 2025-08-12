#this goes from [0, 1] to [lower, upper]
def scale(x, lower, upper):
    return (x * (upper - lower) + lower)
#this goes from [lower, upper] to [0,1]
def unscale(x, lower, upper):
    return (((2.0 * x - upper - lower)/(upper - lower)) + 1)/2
