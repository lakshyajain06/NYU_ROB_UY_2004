def interpolate_triangle(self, t, leg_index):
    ################################################################################################
    # TODO: implement interpolation for all 4 legs here
    ################################################################################################
    current = (t * 6)
    prev = np.floor(current)
    next = np.ceil(current)

    diff = current - prev

    lowest_to_next = self.ee_triangle_positions[leg_index][next] - self.ee_triangle_positions[leg_index][prev]
    
    interp_position = prev + lowest_to_next * diff

    return interp_position