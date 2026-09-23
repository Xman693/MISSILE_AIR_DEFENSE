from PARAMATERS import get_radar_parameters, get_fc_params, get_simulation_parameters

radar_params = get_radar_parameters()
fc_params = get_fc_params()
sim_params = get_simulation_parameters()

dwell_time = radar_params["dwell_time"] # s
radar_beam_change_interval = radar_params["radar_beam_change_interval"] + dwell_time # s

radar_update_interval = radar_params["update_interval"] # s

fc_update_interval = fc_params["fc_update_interval"] # s

dt = sim_params["dt"] # s



radar_beam_change_due = False
radar_update_due = False
fc_update_due = False

    
def update_clocks(radar_beam_change_clock, radar_update_clock, fc_update_clock):
    
        if radar_beam_change_clock >= radar_beam_change_interval:
            radar_beam_change_clock = 0.0 # reset the radar beam change clock
            radar_beam_change_due = True
        else:
            radar_beam_change_due = False
            radar_beam_change_clock += dt
            
        if radar_update_clock >= radar_update_interval:
            radar_update_clock = 0.0 # reset the radar update clock
            radar_update_due = True
        else:
            radar_update_due = False
            radar_update_clock += dt
            
        if fc_update_clock >= fc_update_interval:
            fc_update_clock = 0.0 # reset the fc update clock
            fc_update_due = True
        else:
            fc_update_due = False
            fc_update_clock += dt

        return radar_beam_change_clock, radar_update_clock, fc_update_clock, radar_beam_change_due, radar_update_due, fc_update_due



