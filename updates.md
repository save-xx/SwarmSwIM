# version 0.4.0
### core changes
- Improved the import logic, it now use the following priority order: absolute path, user script directory, execution directory, package directory
- all plugins are now executed and returned in the main simulation tick()
- fully reworked and streamlined the plugin system
- Simulator.class is now a dictionary, using agents names as keys (#3)
- removed Dt from Agent class, now it in inhereted by Simulator (#4)
- iterator on simulator: the simulator is now iterable, returning the agents contained  as key:value
- Corrected "local_forces" planar motion to include the effect of Coriolis

### animation2D
- legend added (#8)
- optional input added `color_by_type` to select if the legend and colorscheme should collect agent of the same time togheter.
- changed represntation to proper NED (north up East right)
- added grid

### currents
- The currents are reworker as an optional plugin
- votex now takes an optional parameter size (default is 100m)
- agents can be set to ignore currents or accept only time independent. None, 'all', 'partial'
- noise class now directly inteacts with the agent's instance to store value