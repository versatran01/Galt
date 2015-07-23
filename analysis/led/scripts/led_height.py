import ortools.linear_solver.pywraplp as plp

solver = plp.Solver('SolveLedHeigth', plp.Solver.GLOP_LINEAR_PROGRAMMING)

# Constants
bed_height = 0.8
bed_width = 1.32
bed_depth = 0.29

row_width = 2.75
tree_height = 3.5
tree_lower = 0.25
tree_upper = tree_height - 0.5

truck_width = 1.52
k = 1.25

# Create two variables in our
led_height = solver.NumVar(0.1, 0.5 * (tree_lower + tree_upper) - bed_height,
                           'led_height')
led_offset = solver.NumVar(0, 0.5 * bed_width, 'led_offset')

# Constraint 1: led should at least see the bottom of the tree
constraint1_upper = tree_lower + k / 2 * (row_width - bed_width) - bed_height
constraint1 = solver.Constraint(-solver.infinity(), constraint1_upper)
constraint1.SetCoefficient(led_height, 1)
constraint1.SetCoefficient(led_offset, -k)

# Constraint 2: led should at least see the top of the tree
constraint2_lower = tree_upper - bed_height - k / 2 * (row_width - bed_width)
constraint2 = solver.Constraint(constraint2_lower, solver.infinity())
constraint2.SetCoefficient(led_height, 1)
constraint2.SetCoefficient(led_offset, k)

# Constraint 3: led should not be blocked by the bed
constraint3_lower = bed_depth
constraint3 = solver.Constraint(constraint3_lower, solver.infinity())
constraint3.SetCoefficient(led_height, 1)
constraint3.SetCoefficient(led_offset, -k)

# Minimize led_height
objective = solver.Objective()
objective.SetCoefficient(led_height, 1)
objective.SetMaximization()

status = solver.Solve()
print('led_height = ', led_height.solution_value())
print('led_offset = ', led_offset.solution_value())

