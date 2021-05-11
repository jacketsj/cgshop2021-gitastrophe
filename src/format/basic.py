from cgshop2021_pyutils import Solution, SolutionStep, SolutionZipWriter, Direction
from cgshop2021_pyutils import Instance    

idb = InstanceDatabase("../../cgshop_2021_instances_01.zip")

for i in idb:
    if i.number_of_robots == 2:
        for r in range(i.number_of_robots):
            print("Robot", i, "starts at", i.start_of(r)," and has to go to ", i.target_of(r))

        for o in i.obstacles:
            print(o, "is blocked")
    solution = Solution(i)

    # First time step in the solution
    step_1 = SolutionStep()
    step_1[2]=Direction.NORTH # Robot 2 moves north
    step_1[1]=Direction.WEST # Robot 1 moves west
    solution.add_step(step_1)

    # Second time step in the solution
    step_2 = SolutionStep()
    step_2[0]=Direction.SOUTH # Robot 0 moves south
    step_2[2]=Direction.WEST # Robot 2 moves west
    solution.add_step(step_2)

    print("Makespan:", solution.makespan)

    print("SUM:", solution.total_moves)

    with SolutionZipWriter("output.zip") as szw:
        szw.add_solution(s1)
        szw.add_solutions([s1, s2])
