    def print_grid(self,grid,size_x):
	for(i in range(len(grid))):
		if(i%size_x == 0):
			print("")
		if(grid[i] > -1):
			print("O", end="", flush=True)
		else:
			print("#", end="", flush=True)