# -----------------------------------------------------------------------
#
#  MAKE-file to build the "solver_simple" example program,
#  demonstrating usage of the VehicleSim API.
#
#  Mechanical Simulation Corp.
# -----------------------------------------------------------------------

BIN_NAME = solver_simple

C_SRCS = \
solver_simple.c            \
../common/vs_get_api.c   \

INC_PATH = -I../common

OBJS   = $(C_SRCS:.c=.o)

solver_loop:	$(OBJS)
		echo "Building $(BIN_NAME)"
		gcc -o $(BIN_NAME) $(OBJS) -ldl -g3 -ggdb3
		echo "Copying to CarSim Data Folder Root"
		cp -ar $(BIN_NAME) ../../../
        
%.o:	%.c
		gcc -c $(INC_PATH) $< -o $@ -g3 -ggdb3

clean:	
		rm -f $(OBJS) $(BIN_NAME)

