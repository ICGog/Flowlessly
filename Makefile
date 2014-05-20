CXX = g++
LIBS = -lgflags -lglog
CPPFLAGS =
OPTFLAGS = -g -O3
OBJ_DIR = .

OBJS = arc.o cost_scaling.o cycle_cancelling.o graph.o statistics.o \
	successive_shortest.o utils.o
BINS = flow_scheduler
OBJ_BIN = $(addprefix $(OBJ_DIR)/, $(BINS))

quiet-command = $(if $(V),$1,$(if $(2),@echo $2 && $1, @$1))

all: $(OBJ_BIN)

$(OBJ_DIR)/flow_scheduler: $(addprefix $(OBJ_DIR)/, $(OBJS))
	$(call quiet-command, \
		$(CXX) $(CPPFLAGS) flow_scheduler.cc $(OPTFLAGS) \
		arc.o cost_scaling.o cycle_cancelling.o graph.o \
		statistics.o successive_shortest.o utils.o \
		$(LIBS) -o flow_scheduler, " DYNLNK flow_scheduler")

# Make object file (generic).
$(OBJ_DIR)/%.o: %.cc %.h
	$(call quiet-command, \
		$(CXX) $(CPPFLAGS) $(OPTFLAGS) -c $< -o $@, "  CXX     $@")

clean:
	rm -f flow_scheduler
	rm -f arc.o
	rm -f cost_scaling.o
	rm -f cycle_cancelling.o
	rm -f graph.o
	rm -f statistics.o
	rm -f successive_shortest.o
	rm -f utils.o
