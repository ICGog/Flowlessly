CXX = g++
LIBS = -lgflags -lglog
CPPFLAGS =
OPTFLAGS = -g -O0
OBJ_DIR = .

OBJS = graph.o min_cost_flow.o
BINS = flow_scheduler
OBJ_BIN = $(addprefix $(OBJ_DIR)/, $(BINS))

quiet-command = $(if $(V),$1,$(if $(2),@echo $2 && $1, @$1))

all: $(OBJ_BIN)

$(OBJ_DIR)/flow_scheduler: $(addprefix $(OBJ_DIR)/, $(OBJS))
	$(call quiet-command, \
		$(CXX) $(CPPFLAGS) flow_scheduler.cc $(OPTFLAGS) \
		graph.o min_cost_flow.o \
		$(LIBS) -o flow_scheduler, " DYNLNK flow_scheduler")

# Make object file (generic).
$(OBJ_DIR)/%.o: %.cc %.h .setup
	$(call quiet-command, \
		$(CXX) $(CPPFLAGS) $(OPTFLAGS) -c $< -o $@, "  CXX     $@")


clean:
	rm flow_scheduler
	rm graph.o
	rm min_cost_flow.o