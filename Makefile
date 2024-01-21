CXX           := g++
CXXFLAGS_COM  := -std=c++17 -Wall -Werror -Wextra -Wconversion -Wpedantic -pedantic-errors -march=native -mtune=native
CXXFLAGS_REL  := -O3
CXXFLAGS_DBG  := -ggdb -O0
CXXFLAGS_SAN  := -fsanitize=address -fsanitize=leak
DEPFLAGS       = -MT $@ -MMD -MP -MF $*
INCLUDE       := -Ivendor/m/include
BINS          := atsc-cc-extractor
DEPFILES      := $(BINS:%=%.rel.d) $(BINS:%=%.dbg.d) $(BINS:%=%.san.d)

.PHONY: all
all : $(BINS) $(BINS:%=%_dbg) $(BINS:%=%_san)

.PHONY: clean
clean :
	rm -f *.o *.d $(BINS) $(BINS:%=%_dbg) $(BINS:%=%_san)

$(BINS) : % : %.rel.o
	$(CXX) $(CXXFLAGS_COM) $(CXXFLAGS_REL) -o $@ $<
.SECONDEXPANSION:
$(BINS:%=%_dbg) : % : $$(subst _dbg,,%).dbg.o
	$(CXX) $(CXXFLAGS_COM) $(CXXFLAGS_DBG) -o $@ $<
$(BINS:%=%_san) : % : $$(subst _san,,%).san.o
	$(CXX) $(CXXFLAGS_COM) $(CXXFLAGS_DBG) $(CXXFLAGS_SAN) -o $@ $<

# Remove default rule, add rules with dependency file
%.o : %.cpp
%.rel.o : %.cpp %.rel.d
	$(CXX) $(DEPFLAGS).rel.d $(CXXFLAGS_COM) $(CXXFLAGS_REL) $(INCLUDE) -c $< -o $@
%.dbg.o : %.cpp %.dbg.d
	$(CXX) $(DEPFLAGS).dbg.d $(CXXFLAGS_COM) $(CXXFLAGS_DBG) $(INCLUDE) -c $< -o $@
%.san.o : %.cpp %.san.d
	$(CXX) $(DEPFLAGS).san.d $(CXXFLAGS_COM) $(CXXFLAGS_DBG) $(CXXFLAGS_SAN) $(INCLUDE) -c $< -o $@

# Add targets for each dependency file
$(DEPFILES) :

# Include auto-generated dependency rules
include $(wildcard $(DEPFILES))
