#TODO: Separate debug/release mode w/ other than -O0

CXX       := g++
CXXFLAGS  := -std=c++17 -ggdb -Wall -Werror -Wextra -Wconversion -Wpedantic -pedantic-errors -O0
DEPFLAGS   = -MT $@ -MMD -MP -MF $*.d
INCLUDE   := -Ivendor/m/include
BINS      := atsc-cc-extractor
DEPFILES  := $(BINS:%=%.d)

.PHONY: all
all : $(BINS)

.PHONY: clean
clean :
	rm -f *.o *.d $(BINS)

# Remove default rule, add rule with dependency file
%.o : %.cpp
%.o : %.cpp %.d
	$(CXX) $(DEPFLAGS) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

$(BINS) : % : %.o
	$(CXX) $(CXXFLAGS) -o $@ $<

# Add targets for each dependency file
$(DEPFILES) :

# Include auto-generated dependency rules
include $(wildcard $(DEPFILES))
