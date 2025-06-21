CXX_SRC = \
    rl_tools/policy.cpp
TARGET_OBJS += $(addsuffix .o,$(addprefix $(TARGET_OBJ_DIR)/,$(basename $(CXX_SRC))))
CXXFLAGS      = $(filter-out -std=gnu17,$(CFLAGS)) -fno-rtti -fno-exceptions -std=c++17 -I $(RL_TOOLS_ROOT)

$(TARGET_OBJ_DIR)/%.o: %.cpp
	$(V1) mkdir -p $(dir $@)
	@echo "%% (c++) $<" "$(STDOUT)"
	$(V1) $(CROSS_CXX) -c -o $@ $(CXXFLAGS) $<