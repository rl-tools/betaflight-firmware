EXTERNAL_SRC_DIR := $(ROOT_DIR)/../src
VPATH := $(VPATH):$(EXTERNAL_SRC_DIR)

CXX_SRC = \
    rl_tools/policy.cpp
TARGET_OBJS += $(addsuffix .o,$(addprefix $(TARGET_OBJ_DIR)/,$(basename $(CXX_SRC))))
CXXFLAGS      = $(filter-out -std=gnu17,$(CFLAGS)) -fno-rtti -fno-exceptions -std=c++17 -I $(RL_TOOLS_ROOT)/include -I $(EXTERNAL_SRC_DIR)

$(TARGET_OBJ_DIR)/%.o: %.cpp
	$(V1) mkdir -p $(dir $@)
	@echo "%% (c++) $<" "$(STDOUT)"
	$(V1) $(CROSS_CXX) -c -o $@ $(CXXFLAGS) $<