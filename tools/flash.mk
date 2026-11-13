###############################################################################
#  flash.mk —— 全局 J-Link 烧录规则
#
#  作用：给任何 STM32CubeMX 生成的 Makefile 工程补上 make flash
#        （CubeMX 的模板里根本没有 flash 目标，所以新工程第一次烧录必然
#         报 "No rule to make target 'flash'"）
#
#  安装（一次性）：
#    1) 本文件放到   D:\STM32CUBE\flash.mk
#    2) 命令行执行   setx MAKEFILES "D:\STM32CUBE\flash.mk"
#    3) 关掉所有终端重新打开（环境变量对已开的窗口不生效）
#
#  之后任何工程目录里都能直接：
#    make flash
#    make flash JLINK_SN=69400138
#    make flash JLINK_DEVICE=STM32H723ZG
#    make jlink-list
#
#  原理：GNU make 启动时会先读环境变量 MAKEFILES 里列出的所有文件
#        （文件不存在也只是静默跳过，不会报错），
#        而且这些文件里的目标 **不会** 被当成默认目标，
#        所以对一个工程的正常 make / make all 完全零影响。
#        好处是：CubeMX 重新生成 Makefile 时也不会把这个能力冲掉。
###############################################################################

# ---- 可覆盖的默认值（工程里若自己定义了同名变量，以工程为准）----
JLINK         ?= JLink.exe
JLINK_DEVICE  ?= STM32H723VG
JLINK_IF      ?= SWD
JLINK_SPEED   ?= 4000
JLINK_SN      ?=

JLINK_SCRIPT   = $(BUILD_DIR)/$(TARGET).flash.jlink
JLINK_LIST    ?= $(BUILD_DIR)/emu-list.jlink

.PHONY: flash download download_jlink jlink-flash jlink-list

# flash 只是个别名，真正的活干在 jlink-flash 里，
# 这样万一哪个工程自带 flash 目标，也不会把它的 recipe 顶掉。
flash download download_jlink: jlink-flash

jlink-flash:
	@$(MAKE) --no-print-directory all
	@echo "si $(JLINK_IF)"                        >  $(JLINK_SCRIPT)
	@echo "speed $(JLINK_SPEED)"                  >> $(JLINK_SCRIPT)
	@echo "device $(JLINK_DEVICE)"                >> $(JLINK_SCRIPT)
	@echo "connect"                               >> $(JLINK_SCRIPT)
	@echo "r"                                     >> $(JLINK_SCRIPT)
	@echo "h"                                     >> $(JLINK_SCRIPT)
	@echo "loadfile $(BUILD_DIR)/$(TARGET).hex"   >> $(JLINK_SCRIPT)
	@echo "r"                                     >> $(JLINK_SCRIPT)
	@echo "g"                                     >> $(JLINK_SCRIPT)
	@echo "qc"                                    >> $(JLINK_SCRIPT)
	@echo "==> burn $(TARGET).hex  ->  $(JLINK_DEVICE)  @ $(JLINK_IF) $(JLINK_SPEED)kHz"
	$(JLINK) -ExitOnError 1 $(if $(JLINK_SN),-SelectEmuBySN $(JLINK_SN),) -CommanderScript $(JLINK_SCRIPT)

# 查本机插着的所有 J-Link 序列号（换探针后先跑这个）
jlink-list:
	@echo "ShowEmuList" >  $(JLINK_LIST)
	@echo "qc"          >> $(JLINK_LIST)
	$(JLINK) -ExitOnError 0 -CommanderScript $(JLINK_LIST)
